#ifndef USB_HOST_H
#define USB_HOST_H

#include "EspUsbHost.h"
#include "usb/usb_host.h"
#include "gamepad.h"

class GamepadHost : public EspUsbHost {
  usb_transfer_t *ledXfer = NULL;
  usb_transfer_t *hidSetupXfer = NULL;
  usb_transfer_t *hidGetReportXfer = NULL;
  uint8_t outEpAddr = 0;
  uint8_t outEpLen = 64;
  uint16_t vendorId = 0;
  uint16_t productId = 0;
  bool ledSent = false;
  bool inputStarted = false;
  uint32_t startInputAt_ms = 0;
  uint32_t nextLedAt_ms = 0;
  uint8_t ledR = 0, ledG = 0, ledB = 255;
  uint8_t initPhase = 0;
  // initPhase: 0=SECTION_ERR, 1=SECTION_ERR, 2=SECTION_ERR, 3=SECTION_ERR, 4=start_IN
  uint8_t inEpAddr = 0;
  uint8_t inRetryCount = 0;
  uint8_t hidInterface = 0xFF;

  void printDeviceIdOnce() {
    if (!deviceHandle || vendorId != 0 || productId != 0) return;

    const usb_device_desc_t *deviceDesc = nullptr;
    if (usb_host_get_device_descriptor(deviceHandle, &deviceDesc) != ESP_OK || deviceDesc == nullptr) {
      Serial.println("Gamepad VID/PID: unavailable");
      return;
    }

    vendorId = deviceDesc->idVendor;
    productId = deviceDesc->idProduct;
    Serial.printf("Gamepad VID:0x%04X PID:0x%04X\n", vendorId, productId);
  }

  void fillPs4LedReport(uint8_t *report, uint8_t reportLen, uint8_t red, uint8_t green, uint8_t blue) {
    memset(report, 0, reportLen);
    report[0] = 0x05; // USB output report ID for DualShock 4
    report[1] = 0xFF; // enable rumble + LED update bits
    report[6] = red;
    report[7] = green;
    report[8] = blue;
  }

  // Beberapa format LED yang diketahui untuk DS4 original dan clone.
  // Diuji bergantian tiap kali ledSent di-reset untuk riset kompatibilitas.
  static constexpr uint8_t kLedFormats[][12] = {
    // {reportId, b1,   b2,   b3,   b4,   b5,   red,  grn,  blu}
    { 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },  // 0: DS4 original (valid_flag0=0x03)
    { 0x05, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },  // 1: DS4 variant (0xFF)
    { 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },  // 2: DS4 variant
    { 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },  // 3: DS4 clone A
    { 0x11, 0xC0, 0x20, 0xF3, 0x04, 0xFF, 0x00, 0x00, 0xFF },  // 4: DS4 BT format over USB
    { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },  // 5: generic HID
  };
  uint8_t ledFormatIdx = 0;

  void sendLedReport(uint8_t red, uint8_t green, uint8_t blue) {
    if (!deviceHandle || ledXfer || outEpAddr == 0 || outEpLen < 11) return;
    if (usb_host_transfer_alloc(outEpLen, 0, &ledXfer) != ESP_OK) return;

    memset(ledXfer->data_buffer, 0, outEpLen);
    const uint8_t *fmt = kLedFormats[ledFormatIdx];
    ledXfer->data_buffer[0] = fmt[0];
    ledXfer->data_buffer[1] = fmt[1];
    ledXfer->data_buffer[2] = fmt[2];
    ledXfer->data_buffer[3] = fmt[3];
    ledXfer->data_buffer[4] = fmt[4];
    ledXfer->data_buffer[5] = fmt[5];
    ledXfer->data_buffer[6] = fmt[6] ? fmt[6] : red;
    ledXfer->data_buffer[7] = fmt[7] ? fmt[7] : green;
    ledXfer->data_buffer[8] = fmt[8] ? fmt[8] : blue;

    Serial.printf("LED format[%u] report[0]=0x%02X report[1]=0x%02X\n",
                  ledFormatIdx, ledXfer->data_buffer[0], ledXfer->data_buffer[1]);

    ledXfer->device_handle = deviceHandle;
    ledXfer->bEndpointAddress = outEpAddr;
    ledXfer->callback = onLedReportDone;
    ledXfer->context = this;
    ledXfer->num_bytes = outEpLen;

    if (usb_host_transfer_submit(ledXfer) != ESP_OK) {
      usb_host_transfer_free(ledXfer);
      ledXfer = NULL;
    }
  }

  void sendHidSetProtocol() {
    if (!deviceHandle || hidSetupXfer || ifNum == 0xFF) return;
    if (usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE, 0, &hidSetupXfer) != ESP_OK) return;

    usb_setup_packet_t setup = {};
    setup.bmRequestType = 0x21; // Host-to-device, class, interface
    setup.bRequest = 0x0B;      // HID SET_PROTOCOL
    setup.wValue = 0x0000;      // boot protocol attempt; fallback keeps SET_IDLE path
    setup.wIndex = ifNum;
    setup.wLength = 0;

    memcpy(hidSetupXfer->data_buffer, setup.val, USB_SETUP_PACKET_SIZE);
    hidSetupXfer->device_handle = deviceHandle;
    hidSetupXfer->callback = onHidSetProtocolDone;
    hidSetupXfer->context = this;
    hidSetupXfer->num_bytes = USB_SETUP_PACKET_SIZE;

    const esp_err_t err = usb_host_transfer_submit_control(clientHandle, hidSetupXfer);
    Serial.printf("SET_PROTOCOL submit err:%d\n", (int)err);
    if (err != ESP_OK) {
      usb_host_transfer_free(hidSetupXfer);
      hidSetupXfer = NULL;
      sendHidSetIdle();
    }
  }

  static void onHidSetProtocolDone(usb_transfer_t *t) {
    GamepadHost *h = (GamepadHost *)t->context;
    Serial.printf("SET_PROTOCOL status:%d bytes:%d\n", (int)t->status, t->actual_num_bytes);
    usb_host_transfer_free(t);
    h->hidSetupXfer = NULL;
    h->sendHidSetIdle();
  }

  void sendHidSetIdle() {
    if (!deviceHandle || hidSetupXfer || ifNum == 0xFF) return;
    if (usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE, 0, &hidSetupXfer) != ESP_OK) return;

    usb_setup_packet_t setup = {};
    setup.bmRequestType = 0x21; // Host-to-device, class, interface
    setup.bRequest = 0x0A;      // HID SET_IDLE
    setup.wValue = 0x0000;      // duration 0, all reports
    setup.wIndex = ifNum;
    setup.wLength = 0;

    memcpy(hidSetupXfer->data_buffer, setup.val, USB_SETUP_PACKET_SIZE);
    hidSetupXfer->device_handle = deviceHandle;
    hidSetupXfer->callback = onHidSetupDone;
    hidSetupXfer->context = this;
    hidSetupXfer->num_bytes = USB_SETUP_PACKET_SIZE;

    const esp_err_t err = usb_host_transfer_submit_control(clientHandle, hidSetupXfer);
    Serial.printf("SET_IDLE submit err:%d\n", (int)err);
    if (err != ESP_OK) {
      usb_host_transfer_free(hidSetupXfer);
      hidSetupXfer = NULL;
      startInputAt_ms = millis() + 300;
    }
  }

  static void onHidSetupDone(usb_transfer_t *t) {
    GamepadHost *h = (GamepadHost *)t->context;
    Serial.printf("SET_IDLE status:%d bytes:%d\n", (int)t->status, t->actual_num_bytes);
    usb_host_transfer_free(t);
    h->hidSetupXfer = NULL;
    h->initPhase = 0;
    h->nextInitPhase();
  }

  void sendHidGetFeature(uint8_t reportId, uint16_t length) {
    if (!deviceHandle || hidGetReportXfer || ifNum == 0xFF) return;
    if (usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE + length, 0, &hidGetReportXfer) != ESP_OK) return;

    usb_setup_packet_t setup = {};
    setup.bmRequestType = 0xA1; // Device-to-host, class, interface
    setup.bRequest = 0x01;      // HID GET_REPORT (feature)
    setup.wValue = (0x0300) | reportId; // feature report
    setup.wIndex = ifNum;
    setup.wLength = length;

    memcpy(hidGetReportXfer->data_buffer, setup.val, USB_SETUP_PACKET_SIZE);
    hidGetReportXfer->device_handle = deviceHandle;
    hidGetReportXfer->callback = onHidGetFeatureDone;
    hidGetReportXfer->context = this;
    hidGetReportXfer->num_bytes = USB_SETUP_PACKET_SIZE + length;

    const esp_err_t err = usb_host_transfer_submit_control(clientHandle, hidGetReportXfer);
    Serial.printf("GET_FEATURE(ID=0x%02X) submit err:%d\n", reportId, (int)err);
    if (err != ESP_OK) {
      usb_host_transfer_free(hidGetReportXfer);
      hidGetReportXfer = NULL;
      // skip to next phase on error
      nextInitPhase();
    }
  }

  void nextInitPhase() {
    initPhase++;
    switch (initPhase) {
      case 1: sendHidGetFeature(0x02, 37); break; // calibration
      case 2: sendHidGetFeature(0x12, 16); break; // pairing info
      case 3: sendHidGetFeature(0xA3, 49); break; // firmware info
      default:
        startInputAt_ms = millis() + 300; // all done → start IN
        break;
    }
  }

  static void onHidGetFeatureDone(usb_transfer_t *t) {
    GamepadHost *h = (GamepadHost *)t->context;
    Serial.printf("GET_FEATURE status:%d bytes:%d\n", (int)t->status, t->actual_num_bytes);
    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > USB_SETUP_PACKET_SIZE) {
      uint8_t *data = t->data_buffer + USB_SETUP_PACKET_SIZE;
      int len = t->actual_num_bytes - USB_SETUP_PACKET_SIZE;
      Serial.print("FEATURE DATA:");
      for (int i = 0; i < (len < 16 ? len : 16); i++) Serial.printf(" %02X", data[i]);
      Serial.printf(" len:%d\n", len);
    }
    usb_host_transfer_free(t);
    h->hidGetReportXfer = NULL;
    h->nextInitPhase();
  }
  static void onLedReportDone(usb_transfer_t *t) {
    GamepadHost *h = (GamepadHost *)t->context;
    Serial.printf("LED report status:%d bytes:%d format[%u]\n",
                  (int)t->status, t->actual_num_bytes, h->ledFormatIdx);
    usb_host_transfer_free(t);
    h->ledXfer = NULL;
    // stop cycling — format[0] sudah cukup untuk DS4 original maupun clone ini
  }

public:
  usb_transfer_t *xfer = NULL;
  uint8_t epLen = 64;
  uint8_t ifNum = 0xFF;

  void cleanup() {
    if (xfer) { usb_host_transfer_free(xfer); xfer = NULL; }
    if (ledXfer) { usb_host_transfer_free(ledXfer); ledXfer = NULL; }
    if (hidSetupXfer) { usb_host_transfer_free(hidSetupXfer); hidSetupXfer = NULL; }
    if (hidGetReportXfer) { usb_host_transfer_free(hidGetReportXfer); hidGetReportXfer = NULL; }
    outEpAddr = 0;
    vendorId = 0;
    productId = 0;
    ledSent = false;
    inputStarted = false;
    ledFormatIdx = 0;
    startInputAt_ms = 0;
    nextLedAt_ms = 0;
    inEpAddr = 0;
    inRetryCount = 0;
    hidInterface = 0xFF;
    if (ifNum != 0xFF && deviceHandle) {
      usb_host_interface_release(clientHandle, deviceHandle, ifNum);
      ifNum = 0xFF;
    }
    if (deviceHandle) { usb_host_device_close(clientHandle, deviceHandle); deviceHandle = NULL; }
    gp.connected = false;
  }

  void onDisconnect(void) override {
    cleanup();
    Serial.println("Disconnected. Waiting...");
    ESP.restart();
  }

  void onConfig(const uint8_t type, const uint8_t *p) override {
    if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      auto *intf = (const usb_intf_desc_t *)p;
      Serial.printf("IF num:%u class:0x%02X subclass:0x%02X proto:0x%02X eps:%u\n",
                    intf->bInterfaceNumber, intf->bInterfaceClass,
                    intf->bInterfaceSubClass, intf->bInterfaceProtocol,
                    intf->bNumEndpoints);
      if (intf->bInterfaceClass != 0x03 || hidInterface != 0xFF) return;
      if (usb_host_interface_claim(clientHandle, deviceHandle, intf->bInterfaceNumber, 0) == ESP_OK) {
        ifNum = intf->bInterfaceNumber;
        hidInterface = intf->bInterfaceNumber;
        Serial.printf("Claim HID IF:%u\n", ifNum);
      }
    }
    if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      printDeviceIdOnce();
      auto *ep = (const usb_ep_desc_t *)p;
      const uint8_t transferType = ep->bmAttributes & 3;
      Serial.printf("EP addr:0x%02X attr:0x%02X type:%u max:%u interval:%u\n",
                    ep->bEndpointAddress, ep->bmAttributes, transferType,
                    ep->wMaxPacketSize, ep->bInterval);
      if (transferType != 3) return;
      if (hidInterface == 0xFF) return;

      if (ep->bEndpointAddress & 0x80) {
        if (xfer != NULL) return;
        epLen = ep->wMaxPacketSize;
        if (usb_host_transfer_alloc(epLen, 0, &xfer) != ESP_OK) return;
        inEpAddr = ep->bEndpointAddress;
        xfer->device_handle = deviceHandle;
        xfer->bEndpointAddress = inEpAddr;
        xfer->callback = onData;
        xfer->context = this;
        Serial.printf("Gamepad ready IN EP:0x%02x\n", ep->bEndpointAddress);
        sendHidSetProtocol();
        return;
      }

      outEpAddr = ep->bEndpointAddress;
      outEpLen = ep->wMaxPacketSize;
      Serial.printf("Gamepad OUT EP:0x%02x len:%u\n", outEpAddr, outEpLen);
    }
  }

  static void onData(usb_transfer_t *t) {
    GamepadHost *h = (GamepadHost *)t->context;
    if (t->status != USB_TRANSFER_STATUS_COMPLETED || t->actual_num_bytes < 1) {
      Serial.printf("IN status:%d bytes:%d retry:%u\n", (int)t->status, t->actual_num_bytes, h->inRetryCount);
      if (h->deviceHandle && h->inEpAddr && h->inRetryCount < 5) {
        h->inRetryCount++;
        usb_host_endpoint_flush(h->deviceHandle, h->inEpAddr);
        usb_host_endpoint_clear(h->deviceHandle, h->inEpAddr);
        h->inputStarted = false;
        h->startInputAt_ms = millis() + 500;
        return;
      }
      // IN recovery exhausted — no GET_REPORT fallback needed; see initPhase chain instead
      gp.connected = false;
      return;
    }
    h->inRetryCount = 0;
    static uint32_t lastRawPrint_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - lastRawPrint_ms >= 1000) {
      lastRawPrint_ms = now_ms;
      Serial.print("RAW HID:");
      const int printLen = t->actual_num_bytes < 16 ? t->actual_num_bytes : 16;
      for (int i = 0; i < printLen; i++) {
        Serial.printf(" %02X", t->data_buffer[i]);
      }
      Serial.printf(" len:%d\n", t->actual_num_bytes);
    }

    if (!h->ledSent && h->outEpAddr && millis() - h->startInputAt_ms > 2000) {
      h->ledSent = true;
      h->ledR = 0; h->ledG = 0; h->ledB = 255;
      h->nextLedAt_ms = millis();
    }

    parsePacket(t->data_buffer, t->actual_num_bytes);
    // Serial.printf("PARSED LX:%4d LY:%4d RX:%4d RY:%4d L2:%3d R2:%3d connected:%d\n",
    //               gp.lx, gp.ly, gp.rx, gp.ry, gp.l2a, gp.r2a, gp.connected ? 1 : 0);
    if (h->xfer) { h->xfer->num_bytes = h->epLen; usb_host_transfer_submit(h->xfer); }
  }

  public:
  void task() {
    EspUsbHost::task();
    const uint32_t now = millis();
    if (!inputStarted && xfer && startInputAt_ms && (int32_t)(now - startInputAt_ms) >= 0) {
      inputStarted = true;
      xfer->num_bytes = epLen;
      const esp_err_t submitErr = usb_host_transfer_submit(xfer);
      Serial.printf("IN delayed submit err:%d len:%u\n", (int)submitErr, epLen);
    }
    // Kirim LED berulang tiap 1500ms supaya clone yang tidak reliable tetap nyala
    if (ledSent && !ledXfer && outEpAddr && nextLedAt_ms && (int32_t)(now - nextLedAt_ms) >= 0) {
      nextLedAt_ms = now + 1500;
      sendLedReport(ledR, ledG, ledB);
    }
    // GPIO polling via GET_REPORT dihapus — initPhase chain sudah mencakup GET_FEATURE handshake
  }
};

#endif
