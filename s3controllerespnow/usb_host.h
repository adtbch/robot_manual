#ifndef USB_HOST_H
#define USB_HOST_H

#include "EspUsbHost.h"
#include "usb/usb_host.h"
#include "gamepad.h"

class GamepadHost : public EspUsbHost {
  usb_transfer_t *xfer = NULL;
  uint8_t epLen = 64;
  uint8_t ifNum = 0xFF;

  void cleanup() {
    if (xfer) { usb_host_transfer_free(xfer); xfer = NULL; }
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
      if (usb_host_interface_claim(clientHandle, deviceHandle, intf->bInterfaceNumber, 0) == ESP_OK)
        ifNum = intf->bInterfaceNumber;
    }
    if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && xfer == NULL) {
      auto *ep = (const usb_ep_desc_t *)p;
      if ((ep->bmAttributes & 3) != 3) return;
      if (!(ep->bEndpointAddress & 0x80)) return;
      epLen = ep->wMaxPacketSize;
      if (usb_host_transfer_alloc(epLen, 0, &xfer) != ESP_OK) return;
      xfer->device_handle = deviceHandle;
      xfer->bEndpointAddress = ep->bEndpointAddress;
      xfer->callback = onData;
      xfer->context = this;
      gp.connected = true;
      Serial.printf("Gamepad ready EP:0x%02x\n", ep->bEndpointAddress);
      xfer->num_bytes = epLen;
      usb_host_transfer_submit(xfer);
    }
  }

  static void onData(usb_transfer_t *t) {
    GamepadHost *h = (GamepadHost *)t->context;
    if (t->status != 0 || t->actual_num_bytes < 1) {
      if (h->xfer) { h->xfer->num_bytes = h->epLen; usb_host_transfer_submit(h->xfer); }
      return;
    }
    parsePacket(t->data_buffer, t->actual_num_bytes);
    if (h->xfer) { h->xfer->num_bytes = h->epLen; usb_host_transfer_submit(h->xfer); }
  }

public:
  void task() { EspUsbHost::task(); }
};

#endif
