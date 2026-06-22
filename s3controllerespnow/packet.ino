/*
 * =====================================================================
 * FILE    : packet.ino
 * PERAN   : Helper functions untuk pembuatan dan manipulasi ControlPacket.
 *
 * ISI FILE INI:
 *   - hitung_checksum()           XOR checksum untuk integritas paket
 *   - terapkan_checksum()         Hitung & isi field checksum
 *   - validasi_paket()            Validasi magic + checksum di penerima
 *   - isi_paket_dari_gamepad()    Isi ControlPacket dari GamepadState
 *   - buat_paket_stop()           Paket kosong saat gamepad disconnect
 *
 * CATATAN:
 *   Checksum = XOR semua byte dari field 'x' sampai 'connected'.
 *   Field 'magic' dan 'checksum' sendiri TIDAK diikutsertakan.
 * =====================================================================
 */

#include "config.h"
#include "gamepad.h"

// =====================================================================
//  FUNGSI: HITUNG XOR CHECKSUM
// =====================================================================

/**
 * Hitung XOR checksum dari ControlPacket (field x sampai connected).
 * Field magic dan checksum sendiri tidak diikutsertakan.
 *
 * @param p Paket yang akan di-checksum
 * @return  Nilai XOR (1 byte)
 */
static uint8_t hitung_checksum(const ControlPacket &p) {
    // Mulai dari byte offset field 'x' (setelah magic + checksum)
    const uint8_t *raw = reinterpret_cast<const uint8_t *>(&p);
    const size_t start = offsetof(ControlPacket, x);
    const size_t end   = sizeof(ControlPacket);

    uint8_t cs = 0;
    for (size_t i = start; i < end; i++) {
        cs ^= raw[i];
    }
    return cs;
}

// =====================================================================
//  FUNGSI: ISI CHECKSUM PADA PAKET
// =====================================================================

/**
 * Hitung checksum dan tulis ke field checksum.
 * Panggil SETELAH semua field lain terisi.
 *
 * @param p Referensi paket
 */
void terapkan_checksum(ControlPacket &p) {
    p.checksum = hitung_checksum(p);
}

// =====================================================================
//  FUNGSI: VALIDASI PAKET (untuk receiver)
// =====================================================================

/**
 * Validasi magic number dan checksum.
 *
 * @param p Paket yang diterima
 * @return true jika valid
 */
bool validasi_paket(const ControlPacket &p) {
    if (p.magic != kPacketMagic) return false;
    if (hitung_checksum(p) != p.checksum) return false;
    return true;
}

// =====================================================================
//  FUNGSI: ISI PAKET DARI GAMEPADSTATE
// =====================================================================

/**
 * Konversi GamepadState (dari USB HID parser) ke ControlPacket.
 * Bitmask tombol menggunakan layout yang sama dengan esp32controllerV2.
 *
 * @param paket  Referensi paket yang akan diisi
 * @param gp     GamepadState dari parser USB HID
 */
void isi_paket_dari_gamepad(ControlPacket &paket, const GamepadState &gp) {
    paket = {};  // zero semua field

    paket.magic = kPacketMagic;

    // --- Analog stik mentah (-128..127) ---
    paket.lx = gp.lx;
    paket.ly = gp.ly;
    paket.rx = gp.rx;
    paket.ry = gp.ry;

    // --- Perintah gerak (dari stik, siap pakai untuk motor) ---
    paket.x = gp.lx;    // Lateral: kanan (+), kiri (-)
    paket.y = gp.ly;    // Maju/mundur: maju (+), mundur (-)
    paket.w = gp.rx;    // Rotasi: CCW (+), CW (-)

    // --- Trigger analog (0..255) ---
    paket.l2Value = gp.l2a;
    paket.r2Value = gp.r2a;

    // --- Gyro tidak tersedia via USB HID ---
    paket.gyrX = 0;
    paket.gyrY = 0;
    paket.gyrZ = 0;

    // --- Bitmask tombol (layout sama dengan esp32controllerV2) ---
    uint32_t btn = 0;
    if (gp.cross)    btn |= (1u << 0);
    if (gp.circle)   btn |= (1u << 1);
    if (gp.triangle) btn |= (1u << 2);
    if (gp.square)   btn |= (1u << 3);
    if (gp.l1)       btn |= (1u << 4);
    if (gp.r1)       btn |= (1u << 5);
    if (gp.l2)       btn |= (1u << 6);   // digital
    if (gp.r2)       btn |= (1u << 7);   // digital
    if (gp.l3)       btn |= (1u << 8);
    if (gp.r3)       btn |= (1u << 9);
    if (gp.up)       btn |= (1u << 10);
    if (gp.down)     btn |= (1u << 11);
    if (gp.left)     btn |= (1u << 12);
    if (gp.right)    btn |= (1u << 13);
    if (gp.share)    btn |= (1u << 14);
    if (gp.options)  btn |= (1u << 15);
    // bit 16 = PS Button (tidak tersedia via USB HID)
    if (gp.touchpad) btn |= (1u << 17);
    paket.buttons = btn;

    // --- Metadata ---
    nomor_urut_paket++;
    paket.seq       = nomor_urut_paket;
    paket.connected = 1;

    // --- Checksum (terakhir, setelah semua field terisi) ---
    terapkan_checksum(paket);
}

// =====================================================================
//  FUNGSI: BUAT PAKET STOP
//  Dikirim saat gamepad disconnect atau tidak ada koneksi.
//  Semua field gerak = 0, connected = 0.
// =====================================================================

/**
 * Isi paket dengan nilai stop (semua gerak = 0, connected = 0).
 * Checksum tetap dihitung agar receiver bisa validasi.
 *
 * @param paket Referensi paket yang akan diisi
 */
void buat_paket_stop(ControlPacket &paket) {
    paket           = {};
    paket.magic     = kPacketMagic;
    paket.connected = 0;

    nomor_urut_paket++;
    paket.seq = nomor_urut_paket;

    terapkan_checksum(paket);
}
