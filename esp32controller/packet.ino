/*
 * =====================================================================
 * FILE    : packet.ino
 * PERAN   : Fungsi-fungsi helper yang berkaitan dengan pembuatan
 *           dan manipulasi paket data.
 *
 * ISI FILE INI:
 *   - hitung_checksum()     (XOR checksum untuk frame UART WSN-31)
 *   - buat_paket_stop()     (paket kosong/stop saat PS4 disconnect)
 *
 * CATATAN:
 *   Struct ControlPacket didefinisikan di config.h (pusat definisi bersama).
 * =====================================================================
 */

#include "config.h"

// =====================================================================
//  FUNGSI: HITUNG XOR CHECKSUM
//  Dipakai oleh wsn_serial.ino untuk validasi integritas frame UART.
// =====================================================================

/**
 * Hitung XOR checksum dari sejumlah byte data.
 *
 * @param data    Pointer ke buffer byte
 * @param panjang Jumlah byte yang akan di-checksum
 * @return        Nilai XOR checksum (1 byte)
 */
uint8_t hitung_checksum(const uint8_t *data, uint16_t panjang) {
    uint8_t cs = 0;
    for (uint16_t i = 0; i < panjang; i++) {
        cs ^= data[i];
    }
    return cs;
}

// =====================================================================
//  FUNGSI: BUAT PAKET STOP
//  Dikirim saat PS4 disconnect atau sistem dalam kondisi error.
//  Semua field gerak = 0, connected = 0.
// =====================================================================

/**
 * Isi paket dengan nilai stop (semua gerak = 0, connected = 0).
 *
 * @param paket Referensi paket yang akan diisi
 */
void buat_paket_stop(ControlPacket &paket) {
    paket           = {};                             // zero semua field
    paket.magic     = kPacketMagic;
    paket.connected = 0;
    nomor_urut_paket++;
    paket.seq = nomor_urut_paket;
}
