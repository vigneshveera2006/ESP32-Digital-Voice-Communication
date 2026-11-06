// =====================================================================
//  live_protocol.h  —  Packet format for the LIVE voice system.
//  The SAME file must be in the transmitter and receiver sketch folders.
//  All multi-byte fields are little-endian (native on ESP32).
// =====================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define LV_MAGIC        0xA7
#define LV_CHANNEL      1          // Wi-Fi channel for ESP-NOW (and later the receiver hotspot)

// Packet types
#define LV_AUDIO        1          // one 20 ms frame of encoded audio
#define LV_REF          2          // original samples of one frame (for SNR/MSE on the dashboard)
#define LV_STOP         3          // transmitter stopped (S3 OFF)
#define LV_TEST         4          // stage 3 link test
#define LV_BEACON       5          // receiver -> transmitter, lets the transmitter learn its MAC

// Encodings (S1 S2)
#define LV_PCM          0          // OFF OFF
#define LV_DPCM         1          // OFF ON
#define LV_DM           2          // ON  OFF
#define LV_ADM          3          // ON  ON

#define LV_FRAME_SAMPLES  160      // 20 ms at 8 kHz
#define LV_MAX_PAYLOAD    200      // ESP-NOW limit is 250 bytes in total

#pragma pack(push, 1)
typedef struct {          // 24 bytes
  uint8_t  magic;         // LV_MAGIC
  uint8_t  type;          // LV_AUDIO, LV_REF, ...
  uint8_t  codec;         // LV_PCM ... LV_ADM
  uint8_t  bits;          // bits per code: 8, 4 or 1
  uint16_t seq;           // frame sequence number (wraps at 65535)
  uint16_t nsamples;      // output samples in this frame (160)
  int16_t  st_y;          // decoder state at frame start: previous reconstructed sample
  uint16_t st_step;       // decoder state: ADM current step size
  uint8_t  st_last;       // decoder state: ADM last bit (0/1, 0xFF = none yet)
  uint8_t  osr;           // oversampling ratio for DM/ADM (1 or 4)
  uint16_t p_step;        // parameter: DPCM/DM step, ADM initial step
  uint16_t p_min;         // parameter: ADM minimum step
  uint16_t p_max;         // parameter: ADM maximum step
  uint8_t  flags;         // LV_REF: part number (0 or 1)
  uint8_t  len;           // payload length in bytes
  uint16_t crc;           // CRC-16/CCITT over header (with crc = 0) + payload
} LvHeader;
#pragma pack(pop)

#define LV_MAX_PACKET (sizeof(LvHeader) + LV_MAX_PAYLOAD)

static inline uint16_t lv_crc16(const uint8_t *d, size_t n, uint16_t crc = 0xFFFF) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Fills in len and crc, copies the payload after the header. Returns total packet length.
static inline size_t lv_finish(uint8_t *pkt, LvHeader *h, const uint8_t *payload, uint8_t len) {
  h->magic = LV_MAGIC;
  h->len = len;
  h->crc = 0;
  memcpy(pkt, h, sizeof(LvHeader));
  if (len) memcpy(pkt + sizeof(LvHeader), payload, len);
  uint16_t c = lv_crc16(pkt, sizeof(LvHeader) + len);
  memcpy(pkt + offsetof(LvHeader, crc), &c, 2);
  h->crc = c;
  return sizeof(LvHeader) + len;
}

// Checks magic, length and CRC of a received packet.
static inline bool lv_valid(const uint8_t *pkt, int n) {
  if (n < (int)sizeof(LvHeader) || n > (int)LV_MAX_PACKET) return false;
  LvHeader h;
  memcpy(&h, pkt, sizeof(h));
  if (h.magic != LV_MAGIC || (int)sizeof(LvHeader) + h.len != n) return false;
  uint8_t tmp[LV_MAX_PACKET];
  memcpy(tmp, pkt, n);
  memset(tmp + offsetof(LvHeader, crc), 0, 2);
  return lv_crc16(tmp, n) == h.crc;
}

// Payload bytes per 20 ms frame for each encoding (OSR 1).
static inline uint8_t lv_payload_bytes(uint8_t codec) {
  switch (codec) {
    case LV_PCM:  return 160;   // 8 bits x 160
    case LV_DPCM: return 80;    // 4 bits x 160
    default:      return 20;    // DM / ADM: 1 bit x 160
  }
}

static inline const char *lv_codec_name(uint8_t c) {
  switch (c) { case LV_PCM: return "PCM"; case LV_DPCM: return "DPCM"; case LV_DM: return "DM"; case LV_ADM: return "ADM"; }
  return "?";
}
