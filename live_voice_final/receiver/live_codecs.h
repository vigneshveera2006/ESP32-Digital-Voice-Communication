// =====================================================================
//  live_codecs.h  —  PCM, DPCM, DM and ADM for the LIVE voice system.
//  The SAME file is in the transmitter and receiver sketch folders, so the
//  receiver's decoder uses exactly the transmitter's arithmetic.
//
//  Samples are 16-bit integers. Codes are packed MSB-first.
//    PCM  8 bit : code = (x + 32768) >> 8              recon = (code << 8) - 32768 + 128
//    DPCM 4 bit : q = round((x - y) / step), -8..7      y = y + q*step       (closed loop)
//    DM   1 bit : bit = (x >= y)                        y = y +/- step
//    ADM  1 bit : as DM, step x1.5 when the bit repeats, x2/3 when it changes, kept in [min, max]
//  DM/ADM may be oversampled (OSR 4 = coded at 32 kHz); the decoder averages
//  each group of OSR outputs back to 8 kHz.
//
//  Decoder state (y, ADM step, ADM last bit) is carried from frame to frame.
//  The transmitter also writes the state at the start of each frame into the
//  packet header, so after a lost packet the receiver resynchronises at once.
// =====================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include "live_protocol.h"

// ---- Parameters (tuned for the transmitter's AGC level: speech peaks ~23000) ----
#define DPCM_STEP        1600
#define DM_STEP_OSR1     2500
#define DM_STEP_OSR4     2400
#define ADM_STEP_OSR1     800
#define ADM_MIN_OSR1       32
#define ADM_MAX_OSR1     6000
#define ADM_STEP_OSR4     400
#define ADM_MIN_OSR4       50
#define ADM_MAX_OSR4     6000

typedef struct {
  int32_t y;        // previous reconstructed sample (DPCM / DM / ADM)
  int32_t step;     // ADM current step
  int32_t last;     // ADM last bit, -1 = none yet
} CodecState;

static inline int32_t lc_clip16(int32_t v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

static inline uint8_t lc_bits(uint8_t codec) { return codec == LV_PCM ? 8 : (codec == LV_DPCM ? 4 : 1); }

// Payload bytes for one 20 ms frame.
static inline uint8_t lc_frame_bytes(uint8_t codec, uint8_t osr) {
  uint32_t codes = (uint32_t)LV_FRAME_SAMPLES * ((codec == LV_DM || codec == LV_ADM) ? osr : 1);
  return (uint8_t)((codes * lc_bits(codec) + 7) / 8);
}

// Fills the parameter fields of a header for this codec/OSR.
static inline void lc_params(uint8_t codec, uint8_t osr, LvHeader *h) {
  h->codec = codec;
  h->bits = lc_bits(codec);
  h->osr = (codec == LV_DM || codec == LV_ADM) ? osr : 1;
  h->p_step = h->p_min = h->p_max = 0;
  if (codec == LV_DPCM) h->p_step = DPCM_STEP;
  if (codec == LV_DM)   h->p_step = (h->osr == 4) ? DM_STEP_OSR4 : DM_STEP_OSR1;
  if (codec == LV_ADM) {
    h->p_step = (h->osr == 4) ? ADM_STEP_OSR4 : ADM_STEP_OSR1;
    h->p_min  = (h->osr == 4) ? ADM_MIN_OSR4  : ADM_MIN_OSR1;
    h->p_max  = (h->osr == 4) ? ADM_MAX_OSR4  : ADM_MAX_OSR1;
  }
}

static inline void lc_reset(CodecState *s, const LvHeader *h) { s->y = 0; s->step = h->p_step; s->last = -1; }

static inline void lc_state_to_header(const CodecState *s, LvHeader *h) {
  h->st_y = (int16_t)s->y;
  h->st_step = (uint16_t)s->step;
  h->st_last = s->last < 0 ? 0xFF : (uint8_t)s->last;
}

static inline void lc_state_from_header(CodecState *s, const LvHeader *h) {
  s->y = h->st_y;
  s->step = h->st_step;
  s->last = h->st_last == 0xFF ? -1 : h->st_last;
}

// ---------------- encoder ----------------
// x: n input samples (n = 160, or 640 for DM/ADM at OSR 4). Writes packed codes to out.
static inline void lc_encode(const LvHeader *h, CodecState *s, const int16_t *x, int n, uint8_t *out) {
  memset(out, 0, lc_frame_bytes(h->codec, h->osr));
  for (int i = 0; i < n; i++) {
    int32_t v = x[i];
    switch (h->codec) {
      case LV_PCM:
        out[i] = (uint8_t)((v + 32768) >> 8);
        break;
      case LV_DPCM: {
        int32_t e = v - s->y, st = h->p_step, hs = st / 2;
        int32_t q = e >= 0 ? (e + hs) / st : -((-e + hs) / st);
        if (q < -8) q = -8;
        if (q > 7) q = 7;
        s->y = lc_clip16(s->y + q * st);
        uint8_t c = (uint8_t)(q + 8);
        out[i >> 1] |= (i & 1) ? c : (uint8_t)(c << 4);
        break;
      }
      case LV_DM: {
        int b = v >= s->y;
        s->y = lc_clip16(b ? s->y + h->p_step : s->y - h->p_step);
        if (b) out[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
        break;
      }
      case LV_ADM: {
        int b = v >= s->y;
        if (s->last >= 0) {
          s->step = (b == s->last) ? ((s->step * 3) >> 1) : ((s->step * 2) / 3);
          if (s->step < h->p_min) s->step = h->p_min;
          if (s->step > h->p_max) s->step = h->p_max;
        }
        s->y = lc_clip16(b ? s->y + s->step : s->y - s->step);
        s->last = b;
        if (b) out[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
        break;
      }
    }
  }
}

// ---------------- decoder ----------------
// Decodes one frame into 160 output samples (averaging OSR codes for DM/ADM).
// Returns false if the payload length does not match the header.
static inline bool lc_decode(const LvHeader *h, CodecState *s, const uint8_t *in, int16_t *out) {
  int osr = (h->codec == LV_DM || h->codec == LV_ADM) ? h->osr : 1;
  if (osr != 1 && osr != 4) return false;
  if (h->len != lc_frame_bytes(h->codec, osr) || h->bits != lc_bits(h->codec)) return false;
  if (h->codec != LV_PCM && h->p_step == 0) return false;
  int shift = osr == 4 ? 2 : 0;
  for (int k = 0; k < LV_FRAME_SAMPLES; k++) {
    int32_t sum = 0;
    for (int j = 0; j < osr; j++) {
      int i = k * osr + j;
      int32_t y = 0;
      switch (h->codec) {
        case LV_PCM:
          y = ((int32_t)in[i] << 8) - 32768 + 128;
          break;
        case LV_DPCM: {
          int32_t c = (i & 1) ? (in[i >> 1] & 0x0F) : (in[i >> 1] >> 4);
          s->y = lc_clip16(s->y + (c - 8) * h->p_step);
          y = s->y;
          break;
        }
        case LV_DM: {
          int b = (in[i >> 3] >> (7 - (i & 7))) & 1;
          s->y = lc_clip16(b ? s->y + h->p_step : s->y - h->p_step);
          y = s->y;
          break;
        }
        case LV_ADM: {
          int b = (in[i >> 3] >> (7 - (i & 7))) & 1;
          if (s->last >= 0) {
            s->step = (b == s->last) ? ((s->step * 3) >> 1) : ((s->step * 2) / 3);
            if (s->step < h->p_min) s->step = h->p_min;
            if (s->step > h->p_max) s->step = h->p_max;
          }
          s->y = lc_clip16(b ? s->y + s->step : s->y - s->step);
          s->last = b;
          y = s->y;
          break;
        }
      }
      sum += y;
    }
    out[k] = (int16_t)(sum >> shift);
  }
  return true;
}
