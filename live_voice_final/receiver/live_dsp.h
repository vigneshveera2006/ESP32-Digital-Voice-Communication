// =====================================================================
//  live_dsp.h  —  Audio filters for the LIVE voice system.
//  The SAME file is in the transmitter and receiver sketch folders.
//
//  Transmitter:  32 kHz microphone -> LowPass32 (3.6 kHz) -> HighPass (300 Hz)
//                -> Agc (steady speech level) -> 32 kHz frame (DM/ADM at OSR 4)
//                and every 4th sample -> 8 kHz frame (PCM, DPCM, DM/ADM at OSR 1)
//  Receiver:     8 kHz decoded frames -> HighPass (300 Hz) -> volume
//                -> Upsampler (x2 to 16 kHz, removes the harsh 8 kHz images) -> I2S
// =====================================================================
#pragma once
#include <math.h>
#include <string.h>
#include <stdint.h>

#define DSP_TAPS 31                 // FIR length for decimator and upsampler

// Windowed-sinc low-pass, cutoff fc (Hz) at sample rate fs, unity DC gain.
static inline void dsp_design_lowpass(float *h, int n, float fc, float fs) {
  const float m = (n - 1) / 2.0f, f = fc / fs;
  float sum = 0;
  for (int i = 0; i < n; i++) {
    float x = i - m;
    float s = (x == 0) ? 2 * f : sinf(2 * (float)M_PI * f * x) / ((float)M_PI * x);
    float w = 0.54f - 0.46f * cosf(2 * (float)M_PI * i / (n - 1));     // Hamming window
    h[i] = s * w;
    sum += h[i];
  }
  for (int i = 0; i < n; i++) h[i] /= sum;
}

// ---------------- Band-limit filter at 32 kHz (transmitter) ----------------
// 63-tap low-pass at 3.6 kHz, run on every 32 kHz microphone sample. Its output is used
// directly for oversampled DM/ADM, and every 4th sample of it gives the 8 kHz signal.
#define LP32_TAPS 63
struct LowPass32 {
  float h[LP32_TAPS];
  float hist[2 * LP32_TAPS];        // doubled buffer: no wrap-around inside the loop
  int pos = 0;
  void init() { dsp_design_lowpass(h, LP32_TAPS, 3600.0f, 32000.0f); memset(hist, 0, sizeof(hist)); pos = 0; }
  float run(float x) {
    hist[pos] = x;
    hist[pos + LP32_TAPS] = x;
    const float *p = &hist[pos + LP32_TAPS];   // p[0] = newest, p[-k] = k samples ago
    float acc = 0;
    for (int k = 0; k < LP32_TAPS; k++) acc += h[k] * p[-k];
    pos = (pos + 1) % LP32_TAPS;
    return acc;
  }
};

// ---------------- 16 kHz -> 8 kHz ----------------
struct Decimator {
  float h[DSP_TAPS];
  float hist[DSP_TAPS];             // last DSP_TAPS input samples (circular)
  int pos = 0, phase = 0;
  void init() { dsp_design_lowpass(h, DSP_TAPS, 3600.0f, 16000.0f); memset(hist, 0, sizeof(hist)); }
  // Feeds one 16 kHz sample; returns true and sets out on every 2nd sample.
  bool push(float x, float &out) {
    hist[pos] = x;
    int p = pos;
    pos = (pos + 1) % DSP_TAPS;
    if (++phase < 2) return false;
    phase = 0;
    float acc = 0;
    for (int k = 0; k < DSP_TAPS; k++) {
      acc += h[k] * hist[p];
      p = (p == 0) ? DSP_TAPS - 1 : p - 1;
    }
    out = acc;
    return true;
  }
};

// ---------------- 8 kHz -> 16 kHz ----------------
struct Upsampler {
  float h[DSP_TAPS];
  float hist[DSP_TAPS];
  int pos = 0;
  void init() { dsp_design_lowpass(h, DSP_TAPS, 3600.0f, 16000.0f); memset(hist, 0, sizeof(hist)); }
  // One 8 kHz sample in, two 16 kHz samples out (zero-stuffing + low-pass, gain 2).
  void push(float x, float &y0, float &y1) {
    for (int ph = 0; ph < 2; ph++) {
      hist[pos] = ph == 0 ? x * 2.0f : 0.0f;
      int p = pos;
      pos = (pos + 1) % DSP_TAPS;
      float acc = 0;
      for (int k = 0; k < DSP_TAPS; k++) {
        acc += h[k] * hist[p];
        p = (p == 0) ? DSP_TAPS - 1 : p - 1;
      }
      (ph == 0 ? y0 : y1) = acc;
    }
  }
};

// ---------------- 2nd-order Butterworth high-pass ----------------
struct HighPass {
  float b0, b1, b2, a1, a2, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  void init(float fc, float fs) {
    float w = 2 * (float)M_PI * fc / fs, c = cosf(w), al = sinf(w) / (2 * 0.7071f), a0 = 1 + al;
    b0 = (1 + c) / 2 / a0; b1 = -(1 + c) / a0; b2 = (1 + c) / 2 / a0;
    a1 = -2 * c / a0;      a2 = (1 - al) / a0;
    x1 = x2 = y1 = y2 = 0;
  }
  float run(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
};

// ---------------- Noise gate (receiver, per 20 ms frame) ----------------
// Measures the background level as the quietest frame of the last 3 s. Frames more than
// ~6 dB above it count as speech; the gate stays open 400 ms after speech, then fades the
// speaker down to -24 dB so the hiss between words disappears. Returns the gain at the start
// and end of the frame (the caller ramps between them, so there are no clicks).
#define GATE_HIST 150
struct NoiseGate {
  float hist[GATE_HIST];
  int pos = 0, n = 0, hang = 0;
  float g = 1.0f;
  bool enabled = true;
  float closedGain = 0.06f;            // -24 dB when nobody speaks
  float ratio = 2.0f;                  // speech must be ~6 dB above the background
  float minOpen = 150.0f;              // ... and above this absolute level (16-bit units)
  int hangFrames = 20;                 // stay open 400 ms after the last speech frame
  uint32_t frames = 0, openFrames = 0;
  void reset() { pos = n = 0; hang = hangFrames; g = 1.0f; }
  void process(const int16_t *s, int len, float &g0, float &g1) {
    float mean = 0;
    for (int i = 0; i < len; i++) mean += s[i];
    mean /= len;
    float sq = 0;
    for (int i = 0; i < len; i++) { float d = s[i] - mean; sq += d * d; }
    float r = sqrtf(sq / len);
    hist[pos] = r;
    pos = (pos + 1) % GATE_HIST;
    if (n < GATE_HIST) n++;
    float floorLvl = r;
    for (int k = 0; k < n; k++) if (hist[k] < floorLvl) floorLvl = hist[k];
    bool speech = n < 25 || r > fmaxf(floorLvl * ratio, minOpen);   // first 0.5 s: stay open while learning
    if (speech) hang = hangFrames; else if (hang > 0) hang--;
    float target = (!enabled || hang > 0) ? 1.0f : closedGain;
    float next = target > g ? target : fmaxf(target, g * 0.7f);   // open at once, close over ~0.15 s
    g0 = g; g1 = next; g = next;
    frames++;
    if (next > 0.5f) openFrames++;
  }
};

// ---------------- Automatic gain control (per 160-sample frame) ----------------
// Brings speech peaks to `target` (16-bit units). Fast to turn down, slow to turn up,
// and holds its gain during silence so background noise is not boosted.
struct Agc {
  float gain = 8.0f;                  // current linear gain
  float target = 23000.0f;            // speech peaks about -3 dBFS
  float maxGain = 200.0f;             // +46 dB at most
  float gateRms = 40.0f;              // below this (16-bit units, ~-58 dBFS) = silence
  float lastPeak = 0, lastRms = 0;
  // Processes n samples in place (float, 16-bit scale in and out), output clipped to +-32767.
  void process(float *x, int n) {
    float peak = 0, sq = 0;
    for (int i = 0; i < n; i++) { float a = fabsf(x[i]); if (a > peak) peak = a; sq += x[i] * x[i]; }
    float rms = sqrtf(sq / n);
    lastPeak = peak; lastRms = rms;
    float want = gain;
    if (rms > gateRms && peak > 0) want = target / peak;
    if (want > maxGain) want = maxGain;
    if (want < 1.0f) want = 1.0f;
    float next = (want < gain) ? want : fminf(want, gain * 1.06f);   // attack now, release ~0.5 dB/frame
    float start = (next < gain) ? next : gain;                         // turning down: apply at once (no overshoot)
    for (int i = 0; i < n; i++) {
      float g = start + (next - start) * (i + 1) / n;                   // turning up: smooth ramp, no clicks
      float v = x[i] * g;
      x[i] = v > 32767.0f ? 32767.0f : (v < -32767.0f ? -32767.0f : v);
    }
    gain = next;
  }
};
