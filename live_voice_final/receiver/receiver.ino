// =====================================================================
//  receiver.ino  —  LIVE wireless voice: RECEIVER
//  Arduino-ESP32 core 2.0.14, board "ESP32 Dev Module".
//  Version: FINAL (PCM, DPCM, DM, ADM + phone dashboard with captive portal).
//
//  ESP-NOW packet -> CRC-16 and sequence check -> read encoding and decoder
//  state from the header -> decode (PCM / DPCM / DM / ADM, live_codecs.h)
//  -> jitter buffer (60 ms) -> 300 Hz high-pass -> volume -> x2 upsampling
//  to 16 kHz -> I2S -> MAX98357A -> speaker.  Lost frames are concealed.
//
//  Phone dashboard: Wi-Fi "VoiceRX" (open network, no password) on channel 1, the same
//  channel as ESP-NOW. Joining it opens the dashboard automatically (captive
//  portal), or open http://192.168.4.1. The web server runs on the other CPU
//  core at low priority, so it cannot interrupt the audio.
//
//  Recording: while audio arrives, the received packets (encoded bits + header)
//  are stored in memory. After S3 OFF the dashboard offers the reconstructed audio
//  as a WAV file, decoded again on this ESP32 with the same decoder.
//
//  SNR / MSE: the transmitter sends the original samples of every 10th frame
//  (LV_REF, same sequence number). They are compared with this receiver's own
//  decoded output of that same frame - exactly aligned. Never played.
//
//  Wiring: MAX98357A BCLK->26, LRC->25, DIN->22, VIN->5 V, GND->GND.
//  Serial Monitor 115200 baud:  + / -  volume,  t  test tone,  g  noise gate on/off,
//                               n  noise removal on/off,  r  reset counters.
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <driver/i2s.h>
#include "live_protocol.h"
#include "live_dsp.h"
#include "live_codecs.h"
#include "dashboard_page.h"

// ---------------- Pins / audio ----------------
#define AMP_BCLK   26
#define AMP_LRC    25
#define AMP_DIN    22
#define PIN_LED    2
#define AMP_PORT   I2S_NUM_0
#define OUT_RATE   16000                // 8 kHz audio upsampled x2
#define PREBUFFER  3                    // frames queued before playback starts (60 ms)
#define MAX_QUEUED 8                    // above this, drop one frame (clock drift)
#define SPEECH_PEAK 23000.0f            // transmitter's speech peak level (its AGC target)

// ---------------- Hotspot ----------------
#define AP_SSID     "VoiceRX"
#define AP_PASSWORD ""         // at least 8 characters
#define SNR_WINDOW  25                  // reference frames in the SNR/MSE average (25 = 5 s)
#define DEC_RING    16                  // recent decoded frames kept for matching references
#define REC_BYTES   100000UL            // recording memory, reserved at boot before Wi-Fi starts
#define REC_LOST    0xEE                // header flag marking a lost frame in the recording

// ---------------- Types (kept above every function: the Arduino IDE inserts
//                  function declarations before the first function) ----------------
typedef struct { uint8_t data[LV_MAX_PACKET]; int len; } RxItem;            // one radio packet
typedef struct { int16_t s[LV_FRAME_SAMPLES]; uint8_t conceal; float qvar; } Frame;   // one decoded 20 ms frame
struct Stats { uint32_t rx, lost, crc, dup, other, underrun, dropped, stateMismatch; };
struct DecSlot { uint16_t seq; bool used; int16_t s[LV_FRAME_SAMPLES]; };
struct Dash {                            // everything the dashboard shows (guarded by dashLock)
  uint8_t  codec, osr, bits, payload;
  bool     haveStream;
  uint32_t rx, lost, crc, stateMismatch, rejected, pps;
  bool     snrValid;
  float    snr, mse, frameSnr;
  int32_t  waveSeq;
  int16_t  orig[LV_FRAME_SAMPLES], recon[LV_FRAME_SAMPLES];
  uint8_t  lastPayload[LV_MAX_PAYLOAD];
  uint8_t  lastLen;
  uint32_t lastAudioMs;
  bool     stopped;
  uint8_t  recCodec, recOsr, recPayload;
};

// ---------------- Voice gate: speaker fully silent when nobody speaks ----------------
// Learns the background level (quietest 20 ms frame of the last 3 s). A frame opens the gate
// only if it is clearly louder than that AND sounds like voice (not hiss: few zero crossings).
// Starts CLOSED, so the burst of noise when the transmitter is switched on is not heard.
// Opens within one frame when you speak, stays open 300 ms after the last word, then fades
// to complete silence in about 0.1 s (no clicks).
struct VoiceGate {
  float hist[GATE_HIST];
  int pos = 0, n = 0, hang = 0;
  float g = 0.0f;                      // current gain (0 = silent, 1 = open)
  bool enabled = true;
  float ratio = 2.5f;                  // speech must be ~8 dB above the background
  float minOpen = 300.0f;              // ... and above this absolute level (16-bit units)
  float learnOpen = 2500.0f;           // first 0.5 s (background not known yet): only clear speech opens
  float maxZcr = 0.45f;                // hiss crosses zero more often than voice does
  int hangFrames = 15;                 // stay open 300 ms after the last speech frame
  uint32_t frames = 0, openFrames = 0;
  void reset() { hang = 0; g = 0.0f; } // keep the learned background, start closed
  void process(const int16_t *s, int len, float &g0, float &g1) {
    float mean = 0;
    for (int i = 0; i < len; i++) mean += s[i];
    mean /= len;
    float sq = 0;
    int zc = 0;
    float prev = s[0] - mean;
    for (int i = 0; i < len; i++) {
      float d = s[i] - mean;
      sq += d * d;
      if ((d >= 0) != (prev >= 0)) zc++;
      prev = d;
    }
    float r = sqrtf(sq / len), zcr = (float)zc / len;
    hist[pos] = r;
    pos = (pos + 1) % GATE_HIST;
    if (n < GATE_HIST) n++;
    float floorLvl = r;
    for (int k = 0; k < n; k++) if (hist[k] < floorLvl) floorLvl = hist[k];
    bool loud = (n < 25) ? r > learnOpen : r > fmaxf(floorLvl * ratio, minOpen);
    bool speech = loud && (zcr < maxZcr || hang > 0);   // "s"/"sh" sounds keep a word going, hiss cannot open it
    if (speech) hang = hangFrames; else if (hang > 0) hang--;
    float target = (!enabled || hang > 0) ? 1.0f : 0.0f;
    float next = target > g ? target : g * 0.6f;
    if (next < 0.02f) next = 0.0f;
    g0 = g; g1 = next; g = next;
    frames++;
    if (next > 0.5f) openFrames++;
  }
};

// ---------------- Noise remover (spectral noise reduction, 8 kHz) ----------------
// Splits the sound into 129 frequency bands every 10 ms (FFT). For each band it knows the noise:
//   - the quantization noise of the codec (PCM / DPCM step size from the packet header), which is
//     the hiss you hear WHILE speaking (it only exists while the signal changes), and
//   - the background noise, learned while nobody speaks.
// Bands holding mostly noise are turned down, bands with voice pass. Everything above 3.4 kHz
// (no speech there, only quantization buzz / idle whine) is removed.
// Adds 10 ms delay. Serial command n = on/off (to compare).
#define NR_N    256                     // FFT size
#define NR_HOP  80                      // 10 ms hop: two hops per 20 ms frame
#define NR_WIN  160                     // analysis window (50 % overlap)
#define NR_BINS (NR_N / 2 + 1)
#define NR_LP_START 105                 // bands from 3.28 kHz fade out ...
#define NR_LP_END   112                 // ... and are fully removed above 3.5 kHz
struct Denoiser {
  float win[NR_WIN], inBuf[NR_WIN], olap[NR_WIN];
  float noise[NR_BINS], gPrev[NR_BINS];
  float re[NR_N], im[NR_N];
  bool enabled = true, primed = false;
  float qVar = 0, winEnergy = 1;        // codec quantization noise (variance) for the current frame
  float alpha = 2.5f;                   // over-subtraction: higher = stronger noise removal
  float floorGain = 0.08f;              // noise bands are turned down to 8 % (-22 dB), not to zero
  void init() {
    for (int i = 0; i < NR_WIN; i++) win[i] = sqrtf(0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / NR_WIN)));
    winEnergy = 0;
    for (int i = 0; i < NR_WIN; i++) winEnergy += win[i] * win[i];
    for (int k = 0; k < NR_BINS; k++) { noise[k] = 0; gPrev[k] = 1.0f; }
    primed = false;
    clear();
  }
  void clear() { memset(inBuf, 0, sizeof(inBuf)); memset(olap, 0, sizeof(olap)); }
  static void fft(float *xr, float *xi, int n, bool inv) {
    for (int i = 1, j = 0; i < n; i++) {
      int bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) { float t = xr[i]; xr[i] = xr[j]; xr[j] = t; t = xi[i]; xi[i] = xi[j]; xi[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
      float ang = 2.0f * (float)M_PI / len * (inv ? 1.0f : -1.0f), wr = cosf(ang), wi = sinf(ang);
      for (int i = 0; i < n; i += len) {
        float cr = 1.0f, ci = 0.0f;
        for (int j = 0; j < len / 2; j++) {
          int a = i + j, b = a + len / 2;
          float tr = xr[b] * cr - xi[b] * ci, ti = xr[b] * ci + xi[b] * cr;
          xr[b] = xr[a] - tr; xi[b] = xi[a] - ti; xr[a] += tr; xi[a] += ti;
          float t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
        }
      }
    }
    if (inv) for (int i = 0; i < n; i++) { xr[i] /= n; xi[i] /= n; }
  }
  // 80 new samples in, 80 cleaned samples out. quiet = nobody is speaking right now.
  void hop(const float *x, float *y, bool quiet) {
    memmove(inBuf, inBuf + NR_HOP, (NR_WIN - NR_HOP) * sizeof(float));
    memcpy(inBuf + NR_WIN - NR_HOP, x, NR_HOP * sizeof(float));
    for (int i = 0; i < NR_N; i++) { re[i] = i < NR_WIN ? inBuf[i] * win[i] : 0.0f; im[i] = 0.0f; }
    fft(re, im, NR_N, false);
    for (int k = 0; k < NR_BINS; k++) {
      float P = re[k] * re[k] + im[k] * im[k];
      if (!primed) noise[k] = P;
      else if (quiet) noise[k] = 0.95f * noise[k] + 0.05f * P;             // learns the noise while nobody speaks
                                                                           // (kept as it is while someone speaks)
      float g = 1.0f;
      if (enabled) {
        float N = fmaxf(noise[k], qVar * winEnergy);                       // background or codec noise, the larger
        float w = 1.0f - alpha * N / (P + 1e-3f);                          // share of this band that is voice
        g = w > 0.0f ? sqrtf(w) : 0.0f;
        if (g < floorGain) g = floorGain;
        g = 0.5f * g + 0.5f * gPrev[k];                                    // smooth in time: no "musical" chirps
      }
      gPrev[k] = g;
      if (enabled && k >= NR_LP_START)                                     // low-pass: nothing above 3.5 kHz
        g = k >= NR_LP_END ? 0.0f : g * (float)(NR_LP_END - k) / (NR_LP_END - NR_LP_START);
      re[k] *= g; im[k] *= g;
      if (k > 0 && k < NR_N / 2) { re[NR_N - k] *= g; im[NR_N - k] *= g; }
    }
    primed = true;
    fft(re, im, NR_N, true);
    for (int i = 0; i < NR_WIN; i++) olap[i] += re[i] * win[i];
    memcpy(y, olap, NR_HOP * sizeof(float));
    memmove(olap, olap + NR_HOP, (NR_WIN - NR_HOP) * sizeof(float));
    memset(olap + NR_WIN - NR_HOP, 0, NR_HOP * sizeof(float));
  }
  // Cleans one 160-sample frame in place.
  void process(int16_t *s, bool quiet, float codecNoiseVar) {
    qVar = codecNoiseVar;
    float in[NR_HOP], out[NR_HOP];
    for (int h = 0; h < 2; h++) {
      for (int i = 0; i < NR_HOP; i++) in[i] = s[h * NR_HOP + i];
      hop(in, out, quiet);
      for (int i = 0; i < NR_HOP; i++) s[h * NR_HOP + i] = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, out[i]));
    }
  }
};

// Quantization noise of the codec (variance, 16-bit units^2): uniform error of +-step/2 -> step^2 / 12.
static float codecNoiseVar(const LvHeader &h) {
  if (h.codec == LV_PCM) return 256.0f * 256.0f / 12.0f;                   // 8-bit PCM: step 256
  if (h.codec == LV_DPCM) return (float)h.p_step * h.p_step / 12.0f;       // DPCM: step from the header
  return 0.0f;                                                             // DM / ADM: learned in the pauses
}

// Volume: at volume V, speech peaks reach V % of full scale. Default 25 (+ / - to change).
static volatile int volumeStep = 25;
static float volumeGain() { return volumeStep / 100.0f * 30000.0f / SPEECH_PEAK; }

// ---------------- Radio receive queue ----------------
static QueueHandle_t rxQueue;
static volatile uint32_t rxQueueDrops = 0;
static void pushItem(const uint8_t *data, int len) {
  if (len <= 0 || len > (int)LV_MAX_PACKET) return;
  RxItem it;
  memcpy(it.data, data, len);
  it.len = len;
  if (xQueueSend(rxQueue, &it, 0) != pdTRUE) rxQueueDrops++;
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { pushItem(data, len); }
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) { pushItem(data, len); }
#endif

// ---------------- Decoded frames -> player ----------------
static QueueHandle_t frameQueue;
static volatile bool flushRequest = false;
static volatile int  toneFrames = 0;      // >0: player plays a test tone
static VoiceGate gate;                    // silences the speaker between words (Serial command g = on/off)
static Denoiser nr;                       // removes the noise while someone speaks (Serial command n = on/off)

// ---------------- Statistics and stream state (packet handler, loop task) ----------------
static Stats sec = {0, 0, 0, 0, 0, 0, 0, 0}, tot = {0, 0, 0, 0, 0, 0, 0, 0};
static volatile uint32_t underrunCount = 0, dropCount = 0;
static bool     streamActive = false, haveExpected = false;
static uint16_t expectedSeq = 0;
static uint8_t  streamCodec = 0, streamOsr = 1, streamLen = 0;
static CodecState runState;
static bool     haveRunState = false;
static uint32_t lastAudioMs = 0;
static volatile bool playing = false;

// ---------------- SNR / MSE measurement ----------------
static DecSlot  decRing[DEC_RING];
static int      decPos = 0;
static int16_t  refBuf[LV_FRAME_SAMPLES];
static int32_t  refSeq = -1;
static uint8_t  refParts = 0;
static double   winSig[SNR_WINDOW], winErr[SNR_WINDOW];
static int      winCount = 0, winPos = 0;

// ---------------- Dashboard data (shared between the audio side and the web task) ----------------
static Dash dash;
static SemaphoreHandle_t dashLock;

// ---------------- Recording of the received stream ----------------
static uint8_t *recBuf = nullptr;
static volatile uint32_t recLen = 0, recFrames = 0, recId = 0;
static volatile bool recActive = false, recFull = false;
static bool stopSeen = true;                 // last stream ended with S3 OFF (not just out of range)
static uint8_t recCodecNow = 0xFF, recOsrNow = 0;

static void recStart(const LvHeader &h) {
  recActive = false;
  recId++;                                   // tells a running download that the old recording is gone
  recLen = 0;
  recFrames = 0;
  recFull = false;
  recActive = recBuf != nullptr;
  recCodecNow = h.codec;
  recOsrNow = h.osr;
  if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(5)) == pdTRUE) {
    dash.recCodec = h.codec; dash.recOsr = h.osr; dash.recPayload = h.len;
    xSemaphoreGive(dashLock);
  }
}

// Appends one frame: its header and its payload exactly as received (lost frames: header only).
static void recAppend(const LvHeader &h, const uint8_t *pl, uint8_t len) {
  if (!recActive) return;
  uint32_t need = sizeof(LvHeader) + len;
  if (recLen + need > REC_BYTES) { recFull = true; recActive = false; return; }
  memcpy(recBuf + recLen, &h, sizeof(LvHeader));
  if (len) memcpy(recBuf + recLen + sizeof(LvHeader), pl, len);
  recLen += need;                            // published only after the data is in place
  recFrames++;
}

// ---------------- Dashboard ----------------
static WebServer server(80);
static DNSServer dns;

static void dashReset() {
  winCount = winPos = 0;
  refSeq = -1;
  refParts = 0;
  for (auto &d : decRing) d.used = false;
  if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(5)) == pdTRUE) {
    dash.snrValid = false;
    dash.waveSeq = -1;
    xSemaphoreGive(dashLock);
  }
}

// Compares a complete reference frame with our decoded frame of the same sequence number.
static void measureReference() {
  for (auto &d : decRing) {
    if (!d.used || d.seq != (uint16_t)refSeq) continue;
    double sig = 0, err = 0;
    for (int i = 0; i < LV_FRAME_SAMPLES; i++) {
      double o = refBuf[i], e = (double)refBuf[i] - d.s[i];
      sig += o * o;
      err += e * e;
    }
    winSig[winPos] = sig;
    winErr[winPos] = err;
    winPos = (winPos + 1) % SNR_WINDOW;
    if (winCount < SNR_WINDOW) winCount++;
    double S = 0, E = 0;
    for (int k = 0; k < winCount; k++) { S += winSig[k]; E += winErr[k]; }
    if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(5)) == pdTRUE) {
      dash.snrValid = S > 0 && E > 0;
      dash.snr = dash.snrValid ? 10.0 * log10(S / E) : 0;
      dash.mse = E / (winCount * LV_FRAME_SAMPLES) / (32768.0 * 32768.0);   // samples scaled to +-1
      dash.frameSnr = (sig > 0 && err > 0) ? 10.0 * log10(sig / err) : 0;
      dash.waveSeq = refSeq;
      memcpy(dash.orig, refBuf, sizeof(refBuf));
      memcpy(dash.recon, d.s, sizeof(d.s));
      xSemaphoreGive(dashLock);
    }
    return;
  }
}

static void queueFrame(const Frame &f) {
  if (xQueueSend(frameQueue, &f, 0) != pdTRUE) { Frame junk; xQueueReceive(frameQueue, &junk, 0); xQueueSend(frameQueue, &f, 0); }
}

static void handleItem(const RxItem &it) {
  if (!lv_valid(it.data, it.len)) { sec.crc++; tot.crc++; return; }
  LvHeader h;
  memcpy(&h, it.data, sizeof(h));
  const uint8_t *pl = it.data + sizeof(LvHeader);

  if (h.type == LV_STOP) {
    if (streamActive) Serial.println(">> Transmitter STOPPED");
    streamActive = false;
    haveExpected = false;
    haveRunState = false;
    flushRequest = true;
    recActive = false;                                           // recording complete
    stopSeen = true;
    if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(5)) == pdTRUE) { dash.stopped = true; xSemaphoreGive(dashLock); }
    return;
  }

  if (h.type == LV_REF) {                                       // original samples for SNR/MSE
    if (h.len != 160 || h.flags > 1) return;
    if ((int32_t)h.seq != refSeq) { refSeq = h.seq; refParts = 0; }
    memcpy(&refBuf[h.flags * 80], pl, 160);
    refParts |= (uint8_t)(1 << h.flags);
    if (refParts == 3) { measureReference(); refParts = 0; }
    return;
  }
  if (h.type != LV_AUDIO) return;

  if (!streamActive || h.codec != streamCodec || h.osr != streamOsr) {
    if (h.codec == LV_DM || h.codec == LV_ADM)
      Serial.printf(">> Live audio STARTED: %s, OSR %u, %u-byte payload (%u kbit/s)\n", lv_codec_name(h.codec), h.osr, h.len, h.len * 8 * 50 / 1000);
    else
      Serial.printf(">> Live audio STARTED: %s %u-bit, %u-byte payload (%u kbit/s)\n", lv_codec_name(h.codec), h.bits, h.len, h.len * 8 * 50 / 1000);
    streamActive = true;
    haveExpected = false;
    haveRunState = false;
    dashReset();                                                 // fresh SNR window
    if (stopSeen || h.codec != recCodecNow || h.osr != recOsrNow) recStart(h);   // new transmission: new recording
    else recActive = recBuf != nullptr && !recFull;              // back in range: continue the same recording
    stopSeen = false;
  }
  lastAudioMs = millis();
  streamCodec = h.codec;
  streamOsr = h.osr;
  streamLen = h.len;

  if (haveExpected) {
    int16_t d = (int16_t)(h.seq - expectedSeq);
    if (d < 0) { sec.dup++; tot.dup++; return; }
    if (d > 0) {
      sec.lost += d; tot.lost += d;
      haveRunState = false;                                      // a frame is missing: resync from the header
      Frame c;
      c.conceal = 1;
      c.qvar = 0;
      for (int k = 0; k < d && k < 3; k++) queueFrame(c);        // keep timing: fill the gap
      LvHeader lostH = h;                                        // mark the missing frames in the recording
      lostH.flags = REC_LOST;
      lostH.len = 0;
      for (int k = 0; k < d && k < 50; k++) recAppend(lostH, nullptr, 0);
    }
  }
  expectedSeq = h.seq + 1;
  haveExpected = true;

  // Decoder state: start from the state written in the header. When no frame was lost,
  // it must equal the state our decoder reached at the end of the previous frame.
  CodecState st;
  lc_state_from_header(&st, &h);
  if (haveRunState && h.codec != LV_PCM &&
      (st.y != runState.y || (h.codec == LV_ADM && (st.step != runState.step || st.last != runState.last)))) {
    sec.stateMismatch++; tot.stateMismatch++;
  }

  Frame f;
  f.conceal = 0;
  f.qvar = codecNoiseVar(h);
  if (!lc_decode(&h, &st, pl, f.s)) { sec.other++; tot.other++; return; }   // inconsistent header
  runState = st;
  haveRunState = true;
  sec.rx++; tot.rx++;
  queueFrame(f);
  recAppend(h, pl, h.len);

  // keep the decoded frame for matching with its reference, and the payload for the dashboard
  DecSlot &slot = decRing[decPos];
  slot.seq = h.seq;
  slot.used = true;
  memcpy(slot.s, f.s, sizeof(f.s));
  decPos = (decPos + 1) % DEC_RING;
  if (xSemaphoreTake(dashLock, 0) == pdTRUE) {
    dash.codec = h.codec; dash.osr = h.osr; dash.bits = h.bits; dash.payload = h.len;
    dash.haveStream = true; dash.stopped = false;
    memcpy(dash.lastPayload, pl, h.len);
    dash.lastLen = h.len;
    dash.lastAudioMs = lastAudioMs;
    xSemaphoreGive(dashLock);
  }
}

// ---------------- Speaker output ----------------
static bool ampInit() {
  i2s_config_t c;
  memset(&c, 0, sizeof(c));
  c.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  c.sample_rate          = OUT_RATE;
  c.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  c.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  c.dma_buf_count        = 4;
  c.dma_buf_len          = LV_FRAME_SAMPLES * 2;      // 20 ms at 16 kHz
  c.use_apll             = false;
  c.tx_desc_auto_clear   = true;
  if (i2s_driver_install(AMP_PORT, &c, 0, NULL) != ESP_OK) return false;
  i2s_pin_config_t p;
  memset(&p, 0, sizeof(p));
  p.mck_io_num = I2S_PIN_NO_CHANGE;
  p.bck_io_num = AMP_BCLK;
  p.ws_io_num = AMP_LRC;
  p.data_out_num = AMP_DIN;
  p.data_in_num = I2S_PIN_NO_CHANGE;
  if (i2s_set_pin(AMP_PORT, &p) != ESP_OK) return false;
  i2s_zero_dma_buffer(AMP_PORT);
  return true;
}

// Plays one 8 kHz frame (already noise-cleaned): high-pass, volume (ramped from fade0 to fade1 across the frame, so
// gain changes never click), upsample, write 20 ms to I2S (blocks until there is room).
static void playFrame(const int16_t *s, float fade0, float fade1) {
  static HighPass hp;
  static Upsampler up;
  static bool init = false;
  static int16_t out[LV_FRAME_SAMPLES * 4];             // 320 stereo frames
  if (!init) { hp.init(300.0f, 8000.0f); up.init(); init = true; }
  float vg = volumeGain();
  for (int i = 0; i < LV_FRAME_SAMPLES; i++) {
    float g = vg * (fade0 + (fade1 - fade0) * (i + 1) / LV_FRAME_SAMPLES);
    float x = hp.run((float)s[i]) * g, y0, y1;
    up.push(x, y0, y1);
    int16_t a = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, y0));
    int16_t b = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, y1));
    out[4 * i] = a; out[4 * i + 1] = a; out[4 * i + 2] = b; out[4 * i + 3] = b;
  }
  size_t w;
  i2s_write(AMP_PORT, out, sizeof(out), &w, portMAX_DELAY);
}

static void playerTask(void *) {
  static int16_t lastGood[LV_FRAME_SAMPLES], silent[LV_FRAME_SAMPLES];
  memset(lastGood, 0, sizeof(lastGood));
  memset(silent, 0, sizeof(silent));
  int concealRun = 0;
  Frame f;
  for (;;) {
    if (flushRequest) { xQueueReset(frameQueue); flushRequest = false; playing = false; gate.reset(); nr.clear(); }

    // Test tone: 440 Hz at the transmitter's speech-peak level, through the same filters and volume
    if (toneFrames > 0) {
      static int16_t tf[LV_FRAME_SAMPLES];
      static float ph = 0;
      for (int i = 0; i < LV_FRAME_SAMPLES; i++) {
        tf[i] = (int16_t)(sinf(ph) * SPEECH_PEAK);
        ph += 2.0f * (float)M_PI * 440.0f / 8000.0f;
        if (ph > 2.0f * (float)M_PI) ph -= 2.0f * (float)M_PI;
      }
      playFrame(tf, 1.0f, 1.0f);
      toneFrames--;
      continue;
    }

    if (!playing) {
      if (uxQueueMessagesWaiting(frameQueue) >= PREBUFFER) { playing = true; concealRun = 0; }
      else { playFrame(silent, 0, 0); continue; }     // keeps the I2S clock and timing running
    }

    if (uxQueueMessagesWaiting(frameQueue) > MAX_QUEUED) { xQueueReceive(frameQueue, &f, 0); dropCount++; }

    bool got = xQueueReceive(frameQueue, &f, 0) == pdTRUE;
    if (got && !f.conceal) {
      nr.process(f.s, gate.hang == 0, f.qvar);              // noise removal first (learns while the gate is closed)
      memcpy(lastGood, f.s, sizeof(lastGood));
      concealRun = 0;
      float g0, g1;
      gate.process(f.s, LV_FRAME_SAMPLES, g0, g1);
      playFrame(f.s, g0, g1);
    } else {
      // a lost frame (got && conceal) or the buffer ran empty (!got):
      // replay the last good frame, fading out, then go silent and re-buffer
      if (!got) underrunCount++;
      concealRun++;
      static const float fades[4] = {1.0f, 0.5f, 0.25f, 0.1f};
      if (concealRun <= 3) playFrame(lastGood, gate.g * fades[concealRun - 1], gate.g * fades[concealRun]);
      else { playing = false; playFrame(silent, 0, 0); }
    }
  }
}

// ---------------- Web server (runs in its own task on core 0) ----------------
static void sendDashboard() { server.send_P(200, "text/html", DASHBOARD_HTML); }

static void sendCaptiveRedirect() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

static String recName(uint8_t c, uint8_t osr) {
  if (c == LV_DM || c == LV_ADM) return String(lv_codec_name(c)) + (osr == 4 ? " (4x oversampled)" : " (1x)");
  return String(lv_codec_name(c)) + (c == LV_PCM ? " 8-bit" : " 4-bit");
}

static void sendData() {
  static Dash d;
  static char buf[4096];
  if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(20)) != pdTRUE) { server.send(503, "text/plain", "busy"); return; }
  memcpy(&d, &dash, sizeof(d));
  xSemaphoreGive(dashLock);

  uint32_t age = d.haveStream ? millis() - d.lastAudioMs : 0xFFFFFFFF;
  const char *status = d.stopped ? "STOPPED" : (d.haveStream && age < 1000 ? "LIVE" : (d.haveStream ? "NO SIGNAL" : "WAITING"));
  float lossPct = (d.rx + d.lost) ? 100.0f * d.lost / (d.rx + d.lost) : 0.0f;
  int n = snprintf(buf, sizeof(buf),
    "{\"status\":\"%s\",\"codecName\":\"%s\",\"bits\":%u,\"osr\":%u,\"payload\":%u,\"kbps\":%u,\"pps\":%lu,"
    "\"rx\":%lu,\"lost\":%lu,\"lossPct\":%.3f,\"crc\":%lu,\"stateMismatch\":%lu,\"rejected\":%lu,"
    "\"snrValid\":%s,\"snr\":%.2f,\"mse\":%.4e,\"frameSnr\":%.2f,\"waveSeq\":%ld,\"volume\":%d,\"uptime\":%lu,"
    "\"recId\":%lu,\"recSec\":%.2f,\"recLive\":%s,\"recFull\":%s,\"recName\":\"%s\",\"recMaxSec\":%lu,",
    status, d.haveStream ? lv_codec_name(d.codec) : "", d.bits, d.osr, d.payload, (unsigned)(d.payload * 8 * 50 / 1000),
    (unsigned long)d.pps, (unsigned long)d.rx, (unsigned long)d.lost, lossPct, (unsigned long)d.crc,
    (unsigned long)d.stateMismatch, (unsigned long)d.rejected, d.snrValid ? "true" : "false",
    d.snr, d.mse, d.frameSnr, (long)d.waveSeq, volumeStep, (unsigned long)(millis() / 1000),
    (unsigned long)recId, recFrames * 0.02f, recActive ? "true" : "false", recFull ? "true" : "false",
    recName(d.recCodec, d.recOsr).c_str(),
    (unsigned long)(d.recPayload ? REC_BYTES / ((sizeof(LvHeader) + d.recPayload) * 50UL) : 0));

  // waveforms of the last measured frame
  n += snprintf(buf + n, sizeof(buf) - n, "\"orig\":[");
  for (int i = 0; d.waveSeq >= 0 && i < LV_FRAME_SAMPLES; i++) n += snprintf(buf + n, sizeof(buf) - n, i ? ",%d" : "%d", d.orig[i]);
  n += snprintf(buf + n, sizeof(buf) - n, "],\"recon\":[");
  for (int i = 0; d.waveSeq >= 0 && i < LV_FRAME_SAMPLES; i++) n += snprintf(buf + n, sizeof(buf) - n, i ? ",%d" : "%d", d.recon[i]);

  // first codes of the latest packet (unpacked) and its first bytes in hex
  n += snprintf(buf + n, sizeof(buf) - n, "],\"codes\":[");
  int show = d.bits == 1 ? 96 : (d.bits == 4 ? 48 : 32);
  for (int i = 0; d.haveStream && i < show && (i * d.bits) / 8 < d.lastLen; i++) {
    int code;
    if (d.bits == 8) code = d.lastPayload[i];
    else if (d.bits == 4) code = (i & 1) ? (d.lastPayload[i >> 1] & 0x0F) : (d.lastPayload[i >> 1] >> 4);
    else code = (d.lastPayload[i >> 3] >> (7 - (i & 7))) & 1;
    n += snprintf(buf + n, sizeof(buf) - n, i ? ",%d" : "%d", code);
  }
  n += snprintf(buf + n, sizeof(buf) - n, "],\"hex\":\"");
  for (int i = 0; d.haveStream && i < 24 && i < d.lastLen; i++) n += snprintf(buf + n, sizeof(buf) - n, "%02X ", d.lastPayload[i]);
  n += snprintf(buf + n, sizeof(buf) - n, "\"}");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

static void makeWavHeader(uint8_t *w, uint32_t fs, uint32_t nSamples) {
  uint32_t dataBytes = nSamples * 2, riff = 36 + dataBytes, byteRate = fs * 2, fmtSize = 16;
  uint16_t fmt = 1, ch = 1, align = 2, bps = 16;
  memcpy(w, "RIFF", 4);  memcpy(w + 4, &riff, 4);   memcpy(w + 8, "WAVEfmt ", 8);  memcpy(w + 16, &fmtSize, 4);
  memcpy(w + 20, &fmt, 2); memcpy(w + 22, &ch, 2);  memcpy(w + 24, &fs, 4);        memcpy(w + 28, &byteRate, 4);
  memcpy(w + 32, &align, 2); memcpy(w + 34, &bps, 2); memcpy(w + 36, "data", 4);   memcpy(w + 40, &dataBytes, 4);
}

// Serves the recording as a 16-bit, 8 kHz mono WAV, decoding the stored packets on the fly
// with the same decoder that fed the speaker, then cleaned exactly like the speaker sound
// (noise remover, voice gate, 300 Hz high-pass). /rec.wav?raw=1 gives the uncleaned decoder output.
// Supports Range requests (needed by iPhones).
static Denoiser recNr;                     // separate copies for the web task (the player keeps its own)
static VoiceGate recGate;
static HighPass recHp;
static void sendRecording() {
  uint32_t id = recId, len = recLen, frames = recFrames;
  if (!recBuf || frames == 0) { server.send(404, "text/plain", "No recording yet"); return; }
  uint32_t total = 44 + frames * LV_FRAME_SAMPLES * 2, start = 0, end = total - 1;
  bool partial = false;
  if (server.hasHeader("Range")) {
    String r = server.header("Range");                       // "bytes=a-b" or "bytes=a-"
    if (r.startsWith("bytes=")) {
      int dash_ = r.indexOf('-');
      String a = r.substring(6, dash_), b = r.substring(dash_ + 1);
      if (a.length()) start = a.toInt();
      if (b.length()) end = b.toInt();
      if (end >= total) end = total - 1;
      if (start > end) { server.sendHeader("Content-Range", "bytes */" + String(total)); server.send(416, "text/plain", ""); return; }
      partial = true;
    }
  }
  server.sendHeader("Accept-Ranges", "bytes");
  server.sendHeader("Cache-Control", "no-store");
  if (server.hasArg("dl")) {
    char name[64];
    snprintf(name, sizeof(name), "attachment; filename=\"reconstructed_%s_%lu.wav\"", lv_codec_name(dash.recCodec), (unsigned long)id);
    server.sendHeader("Content-Disposition", name);
  }
  if (partial) server.sendHeader("Content-Range", "bytes " + String(start) + "-" + String(end) + "/" + String(total));
  server.setContentLength(end - start + 1);
  server.send(partial ? 206 : 200, "audio/wav", "");

  WiFiClient client = server.client();
  static uint8_t out[1024];
  size_t o = 0;
  uint32_t pos = 0;
  auto emit = [&](const uint8_t *b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++, pos++) {
      if (pos < start || pos > end) continue;
      out[o++] = b[i];
      if (o == sizeof(out)) { client.write(out, o); o = 0; }
    }
  };
  uint8_t hdr[44];
  makeWavHeader(hdr, 8000, frames * LV_FRAME_SAMPLES);
  emit(hdr, 44);
  bool raw = server.hasArg("raw");
  recNr.init();
  recGate = VoiceGate();
  recHp.init(300.0f, 8000.0f);
  uint32_t off = 0;
  int16_t pcm[LV_FRAME_SAMPLES];
  for (uint32_t k = 0; k < frames && pos <= end && client.connected(); k++) {
    if (recId != id || off + sizeof(LvHeader) > len) break;   // a new recording started: stop
    LvHeader h;
    memcpy(&h, recBuf + off, sizeof(h));
    const uint8_t *pl = recBuf + off + sizeof(LvHeader);
    off += sizeof(LvHeader) + h.len;
    CodecState st;
    lc_state_from_header(&st, &h);
    if (h.flags == REC_LOST || !lc_decode(&h, &st, pl, pcm)) memset(pcm, 0, sizeof(pcm));
    if (!raw) {                                                // same cleaning as the speaker
      recNr.process(pcm, recGate.hang == 0, codecNoiseVar(h));
      float g0, g1;
      recGate.process(pcm, LV_FRAME_SAMPLES, g0, g1);
      for (int i = 0; i < LV_FRAME_SAMPLES; i++) {
        float g = g0 + (g1 - g0) * (i + 1) / LV_FRAME_SAMPLES;
        pcm[i] = (int16_t)fmaxf(-32767.0f, fminf(32767.0f, recHp.run((float)pcm[i]) * g));
      }
    }
    emit((const uint8_t *)pcm, sizeof(pcm));
  }
  if (o) client.write(out, o);
}

static void sendCmd() {
  String c = server.arg("c");
  if (c == "up" && volumeStep < 100) volumeStep++;
  else if (c == "down" && volumeStep > 0) volumeStep--;
  else if (c == "tone") toneFrames = 75;
  server.send(200, "text/plain", "ok");
}

static void webTask(void *) {
  for (;;) {
    dns.processNextRequest();
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void webInit() {
  WiFi.softAP(AP_SSID, AP_PASSWORD, LV_CHANNEL);
  delay(100);
  static const char *hdrKeys[] = {"Range"};
  server.collectHeaders(hdrKeys, 1);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());                     // every name -> this receiver (captive portal)
  server.on("/", sendDashboard);
  server.on("/data", sendData);
  server.on("/cmd", sendCmd);
  server.on("/rec.wav", sendRecording);
  // Phones check these addresses to detect a sign-in page; answering with a redirect
  // makes the phone open the dashboard by itself.
  const char *probes[] = {"/generate_204", "/gen_204", "/hotspot-detect.html", "/library/test/success.html",
                          "/connecttest.txt", "/ncsi.txt", "/redirect", "/canonical.html", "/success.txt", "/fwlink"};
  for (const char *p : probes) server.on(p, sendCaptiveRedirect);
  server.onNotFound(sendCaptiveRedirect);
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 6144, NULL, 1, NULL, 0);
  Serial.printf("[WEB] Wi-Fi \"%s\" (password %s). The dashboard opens by itself, or go to http://%s\n",
                AP_SSID, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
}

static void sendBeacon() {
  static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t pkt[LV_MAX_PACKET];
  LvHeader h;
  memset(&h, 0, sizeof(h));
  h.type = LV_BEACON;
  size_t n = lv_finish(pkt, &h, nullptr, 0);
  esp_now_send(BCAST, pkt, n);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_LED, OUTPUT);
  Serial.println();
  Serial.println("=== LIVE VOICE RECEIVER (final: PCM, DPCM, DM, ADM + dashboard) ===");

  // Reserve the recording memory first, while the heap is still in one large piece.
  recBuf = (uint8_t *)malloc(REC_BYTES);
  Serial.printf("[REC] %s %lu bytes for the recording\n", recBuf ? "reserved" : "COULD NOT reserve", (unsigned long)REC_BYTES);
  dashLock = xSemaphoreCreateMutex();
  memset(&dash, 0, sizeof(dash));
  dash.waveSeq = -1;
  rxQueue = xQueueCreate(16, sizeof(RxItem));
  frameQueue = xQueueCreate(MAX_QUEUED + 4, sizeof(Frame));
  nr.init();
  if (!ampInit()) { Serial.println("ERR: amplifier I2S init failed"); while (true) delay(1000); }
  xTaskCreatePinnedToCore(playerTask, "player", 4096, NULL, 3, NULL, 1);

  WiFi.mode(WIFI_AP_STA);                  // hotspot for the phone + station interface for ESP-NOW
  WiFi.disconnect();
  WiFi.setSleep(false);
  webInit();                               // starts the hotspot on channel LV_CHANNEL
  esp_wifi_set_channel(LV_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ERR: ESP-NOW init failed"); while (true) delay(1000); }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t p;
  memset(&p, 0, sizeof(p));
  memset(p.peer_addr, 0xFF, 6);
  p.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&p);

  Serial.printf("ESP-NOW MAC %s, channel %d. Volume %d (+ / - volume, t test tone, g gate on/off, n noise removal on/off).\n",
                WiFi.macAddress().c_str(), LV_CHANNEL, volumeStep);
  Serial.println("Waiting for the transmitter (turn S3 ON there).");
}

void loop() {
  static uint32_t lastReport = 0, lastBeacon = 0, urRep = 0, dropRep = 0;
  RxItem it;
  while (xQueueReceive(rxQueue, &it, pdMS_TO_TICKS(5)) == pdTRUE) handleItem(it);

  if (millis() - lastBeacon >= 1000) { lastBeacon = millis(); sendBeacon(); }

  if (streamActive && millis() - lastAudioMs > 1000) {
    Serial.println("!! No audio for 1 s - transmitter off or out of range");
    streamActive = false;
    haveExpected = false;
    haveRunState = false;
    recActive = false;                                  // keep what was recorded so far
  }
  digitalWrite(PIN_LED, playing ? HIGH : LOW);

  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    uint32_t ur = underrunCount, dr = dropCount;
    sec.underrun = ur - urRep; sec.dropped = dr - dropRep; urRep = ur; dropRep = dr;
    tot.underrun = ur; tot.dropped = dr;
    if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(5)) == pdTRUE) {
      dash.rx = tot.rx; dash.lost = tot.lost; dash.crc = tot.crc;
      dash.stateMismatch = tot.stateMismatch; dash.rejected = tot.other; dash.pps = sec.rx;
      xSemaphoreGive(dashLock);
    }
    if (streamActive || sec.rx || sec.crc) {
      uint32_t exp = sec.rx + sec.lost, texp = tot.rx + tot.lost;
      char desc[24], snrTxt[12];
      if (streamCodec == LV_DM || streamCodec == LV_ADM) snprintf(desc, sizeof(desc), "%s OSR%u", lv_codec_name(streamCodec), streamOsr);
      else snprintf(desc, sizeof(desc), "%s %u-bit", lv_codec_name(streamCodec), lc_bits(streamCodec));
      snprintf(snrTxt, sizeof(snrTxt), "-");
      if (xSemaphoreTake(dashLock, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (dash.snrValid) snprintf(snrTxt, sizeof(snrTxt), "%.1f dB", dash.snr);
        xSemaphoreGive(dashLock);
      }
      uint32_t gf = gate.frames, go = gate.openFrames;
      gate.frames = gate.openFrames = 0;
      Serial.printf("%s %-9s %2u kbit/s | rx %2lu/s lost %lu (%.1f%%) CRC %lu state-check %lu | SNR %s | buffer %u, gaps filled %lu | total lost %.2f%% | vol %d, gate %s %lu%% open\n",
                    playing ? "PLAY" : "wait", desc, (unsigned)(streamLen * 8 * 50 / 1000), (unsigned long)sec.rx,
                    (unsigned long)sec.lost, exp ? 100.0f * sec.lost / exp : 0.0f, (unsigned long)sec.crc,
                    (unsigned long)sec.stateMismatch, snrTxt,
                    (unsigned)uxQueueMessagesWaiting(frameQueue), (unsigned long)sec.underrun,
                    texp ? 100.0f * tot.lost / texp : 0.0f, volumeStep, gate.enabled ? "on" : "OFF",
                    (unsigned long)(gf ? 100 * go / gf : 0));
      if (sec.other) Serial.printf("   %lu frames rejected: header does not match its payload\n", (unsigned long)sec.other);
    }
    if (rxQueueDrops) { Serial.printf("   radio queue full, dropped %lu\n", (unsigned long)rxQueueDrops); rxQueueDrops = 0; }
    sec = {0, 0, 0, 0, 0, 0, 0, 0};
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '+' || c == '-') {
      int v = volumeStep + (c == '+' ? 1 : -1);
      volumeStep = v < 0 ? 0 : (v > 100 ? 100 : v);
      Serial.printf("[VOL] %d%s\n", volumeStep, volumeStep > 30 ? "  (high: may buzz on loud words)" : "");
    } else if (c == 'r' || c == 'R') {
      tot = {0, 0, 0, 0, 0, 0, 0, 0};
      Serial.println("[counters reset]");
    } else if (c == 'g' || c == 'G') {
      gate.enabled = !gate.enabled;
      Serial.printf("[GATE] noise gate %s\n", gate.enabled ? "ON (hiss between words muted)" : "OFF (everything played)");
    } else if (c == 'n' || c == 'N') {
      nr.enabled = !nr.enabled;
      Serial.printf("[NR] noise removal %s\n", nr.enabled ? "ON" : "OFF");
    } else if (c == 't' || c == 'T') {
      toneFrames = 75;                       // 1.5 s
      Serial.printf("[TONE] 440 Hz, 1.5 s, through the voice path at volume %d\n", volumeStep);
    }
  }
}