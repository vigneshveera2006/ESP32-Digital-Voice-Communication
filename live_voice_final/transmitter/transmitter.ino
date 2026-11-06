// =====================================================================
//  transmitter.ino  —  LIVE wireless voice: TRANSMITTER
//  Arduino-ESP32 core 2.0.14, board "ESP32 Dev Module".
//  Version: FINAL (PCM, DPCM, DM, ADM + reference frames for the receiver's SNR/MSE).
//
//  INMP441 (32 kHz) -> low-pass 3.6 kHz -> 300 Hz high-pass -> automatic gain
//  -> silence gate (only voice is sent; in pauses the gain is held and real silence is sent)
//  -> 20 ms frames -> PCM / DPCM / DM / ADM encoder (chosen with S1, S2)
//  -> packet (header with sequence number, encoder state, parameters, CRC-16)
//  -> ESP-NOW.  Every 10th frame the original (encoder input) samples are also sent,
//  so the receiver can measure SNR and MSE on exactly aligned samples.
//
//  Encodings:  S1 S2 = OFF OFF PCM 8-bit (64 kbit/s)   OFF ON DPCM 4-bit (32 kbit/s)
//                      ON  OFF DM 1-bit                ON  ON ADM 1-bit
//  DM/ADM run 4x oversampled (32 kbit/s) by default; type  o + Enter  in the
//  Serial Monitor while stopped to switch to 1x (8 kbit/s) and back.
//
//  Wiring: INMP441 VDD->3V3, GND->GND, L/R->GND, SCK->26, WS->25, SD->33
//          S1->18, S2->19, S3->21 (other side to GND). LED = on-board GPIO 2.
//  S3 ON = transmit, S3 OFF = stop.  Serial Monitor 115200 baud.
// =====================================================================
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <driver/i2s.h>
#include "live_protocol.h"
#include "live_dsp.h"
#include "live_codecs.h"

// ---------------- Pins ----------------
#define MIC_SCK   26
#define MIC_WS    25
#define MIC_SD    33
#define PIN_S1    18
#define PIN_S2    19
#define PIN_S3    21
#define PIN_LED   2

// ---------------- Audio ----------------
#define MIC_PORT     I2S_NUM_0
#define MIC_RATE     32000
#define MIC_FRAMES   (LV_FRAME_SAMPLES * 4)       // 640 frames at 32 kHz = 20 ms
#define REF_EVERY    10                           // send a reference frame every 10th frame (5 per second)
#define NOISE_HIST   100                          // room-noise floor = quietest frame of the last 2 s
#define VOICE_RATIO  3.0f                         // voice must be ~10 dB above the room noise ...
#define VOICE_MIN_RMS 40.0f                       // ... and above this level (16-bit units, ~-58 dBFS)
#define VOICE_HANG   15                           // keep sending 300 ms after the last word

// ---------------- Types (above every function, see the Arduino IDE note in receiver.ino) ----------------
struct Sw { uint8_t pin; bool state; bool last; uint32_t since; };

static int32_t micBuf[MIC_FRAMES * 2];             // stereo slots
static int     micSlot = -1;                       // auto-detected slot carrying the microphone
static LowPass32 lp32;
static HighPass  hpf;
static Agc       agc;
static float     work32[MIC_FRAMES];
static int16_t   frame32[MIC_FRAMES];              // 32 kHz, for DM/ADM at OSR 4
static int16_t   frame8[LV_FRAME_SAMPLES];         // 8 kHz, for PCM, DPCM, DM/ADM at OSR 1

// ---------------- Radio ----------------
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t  peerMac[6];
static bool     havePeer = false;
static volatile bool beaconSeen = false;
static uint8_t  beaconMac[6];
static volatile uint32_t sentOk = 0, sentFail = 0;
static volatile uint32_t lastDeliveredMs = 0;        // last packet the receiver acknowledged
static uint32_t sendCalls = 0, skippedFrames = 0, radioResets = 0;
#define MAX_IN_FLIGHT   6                            // packets sent but not yet confirmed
#define LINK_LOST_MS    1500                         // no delivery for this long -> reset the radio

// ---------------- State ----------------
static bool       running = false;
static uint8_t    codec = LV_PCM;
static uint8_t    dmOsr = 4;                       // oversampling for DM/ADM (4 or 1)
static uint8_t    runOsr = 1;
static CodecState enc;
static uint16_t   seq = 0;
static uint32_t   framesSent = 0, sendErr = 0;
static float      inLevelDb = -120;                // microphone level before gain
static float      noiseFloorDb = -120;             // room noise level before gain
static bool       txOpen = false;                  // true = voice detected, sound is being sent

static Sw sw[3] = {{PIN_S1, false, false, 0}, {PIN_S2, false, false, 0}, {PIN_S3, false, false, 0}};

// ---------------- Switches (debounced) ----------------
static void readSwitches() {
  uint32_t now = millis();
  for (auto &s : sw) {
    bool raw = digitalRead(s.pin) == LOW;
    if (raw != s.last) { s.last = raw; s.since = now; }
    else if (raw != s.state && now - s.since >= 30) s.state = raw;
  }
}
static uint8_t selectedCodec() { return (sw[0].state ? 2 : 0) | (sw[1].state ? 1 : 0); }

// ---------------- ESP-NOW ----------------
static void onSent(const uint8_t *mac, esp_now_send_status_t st) {
  if (st == ESP_NOW_SEND_SUCCESS) { sentOk++; lastDeliveredMs = millis(); } else sentFail++;
}
static void handleRx(const uint8_t *mac, const uint8_t *data, int len) {
  if (lv_valid(data, len) && data[1] == LV_BEACON && !beaconSeen) { memcpy(beaconMac, mac, 6); beaconSeen = true; }
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handleRx(info->src_addr, data, len); }
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleRx(mac, data, len); }
#endif
static bool addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p;
  memset(&p, 0, sizeof(p));
  memcpy(p.peer_addr, mac, 6);
  p.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&p) == ESP_OK;
}
static void sendRaw(const uint8_t *pkt, size_t n) {
  if (esp_now_send(havePeer ? peerMac : BCAST, pkt, n) == ESP_OK) sendCalls++;
  else sendErr++;
}

// Packets handed to the radio whose delivery result has not come back yet.
static uint32_t inFlight() { uint32_t done = sentOk + sentFail; return sendCalls > done ? sendCalls - done : 0; }

static void radioInit() {
  esp_now_init();
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);
  addPeer(BCAST);
}

// Self-repair: after a long time out of range the radio can stay stuck. Restart ESP-NOW,
// forget the receiver and broadcast until its beacon is heard again (within ~1 s once in range).
static void radioReset() {
  esp_now_deinit();
  radioInit();
  havePeer = false;
  beaconSeen = false;
  sendCalls = sentOk + sentFail;
  lastDeliveredMs = millis();
  radioResets++;
  Serial.println("!! No delivery for 1.5 s - receiver out of range. Radio reset; searching for the receiver...");
}

// ---------------- Microphone ----------------
static bool micInit() {
  i2s_config_t c;
  memset(&c, 0, sizeof(c));
  c.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  c.sample_rate          = MIC_RATE;
  c.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
  c.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  c.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  c.dma_buf_count        = 4;
  c.dma_buf_len          = MIC_FRAMES;
  c.use_apll             = false;
  if (i2s_driver_install(MIC_PORT, &c, 0, NULL) != ESP_OK) return false;
  i2s_pin_config_t p;
  memset(&p, 0, sizeof(p));
  p.mck_io_num = I2S_PIN_NO_CHANGE;
  p.bck_io_num = MIC_SCK;
  p.ws_io_num = MIC_WS;
  p.data_out_num = I2S_PIN_NO_CHANGE;
  p.data_in_num = MIC_SD;
  if (i2s_set_pin(MIC_PORT, &p) != ESP_OK) return false;
  i2s_zero_dma_buffer(MIC_PORT);
  return true;
}

// Reads 20 ms from the microphone and builds frame32 (640) and frame8 (160).
static bool captureFrame() {
  size_t bytes = 0;
  i2s_read(MIC_PORT, micBuf, sizeof(micBuf), &bytes, portMAX_DELAY);
  int frames = bytes / 8;
  if (frames != MIC_FRAMES) return false;
  if (micSlot < 0) {                                       // which slot holds the microphone?
    double e0 = 0, e1 = 0;
    for (int i = 0; i < frames; i++) { e0 += fabs((double)(micBuf[2 * i] >> 8)); e1 += fabs((double)(micBuf[2 * i + 1] >> 8)); }
    if (e0 == 0 && e1 == 0) return false;
    micSlot = e0 >= e1 ? 0 : 1;
    Serial.printf(">> Microphone found in I2S slot %d\n", micSlot);
  }
  float sq = 0;
  for (int i = 0; i < MIC_FRAMES; i++) {
    float x = (float)(micBuf[2 * i + micSlot] >> 8) / 256.0f;     // 24-bit -> 16-bit scale
    float y = hpf.run(lp32.run(x));
    work32[i] = y;
    sq += y * y;
  }
  float rms = sqrtf(sq / MIC_FRAMES);
  inLevelDb = rms > 0 ? 20 * log10f(rms / 32768.0f) : -120;

  // Room-noise floor = quietest frame of the last 2 s (before any gain). Only sound clearly
  // (10 dB) above it counts as voice. In silence the AGC must NOT turn the gain up (that is
  // what made the noise grow when nobody talked), and the frame is faded to real silence.
  static float lvlHist[NOISE_HIST];
  static int lvlPos = 0, lvlN = 0, voiceHang = 0;
  static float txGate = 0.0f;
  lvlHist[lvlPos] = rms;
  lvlPos = (lvlPos + 1) % NOISE_HIST;
  if (lvlN < NOISE_HIST) lvlN++;
  float floorLvl = rms;
  for (int k = 0; k < lvlN; k++) if (lvlHist[k] < floorLvl) floorLvl = lvlHist[k];
  float voiceThr = fmaxf(VOICE_MIN_RMS, floorLvl * VOICE_RATIO);
  bool voice = lvlN >= 10 && rms > voiceThr;               // first 0.2 s: learning the room noise
  if (voice) voiceHang = VOICE_HANG; else if (voiceHang > 0) voiceHang--;
  noiseFloorDb = floorLvl > 0 ? 20 * log10f(floorLvl / 32768.0f) : -120;
  agc.gateRms = voiceThr;                                  // AGC adapts only on voice, holds its gain otherwise
  agc.process(work32, MIC_FRAMES);
  float g0 = txGate, target = voiceHang > 0 ? 1.0f : 0.0f;
  txGate = target > txGate ? target : txGate * 0.6f;       // open at once, close over ~0.1 s
  if (txGate < 0.02f) txGate = 0.0f;
  for (int i = 0; i < MIC_FRAMES; i++) work32[i] *= g0 + (txGate - g0) * (i + 1) / MIC_FRAMES;
  txOpen = txGate > 0.5f;
  for (int i = 0; i < MIC_FRAMES; i++) frame32[i] = (int16_t)lrintf(work32[i]);
  for (int k = 0; k < LV_FRAME_SAMPLES; k++) frame8[k] = frame32[4 * k];
  return true;
}

// ---------------- Encoding and sending ----------------
static void startHeader(LvHeader *h) {
  memset(h, 0, sizeof(*h));
  h->type = LV_AUDIO;
  h->seq = seq;
  h->nsamples = LV_FRAME_SAMPLES;
  lc_params(codec, runOsr, h);
}

// Reference frame for the receiver's SNR/MSE: the encoder's input for this frame, at 8 kHz,
// aligned with what the decoder outputs (for OSR 4: the average of each block of 4, exactly
// as the decoder averages). Sent as two LV_REF packets carrying the same sequence number.
// Used only for the dashboard measurements, never played.
static void sendReference(uint16_t frameSeq, bool os4) {
  static int16_t ref[LV_FRAME_SAMPLES];
  static uint8_t pkt[LV_MAX_PACKET];
  for (int k = 0; k < LV_FRAME_SAMPLES; k++) {
    if (os4) ref[k] = (int16_t)(((int32_t)frame32[4 * k] + frame32[4 * k + 1] + frame32[4 * k + 2] + frame32[4 * k + 3]) >> 2);
    else ref[k] = frame8[k];
  }
  for (uint8_t part = 0; part < 2; part++) {
    LvHeader h;
    startHeader(&h);
    h.type = LV_REF;
    h.seq = frameSeq;
    h.flags = part;                                   // part 0 = samples 0..79, part 1 = 80..159
    size_t n = lv_finish(pkt, &h, (const uint8_t *)&ref[part * 80], 160);
    sendRaw(pkt, n);
  }
}

static void sendFrame() {
  static uint8_t payload[LV_MAX_PAYLOAD], pkt[LV_MAX_PACKET];
  LvHeader h;
  startHeader(&h);
  lc_state_to_header(&enc, &h);                     // decoder state at the START of this frame
  bool os4 = (codec == LV_DM || codec == LV_ADM) && runOsr == 4;
  lc_encode(&h, &enc, os4 ? frame32 : frame8, os4 ? MIC_FRAMES : LV_FRAME_SAMPLES, payload);
  size_t n = lv_finish(pkt, &h, payload, lc_frame_bytes(codec, runOsr));
  sendRaw(pkt, n);
  if (seq % REF_EVERY == 0) sendReference(seq, os4);  // 5 times per second
  seq++;
  framesSent++;
}

static void describe(char *buf, size_t n, uint8_t c, uint8_t osr) {
  uint32_t kbps = lc_frame_bytes(c, osr) * 8 * 50 / 1000;
  if (c == LV_DM || c == LV_ADM) snprintf(buf, n, "%s OSR%u %lu kbit/s", lv_codec_name(c), osr, (unsigned long)kbps);
  else snprintf(buf, n, "%s %u-bit %lu kbit/s", lv_codec_name(c), lc_bits(c), (unsigned long)kbps);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_S1, INPUT_PULLUP);
  pinMode(PIN_S2, INPUT_PULLUP);
  pinMode(PIN_S3, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  Serial.println();
  Serial.println("=== LIVE VOICE TRANSMITTER (final: PCM, DPCM, DM, ADM) ===");

  lp32.init();
  hpf.init(300.0f, (float)MIC_RATE);
  if (!micInit()) { Serial.println("ERR: microphone I2S init failed"); while (true) delay(1000); }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(LV_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ERR: ESP-NOW init failed"); while (true) delay(1000); }
  esp_now_deinit();
  radioInit();

  Serial.printf("MAC %s, channel %d. Waiting for the receiver's beacon.\n", WiFi.macAddress().c_str(), LV_CHANNEL);
  Serial.println("Choose the encoding with S1/S2, then turn S3 ON to talk. Type o + Enter (while stopped) to switch DM/ADM between OSR 4 and OSR 1.");
  for (int i = 0; i < 5; i++) { readSwitches(); delay(10); }
}

void loop() {
  static uint32_t lastReport = 0, lastBlink = 0, okRep = 0, failRep = 0, sentRep = 0;
  char desc[48];

  bool frameReady = captureFrame();            // paced by the microphone: one frame every 20 ms
  readSwitches();

  if (beaconSeen && !havePeer && addPeer(beaconMac)) {
    memcpy(peerMac, beaconMac, 6);
    havePeer = true;
    Serial.printf(">> Receiver found: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4], peerMac[5]);
  }

  bool s3 = sw[2].state;
  if (s3 && !running) {
    codec = selectedCodec();
    runOsr = (codec == LV_DM || codec == LV_ADM) ? dmOsr : 1;
    LvHeader h;
    startHeader(&h);
    lc_reset(&enc, &h);                        // fresh encoder state for every start
    running = true;
    lastDeliveredMs = millis();
    describe(desc, sizeof(desc), codec, runOsr);
    Serial.printf(">> START: %s\n", desc);
  } else if (!s3 && running) {
    running = false;
    uint8_t pkt[LV_MAX_PACKET];
    LvHeader h;
    memset(&h, 0, sizeof(h));
    h.type = LV_STOP;
    h.seq = seq;
    size_t n = lv_finish(pkt, &h, nullptr, 0);
    for (int i = 0; i < 3; i++) { sendRaw(pkt, n); delay(3); }
    Serial.println(">> STOP. Change S1/S2 if you like, then S3 ON again.");
  }

  if (running && frameReady && micSlot >= 0) {
    if (inFlight() > MAX_IN_FLIGHT) skippedFrames++;        // radio still busy retrying: drop, don't pile up
    else sendFrame();
  }
  if (running && havePeer && millis() - lastDeliveredMs > LINK_LOST_MS) radioReset();

  if (running) digitalWrite(PIN_LED, HIGH);
  else if (millis() - lastBlink >= 500) { lastBlink = millis(); digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }

  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    uint32_t ok = sentOk, fail = sentFail;
    uint8_t c = running ? codec : selectedCodec();
    describe(desc, sizeof(desc), c, running ? runOsr : ((c == LV_DM || c == LV_ADM) ? dmOsr : 1));
    Serial.printf("%s %-22s | mic %6.1f dBFS (noise %6.1f, %s) gain %+5.1f dB | sent %lu/s delivered %lu lost-in-air %lu | %s\n",
                  running ? "TX " : "off", desc, inLevelDb, noiseFloorDb, txOpen ? "VOICE" : "quiet", 20 * log10f(agc.gain),
                  (unsigned long)(framesSent - sentRep), (unsigned long)(ok - okRep), (unsigned long)(fail - failRep),
                  havePeer ? "receiver found" : "no receiver yet");
    if (sendErr || skippedFrames) {
      Serial.printf("   send errors %lu, frames skipped (radio busy) %lu, radio resets so far %lu\n",
                    (unsigned long)sendErr, (unsigned long)skippedFrames, (unsigned long)radioResets);
      sendErr = 0;
      skippedFrames = 0;
    }
    okRep = ok; failRep = fail; sentRep = framesSent;
  }

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'o' || ch == 'O') {
      if (running) Serial.println(">> Turn S3 OFF first, then change OSR.");
      else { dmOsr = dmOsr == 4 ? 1 : 4; Serial.printf(">> DM/ADM oversampling now OSR %u (%u kbit/s)\n", dmOsr, dmOsr * 8); }
    }
  }
}