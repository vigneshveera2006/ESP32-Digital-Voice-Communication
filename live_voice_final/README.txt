LIVE WIRELESS VOICE SYSTEM - FINAL PROGRAMS
Arduino IDE, board package "esp32 by Espressif Systems" version 2.0.14, board "ESP32 Dev Module".
No extra libraries needed.

transmitter\  (transmitter.ino, live_protocol.h, live_dsp.h, live_codecs.h)                    -> TRANSMITTER board
receiver\     (receiver.ino, live_protocol.h, live_dsp.h, live_codecs.h, dashboard_page.h) -> RECEIVER board
Keep every .h file next to its .ino and keep the folder names.

USE
1. Power both boards. The receiver's blue LED lights while it plays audio.
2. Transmitter: S1/S2 choose the encoding (OFF OFF PCM, OFF ON DPCM, ON OFF DM, ON ON ADM). S3 ON = talk, S3 OFF = stop.
   Serial command (transmitter, while S3 OFF): o = DM/ADM oversampling 4x (32 kbit/s) <-> 1x (8 kbit/s).
3. Phone: join Wi-Fi "VoiceRX", password voice1234. The dashboard opens by itself, or open http://192.168.4.1
4. After S3 OFF the dashboard plays the reconstructed audio of that transmission and offers it as a WAV
   download (PCM up to 10.9 s, DPCM and 4x DM/ADM 19.2 s, 1x DM/ADM 45.5 s). A new S3 ON replaces it.
5. Receiver Serial Monitor (115200): + / - volume (default 8, try up to 12), t test tone,
   g noise gate on/off (mutes the hiss between words), r reset counters.
6. Out of range: the transmitter resets its radio after 1.5 s without delivery and reconnects by itself
   when back in range. No button needed.
