/*
 * esp32_crsf_wifi.ino
 * -------------------
 * CRSF-zu-WiFi-Bruecke fuer den Hexapod, fuer Seeed Studio XIAO ESP32-C6.
 *
 * Steckt am externen Modulschacht der RadioMaster TX16S (EdgeTX: externes
 * Modul = CRSF), liest die RC-Kanaele ueber UART und schickt walk/halt/pose/
 * set_gait-Befehle per WLAN an den Hexapod-Webserver (POST /command) --
 * dieselbe API, die auch das Webinterface nutzt.
 *
 * Portierung von tools/hexapod_crsf.py (gleiches Mapping + Failsafe).
 *
 * WLAN-Einrichtung: Beim ersten Start (oder BOOT-Taste gedrueckt halten)
 * oeffnet das Modul einen Hotspot "Hexapod-CRSF". Damit verbinden, im Browser
 * SSID/Passwort und die Robot-URL eingeben -- wird im NVS gespeichert.
 *
 * Benoetigte Library: "WiFiManager" von tzapu (Library Manager).
 * Board: "XIAO_ESP32C6" (ESP32 Arduino Core >= 3.0).
 */
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>      // tzapu/WiFiManager
#include <Preferences.h>

// ----------------------------- Hardware-Pins -----------------------------
// CRSF-Signal (vom Modulschacht) an diesen RX-Pin. CRSF ist nicht-invertiertes
// 3,3V-TTL -> direkt an einen GPIO. TX wird nicht gebraucht.
#define CRSF_RX_PIN   D7      // XIAO-Label D7 (GPIO17)
#define CRSF_TX_PIN   -1
#define CRSF_BAUD     400000  // wie hexapod_crsf.py; CRSF-Standard waere 420000
#define BOOT_BUTTON   9       // GPIO9 = BOOT-Taste (Config-Portal erzwingen)

HardwareSerial CRSF(1);       // UART1

// ----------------------------- Mapping -----------------------------------
// Kanal-Index 0-basiert (CH1..CH16). -1 = deaktiviert.  (wie hexapod_crsf.py)
const int CH_VX = 1;     // CH2 rechter Stick vert  -> vorwaerts/rueckwaerts
const int CH_VY = 0;     // CH1 rechter Stick hor   -> seitwaerts
const int CH_OMEGA = 3;  // CH4 linker Stick hor    -> drehen
const int CH_HEIGHT = 2; // CH3 linker Stick vert   -> Hoehe
const int CH_GAIT = -1;  // Schalter -> Gangart
const int CH_TZ = -1;    // Poti -> Koerperhoehe tz
const int CH_TX = -1;    // Slider -> tx
const int CH_TY = -1;    // Poti -> ty
const int CH_MODE = -1;  // Schalter -> Pose-Modus (Sticks -> roll/pitch/yaw)
const int CH_HALT = -1;  // Schalter -> Not-Halt

const bool INV_VX = false, INV_VY = true, INV_OMEGA = true, INV_HEIGHT = false;
const bool INV_TZ = false, INV_TX = false, INV_TY = false;
const float VX_MAX = 40, VY_MAX = 40, OMEGA_MAX = 30;
const float H_MIN = 15, H_MAX = 50;
const float TZ_MIN = -20, TZ_MAX = 40, TX_MIN = -30, TX_MAX = 30, TY_MIN = -30, TY_MAX = 30;
const float ROLL_MAX = 18, PITCH_MAX = 18, YAW_MAX = 18;
const char* GAITS[] = {"tripod", "tetrapod", "ripple", "wave"};
const int N_GAITS = 4;
const float DEAD = 0.06;
const float SEND_HZ = 15.0;

// ----------------------------- CRSF ---------------------------------------
static const uint8_t RC_CHANNELS_PACKED = 0x16;

uint8_t crc8(const uint8_t* data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
  }
  return crc;
}

// 11-bit-gepackte Kanaele entpacken (16 Stueck)
void unpackChannels(const uint8_t* p, int len, uint16_t* ch) {
  uint32_t bits = 0; int nbits = 0, idx = 0;
  for (int i = 0; i < len && idx < 16; i++) {
    bits |= (uint32_t)p[i] << nbits; nbits += 8;
    while (nbits >= 11 && idx < 16) {
      ch[idx++] = bits & 0x7FF; bits >>= 11; nbits -= 11;
    }
  }
}

// CRSF 172..1811, Mitte 992 -> -1..+1
float normCh(uint16_t v) {
  float r = (float)((int)v - 992) / 819.0f;
  return r < -1 ? -1 : (r > 1 ? 1 : r);
}

uint8_t rxbuf[64]; int rxlen = 0;
uint16_t channels[16]; bool haveChannels = false;
uint32_t lastFrameMs = 0;

void feedCrsf() {
  while (CRSF.available()) {
    if (rxlen < (int)sizeof(rxbuf)) rxbuf[rxlen++] = CRSF.read();
    else { memmove(rxbuf, rxbuf + 1, --rxlen); rxbuf[rxlen++] = CRSF.read(); }
    // Frames aus dem Puffer ziehen (Resync wie in der Python-Referenz)
    while (rxlen >= 2) {
      int length = rxbuf[1];
      if (length < 2 || length > 62) { memmove(rxbuf, rxbuf + 1, --rxlen); continue; }
      if (rxlen < length + 2) break;
      if (crc8(rxbuf + 2, length - 1) == rxbuf[length + 1]) {
        if (rxbuf[2] == RC_CHANNELS_PACKED) {
          unpackChannels(rxbuf + 3, length - 2, channels);
          haveChannels = true; lastFrameMs = millis();
        }
        memmove(rxbuf, rxbuf + length + 2, rxlen - (length + 2));
        rxlen -= (length + 2);
      } else {
        memmove(rxbuf, rxbuf + 1, --rxlen);
      }
    }
  }
}

// ----------------------------- Helpers ------------------------------------
float chn(int i) { return (i >= 0 && i < 16) ? normCh(channels[i]) : 0.0f; }
float applyv(float v, bool inv, float dz) {
  if (inv) v = -v;
  if (fabs(v) < dz) return 0.0f;
  float s = (fabs(v) - dz) / (1.0f - dz);
  return (v > 0 ? 1 : -1) * s;
}
float potv(int i, bool inv, float lo, float hi) {
  if (i < 0) return 0.0f;
  return lo + (applyv(chn(i), inv, 0.0f) + 1.0f) / 2.0f * (hi - lo);
}
bool swOn(int i) { return i >= 0 && chn(i) > 0.3f; }
float r2(float v) { return roundf(v * 2) / 2.0f; }

// ----------------------------- HTTP ---------------------------------------
Preferences prefs;
String robotUrl = "http://hexapod.local:8000";
WiFiClient wifiClient;

void sendCmd(const String& json) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.setConnectTimeout(300);
  http.setTimeout(300);
  if (http.begin(wifiClient, robotUrl + "/command")) {
    http.addHeader("Content-Type", "application/json");
    http.POST(json);
    http.end();
  }
}

// ----------------------------- Status-LED ---------------------------------
void led(bool on) { digitalWrite(LED_BUILTIN, on ? LOW : HIGH); } // XIAO: aktiv-LOW

// ----------------------------- WiFiManager --------------------------------
void setupWifi(bool forcePortal) {
  WiFiManager wm;
  prefs.begin("crsf", false);
  robotUrl = prefs.getString("url", robotUrl);
  WiFiManagerParameter pUrl("url", "Robot-URL", robotUrl.c_str(), 64);
  wm.addParameter(&pUrl);
  wm.setConfigPortalTimeout(180);
  bool ok;
  if (forcePortal) ok = wm.startConfigPortal("Hexapod-CRSF");
  else             ok = wm.autoConnect("Hexapod-CRSF");
  if (strlen(pUrl.getValue()) > 0) {
    robotUrl = pUrl.getValue();
    prefs.putString("url", robotUrl);
  }
  prefs.end();
  if (!ok) ESP.restart();
}

// ----------------------------- Setup/Loop ---------------------------------
bool walking = false; int gaitIdx = -1; String lastPose = "";
uint32_t lastSendMs = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); led(false);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  CRSF.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  bool force = (digitalRead(BOOT_BUTTON) == LOW); // BOOT gehalten -> Portal
  setupWifi(force);
  Serial.printf("WLAN ok: %s  ->  %s\n", WiFi.localIP().toString().c_str(), robotUrl.c_str());
}

void loop() {
  feedCrsf();
  uint32_t now = millis();

  // Failsafe: kein CRSF-Signal -> Halt
  if (walking && lastFrameMs && now - lastFrameMs > 500) {
    sendCmd("{\"action\":\"halt\"}"); walking = false;
  }

  // Status-LED: kein WLAN=schnell blinken, kein CRSF=langsam, beides ok=an
  bool link = haveChannels && (now - lastFrameMs < 500);
  if (WiFi.status() != WL_CONNECTED) led((now / 120) % 2);
  else if (!link)                    led((now / 500) % 2);
  else                               led(true);

  if (!haveChannels || now - lastSendMs < 1000.0 / SEND_HZ) return;
  lastSendMs = now;

  // --- Not-Halt-Schalter ---
  if (swOn(CH_HALT)) { sendCmd("{\"action\":\"halt\"}"); walking = false; }

  // --- Gangart-Schalter ---
  if (CH_GAIT >= 0) {
    float gv = chn(CH_GAIT);
    int gi = (int)roundf((gv + 1) / 2 * (N_GAITS - 1));
    gi = gi < 0 ? 0 : (gi >= N_GAITS ? N_GAITS - 1 : gi);
    if (gi != gaitIdx) {
      gaitIdx = gi;
      sendCmd(String("{\"action\":\"set_gait\",\"gait\":\"") + GAITS[gi] + "\"}");
    }
  }

  // --- Translationen (Regler) ---
  float tz = CH_TZ >= 0 ? potv(CH_TZ, INV_TZ, TZ_MIN, TZ_MAX) : 0;
  float tx = CH_TX >= 0 ? potv(CH_TX, INV_TX, TX_MIN, TX_MAX) : 0;
  float ty = CH_TY >= 0 ? potv(CH_TY, INV_TY, TY_MIN, TY_MAX) : 0;

  float roll = 0, pitch = 0, yaw = 0;
  if (swOn(CH_MODE)) {
    // Pose-Modus: Sticks -> Neigung
    roll = applyv(chn(CH_VY), INV_VY, DEAD) * ROLL_MAX;
    pitch = applyv(chn(CH_VX), INV_VX, DEAD) * PITCH_MAX;
    yaw = applyv(chn(CH_OMEGA), INV_OMEGA, DEAD) * YAW_MAX;
    if (walking) { sendCmd("{\"action\":\"halt\"}"); walking = false; }
  } else {
    // Bewegungs-Modus: Sticks -> walk
    float vx = applyv(chn(CH_VX), INV_VX, DEAD) * VX_MAX;
    float vy = applyv(chn(CH_VY), INV_VY, DEAD) * VY_MAX;
    float om = applyv(chn(CH_OMEGA), INV_OMEGA, DEAD) * OMEGA_MAX;
    float height = H_MIN + (applyv(chn(CH_HEIGHT), INV_HEIGHT, 0.0f) + 1) / 2 * (H_MAX - H_MIN);
    if (fabs(vx) > 0 || fabs(vy) > 0 || fabs(om) > 0) {
      char buf[160];
      snprintf(buf, sizeof(buf),
        "{\"action\":\"walk\",\"vx\":%.1f,\"vy\":%.1f,\"omega_deg\":%.1f,"
        "\"height\":%.1f,\"steps\":30,\"rate_hz\":40}",
        r2(vx), r2(vy), r2(om), r2(height));
      sendCmd(buf); walking = true;
    } else if (walking) {
      sendCmd("{\"action\":\"halt\"}"); walking = false;
    }
  }

  // --- Pose (6-DOF) nur bei Aenderung senden ---
  char key[96];
  snprintf(key, sizeof(key), "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
           r2(roll), r2(pitch), r2(yaw), r2(tx), r2(ty), r2(tz));
  if (lastPose != key) {
    char buf[200];
    snprintf(buf, sizeof(buf),
      "{\"action\":\"pose\",\"roll_deg\":%.1f,\"pitch_deg\":%.1f,\"yaw_deg\":%.1f,"
      "\"tx\":%.1f,\"ty\":%.1f,\"tz\":%.1f}",
      r2(roll), r2(pitch), r2(yaw), r2(tx), r2(ty), r2(tz));
    sendCmd(buf); lastPose = key;
  }
}
