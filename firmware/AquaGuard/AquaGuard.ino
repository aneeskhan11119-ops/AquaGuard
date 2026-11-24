/*
 * =====================================================================
 *  Automatic Water-Level Controller with Source-Empty Protection
 *  ESP32 firmware - budget build (ultrasonic + floats + ACS712 + flow
 *  sensor + MQTT phone app + Telegram alerts)
 * =====================================================================
 *  Motor 1 = borehole pump  -> source tank (sump)
 *  Motor 2 = roof pump      -> overhead (roof) tank
 *
 *  Build   : 12 V DC model - 2 x 12 V diaphragm pumps switched directly by a
 *            4-channel 5 V relay module, 12 V 5 A adapter, LM2596 12 V -> 5 V
 *  Sensors : 2 x JSN-SR04T ultrasonic (source tank, roof tank)
 *            1 x float switch (source tank EMPTY backup)
 *            2 x ACS712-5A current sensor (one per pump, DC)
 *            optional: roof FULL float, YF-S201 flow sensor (off by default)
 *  Outputs : relay IN1 = pump 1, IN2 = pump 2; optional LEDs and buzzer
 *  Network : Wi-Fi + built-in web page for the "Water Controller" phone app
 *            (optional: MQTT cloud, Telegram bot alerts)
 *
 *  The control logic runs in loop() and NEVER waits for Wi-Fi.
 *  All network code runs in a separate task on core 0.
 *
 *  Board     : "ESP32 Dev Module"  (Arduino-ESP32 core 2.0.x or 3.x)
 *  Libraries : PubSubClient (Nick O'Leary), ArduinoJson 7.x (Benoit Blanchon)
 *
 *  !!! Current thresholds and tank distances below are PLACEHOLDERS.
 *  !!! Calibrate them on your own pumps and tanks (Serial Monitor or app)
 *  !!! before leaving the system unattended.
 *  !!! Mains wiring must be checked by a qualified electrician.
 *
 *  ---------------------------------------------------------------
 *  PHONE APP / BROWSER (home Wi-Fi)
 *  ---------------------------------------------------------------
 *  The ESP32 serves the control page itself. In the app enter the address
 *  printed in the Serial Monitor (e.g. 192.168.1.50) or  watertank.local
 *     GET  /             control page
 *     GET  /api/status   status JSON (same format as the MQTT status below)
 *     GET  /api/config   settings + schedules JSON
 *     POST /api/cmd      JSON command (same commands as MQTT cmd below)
 *
 *  ---------------------------------------------------------------
 *  MQTT API (only if USE_MQTT is true)          base = wlc/<DEVICE_ID>
 *  ---------------------------------------------------------------
 *  wlc/wlc01/status  (device -> app, retained; on every change, every 5 s while a
 *                     motor runs, otherwise every 30 s - keeps EMQX free-tier traffic low)
 *     {"src":{"pct":63,"cm":102.5,"step":"HALF","valid":true,"float":true,"ok":true},
 *      "roof":{"pct":20,"cm":90.1,"step":"LOW","valid":true,"fullFloat":false},
 *      "flow":{"lpm":12.1,"litres":1234.5},
 *      "m1":{"on":true,"mode":"auto","manualOn":false,"state":"RUNNING","amps":4.2,
 *            "fails":0,"runSec":125,"restLeftSec":0,"lastRunSec":600,"lastRunAt":1760500000,"sched":false},
 *      "m2":{...same...},
 *      "alert":"last alert text","uptime":12345,"time":1760500000,"rssi":-61}
 *  wlc/wlc01/event   (device -> app) plain-text alert messages
 *  wlc/wlc01/avail   (device -> app, retained) "online" / "offline" (offline = LWT)
 *  wlc/wlc01/config  (device -> app, retained) {"params":{...},"schedules":[...]}
 *  wlc/wlc01/cmd     (app -> device) JSON commands:
 *     {"cmd":"on","motor":1}            switch motor ON  (puts it in manual)
 *     {"cmd":"off","motor":2}           switch motor OFF (puts it in manual)
 *     {"cmd":"mode","motor":1,"value":"auto"}   or "manual"
 *     {"cmd":"reset"}                   clear lockouts
 *     {"cmd":"set","values":{"roofStartPct":30,"m1DryA":2.2}}
 *     {"cmd":"schedules","list":[{"motor":2,"days":127,"on":"06:00","off":"07:00","en":true}]}
 *        days = bitmask, Sun=1 Mon=2 Tue=4 Wed=8 Thu=16 Fri=32 Sat=64 (127 = every day)
 *     {"cmd":"getConfig"}               re-publish config
 *     {"cmd":"status"}                  publish status now
 *     {"cmd":"defaults"}                restore default settings
 * =====================================================================
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>

// =====================================================================
// 1. YOUR ACCOUNT SETTINGS  (edit these)
// =====================================================================
#define WIFI_SSID      ""                       // optional: Wi-Fi typed in the code (normally use the phone setup page)
#define WIFI_PASS      ""
// Extra networks the ESP32 may use (it joins whichever one is available).
// Leave a name as "" to skip it.
#define WIFI2_SSID     ""
#define WIFI2_PASS     ""
#define WIFI3_SSID     ""
#define WIFI3_PASS     ""
#define WIFI4_SSID     ""
#define WIFI4_PASS     ""
#define WIFI5_SSID     ""
#define WIFI5_PASS     ""

// Wi-Fi setup mode: if no saved Wi-Fi connects within 1 minute, or you hold the BOOT
// button for 5 s, the ESP32 opens its own hotspot with a setup page (saves up to 3 networks).
#define PORTAL_SSID    "AquaGuard-Setup"
#define PORTAL_PASS    "aquaguard"
const uint32_t PORTAL_AUTO_MS = 60000;

#define MDNS_NAME      "watertank"     // app address: watertank.local

// Cloud connection (EMQX) used by the remote "Water Controller" app on any Wi-Fi.
#define USE_MQTT       true
#define MQTT_HOST      "YOUR_EMQX_ADDRESS.emqxsl.com"   // EMQX > Overview
#define MQTT_PORT      8883            // 8883 = TLS (EMQX / HiveMQ Cloud), 1883 = plain
#define MQTT_USE_TLS   true
#define MQTT_USER      "YOUR_EMQX_USERNAME"   // EMQX > Access Control > Authentication
#define MQTT_PASS      "YOUR_EMQX_PASSWORD"
#define DEVICE_ID      "wlc01"         // topics become wlc/wlc01/...

#define TG_ENABLED     false           // true = send alerts to Telegram (fill token + chat id)
#define TG_BOT_TOKEN   "123456789:AAxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define TG_CHAT_ID     "123456789"

#define TZ_POSIX       "PKT-5"         // Pakistan time (UTC+5) for schedules

// ---- What is fitted on this build ----
#define USE_CURRENT_SENSORS false  // true only if ACS712 current sensors are wired to pins 34 and 35
#define CURRENT_DC       true    // true = 12 V DC pumps (ACS712 reads DC), false = 220 V AC motors
#define USE_SRC_FLOAT    false   // true only if a source-tank float switch is wired to pin 32 (else the ultrasonic level is used)
#define USE_ROOF_FLOAT   false   // true only if a roof-tank FULL float is wired to pin 13
#define USE_FLOW_SENSOR  false   // true only if a YF-S201 flow sensor is wired to pin 4

// true = short timings for bench testing with bulbs (minutes become seconds,
// 10 s boot delay). Set back to false before connecting real motors!
#define BENCH_MODE     false

#define TOPIC_BASE  "wlc/" DEVICE_ID
#define T_STATUS    TOPIC_BASE "/status"
#define T_EVENT     TOPIC_BASE "/event"
#define T_CMD       TOPIC_BASE "/cmd"
#define T_CONFIG    TOPIC_BASE "/config"
#define T_AVAIL     TOPIC_BASE "/avail"

// =====================================================================
// 2. PIN MAP
// =====================================================================
const int PIN_SRC_TRIG   = 26;  // JSN-SR04T source tank TRIG
const int PIN_SRC_ECHO   = 27;  // ECHO via 1k/2k divider (5 V -> 3.3 V)
const int PIN_ROOF_TRIG  = 25;  // JSN-SR04T roof tank TRIG
const int PIN_ROOF_ECHO  = 33;  // ECHO via 1k/2k divider
const int PIN_FLOAT_SRC  = 32;  // source EMPTY float to GND: CLOSED = water present
const int PIN_FLOAT_ROOF = 13;  // roof FULL float to GND: CLOSED = not full, OPENS when full
const int PIN_CT1        = 34;  // ACS712 Motor 1, OUT via 10k/20k divider (ADC1 only)
const int PIN_CT2        = 35;  // ACS712 Motor 2, OUT via 10k/20k divider
const int PIN_FLOW       = 4;   // YF-S201 signal via 10k/20k divider
const int PIN_RELAY1     = 16;  // relay IN1 -> contactor 1 (Motor 1)
const int PIN_RELAY2     = 17;  // relay IN2 -> contactor 2 (Motor 2)
const int PIN_LED_M1     = 19;  // green: Motor 1 running
const int PIN_LED_M2     = 21;  // green: Motor 2 running
const int PIN_LED_FAULT  = 22;  // red: steady = lockout, blinking = warning
const int PIN_BUZZER     = 23;  // active buzzer via NPN transistor
const int PIN_RESET      = 14;  // push button to GND, hold 2 s to clear lockouts

#define RELAY_ON   LOW          // most opto relay modules are active-LOW
#define RELAY_OFF  HIGH         // swap both if your module is active-HIGH

// =====================================================================
// 3. FIXED SETTINGS
// =====================================================================
// ACS712 output (5 V supply): 2.5 V at 0 A. 5A model = 185 mV/A (20A model = 100 mV/A).
// The 10k/20k divider scales it by 2/3 -> the 5A model gives 123 mV/A at the ESP32 pin.
const float    ACS_MV_PER_A      = 123.3f; // CALIBRATE against a multimeter in series with the pump
const float    ACS_NOISE_A       = 0.08f;  // readings below this count as 0 A

const uint32_t BOOT_DELAY_MS     = BENCH_MODE ? 10000UL : 120000UL; // after power returns
const uint32_t STAGGER_MS        = BENCH_MODE ? 3000UL  : 15000UL;  // gap between motor starts
const uint32_t DRY_CONFIRM_MS    = 3000;   // current AND flow low this long -> dry trip
const uint32_t SINGLE_CONFIRM_MS = 20000;  // only ONE of them low this long -> dry trip
const uint32_t OC_CONFIRM_MS     = 2000;   // overcurrent must persist this long
const uint32_t FLOAT_DEBOUNCE_MS = 3000;   // float must be stable this long (ripples)
const uint32_t US_PERIOD_MS      = 600;    // one ultrasonic tank reading every 600 ms
const uint32_t US_INVALID_MS     = 10000;  // no valid echo this long -> sensor fault
const uint32_t STUCK_CONFIRM_MS  = 10000;  // current while OFF this long -> stuck contactor
const uint32_t DISAGREE_MS       = 60000;  // float vs ultrasonic disagreement
const uint32_t ALERT_REPEAT_MS   = 10UL * 60000UL; // same alert at most every 10 min
const float    TIME_SCALE        = BENCH_MODE ? (1.0f / 60.0f) : 1.0f;
const float    US_MIN_CM         = 20.0f;  // JSN-SR04T blind zone

// =====================================================================
// 4. TUNABLE SETTINGS (defaults; changeable from the app, saved in flash)
// =====================================================================
struct Cfg {
  uint32_t magic;
  // Tank geometry: distance from sensor face to water surface, in cm
  float srcEmptyCm, srcFullCm, roofEmptyCm, roofFullCm;
  // Level thresholds in percent
  float srcEmptyPct, srcOkPct, srcStartPct, srcStopPct;
  float roofStartPct, roofStopPct;
  // Current thresholds in amps (CALIBRATE)
  float m1DryA, m1OcA, m2DryA, m2OcA;
  // Flow sensor
  float useFlow, m1MinFlowLpm, flowPulsesPerL;
  // Timings
  float m1MaxRunMin, m2MaxRunMin, m1RestMin, m2RestMin;
  float m1ProdSec, m2ProdSec, m1MaxFails, m2MaxFails;
  float primeSec, minOffMin, m2NoRiseMin, m2NoRisePct;
};
const uint32_t CFG_MAGIC = 0x574C4302;   // changed: old saved settings are replaced by these defaults

// =====================================================================
// 5. TYPES AND GLOBALS
// =====================================================================
struct Sched { uint8_t motor; uint8_t days; uint8_t en; uint8_t pad; uint16_t onMin; uint16_t offMin; };
const int MAX_SCHED = 8;

enum MState : uint8_t { ST_IDLE, ST_RUNNING, ST_REST, ST_LOCKOUT };
enum Trip { TRIP_NONE, TRIP_DRY, TRIP_OC, TRIP_TIMEOUT, TRIP_NORISE };
enum AlertId {
  AL_NONE = -1, AL_BOOT = 0, AL_SRC_EMPTY, AL_SRC_FULL, AL_ROOF_FULL, AL_DRY1, AL_DRY2,
  AL_LOCK1, AL_LOCK2, AL_SENS_SRC, AL_SENS_ROOF, AL_DISAGREE_SRC, AL_DISAGREE_ROOF,
  AL_STUCK1, AL_STUCK2, AL_COUNT
};
enum CmdType : uint8_t { CMD_MODE, CMD_POWER, CMD_RESET };
struct Cmd { CmdType type; uint8_t motor; uint8_t val; };

struct Tank {
  int trig, echo;
  float hist[5]; uint8_t n, idx;
  float cm, pct; bool valid; uint32_t lastGood;
  int shownPct; bool shownValid;   // last value sent to the app
};

struct DIn { int pin; bool stable, lastRaw; uint32_t changed; };

struct Motor {
  uint8_t id; int relay, ct;
  MState state; bool autoMode, manualOn, on, resumeFill;
  uint8_t fails;
  uint32_t runStart, stopTime, restUntil, dryStart, singleStart, ocStart, lastMeasure;
  uint32_t wetMs, lastWetTick, riseRefTime, stuckSince;
  float riseRefPct, amps, runStartPct;
  uint32_t lastRunSec; time_t lastRunEpoch;
};

struct MotorSnap {
  bool on, autoMode, manualOn, sched; uint8_t state, fails; float amps;
  uint32_t runSec, restLeftSec, lastRunSec; time_t lastRunEpoch;
};
struct Snapshot {
  float srcPct, srcCm, roofPct, roofCm; bool srcValid, roofValid, srcFloatWater, roofFullFloat, sourceOk;
  float flowLpm; double litres; MotorSnap m[2]; char lastAlert[160]; uint32_t uptime;
};

struct MParams { float dryA, ocA, maxRunMs, restMs, prodMs, maxFails, noRiseMs, noRisePct; bool useFlow; };
struct Demand  { bool startOk; const char* stop; bool stopIsFull; const char* interlock; float targetPct; bool targetValid; };

// Tunable-settings table (kept after all type definitions so Arduino's
// automatic function prototypes always see every type they use)
struct Param { const char* key; size_t off; float mn, mx; };
#define PRM(k, mn, mx) { #k, offsetof(Cfg, k), mn, mx }
const Param PARAMS[] = {
  PRM(srcEmptyCm, 25, 600),   PRM(srcFullCm, 20, 400),
  PRM(roofEmptyCm, 25, 600),  PRM(roofFullCm, 20, 400),
  PRM(srcEmptyPct, 0, 50),    PRM(srcOkPct, 5, 80),
  PRM(srcStartPct, 5, 90),    PRM(srcStopPct, 50, 100),
  PRM(roofStartPct, 5, 90),   PRM(roofStopPct, 50, 100),
  PRM(m1DryA, 0, 20),         PRM(m1OcA, 1, 25),
  PRM(m2DryA, 0, 20),         PRM(m2OcA, 1, 25),
  PRM(useFlow, 0, 1),         PRM(m1MinFlowLpm, 0, 30),     PRM(flowPulsesPerL, 100, 2000),
  PRM(m1MaxRunMin, 1, 240),   PRM(m2MaxRunMin, 1, 240),
  PRM(m1RestMin, 1, 240),     PRM(m2RestMin, 1, 240),
  PRM(m1ProdSec, 10, 1800),   PRM(m2ProdSec, 10, 1800),
  PRM(m1MaxFails, 1, 20),     PRM(m2MaxFails, 1, 20),
  PRM(primeSec, 3, 120),      PRM(minOffMin, 0, 30),
  PRM(m2NoRiseMin, 0, 60),    PRM(m2NoRisePct, 0, 50),
};
const int N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

Cfg defaultCfg() {
  Cfg c;
  c.magic = CFG_MAGIC;
  c.srcEmptyCm   = 60;   c.srcFullCm  = 25;   // MEASURE your tanks (sensor face to water, cm)
  c.roofEmptyCm  = 60;   c.roofFullCm = 25;   // keep FULL >= 25 cm: the sensor can't see closer
  c.srcEmptyPct  = 10;   // Motor 2 stops at/below this (keep above its suction inlet)
  c.srcOkPct     = 30;   // Motor 2 allowed again only after source recovers to this
  c.srcStartPct  = 40;   // Motor 1 starts below this
  c.srcStopPct   = 95;   // Motor 1 stops at this
  c.roofStartPct = 30;   // Motor 2 starts below this
  c.roofStopPct  = 95;   // Motor 2 stops at this
  c.m1DryA = 0.30; c.m1OcA = 3.0;           // CALIBRATE (12 V mini pumps: about 0.5-1.5 A wet)
  c.m2DryA = 0.30; c.m2OcA = 3.0;           // CALIBRATE
  c.useFlow = USE_FLOW_SENSOR ? 1 : 0;   c.m1MinFlowLpm = 1.0;  c.flowPulsesPerL = 450;  // YF-S201: F = 7.5 x L/min
  c.m1MaxRunMin = 45;  c.m2MaxRunMin = 30;   // ~1.5 x normal fill time
  c.m1RestMin   = 30;  c.m2RestMin   = 10;   // borehole recovery / air-lock rest
  c.m1ProdSec   = 90;  c.m2ProdSec   = 60;   // run with water this long = good attempt
  c.m1MaxFails  = 4;   c.m2MaxFails  = 3;    // useless attempts in a row -> lockout
  c.primeSec    = 10;  // ignore current/flow for this long after start (inrush + priming)
  c.minOffMin   = 3;   // minimum off time per motor (short-cycle protection)
  c.m2NoRiseMin = 6;   // roof level must rise...
  c.m2NoRisePct = 2;   // ...at least this many % per window, else air lock / leak (0 = off)
  return c;
}

inline float& cfgRef(Cfg& c, const Param& p) {
  return *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(&c) + p.off);
}
inline float cfgGet(const Cfg& c, const Param& p) {
  return *reinterpret_cast<const float*>(reinterpret_cast<const uint8_t*>(&c) + p.off);
}

// Shared between control loop (core 1) and network task (core 0); guarded by gMux
Cfg gCfg;
Sched gSched[MAX_SCHED];
uint8_t gSchedCount = 0;
Snapshot gSnap;
volatile bool gSnapDirty = false;
volatile bool gAnyMotorOn = false;
SemaphoreHandle_t gMux;
QueueHandle_t cmdQ, alertQ;
Preferences prefs;

// Control-loop state
Tank srcTank  = { PIN_SRC_TRIG,  PIN_SRC_ECHO  };
Tank roofTank = { PIN_ROOF_TRIG, PIN_ROOF_ECHO };
DIn fSrc  = { PIN_FLOAT_SRC };
DIn fRoof = { PIN_FLOAT_ROOF };
Motor M1, M2;
bool srcFloatWater = false, roofFullFloat = false, sourceOk = false;
bool schedOn[3] = { false, false, false };
volatile uint32_t flowPulses = 0;
portMUX_TYPE flowMux = portMUX_INITIALIZER_UNLOCKED;
float flowLpm = 0; double litresTotal = 0;
uint32_t bootTime = 0, lastAnyStart = 0, beepUntil = 0;
uint32_t alertLast[AL_COUNT];
char lastAlertMsg[160] = "";

// Network objects (used only by the network task)
WiFiClientSecure mqttSecure;
WiFiClient mqttPlain;
WiFiClientSecure tgClient;
PubSubClient mqtt;
WebServer web(80);
DNSServer dnsServer;
volatile bool gPortalOn = false, gPortalRequest = false, gTryNewNet = false, gJoining = false;
bool gWebStarted = false;
uint32_t gPortalSince = 0;
const int PIN_BOOT_BTN = 0;      // the BOOT button on the ESP32 board: hold 5 s = Wi-Fi setup mode

// Wi-Fi setup mode functions (declared here so the Arduino IDE always finds them)
String netLabel(int k);
void loadSavedNetworks();
void saveNetwork(const String& ssid, const String& pass);
void forgetSavedNetworks();
bool anyNetworkConfigured();
void servicePortal();
bool joinKnownWifi();
void startPortal();
void stopPortal();
String htmlEsc(const String& in);
void sendPortalPage();
void handleWifiSave();
void handleWifiClear();
bool portalClient();
void redirectToPortal();
void startWebOnce();
void updateSetupButton();

// =====================================================================
// 6. SMALL HELPERS
// =====================================================================
void lockShared()   { xSemaphoreTake(gMux, portMAX_DELAY); }
void unlockShared() { xSemaphoreGive(gMux); }

void beep(uint32_t ms) { beepUntil = millis() + ms; }

// Non-zero timestamp (0 means "not started"). Never ahead of 'now', so 'now - t' cannot underflow.
inline uint32_t nz(uint32_t t) { return t ? t : 1; }

// Sends an alert to Serial, MQTT and Telegram. Same id is repeated at most every 10 min.
void alert(int id, const char* fmt, ...) {
  uint32_t now = millis();
  if (id >= 0) {
    if (alertLast[id] && (now - alertLast[id]) < ALERT_REPEAT_MS) return;
    alertLast[id] = nz(now);
  }
  char buf[160];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.printf("ALERT: %s\n", buf);
  strlcpy(lastAlertMsg, buf, sizeof(lastAlertMsg));
  xQueueSend(alertQ, buf, 0);   // dropped if queue full; never blocks the control loop
  gSnapDirty = true;
  beep(200);
}

const char* stateName(uint8_t s) {
  static const char* n[] = { "IDLE", "RUNNING", "REST", "LOCKOUT" };
  return s < 4 ? n[s] : "?";
}
const char* stepName(float pct, bool valid) {
  if (!valid) return "UNKNOWN";
  if (pct < 15) return "EMPTY";
  if (pct < 40) return "LOW";
  if (pct < 85) return "HALF";
  return "FULL";
}
const char* tripText(Trip t) {
  switch (t) {
    case TRIP_DRY:     return "not sucking water (dry run)";
    case TRIP_OC:      return "overcurrent / jammed";
    case TRIP_TIMEOUT: return "max run time reached";
    case TRIP_NORISE:  return "roof level not rising (air lock / leak)";
    default:           return "";
  }
}
float round1(float x) { return roundf(x * 10.0f) / 10.0f; }
const char* tripTextFor(uint8_t id, Trip t) {
  if (id == 1 && t == TRIP_NORISE) return "borehole dry (source tank not filling)";
  return tripText(t);
}

// =====================================================================
// 7. SENSORS
// =====================================================================
void tankBegin(Tank& t) {
  pinMode(t.trig, OUTPUT); digitalWrite(t.trig, LOW);
  pinMode(t.echo, INPUT);
  t.n = t.idx = 0; t.cm = -1; t.pct = -1; t.valid = false; t.lastGood = 0;
}

float pingCm(const Tank& t) {
  digitalWrite(t.trig, LOW);  delayMicroseconds(4);
  digitalWrite(t.trig, HIGH); delayMicroseconds(20);
  digitalWrite(t.trig, LOW);
  unsigned long us = pulseIn(t.echo, HIGH, 40000UL);
  if (!us) return -1;
  return us * 0.0343f / 2.0f;
}

void sortSmall(float* a, int n) {
  for (int i = 1; i < n; i++) { float k = a[i]; int j = i - 1; while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; } a[j + 1] = k; }
}

// 3 pings -> median, then median of the last 5 results (kills ripples and odd echoes)
void tankRead(Tank& t, float emptyCm, float fullCm) {
  float r[3]; int ok = 0;
  for (int i = 0; i < 3; i++) {
    float d = pingCm(t);
    if (d >= US_MIN_CM && d <= emptyCm + 50) r[ok++] = d;
    delay(50);
  }
  uint32_t now = millis();
  if (ok > 0) {
    sortSmall(r, ok);
    float m = r[ok / 2];
    t.hist[t.idx] = m; t.idx = (t.idx + 1) % 5; if (t.n < 5) t.n++;
    float s[5]; for (int i = 0; i < t.n; i++) s[i] = t.hist[i];
    sortSmall(s, t.n);
    t.cm = s[t.n / 2];
    t.lastGood = now; t.valid = true;
    float span = emptyCm - fullCm; if (span < 5) span = 5;
    t.pct = constrain((emptyCm - t.cm) / span * 100.0f, 0.0f, 100.0f);
    int ip = (int)lroundf(t.pct);
    if (ip != t.shownPct || !t.shownValid) { t.shownPct = ip; t.shownValid = true; gSnapDirty = true; }  // level changed: update the app now
  } else if (now - t.lastGood > US_INVALID_MS) {
    t.valid = false;
    if (t.shownValid) { t.shownValid = false; gSnapDirty = true; }
  }
}

void dinBegin(DIn& d) {
  pinMode(d.pin, INPUT_PULLUP); delay(5);
  d.lastRaw = d.stable = digitalRead(d.pin); d.changed = millis();
}
void dinUpdate(DIn& d) {
  bool r = digitalRead(d.pin);
  if (r != d.lastRaw) { d.lastRaw = r; d.changed = millis(); }
  if (r != d.stable && (millis() - d.changed) >= FLOAT_DEBOUNCE_MS) d.stable = r;
}

// Zero-current voltage of each ACS712, measured at boot while all pumps are off (DC mode)
float ctZeroMv[40];

float readMeanMv(int pin, uint32_t us) {
  double sum = 0; uint32_t n = 0; uint32_t t0 = micros();
  while ((micros() - t0) < us) { sum += analogReadMilliVolts(pin); n++; }
  return n ? (float)(sum / n) : 0;
}

void calibrateCurrentZero() {
  if (!USE_CURRENT_SENSORS) { Serial.println("Current sensors: not fitted (dry-run detection uses tank level rise)"); return; }
  ctZeroMv[PIN_CT1] = readMeanMv(PIN_CT1, 400000UL);
  ctZeroMv[PIN_CT2] = readMeanMv(PIN_CT2, 400000UL);
  Serial.printf("Current sensor zero: CT1 %.0f mV, CT2 %.0f mV (expect about 1667 mV)\n", ctZeroMv[PIN_CT1], ctZeroMv[PIN_CT2]);
}

// Pump current in amps. DC: average minus zero (direction ignored). AC: RMS with offset removed.
// A disconnected sensor reads 0 A, which counts as dry (fail-safe).
float readAmps(int pin) {
  if (!USE_CURRENT_SENSORS) return 0;                      // no current sensors fitted: never read the floating pins
  if (CURRENT_DC) {
    float mv = readMeanMv(pin, 100000UL) - ctZeroMv[pin];
    if (ctZeroMv[pin] < 300) mv = 0;                       // sensor missing at boot: treat as 0 A
    float a = fabsf(mv) / ACS_MV_PER_A;
    return a < ACS_NOISE_A ? 0 : a;
  }
  double sum = 0, sq = 0; uint32_t n = 0;
  uint32_t t0 = micros();
  while ((micros() - t0) < 100000UL) {          // 100 ms = 5 mains cycles at 50 Hz
    double v = analogReadMilliVolts(pin);
    sum += v; sq += v * v; n++;
  }
  if (!n) return 0;
  double mean = sum / n, var = sq / n - mean * mean;
  if (var < 0) var = 0;
  float a = (float)sqrt(var) / ACS_MV_PER_A;
  return a < ACS_NOISE_A ? 0 : a;
}

void IRAM_ATTR onFlowPulse() {
  portENTER_CRITICAL_ISR(&flowMux); flowPulses++; portEXIT_CRITICAL_ISR(&flowMux);
}
void updateFlow(const Cfg& c) {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 1000) return;
  uint32_t dt = now - last; last = now;
  portENTER_CRITICAL(&flowMux); uint32_t p = flowPulses; flowPulses = 0; portEXIT_CRITICAL(&flowMux);
  float litres = p / c.flowPulsesPerL;
  flowLpm = litres * 60000.0f / dt;
  litresTotal += litres;
}

void updateSensors(const Cfg& c) {
  dinUpdate(fSrc); dinUpdate(fRoof);
  srcFloatWater = USE_SRC_FLOAT ? (fSrc.stable == LOW) : true;    // wire break -> "empty" (fail-safe)
  roofFullFloat = USE_ROOF_FLOAT && (fRoof.stable == HIGH);  // wire break -> "full" (fail-safe)

  static uint32_t lastUs = 0; static bool which = false;
  if (millis() - lastUs >= US_PERIOD_MS) {
    lastUs = millis(); which = !which;
    if (which) tankRead(srcTank, c.srcEmptyCm, c.srcFullCm);
    else       tankRead(roofTank, c.roofEmptyCm, c.roofFullCm);
  }
  updateFlow(c);
}

// Source-empty hysteresis for Motor 2: off at srcEmptyPct (or float), on again at srcOkPct
void updateSourceOk(const Cfg& c) {
  bool hardEmpty = !srcFloatWater || (srcTank.valid && srcTank.pct <= c.srcEmptyPct);
  if (hardEmpty) sourceOk = false;
  else if (srcTank.valid && srcTank.pct >= c.srcOkPct) sourceOk = true;
}

void updateSchedules() {
  static uint32_t last = 0;
  if (millis() - last < 5000) return;
  last = millis();
  schedOn[1] = schedOn[2] = false;
  time_t now = time(nullptr);
  if (now < 1700000000) return;              // clock not set yet (no NTP since boot)
  struct tm t; localtime_r(&now, &t);
  int cur = t.tm_hour * 60 + t.tm_min;
  int today = t.tm_wday, yday = (t.tm_wday + 6) % 7;
  lockShared();
  for (int i = 0; i < gSchedCount; i++) {
    const Sched& s = gSched[i];
    if (!s.en || s.motor < 1 || s.motor > 2) continue;
    bool in;
    if (s.onMin <= s.offMin) in = ((s.days >> today) & 1) && cur >= s.onMin && cur < s.offMin;
    else in = (((s.days >> today) & 1) && cur >= s.onMin) || (((s.days >> yday) & 1) && cur < s.offMin);
    if (in) schedOn[s.motor] = true;
  }
  unlockShared();
}

// =====================================================================
// 8. MOTOR CONTROL
// =====================================================================
MParams paramsFor(const Motor& m, const Cfg& c) {
  MParams p;
  if (m.id == 1) {
    p.dryA = c.m1DryA; p.ocA = c.m1OcA;
    p.maxRunMs = c.m1MaxRunMin * 60000.0f * TIME_SCALE;
    p.restMs   = c.m1RestMin   * 60000.0f * TIME_SCALE;
    p.prodMs   = c.m1ProdSec * 1000.0f; p.maxFails = c.m1MaxFails;
    // Borehole dry check: the source tank must keep rising while Motor 1 runs
    p.noRiseMs = c.m2NoRiseMin * 60000.0f * TIME_SCALE; p.noRisePct = c.m2NoRisePct;
    p.useFlow = c.useFlow >= 0.5f;
  } else {
    p.dryA = c.m2DryA; p.ocA = c.m2OcA;
    p.maxRunMs = c.m2MaxRunMin * 60000.0f * TIME_SCALE;
    p.restMs   = c.m2RestMin   * 60000.0f * TIME_SCALE;
    p.prodMs   = c.m2ProdSec * 1000.0f; p.maxFails = c.m2MaxFails;
    p.noRiseMs = c.m2NoRiseMin * 60000.0f * TIME_SCALE; p.noRisePct = c.m2NoRisePct;
    p.useFlow = false;                       // flow sensor is on the bore line only
  }
  return p;
}

uint32_t minOffMs(const Cfg& c) { return (uint32_t)(c.minOffMin * 60000.0f * TIME_SCALE); }
bool minOffOk(const Motor& m, const Cfg& c) { return (millis() - m.stopTime) >= minOffMs(c); }
bool staggerOk()   { return (millis() - lastAnyStart) >= STAGGER_MS; }
bool bootDelayOk() { return (millis() - bootTime) >= BOOT_DELAY_MS; }

void saveLock(const Motor& m, bool v) { lockShared(); prefs.putBool(m.id == 1 ? "l1" : "l2", v); unlockShared(); }
void saveMode(const Motor& m)         { lockShared(); prefs.putBool(m.id == 1 ? "a1" : "a2", m.autoMode); unlockShared(); }

void startMotor(Motor& m) {
  digitalWrite(m.relay, RELAY_ON);
  m.on = true; m.state = ST_RUNNING;
  m.runStart = millis();
  m.dryStart = m.singleStart = m.ocStart = 0;
  m.wetMs = 0; m.lastWetTick = millis();
  m.riseRefTime = 0; m.riseRefPct = 0;
  m.runStartPct = (m.id == 1) ? srcTank.pct : roofTank.pct;
  m.lastMeasure = millis() - 500; m.amps = 0;
  lastAnyStart = millis();
  gSnapDirty = true;
  Serial.printf("[M%u] START\n", m.id);
  if (m.autoMode && !m.resumeFill) {      // tell the user an automatic start happened (not on every retry)
    if (m.id == 1)
      alert(AL_NONE, "Borehole pump (Motor 1) switched ON automatically - source tank at %.0f%%", srcTank.pct);
    else
      alert(AL_NONE, "Roof pump (Motor 2) switched ON automatically - roof tank at %.0f%%", roofTank.pct);
  }
}

void stopMotor(Motor& m) {
  digitalWrite(m.relay, RELAY_OFF);
  if (m.on) {
    m.stopTime = millis();
    m.lastRunSec = (m.stopTime - m.runStart) / 1000;
    time_t now = time(nullptr);
    m.lastRunEpoch = (now > 1700000000) ? now : 0;
    gSnapDirty = true;
    Serial.printf("[M%u] STOP after %lu s\n", m.id, (unsigned long)m.lastRunSec);
  }
  m.on = false;
}

void enterRest(Motor& m, uint32_t restMs) {
  stopMotor(m);
  m.state = ST_REST;
  m.restUntil = millis() + restMs;
}

void enterLockout(Motor& m) {
  stopMotor(m);
  m.state = ST_LOCKOUT; m.manualOn = false;
  saveLock(m, true);
}

// Checks a running motor. Returns a trip reason, or TRIP_NONE.
Trip checkRunning(Motor& m, const Cfg& c, const MParams& p, float targetPct, bool targetValid) {
  if (millis() - m.lastMeasure >= 500) { m.amps = readAmps(m.ct); m.lastMeasure = millis(); }
  uint32_t now = millis();
  uint32_t ran = now - m.runStart;
  if (ran < (uint32_t)(c.primeSec * 1000.0f)) { m.lastWetTick = now; return TRIP_NONE; }  // inrush + priming

  if (m.amps > p.ocA) {
    if (!m.ocStart) m.ocStart = nz(now);
    if (now - m.ocStart >= OC_CONFIRM_MS) return TRIP_OC;
  } else m.ocStart = 0;

  bool curLow  = USE_CURRENT_SENSORS && m.amps < p.dryA;                              // also catches an unplugged ACS712
  bool flowLow = p.useFlow && flowLpm < c.m1MinFlowLpm;
  bool bothLow = p.useFlow ? (curLow && flowLow) : curLow;
  bool anyLow  = curLow || flowLow;

  if (!anyLow) m.wetMs += now - m.lastWetTick;                // time actually pumping water
  m.lastWetTick = now;

  if (bothLow) {                                               // both agree: fast trip
    if (!m.dryStart) m.dryStart = nz(now);
    if (now - m.dryStart >= DRY_CONFIRM_MS) return TRIP_DRY;
  } else m.dryStart = 0;

  if (p.useFlow && anyLow) {                                   // only one says dry: slow trip
    if (!m.singleStart) m.singleStart = nz(now);
    if (now - m.singleStart >= SINGLE_CONFIRM_MS) return TRIP_DRY;
  } else m.singleStart = 0;

  if (ran >= (uint32_t)p.maxRunMs) return TRIP_TIMEOUT;

  if (m.id == 1 && M2.on) m.riseRefTime = 0;                  // roof pump is draining the source: restart the window
  if (p.noRiseMs > 0 && targetValid) {                         // tank level must keep rising
    if (!m.riseRefTime) { m.riseRefTime = nz(now); m.riseRefPct = targetPct; }
    else if (now - m.riseRefTime >= (uint32_t)p.noRiseMs) {
      if (targetPct - m.riseRefPct < p.noRisePct) return TRIP_NORISE;
      m.riseRefTime = nz(now); m.riseRefPct = targetPct;
    }
  }
  return TRIP_NONE;
}

// What the tank levels ask Motor 1 (borehole -> source tank) to do
Demand demandM1(const Cfg& c) {
  Demand d = { false, nullptr, false, nullptr, srcTank.pct, srcTank.valid };
  float startAt = (schedOn[1] || M1.resumeFill) ? c.srcStopPct - 5 : c.srcStartPct;
  d.startOk = srcTank.valid && srcTank.pct < startAt;
  if (srcTank.valid && srcTank.pct >= c.srcStopPct) { d.stop = "source tank full"; d.stopIsFull = true; }
  else if (!srcTank.valid && M1.autoMode) d.stop = "source level sensor not responding";
  return d;
}

// What the tank levels ask Motor 2 (source -> roof tank) to do
Demand demandM2(const Cfg& c) {
  Demand d = { false, nullptr, false, nullptr, roofTank.pct, roofTank.valid };
  float startAt = (schedOn[2] || M2.resumeFill) ? c.roofStopPct - 5 : c.roofStartPct;
  d.startOk = roofTank.valid && srcTank.valid && !roofFullFloat && roofTank.pct < startAt;
  if (roofFullFloat) { d.stop = "roof tank full (backup float)"; d.stopIsFull = true; }
  else if (roofTank.valid && roofTank.pct >= c.roofStopPct) { d.stop = "roof tank full"; d.stopIsFull = true; }
  else if (M2.autoMode && (!roofTank.valid || !srcTank.valid)) d.stop = "level sensor not responding";

  // SOURCE-EMPTY PROTECTION: active in AUTO and MANUAL, cannot be bypassed from the app
  bool hardEmpty = !srcFloatWater || (srcTank.valid && srcTank.pct <= c.srcEmptyPct);
  if (M2.autoMode) { if (!sourceOk) d.interlock = "source tank empty - waiting for refill"; }
  else if (hardEmpty) d.interlock = "source tank empty";
  return d;
}

void updateMotor(Motor& m, const Cfg& c, const Demand& d) {
  MParams p = paramsFor(m, c);
  uint32_t now = millis();

  switch (m.state) {
    case ST_LOCKOUT:
      stopMotor(m); m.manualOn = false;
      break;

    case ST_REST:
      stopMotor(m);
      if (!m.autoMode) m.state = ST_IDLE;                         // user took manual control
      else if ((int32_t)(now - m.restUntil) >= 0) m.state = ST_IDLE;
      break;

    case ST_IDLE: {
      digitalWrite(m.relay, RELAY_OFF);   // keep the relay off while idle
      bool want = false;
      if (m.autoMode) {
        if (m.id == 2 && d.startOk && d.interlock)
          alert(AL_SRC_EMPTY, "Source tank empty - Motor 2 locked until the source refills");
        want = d.startOk && !d.interlock && bootDelayOk();
      } else if (m.manualOn) {
        const char* why = d.interlock ? d.interlock : (d.stopIsFull ? d.stop : nullptr);
        if (why) { m.manualOn = false; alert(AL_NONE, "Motor %u: manual ON refused - %s", m.id, why); }
        want = m.manualOn;
      }
      if (want && minOffOk(m, c) && staggerOk()) startMotor(m);
      break;
    }

    case ST_RUNNING: {
      if (m.autoMode) {
        if (d.stop) {
          stopMotor(m); m.state = ST_IDLE;
          if (d.stopIsFull) {
            m.fails = 0; m.resumeFill = false;
            alert(m.id == 1 ? AL_SRC_FULL : AL_ROOF_FULL, "Motor %u stopped: %s", m.id, d.stop);
          } else alert(m.id == 1 ? AL_SENS_SRC : AL_SENS_ROOF, "Motor %u stopped: %s", m.id, d.stop);
          break;
        }
        if (d.interlock) {                                        // source empty: not a fault
          stopMotor(m); m.state = ST_IDLE;
          alert(AL_SRC_EMPTY, "Motor %u stopped: %s", m.id, d.interlock);
          break;
        }
      } else {
        if (!m.manualOn) { stopMotor(m); m.state = ST_IDLE; break; }
        if (d.stopIsFull || d.interlock) {
          stopMotor(m); m.state = ST_IDLE; m.manualOn = false;
          alert(AL_NONE, "Motor %u (manual) stopped: %s", m.id, d.interlock ? d.interlock : d.stop);
          break;
        }
      }

      Trip t = checkRunning(m, c, p, d.targetPct, d.targetValid);
      if (t == TRIP_NONE) break;

      if (t == TRIP_OC) {
        enterLockout(m);
        alert(m.id == 1 ? AL_LOCK1 : AL_LOCK2, "Motor %u LOCKED OUT: overcurrent / jammed motor. Check the pump, then Reset.", m.id);
        break;
      }
      if (!m.autoMode) {                                          // manual: just stop and report
        stopMotor(m); m.state = ST_IDLE; m.manualOn = false;
        alert(AL_NONE, "Motor %u (manual) stopped: %s", m.id, tripTextFor(m.id, t));
        break;
      }
      // Motor 2 dry while the source is nearly empty = source ran out, not a pump fault
      if (m.id == 2 && t == TRIP_DRY && (!srcFloatWater || (srcTank.valid && srcTank.pct < c.srcOkPct))) {
        sourceOk = false; stopMotor(m); m.state = ST_IDLE;
        alert(AL_SRC_EMPTY, "Motor 2 stopped: source tank nearly empty - waiting for refill");
        break;
      }
      // Low-yield logic: a dry trip after a productive run is normal -> rest and continue
      bool productive;
      if (m.id == 1 && t == TRIP_NORISE)      // borehole gave water earlier in this run, then dried up
        productive = d.targetValid && (d.targetPct - m.runStartPct) >= p.noRisePct;
      else
        productive = (t == TRIP_DRY) && m.wetMs >= (uint32_t)p.prodMs;
      if (productive) m.fails = 0; else m.fails++;
      m.resumeFill = true;
      if (m.fails >= (uint8_t)p.maxFails) {
        enterLockout(m);
        alert(m.id == 1 ? AL_LOCK1 : AL_LOCK2, "Motor %u LOCKED OUT after %u failed attempts (%s). Check it, then press Reset.",
              m.id, m.fails, tripTextFor(m.id, t));
      } else {
        enterRest(m, (uint32_t)p.restMs);
        unsigned long restMin = (unsigned long)(p.restMs / 60000.0f + 0.5f);
        if (productive && m.id == 1 && t == TRIP_NORISE)
          alert(AL_DRY1, "Borehole ran dry: pump stopped - resting %lu min, then trying again", restMin);
        else if (productive)
          alert(m.id == 1 ? AL_DRY1 : AL_DRY2, "Motor %u: water stopped after %lu s of pumping - resting %lu min, then continuing",
                m.id, (unsigned long)(m.wetMs / 1000), restMin);
        else
          alert(m.id == 1 ? AL_DRY1 : AL_DRY2, "Motor %u %s (attempt %u of %u) - retry in %lu min",
                m.id, tripTextFor(m.id, t), m.fails, (unsigned)p.maxFails, restMin);
      }
      break;
    }
  }
}

// Current while the pump is OFF means a stuck relay or a wiring bypass
void checkIdleCurrent(Motor& m, const Cfg& c) {
  static uint32_t lastRead[3] = { 0, 0, 0 };
  if (m.on) { m.stuckSince = 0; return; }
  if (millis() - m.stopTime < 5000 || millis() - lastRead[m.id] < 3000) return;
  lastRead[m.id] = millis();
  m.amps = readAmps(m.ct);
  float th = fmaxf(1.0f, (m.id == 1 ? c.m1DryA : c.m2DryA) * 0.5f);
  if (m.amps >= th) {
    if (!m.stuckSince) m.stuckSince = nz(millis());
    else if (millis() - m.stuckSince >= STUCK_CONFIRM_MS)
      alert(m.id == 1 ? AL_STUCK1 : AL_STUCK2, "WARNING: Motor %u draws %.1f A while switched OFF - relay may be stuck!", m.id, m.amps);
  } else m.stuckSince = 0;
}

void checkSensorAlerts(const Cfg& c) {
  if (millis() - bootTime < 15000) return;
  if (!srcTank.valid)  alert(AL_SENS_SRC,  "Source tank level sensor not responding - automatic pumping paused");
  if (!roofTank.valid) alert(AL_SENS_ROOF, "Roof tank level sensor not responding - Motor 2 auto paused");

  static uint32_t dsSrc = 0, dsRoof = 0;
  bool badSrc  = !srcFloatWater && srcTank.valid && srcTank.pct > c.srcOkPct + 15;
  bool badRoof = roofFullFloat && roofTank.valid && roofTank.pct < 70;
  if (badSrc)  { if (!dsSrc)  dsSrc  = nz(millis()); else if (millis() - dsSrc  > DISAGREE_MS)
                   alert(AL_DISAGREE_SRC, "Source float says EMPTY but ultrasonic says %.0f%% - check float/sensor", srcTank.pct); }
  else dsSrc = 0;
  if (badRoof) { if (!dsRoof) dsRoof = nz(millis()); else if (millis() - dsRoof > DISAGREE_MS)
                   alert(AL_DISAGREE_ROOF, "Roof float says FULL but ultrasonic says %.0f%% - check float/sensor", roofTank.pct); }
  else dsRoof = 0;
}

void clearLockouts() {
  Motor* ms[2] = { &M1, &M2 };
  for (int i = 0; i < 2; i++) {
    Motor& m = *ms[i];
    if (m.state == ST_LOCKOUT || m.state == ST_REST) m.state = ST_IDLE;
    m.fails = 0; m.resumeFill = false;
    saveLock(m, false);
  }
  alert(AL_NONE, "Lockouts cleared - system back to normal");
}

void processCommands() {
  Cmd c;
  while (xQueueReceive(cmdQ, &c, 0) == pdTRUE) {
    if (c.type == CMD_RESET) { clearLockouts(); continue; }
    Motor& m = (c.motor == 1) ? M1 : M2;
    if (c.type == CMD_MODE) {
      bool toAuto = c.val;
      if (toAuto != m.autoMode) {
        m.autoMode = toAuto;
        m.manualOn = toAuto ? false : m.on;   // switching to manual keeps the current on/off state
        saveMode(m);
        Serial.printf("[M%u] mode -> %s\n", m.id, toAuto ? "AUTO" : "MANUAL");
      }
    } else if (c.type == CMD_POWER) {
      if (m.state == ST_LOCKOUT && c.val) { alert(AL_NONE, "Motor %u is LOCKED OUT - press Reset first", m.id); continue; }
      if (m.autoMode) { m.autoMode = false; saveMode(m); }
      m.manualOn = c.val;
      Serial.printf("[M%u] manual %s\n", m.id, c.val ? "ON" : "OFF");
    }
    gSnapDirty = true;
  }
}

void updateResetButton() {
  static uint32_t downSince = 0; static bool handled = false;
  if (digitalRead(PIN_RESET) == LOW) {
    if (!downSince) downSince = nz(millis());
    if (!handled && millis() - downSince >= 2000) { handled = true; clearLockouts(); }
  } else { downSince = 0; handled = false; }
}

void updateSetupButton() {
  static uint32_t downSince = 0; static bool fired = false;
  if (digitalRead(PIN_BOOT_BTN) == LOW) {
    if (!downSince) downSince = nz(millis());
    if (!fired && millis() - downSince >= 5000) { fired = true; gPortalRequest = true; beep(300); }
  } else { downSince = 0; fired = false; }
}

void updateIndicators() {
  digitalWrite(PIN_LED_M1, M1.on ? HIGH : LOW);
  digitalWrite(PIN_LED_M2, M2.on ? HIGH : LOW);
  bool lockout   = M1.state == ST_LOCKOUT || M2.state == ST_LOCKOUT;
  bool sensorBad = (millis() - bootTime > 15000) && (!srcTank.valid || !roofTank.valid);
  bool warn      = sensorBad || M1.state == ST_REST || M2.state == ST_REST || !sourceOk;
  bool blink     = (millis() / 500) % 2;
  digitalWrite(PIN_LED_FAULT, lockout ? HIGH : (warn && blink ? HIGH : LOW));
  bool buzz = (int32_t)(beepUntil - millis()) > 0;
  if (lockout || sensorBad) buzz = buzz || (millis() % 5000) < 150;   // short beep every 5 s
  digitalWrite(PIN_BUZZER, buzz ? HIGH : LOW);
}

void fillMotorSnap(MotorSnap& s, const Motor& m) {
  s.on = m.on; s.autoMode = m.autoMode; s.manualOn = m.manualOn; s.sched = schedOn[m.id];
  s.state = m.state; s.fails = m.fails; s.amps = m.amps;
  s.runSec = m.on ? (millis() - m.runStart) / 1000 : 0;
  int32_t left = (int32_t)(m.restUntil - millis());
  s.restLeftSec = (m.state == ST_REST && left > 0) ? left / 1000 : 0;
  s.lastRunSec = m.lastRunSec; s.lastRunEpoch = m.lastRunEpoch;
}

void updateSnapshot() {
  static uint32_t last = 0;
  if (millis() - last < 1000 && !gSnapDirty) return;
  last = millis();
  lockShared();
  gSnap.srcPct = srcTank.pct;   gSnap.srcCm = srcTank.cm;   gSnap.srcValid = srcTank.valid;
  gSnap.roofPct = roofTank.pct; gSnap.roofCm = roofTank.cm; gSnap.roofValid = roofTank.valid;
  gSnap.srcFloatWater = srcFloatWater; gSnap.roofFullFloat = roofFullFloat; gSnap.sourceOk = sourceOk;
  gSnap.flowLpm = flowLpm; gSnap.litres = litresTotal;
  fillMotorSnap(gSnap.m[0], M1); fillMotorSnap(gSnap.m[1], M2);
  strlcpy(gSnap.lastAlert, lastAlertMsg, sizeof(gSnap.lastAlert));
  gSnap.uptime = millis() / 1000;
  unlockShared();
  gAnyMotorOn = M1.on || M2.on;
}

void printStatus() {
  static uint32_t last = 0;
  if (millis() - last < 10000) return;
  last = millis();
  Serial.printf("SRC %3.0f%% %5.1fcm %s float:%s ok:%d | ROOF %3.0f%% %5.1fcm %s fullFloat:%d | "
                "M1 %s %s %.2fA flow %.1fL/min f:%u | M2 %s %s %.2fA f:%u%s\n",
                srcTank.pct, srcTank.cm, srcTank.valid ? "" : "(NO ECHO)", srcFloatWater ? "water" : "EMPTY", sourceOk,
                roofTank.pct, roofTank.cm, roofTank.valid ? "" : "(NO ECHO)", roofFullFloat,
                M1.autoMode ? "AUTO" : "MAN", stateName(M1.state), M1.amps, flowLpm, M1.fails,
                M2.autoMode ? "AUTO" : "MAN", stateName(M2.state), M2.amps, M2.fails,
                bootDelayOk() ? "" : "  [boot delay]");
}

// =====================================================================
// 9. NETWORK TASK (core 0): Wi-Fi, MQTT, NTP time, Telegram
// =====================================================================
void sanitizeCfg(Cfg& c) {
  if (c.srcFullCm  > c.srcEmptyCm  - 10) c.srcFullCm  = c.srcEmptyCm  - 10;
  if (c.roofFullCm > c.roofEmptyCm - 10) c.roofFullCm = c.roofEmptyCm - 10;
  if (c.srcStartPct  > c.srcStopPct  - 10) c.srcStartPct  = c.srcStopPct  - 10;
  if (c.roofStartPct > c.roofStopPct - 10) c.roofStartPct = c.roofStopPct - 10;
  if (c.srcOkPct < c.srcEmptyPct + 5) c.srcOkPct = c.srcEmptyPct + 5;
  if (c.m1OcA < c.m1DryA + 1) c.m1OcA = c.m1DryA + 1;
  if (c.m2OcA < c.m2DryA + 1) c.m2OcA = c.m2DryA + 1;
}

bool applyParam(const char* key, float v) {      // caller holds gMux
  if (isnan(v)) return false;
  for (int i = 0; i < N_PARAMS; i++) {
    if (!strcmp(PARAMS[i].key, key)) { cfgRef(gCfg, PARAMS[i]) = constrain(v, PARAMS[i].mn, PARAMS[i].mx); return true; }
  }
  return false;
}

String buildConfigJson() {
  JsonDocument doc;
  lockShared();
  JsonObject p = doc["params"].to<JsonObject>();
  for (int i = 0; i < N_PARAMS; i++) p[PARAMS[i].key] = round1(cfgGet(gCfg, PARAMS[i]));
  JsonArray a = doc["schedules"].to<JsonArray>();
  for (int i = 0; i < gSchedCount; i++) {
    const Sched& s = gSched[i];
    JsonObject o = a.add<JsonObject>();
    char t[8];
    o["motor"] = s.motor; o["days"] = s.days; o["en"] = (bool)s.en;
    snprintf(t, sizeof(t), "%02u:%02u", s.onMin / 60, s.onMin % 60);   o["on"]  = String(t);
    snprintf(t, sizeof(t), "%02u:%02u", s.offMin / 60, s.offMin % 60); o["off"] = String(t);
  }
  unlockShared();
  String out; serializeJson(doc, out);
  return out;
}

void publishConfig() {
  if (!USE_MQTT || !mqtt.connected()) return;
  String out = buildConfigJson();
  mqtt.publish(T_CONFIG, (const uint8_t*)out.c_str(), out.length(), true);
}

size_t buildStatusJson(char* buf, size_t len) {
  Snapshot s;
  lockShared(); s = gSnap; unlockShared();
  JsonDocument doc;
  JsonObject src = doc["src"].to<JsonObject>();
  src["pct"] = s.srcValid ? (int)lroundf(s.srcPct) : -1;
  src["cm"] = round1(s.srcCm); src["step"] = stepName(s.srcPct, s.srcValid);
  src["valid"] = s.srcValid; src["float"] = s.srcFloatWater; src["ok"] = s.sourceOk;
  JsonObject roof = doc["roof"].to<JsonObject>();
  roof["pct"] = s.roofValid ? (int)lroundf(s.roofPct) : -1;
  roof["cm"] = round1(s.roofCm); roof["step"] = stepName(s.roofPct, s.roofValid);
  roof["valid"] = s.roofValid; roof["fullFloat"] = s.roofFullFloat;
  JsonObject fl = doc["flow"].to<JsonObject>();
  fl["lpm"] = round1(s.flowLpm); fl["litres"] = round1((float)s.litres);
  for (int i = 0; i < 2; i++) {
    const MotorSnap& m = s.m[i];
    JsonObject o = doc[i == 0 ? "m1" : "m2"].to<JsonObject>();
    o["on"] = m.on; o["mode"] = m.autoMode ? "auto" : "manual"; o["manualOn"] = m.manualOn;
    o["state"] = stateName(m.state); o["amps"] = round1(m.amps); o["fails"] = m.fails;
    o["runSec"] = m.runSec; o["restLeftSec"] = m.restLeftSec;
    o["lastRunSec"] = m.lastRunSec; o["lastRunAt"] = (long)m.lastRunEpoch; o["sched"] = m.sched;
  }
  doc["alert"] = String(s.lastAlert);
  doc["uptime"] = s.uptime;
  time_t now = time(nullptr);
  doc["time"] = (now > 1700000000) ? (long)now : 0L;
  doc["rssi"] = WiFi.RSSI();
  doc["bench"] = BENCH_MODE;
  return serializeJson(doc, buf, len);
}

void publishStatus() {
  gSnapDirty = false;
  if (!USE_MQTT || !mqtt.connected()) return;
  char buf[1024];
  size_t n = buildStatusJson(buf, sizeof(buf));
  mqtt.publish(T_STATUS, (const uint8_t*)buf, n, true);
}

// Handles one JSON command from the web page or MQTT. Returns false if unknown/invalid.
bool handleCommand(JsonDocument& doc) {
  const char* cmd = doc["cmd"] | "";
  int motor = doc["motor"] | 0;

  if (!strcmp(cmd, "on") || !strcmp(cmd, "off")) {
    if (motor < 1 || motor > 2) return false;
    Cmd c = { CMD_POWER, (uint8_t)motor, (uint8_t)(!strcmp(cmd, "on") ? 1 : 0) };
    xQueueSend(cmdQ, &c, 0);
  } else if (!strcmp(cmd, "mode")) {
    if (motor < 1 || motor > 2) return false;
    const char* v = doc["value"] | "";
    Cmd c = { CMD_MODE, (uint8_t)motor, (uint8_t)(!strcmp(v, "auto") ? 1 : 0) };
    xQueueSend(cmdQ, &c, 0);
  } else if (!strcmp(cmd, "reset")) {
    Cmd c = { CMD_RESET, 0, 0 };
    xQueueSend(cmdQ, &c, 0);
  } else if (!strcmp(cmd, "set")) {
    lockShared();
    JsonObject vals = doc["values"];
    for (JsonPair kv : vals) applyParam(kv.key().c_str(), kv.value().as<float>());
    if (doc["key"].is<const char*>()) applyParam(doc["key"].as<const char*>(), doc["value"].as<float>());
    sanitizeCfg(gCfg);
    prefs.putBytes("cfg", &gCfg, sizeof(Cfg));
    unlockShared();
    publishConfig();
  } else if (!strcmp(cmd, "schedules")) {
    Sched tmp[MAX_SCHED]; int n = 0;
    JsonArray list = doc["list"];
    for (JsonObject o : list) {
      if (n >= MAX_SCHED) break;
      int mot = o["motor"] | 0;
      const char* on = o["on"] | ""; const char* off = o["off"] | "";
      int h1, mi1, h2, mi2;
      if (mot < 1 || mot > 2) continue;
      if (sscanf(on, "%d:%d", &h1, &mi1) != 2 || sscanf(off, "%d:%d", &h2, &mi2) != 2) continue;
      if (h1 < 0 || h1 > 23 || h2 < 0 || h2 > 23 || mi1 < 0 || mi1 > 59 || mi2 < 0 || mi2 > 59) continue;
      bool en = o["en"] | true;
      int days = o["days"] | 127;
      tmp[n].motor = mot; tmp[n].days = (uint8_t)(days & 127); tmp[n].en = en; tmp[n].pad = 0;
      tmp[n].onMin = h1 * 60 + mi1; tmp[n].offMin = h2 * 60 + mi2;
      n++;
    }
    lockShared();
    memcpy(gSched, tmp, sizeof(Sched) * n); gSchedCount = n;
    prefs.putUChar("schn", n);
    if (n) prefs.putBytes("sch", gSched, sizeof(Sched) * n);
    unlockShared();
    publishConfig();
  } else if (!strcmp(cmd, "status")) {
    // gSnapDirty below makes the network task publish status right away
  } else if (!strcmp(cmd, "getConfig")) {
    publishConfig();
  } else if (!strcmp(cmd, "defaults")) {
    lockShared(); gCfg = defaultCfg(); prefs.putBytes("cfg", &gCfg, sizeof(Cfg)); unlockShared();
    publishConfig();
  } else {
    return false;
  }
  gSnapDirty = true;
  return true;
}

void onMqtt(char* topic, byte* payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) { Serial.println("MQTT: bad JSON command"); return; }
  handleCommand(doc);
}

// =====================================================================
// 9b. BUILT-IN WEB PAGE (opened by the "Water Controller" phone app)
// =====================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Controller</title>
<style>
:root{--bg:#eef3f7;--card:#fff;--ink:#14212b;--mute:#66788a;--line:#d9e2ea;--water:#2b8fd6;--water2:#7cc4f2;--ok:#1f9d55;--warn:#d98e04;--bad:#d64545;--btn:#0f6e8c}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--ink)}
header{background:#0f6e8c;color:#fff;padding:14px 16px;display:flex;justify-content:space-between;align-items:center}
header h1{font-size:18px;margin:0}#net{font-size:12px;padding:3px 8px;border-radius:10px;background:#ffffff33}
main{padding:12px;max-width:720px;margin:auto}
.card{background:var(--card);border-radius:14px;padding:14px;margin-bottom:12px;box-shadow:0 1px 3px #0001}
.card h2{font-size:15px;margin:0 0 10px}.row{display:flex;gap:12px}.row>*{flex:1}
#alert{display:none;border-left:4px solid var(--warn);font-size:14px}
.tank{text-align:center}.tank .box{position:relative;height:150px;border:2px solid #9fb3c4;border-top:none;border-radius:0 0 12px 12px;overflow:hidden;background:#f6fafd;margin:6px auto;width:90px}
.tank .fill{position:absolute;bottom:0;left:0;right:0;background:linear-gradient(var(--water2),var(--water));transition:height .8s}
.tank .pct{font-size:26px;font-weight:700}.small{font-size:12px;color:var(--mute)}
.pill{display:inline-block;font-size:12px;font-weight:700;padding:3px 9px;border-radius:10px;color:#fff;background:var(--mute)}
.RUNNING{background:var(--ok)}.REST{background:var(--warn)}.LOCKOUT{background:var(--bad)}
.mhead{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.stats{display:flex;gap:14px;flex-wrap:wrap;font-size:13px;color:var(--mute);margin:6px 0 10px}.stats b{color:var(--ink)}
.seg{display:flex;border:1px solid var(--line);border-radius:10px;overflow:hidden;margin-bottom:8px}
.seg button{flex:1;border:0;background:#fff;padding:9px;font-size:14px;color:var(--ink)}.seg button.on{background:var(--btn);color:#fff}
.btns{display:flex;gap:8px}.btns button,.big{flex:1;border:0;border-radius:10px;padding:11px;font-size:15px;font-weight:600;color:#fff;background:var(--btn)}
.btn-on{background:var(--ok)!important}.btn-off{background:var(--bad)!important}.big{width:100%;background:#475b6b}
details summary{font-weight:600;cursor:pointer;font-size:15px}
.set{display:flex;justify-content:space-between;align-items:center;gap:8px;padding:6px 0;border-bottom:1px solid var(--line);font-size:13px}
.set input{width:80px;padding:6px;border:1px solid var(--line);border-radius:6px;text-align:right}
.grp{margin-top:12px;font-weight:700;font-size:13px;color:var(--btn)}
.sch{border:1px solid var(--line);border-radius:10px;padding:8px;margin:8px 0;font-size:13px}
.sch select,.sch input[type=time]{padding:5px;border:1px solid var(--line);border-radius:6px}
.days label{margin-right:6px;white-space:nowrap}.days{margin-top:6px}
.link{background:none;border:0;color:var(--bad);font-size:13px;padding:4px}
.add{background:#fff;border:1px dashed var(--btn);color:var(--btn);border-radius:10px;padding:8px;width:100%;margin-top:4px}
</style></head><body>
<header><h1>&#128167; Water Controller</h1><span id="net">connecting&hellip;</span></header>
<main>
<div class="card" id="alert"></div>
<div class="card"><div class="row">
 <div class="tank"><h2>Source tank</h2><div class="box"><div class="fill" id="srcFill"></div></div><div class="pct" id="srcPct">--</div><div class="small" id="srcInfo"></div></div>
 <div class="tank"><h2>Roof tank</h2><div class="box"><div class="fill" id="roofFill"></div></div><div class="pct" id="roofPct">--</div><div class="small" id="roofInfo"></div></div>
</div></div>
<div id="motors"></div>
<div class="card"><button class="big" onclick="doReset()">Reset lockouts</button>
<div class="small" style="margin-top:8px" id="meta"></div></div>
<div class="card"><details id="setBox"><summary>Settings</summary><div id="settings"></div>
<div class="btns" style="margin-top:12px"><button onclick="saveSettings()">Save settings</button></div></details></div>
<div class="card"><details><summary>Schedules</summary>
<div class="small" style="margin-top:6px">During a window the motor fills its tank to full (Auto mode only).</div>
<div id="scheds"></div><button class="add" onclick="addSched()">+ Add schedule</button>
<div class="btns" style="margin-top:10px"><button onclick="saveScheds()">Save schedules</button></div></details></div>
</main>
<script>
const NAMES={1:'Motor 1 - Borehole pump',2:'Motor 2 - Roof pump'};
const GROUPS=[
['Tank size (distance from sensor to water)',[['srcEmptyCm','Source tank EMPTY (cm)'],['srcFullCm','Source tank FULL (cm)'],['roofEmptyCm','Roof tank EMPTY (cm)'],['roofFullCm','Roof tank FULL (cm)']]],
['Levels (%)',[['srcStartPct','Motor 1 starts below'],['srcStopPct','Motor 1 stops at'],['roofStartPct','Motor 2 starts below'],['roofStopPct','Motor 2 stops at'],['srcEmptyPct','Source EMPTY (Motor 2 blocked) at'],['srcOkPct','Motor 2 allowed again at']]],
['Current (A) - calibrate',[['m1DryA','Motor 1 dry-run below'],['m1OcA','Motor 1 overcurrent above'],['m2DryA','Motor 2 dry-run below'],['m2OcA','Motor 2 overcurrent above']]],
['Flow sensor (bore line)',[['useFlow','Use flow sensor (1 = yes, 0 = no)'],['m1MinFlowLpm','No-flow below (L/min)'],['flowPulsesPerL','Pulses per litre']]],
['Timing',[['primeSec','Priming time after start (s)'],['minOffMin','Minimum off time (min)'],['m1MaxRunMin','Motor 1 max run (min)'],['m2MaxRunMin','Motor 2 max run (min)'],['m1RestMin','Motor 1 rest after dry (min)'],['m2RestMin','Motor 2 rest after dry (min)'],['m1ProdSec','Motor 1 productive run (s)'],['m2ProdSec','Motor 2 productive run (s)'],['m1MaxFails','Motor 1 failed tries before lockout'],['m2MaxFails','Motor 2 failed tries before lockout'],['m2NoRiseMin','Roof must rise every (min, 0 = off)'],['m2NoRisePct','...by at least (%)']]]];
const DAYS=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
let scheds=[];
const $=id=>document.getElementById(id);
async function api(p,b){const r=await fetch(p,b?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)}:{cache:'no-store'});if(!r.ok)throw new Error(r.status);return r.json();}
function dur(s){if(!s)return '0s';const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;return h?h+'h '+m+'m':m?m+'m '+x+'s':x+'s';}
function tank(k,t,extra){$(k+'Fill').style.height=(t.valid?t.pct:0)+'%';$(k+'Pct').textContent=t.valid?t.pct+'%':'--';$(k+'Info').innerHTML=(t.valid?t.step:'<b style="color:#d64545">SENSOR FAULT</b>')+'<br>'+extra;}
function motorCard(n,m,s){
 let st=m.state, label=st; if(st=='REST')label='REST '+dur(m.restLeftSec);
 let stats='<span>Current <b>'+m.amps.toFixed(1)+' A</b></span>';
 if(m.on)stats+='<span>Running <b>'+dur(m.runSec)+'</b></span>';
 if(n==1)stats+='<span>Flow <b>'+s.flow.lpm.toFixed(1)+' L/min</b></span>';
 if(m.lastRunSec)stats+='<span>Last run <b>'+dur(m.lastRunSec)+'</b></span>';
 if(m.fails)stats+='<span>Failed tries <b>'+m.fails+'</b></span>';
 if(m.sched)stats+='<span><b>Schedule active</b></span>';
 return '<div class="card"><div class="mhead"><h2 style="margin:0">'+NAMES[n]+'</h2><span class="pill '+st+'">'+(m.on?'ON - ':'')+label+'</span></div>'+
 '<div class="stats">'+stats+'</div>'+
 '<div class="seg"><button class="'+(m.mode=='auto'?'on':'')+'" onclick="setMode('+n+',\'auto\')">Auto</button><button class="'+(m.mode=='manual'?'on':'')+'" onclick="setMode('+n+',\'manual\')">Manual</button></div>'+
 '<div class="btns"><button class="btn-on" onclick="power('+n+',1)">ON</button><button class="btn-off" onclick="power('+n+',0)">OFF</button></div></div>';}
function render(s){
 tank('src',s.src,'Float: '+(s.src.float?'water':'<b style="color:#d64545">EMPTY</b>')+(s.src.ok?'':' &middot; refilling'));
 tank('roof',s.roof,s.roof.fullFloat?'<b>Full float tripped</b>':'');
 $('motors').innerHTML=motorCard(1,s.m1,s)+motorCard(2,s.m2,s);
 const a=$('alert');if(s.alert){a.style.display='block';a.textContent=s.alert;}else a.style.display='none';
 let t=s.time?new Date(s.time*1000).toLocaleString():'clock not set';
 $('meta').textContent='Updated '+t+' · Wi-Fi '+s.rssi+' dBm · up '+dur(s.uptime)+(s.bench?' · BENCH MODE':'');}
function online(ok){const n=$('net');n.textContent=ok?'online':'offline';n.style.background=ok?'#1f9d55':'#d64545';}
async function poll(){try{render(await api('/api/status'));online(true);}catch(e){online(false);}}
async function send(c){try{await api('/api/cmd',c);}catch(e){alert('Command failed - check Wi-Fi');}setTimeout(poll,300);}
function power(n,on){if(on&&!confirm('Switch '+NAMES[n]+' ON (manual mode)?\nSafety protections stay active.'))return;send({cmd:on?'on':'off',motor:n});}
function setMode(n,v){send({cmd:'mode',motor:n,value:v});}
function doReset(){if(confirm('Clear lockouts? Make sure the fault has been checked first.'))send({cmd:'reset'});}
async function loadConfig(){try{const c=await api('/api/config');let h='';for(const g of GROUPS){h+='<div class="grp">'+g[0]+'</div>';for(const p of g[1])h+='<div class="set"><span>'+p[1]+'</span><input type="number" step="any" id="p_'+p[0]+'" value="'+c.params[p[0]]+'"></div>';}$('settings').innerHTML=h;scheds=c.schedules||[];drawScheds();}catch(e){setTimeout(loadConfig,3000);}}
async function saveSettings(){const v={};for(const g of GROUPS)for(const p of g[1]){const x=parseFloat($('p_'+p[0]).value);if(!isNaN(x))v[p[0]]=x;}await send({cmd:'set',values:v});setTimeout(loadConfig,600);alert('Settings saved');}
function drawScheds(){let h='';scheds.forEach((s,i)=>{h+='<div class="sch"><select onchange="scheds['+i+'].motor=+this.value"><option value="1"'+(s.motor==1?' selected':'')+'>Motor 1</option><option value="2"'+(s.motor==2?' selected':'')+'>Motor 2</option></select> from <input type="time" value="'+s.on+'" onchange="scheds['+i+'].on=this.value"> to <input type="time" value="'+s.off+'" onchange="scheds['+i+'].off=this.value"> <label><input type="checkbox"'+(s.en?' checked':'')+' onchange="scheds['+i+'].en=this.checked"> on</label> <button class="link" onclick="scheds.splice('+i+',1);drawScheds()">remove</button><div class="days">'+DAYS.map((d,k)=>'<label><input type="checkbox"'+((s.days>>k)&1?' checked':'')+' onchange="scheds['+i+'].days^='+(1<<k)+'">'+d+'</label>').join('')+'</div></div>';});$('scheds').innerHTML=h||'<div class="small" style="margin:8px 0">No schedules.</div>';}
function addSched(){if(scheds.length>=8)return alert('Maximum 8 schedules');scheds.push({motor:2,on:'06:00',off:'07:00',days:127,en:true});drawScheds();}
async function saveScheds(){await send({cmd:'schedules',list:scheds});alert('Schedules saved');}
poll();setInterval(poll,2000);loadConfig();
</script></body></html>)rawliteral";

void webSendJson(int code, const char* body) {
  web.sendHeader("Cache-Control", "no-store");
  web.send(code, "application/json", body);
}

void setupWeb() {
  web.on("/", HTTP_GET, []() { if (portalClient()) { sendPortalPage(); return; } web.send_P(200, "text/html", INDEX_HTML); });
  web.on("/wifi", HTTP_GET, []() { sendPortalPage(); });
  web.on("/wifisave", HTTP_POST, []() { handleWifiSave(); });
  web.on("/wificlear", HTTP_POST, []() { handleWifiClear(); });
  {
    const char* probes[] = { "/generate_204", "/gen_204", "/hotspot-detect.html", "/library/test/success.html",
                             "/connecttest.txt", "/ncsi.txt", "/fwlink", "/redirect", "/canonical.html", "/success.txt" };
    for (const char* u : probes) web.on(u, HTTP_GET, []() { if (portalClient()) redirectToPortal(); else web.send(404, "text/plain", "Not found"); });
  }
  web.on("/api/status", HTTP_GET, []() {
    char buf[1024];
    buildStatusJson(buf, sizeof(buf));
    webSendJson(200, buf);
  });
  web.on("/api/config", HTTP_GET, []() {
    String out = buildConfigJson();
    webSendJson(200, out.c_str());
  });
  web.on("/api/cmd", HTTP_POST, []() {
    JsonDocument doc;
    String body = web.arg("plain");
    if (deserializeJson(doc, body.c_str(), body.length())) { webSendJson(400, "{\"ok\":false,\"error\":\"bad json\"}"); return; }
    bool ok = handleCommand(doc);
    webSendJson(ok ? 200 : 400, ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"unknown command\"}");
  });
  web.onNotFound([]() { if (portalClient()) redirectToPortal(); else web.send(404, "text/plain", "Not found"); });
  web.begin();
}

bool mqttConnect() {
  char cid[48];
  snprintf(cid, sizeof(cid), "wlc-%s-%08lx", DEVICE_ID, (unsigned long)(ESP.getEfuseMac() & 0xFFFFFFFF));
  Serial.printf("MQTT: connecting to %s:%d ...\n", MQTT_HOST, MQTT_PORT);
  if (!mqtt.connect(cid, MQTT_USER, MQTT_PASS, T_AVAIL, 1, true, "offline")) {
    Serial.printf("MQTT: failed, state %d\n", mqtt.state());
    return false;
  }
  mqtt.publish(T_AVAIL, "online", true);
  mqtt.subscribe(T_CMD, 1);
  publishConfig();
  publishStatus();
  Serial.println("MQTT: connected");
  return true;
}

bool sendTelegram(const char* text) {
  if (!TG_ENABLED) return true;
  HTTPClient http;
  String url = String("https://api.telegram.org/bot") + TG_BOT_TOKEN + "/sendMessage";
  if (!http.begin(tgClient, url)) return false;
  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/json");
  JsonDocument d;
  d["chat_id"] = TG_CHAT_ID;
  d["text"] = String("[") + DEVICE_ID + "] " + text;
  String body; serializeJson(d, body);
  int code = http.POST(body);
  http.end();
  if (code != 200) Serial.printf("Telegram: HTTP %d\n", code);
  return code == 200;
}

// ---- Wi-Fi: networks saved from the setup page (up to 3) + the 5 slots in this code ----
// Index 0-2 = saved from the phone (newest first), 3-7 = code slots 1-5.
const int NET_SAVED = 3, NET_TOTAL = 8;
String gWifiSsid[NET_TOTAL], gWifiPass[NET_TOTAL];
String gWifiLastMsg = "";

String netLabel(int k) {
  if (k < NET_SAVED) return String("saved network ") + String(k + 1);
  return String("code slot ") + String(k - NET_SAVED + 1);
}

void loadSavedNetworks() {
  Preferences wp;
  wp.begin("wifi", true);
  for (int i = 0; i < NET_SAVED; i++) {
    gWifiSsid[i] = wp.getString((String("s") + i).c_str(), "");
    gWifiPass[i] = wp.getString((String("p") + i).c_str(), "");
  }
  wp.end();
}

void saveNetwork(const String& ssid, const String& pass) {
  // newest goes first; an existing entry with the same name is replaced
  String s[NET_SAVED], p[NET_SAVED];
  int n = 0;
  s[n] = ssid; p[n] = pass; n++;
  for (int i = 0; i < NET_SAVED && n < NET_SAVED; i++) {
    if (gWifiSsid[i].length() && gWifiSsid[i] != ssid) { s[n] = gWifiSsid[i]; p[n] = gWifiPass[i]; n++; }
  }
  Preferences wp;
  wp.begin("wifi", false);
  for (int i = 0; i < NET_SAVED; i++) {
    String sv = i < n ? s[i] : String(""), pv = i < n ? p[i] : String("");
    wp.putString((String("s") + i).c_str(), sv);
    wp.putString((String("p") + i).c_str(), pv);
    gWifiSsid[i] = sv; gWifiPass[i] = pv;
  }
  wp.end();
}

void forgetSavedNetworks() {
  Preferences wp;
  wp.begin("wifi", false);
  wp.clear();
  wp.end();
  for (int i = 0; i < NET_SAVED; i++) { gWifiSsid[i] = ""; gWifiPass[i] = ""; }
}

bool anyNetworkConfigured() {
  for (int k = 0; k < NET_TOTAL; k++) if (gWifiSsid[k].length()) return true;
  return false;
}

void servicePortal() {
  if (gPortalOn) { dnsServer.processNextRequest(); web.handleClient(); }
}

// Scan, show what the ESP32 can see, then join a network from the list.
// Networks saved from the phone are tried first, then the code slots, strongest first.
bool joinKnownWifi() {
  Serial.println("WiFi: scanning ...");
  WiFi.disconnect(false);
  vTaskDelay(pdMS_TO_TICKS(100));
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("WiFi: NO networks found at all (check the antenna / power)");
    gWifiLastMsg = "No Wi-Fi networks found.";
    WiFi.scanDelete();
    return false;
  }
  Serial.printf("WiFi: %d networks found:\n", n);
  int candIdx[NET_TOTAL]; int candRssi[NET_TOTAL]; int candCount = 0;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    int known = -1;
    for (int k = 0; k < NET_TOTAL; k++) if (gWifiSsid[k].length() && s == gWifiSsid[k]) { known = k; break; }
    Serial.printf("   %-32s %4d dBm  %s\n", s.c_str(), (int)WiFi.RSSI(i), known >= 0 ? "<-- IN YOUR LIST" : "");
    if (known >= 0) {
      bool dup = false;
      for (int c = 0; c < candCount; c++) if (candIdx[c] == known) dup = true;
      if (!dup && candCount < NET_TOTAL) { candIdx[candCount] = known; candRssi[candCount] = (int)WiFi.RSSI(i); candCount++; }
    }
  }
  WiFi.scanDelete();
  if (candCount == 0) {
    Serial.println("WiFi: none of your networks is in range.");
    Serial.println("      Your list:");
    for (int k = 0; k < NET_TOTAL; k++) if (gWifiSsid[k].length()) Serial.printf("        %s: \"%s\"\n", netLabel(k).c_str(), gWifiSsid[k].c_str());
    Serial.println("      Check: hotspot ON, band 2.4 GHz, and the name typed EXACTLY (capitals, spaces).");
    Serial.println("      Or hold the BOOT button 5 s and add your Wi-Fi from your phone.");
    gWifiLastMsg = "None of the saved networks is in range.";
    return false;
  }
  // saved-from-phone networks first (newest first), then code slots; strongest first within each group
  for (int a = 0; a < candCount; a++)
    for (int b = a + 1; b < candCount; b++) {
      int ga = candIdx[a] < NET_SAVED ? 0 : 1, gb = candIdx[b] < NET_SAVED ? 0 : 1;
      bool swap = (gb < ga) || (gb == ga && ga == 0 && candIdx[b] < candIdx[a]) || (gb == ga && ga == 1 && candRssi[b] > candRssi[a]);
      if (swap) { int t = candIdx[a]; candIdx[a] = candIdx[b]; candIdx[b] = t; t = candRssi[a]; candRssi[a] = candRssi[b]; candRssi[b] = t; }
    }
  for (int c = 0; c < candCount; c++) {
    int k = candIdx[c];
    Serial.printf("WiFi: joining \"%s\" (%s) ...\n", gWifiSsid[k].c_str(), netLabel(k).c_str());
    gJoining = true;
    WiFi.begin(gWifiSsid[k].c_str(), gWifiPass[k].c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { servicePortal(); vTaskDelay(pdMS_TO_TICKS(50)); }
    gJoining = false;
    if (WiFi.status() == WL_CONNECTED) { gWifiLastMsg = String("Connected to \"") + gWifiSsid[k] + "\"."; return true; }
    Serial.printf("WiFi: could NOT join \"%s\" (status %d) - the PASSWORD (%s) is probably wrong\n",
                  gWifiSsid[k].c_str(), (int)WiFi.status(), netLabel(k).c_str());
    gWifiLastMsg = String("Could not join \"") + gWifiSsid[k] + "\" - the password is probably wrong.";
    WiFi.disconnect(false);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  return false;
}

// ---- Setup mode: the ESP32 opens its own hotspot with a Wi-Fi setup page ----
void startPortal() {
  if (gPortalOn) { gPortalSince = millis(); return; }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(PORTAL_SSID, PORTAL_PASS);
  vTaskDelay(pdMS_TO_TICKS(200));
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());
  startWebOnce();
  gPortalOn = true;
  gPortalSince = millis();
  beep(300);
  Serial.println("==============================================");
  Serial.printf("SETUP MODE: on your phone, join Wi-Fi \"%s\" (password: %s)\n", PORTAL_SSID, PORTAL_PASS);
  Serial.printf("            the setup page opens by itself, or go to http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("==============================================");
}

void stopPortal() {
  if (!gPortalOn) return;
  dnsServer.stop();
  WiFi.softAPdisconnect(true);          // AP off; the Wi-Fi connection stays up
  gPortalOn = false;
  Serial.println("SETUP MODE: closed");
}

String htmlEsc(const String& in) {
  String o; o.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char ch = in[i];
    if (ch == '&') o += "&amp;"; else if (ch == '<') o += "&lt;"; else if (ch == '>') o += "&gt;";
    else if (ch == '"') o += "&quot;"; else if (ch == '\'') o += "&#39;"; else o += ch;
  }
  return o;
}

const char PORTAL_HEAD[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AquaGuard Wi-Fi setup</title><style>
body{margin:0;font-family:system-ui,sans-serif;background:#eef3f8;color:#10233a}
.top{background:#0b6fb8;color:#fff;padding:16px}.top b{font-size:20px}.top div{font-size:13px;opacity:.9}
.c{background:#fff;border-radius:14px;margin:12px;padding:14px;box-shadow:0 2px 8px rgba(0,0,0,.06)}
h2{font-size:16px;margin:0 0 8px}.s{font-size:13px;color:#4b5d73}
.net{display:block;width:100%;text-align:left;border:1px solid #d9e3ee;background:#fff;border-radius:10px;padding:11px;margin:6px 0;font-size:15px}
.net span{float:right;color:#4b5d73;font-size:13px}
label{display:block;font-size:13px;font-weight:600;margin:10px 0 4px}
input[type=text],input[type=password]{width:100%;box-sizing:border-box;padding:11px;font-size:16px;border:1px solid #c9d6e3;border-radius:10px}
.b{width:100%;border:0;border-radius:10px;padding:13px;font-size:16px;font-weight:700;color:#fff;background:#0b6fb8;margin-top:12px}
.g{background:#fff;color:#b42020;border:1px solid #e8b4b4}.m{background:#fff4df;color:#6b4300;border-radius:10px;padding:10px;font-size:14px}
</style></head><body><div class="top"><b>AquaGuard</b><div>Wi-Fi setup</div></div>)HTML";

void sendPortalPage() {
  int n = gJoining ? 0 : WiFi.scanNetworks();
  String h; h.reserve(6000);
  h += FPSTR(PORTAL_HEAD);
  if (gWifiLastMsg.length()) h += "<div class='c m'>" + htmlEsc(gWifiLastMsg) + "</div>";
  if (WiFi.status() == WL_CONNECTED) h += "<div class='c s'>Now connected to <b>" + htmlEsc(WiFi.SSID()) + "</b>.</div>";
  h += "<div class='c'><h2>1. Tap your Wi-Fi</h2><div class='s'>Only 2.4 GHz networks are shown.</div>";
  if (gJoining) h += "<p class='s'>AquaGuard is connecting right now. Reload this page in 20 seconds.</p>";
  else if (n <= 0) h += "<p class='s'>No networks found. Reload this page.</p>";
  for (int i = 0; i < n && i < 20; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;
    String e = htmlEsc(s);
    h += "<button class='net' type='button' data-s='" + e + "' onclick=\"document.getElementById('s').value=this.dataset.s;document.getElementById('p').focus()\">" + e + "<span>" + String((int)WiFi.RSSI(i)) + " dBm</span></button>";
  }
  WiFi.scanDelete();
  h += "</div><div class='c'><h2>2. Type the password</h2><form method='POST' action='/wifisave'>"
       "<label for='s'>Wi-Fi name</label><input type='text' id='s' name='s' maxlength='32' autocapitalize='none' autocorrect='off' required>"
       "<label for='p'>Password</label><input type='password' id='p' name='p' maxlength='63'>"
       "<label style='font-weight:400'><input type='checkbox' onclick=\"var p=document.getElementById('p');p.type=this.checked?'text':'password'\"> Show password</label>"
       "<button class='b' type='submit'>Save and connect</button></form></div>";
  h += "<div class='c'><h2>Saved networks</h2>";
  bool any = false;
  for (int i = 0; i < NET_SAVED; i++) if (gWifiSsid[i].length()) { any = true; h += "<div class='s'>&#8226; " + htmlEsc(gWifiSsid[i]) + "</div>"; }
  if (!any) h += "<div class='s'>None yet.</div>";
  if (any) h += "<form method='POST' action='/wificlear'><button class='b g' type='submit'>Forget saved networks</button></form>";
  h += "</div></body></html>";
  web.send(200, "text/html", h);
}

void handleWifiSave() {
  String s = web.arg("s"); s.trim();
  String p = web.arg("p");
  String msg;
  if (s.length() == 0 || s.length() > 32) msg = "Please type a Wi-Fi name.";
  else if (p.length() > 0 && p.length() < 8) msg = "A Wi-Fi password has at least 8 characters.";
  if (msg.length()) { gWifiLastMsg = msg; web.sendHeader("Location", "/wifi"); web.send(303); return; }
  saveNetwork(s, p);
  gWifiLastMsg = String("Saved \"") + s + "\". Trying to connect ...";
  gTryNewNet = true;
  String h; h += FPSTR(PORTAL_HEAD);
  h += "<div class='c'><h2>Saved &#10003;</h2><p>AquaGuard is now connecting to <b>" + htmlEsc(s) + "</b>.</p>"
       "<p class='s'>If it connects, this setup Wi-Fi closes and the AquaGuard app shows <b>Online</b> within a minute.</p>"
       "<p class='s'>If <b>" PORTAL_SSID "</b> is still here after 1 minute, join it again: the page will say what went wrong (usually the password).</p></div></body></html>";
  web.send(200, "text/html", h);
}

void handleWifiClear() {
  forgetSavedNetworks();
  gWifiLastMsg = "Saved networks forgotten.";
  web.sendHeader("Location", "/wifi");
  web.send(303);
}

bool portalClient() { return gPortalOn && web.client().localIP() == WiFi.softAPIP(); }

void redirectToPortal() {
  web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/wifi");
  web.send(302, "text/plain", "");
}

void startWebOnce() {
  if (gWebStarted) return;
  gWebStarted = true;
  setupWeb();
}

void netTask(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  loadSavedNetworks();
  const char* ssids[5] = { WIFI_SSID, WIFI2_SSID, WIFI3_SSID, WIFI4_SSID, WIFI5_SSID };
  const char* passes[5] = { WIFI_PASS, WIFI2_PASS, WIFI3_PASS, WIFI4_PASS, WIFI5_PASS };
  for (int i = 0; i < 5; i++) { gWifiSsid[NET_SAVED + i] = ssids[i]; gWifiSsid[NET_SAVED + i].trim(); gWifiPass[NET_SAVED + i] = passes[i]; }

  if (USE_MQTT) {
    if (MQTT_USE_TLS) { mqttSecure.setInsecure(); mqtt.setClient(mqttSecure); }
    else mqtt.setClient(mqttPlain);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(2048);
    mqtt.setKeepAlive(30);
    mqtt.setCallback(onMqtt);
  }
  tgClient.setInsecure();

  bool started = false, wasConnected = false;
  uint32_t lastWifiTry = millis() - 20000, lastMqttTry = 0, lastPub = 0;
  uint32_t offlineSince = millis();
  if (!anyNetworkConfigured()) {
    Serial.println("WiFi: no Wi-Fi saved yet - opening setup mode");
    startPortal();
  }

  for (;;) {
    if (gPortalRequest) { gPortalRequest = false; startPortal(); }
    servicePortal();
    if (gTryNewNet && WiFi.status() == WL_CONNECTED) { WiFi.disconnect(false); vTaskDelay(pdMS_TO_TICKS(300)); }

    if (WiFi.status() != WL_CONNECTED) {
      if (wasConnected) { Serial.println("WiFi: lost - controller keeps running offline"); wasConnected = false; offlineSince = millis(); }
      if (!gPortalOn && millis() - offlineSince > PORTAL_AUTO_MS) startPortal();
      bool phoneOnPortal = gPortalOn && WiFi.softAPgetStationNum() > 0;
      uint32_t retryMs = gPortalOn ? 60000UL : 15000UL;
      if (gTryNewNet || (anyNetworkConfigured() && !phoneOnPortal && millis() - lastWifiTry > retryMs)) {
        gTryNewNet = false;
        joinKnownWifi();                       // scans, prints what it sees, joins a network from your list
        lastWifiTry = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(gPortalOn ? 10 : 250));
      continue;
    }
    if (gPortalOn && millis() - gPortalSince > 600000UL) stopPortal();   // opened by the button but not used
    if (!wasConnected) {
      wasConnected = true;
      if (gPortalOn) stopPortal();
      Serial.println("==============================================");
      Serial.printf("WiFi connected to \"%s\". Local page:  %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      Serial.printf("            (or try:  %s.local)\n", MDNS_NAME);
      Serial.println("==============================================");
    }
    if (!started) {
      started = true;
      configTzTime(TZ_POSIX, "pool.ntp.org", "time.google.com");
      if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", 80);
      startWebOnce();
    }

    web.handleClient();

    if (USE_MQTT) {
      if (!mqtt.connected() && millis() - lastMqttTry > 5000) { lastMqttTry = millis(); mqttConnect(); }
      if (mqtt.connected()) {
        mqtt.loop();
        uint32_t every = gAnyMotorOn ? 5000UL : 15000UL;
        if ((gSnapDirty && millis() - lastPub > 1000) || millis() - lastPub > every) { lastPub = millis(); publishStatus(); }
      }
    }

    char msg[160];
    if (xQueueReceive(alertQ, msg, 0) == pdTRUE) {
      if (USE_MQTT && mqtt.connected()) mqtt.publish(T_EVENT, msg);
      if (TG_ENABLED && !sendTelegram(msg)) sendTelegram(msg);   // one retry
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =====================================================================
// 10. SETUP / LOOP
// =====================================================================
void initMotor(Motor& m, uint8_t id, int relay, int ct) {
  memset(&m, 0, sizeof(Motor));
  m.id = id; m.relay = relay; m.ct = ct;
  m.state = ST_IDLE;
  m.autoMode = prefs.getBool(id == 1 ? "a1" : "a2", true);
  if (prefs.getBool(id == 1 ? "l1" : "l2", false)) m.state = ST_LOCKOUT;   // lockout survives power cuts
  m.stopTime = millis() - minOffMs(gCfg);
}

void setup() {
  // Relays OFF first, before anything else
  pinMode(PIN_RELAY1, OUTPUT); digitalWrite(PIN_RELAY1, RELAY_OFF);
  pinMode(PIN_RELAY2, OUTPUT); digitalWrite(PIN_RELAY2, RELAY_OFF);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Water-Level Controller starting ===");

  pinMode(PIN_BUZZER, OUTPUT);   digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LED_M1, OUTPUT);   pinMode(PIN_LED_M2, OUTPUT);   pinMode(PIN_LED_FAULT, OUTPUT);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_CT1, ADC_11db);
  analogSetPinAttenuation(PIN_CT2, ADC_11db);

  calibrateCurrentZero();          // relays are already OFF, so this is the 0 A reading
  tankBegin(srcTank); tankBegin(roofTank);
  dinBegin(fSrc); dinBegin(fRoof);
  srcFloatWater = USE_SRC_FLOAT ? (fSrc.stable == LOW) : true;
  roofFullFloat = USE_ROOF_FLOAT && (fRoof.stable == HIGH);

  if (USE_FLOW_SENSOR) {
    pinMode(PIN_FLOW, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, FALLING);
  } else {
    pinMode(PIN_FLOW, INPUT_PULLDOWN);
  }

  gMux   = xSemaphoreCreateMutex();
  cmdQ   = xQueueCreate(10, sizeof(Cmd));
  alertQ = xQueueCreate(10, 160);
  memset(alertLast, 0, sizeof(alertLast));

  prefs.begin("wlc", false);
  gCfg = defaultCfg();
  if (prefs.getBytesLength("cfg") == sizeof(Cfg)) {
    Cfg t; prefs.getBytes("cfg", &t, sizeof(Cfg));
    if (t.magic == CFG_MAGIC) gCfg = t;
  }
  gSchedCount = prefs.getUChar("schn", 0);
  if (gSchedCount > MAX_SCHED) gSchedCount = 0;
  if (gSchedCount) prefs.getBytes("sch", gSched, sizeof(Sched) * gSchedCount);

  initMotor(M1, 1, PIN_RELAY1, PIN_CT1);
  initMotor(M2, 2, PIN_RELAY2, PIN_CT2);

  bootTime = millis();
  lastAnyStart = millis() - STAGGER_MS;

  // Hardware watchdog on the control loop: reboot (relays default OFF) if it ever hangs
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wcfg = { .timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_reconfigure(&wcfg) != ESP_OK) esp_task_wdt_init(&wcfg);
#else
  esp_task_wdt_init(10, true);
#endif
  esp_task_wdt_add(NULL);

  xTaskCreatePinnedToCore(netTask, "net", 16384, nullptr, 1, nullptr, 0);

  if (M1.state == ST_LOCKOUT || M2.state == ST_LOCKOUT)
    alert(AL_BOOT, "Controller restarted - a LOCKOUT from before the power cut is still active");
  else
    alert(AL_BOOT, "Controller started (power restored). Automatic pumping begins in %lu s.", (unsigned long)(BOOT_DELAY_MS / 1000));
}

void loop() {
  esp_task_wdt_reset();

  Cfg c;
  lockShared(); c = gCfg; unlockShared();

  processCommands();
  updateSensors(c);
  updateSourceOk(c);
  updateSchedules();

  Demand d1 = demandM1(c);
  Demand d2 = demandM2(c);
  updateMotor(M1, c, d1);
  updateMotor(M2, c, d2);
  checkIdleCurrent(M1, c);
  checkIdleCurrent(M2, c);

  checkSensorAlerts(c);
  updateResetButton();
  updateSetupButton();
  updateIndicators();
  updateSnapshot();
  printStatus();
  delay(20);
}
