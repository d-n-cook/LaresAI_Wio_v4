// Minimal one-way LoRa test for the Seeed Wio-SX1262 + XIAO ESP32S3 kit (US 915 MHz).
// Pin map taken from the Seeed variant: CS=41, DIO1=39, RESET=42, BUSY=40,
// SPI SCK=7 MISO=8 MOSI=9. DIO3 drives the TCXO (1.8V), DIO2 is the RF switch.
//
// NODE_ROLE 0 = sender (broadcasts structured telemetry every ~20-30s)
// NODE_ROLE 1 = receiver/gateway (validates + prints each frame with RSSI/SNR)
//
// Over-the-air frame (NMEA-style, kept compact for LoRa):
//   $LARES,id=59,seq=42,typ=SENSOR,zone=1,temp=74.3,hum=41.0,airq=95,gasq=120,smoke=0,fire=0,gas=0,pump=0,cls=Normal,conf=0.98*A7
// where *A7 is an XOR checksum of everything between '$' and '*'. The id field is
// the main node's cameraID/deviceID, so the many-devices -> few-gateways model
// works; (id,seq) lets the cloud dedupe frames heard by overlapping gateways.
// Telemetry values are pushed from the main (Device) node over UART (SENSOR_DATA
// packet); see buildTelemetryFrame.

#include <RadioLib.h>
#include <Wire.h>
#include <MAX30105.h>

#ifndef NODE_ROLE
#define NODE_ROLE 0 // default to sender if not set by build flag
#endif

#ifndef NODE_ID
#define NODE_ID 1 // unique per transmitting device; set via build flag
#endif

#if NODE_ROLE == 0
// Sender-only: bring up WiFi + ElegantOTA so this board can be flashed over the
// air before the enclosure is sealed. Guarded by NODE_ROLE so the receiver env
// (which does not pull in the ElegantOTA lib) still compiles.
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Preferences.h>

// UART link to the main (Device) node — receives the WiFi credentials at boot so
// the SSID/password no longer have to be hard-coded here. UART1 with a crossover
// to the Device board's UART2:
//   Wio RX (GPIO1) ← Device TX (GPIO2)
//   Wio TX (GPIO2) → Device RX (GPIO1)
#define DEVICE_UART_RX 1   // D0
#define DEVICE_UART_TX 2   // D1
static HardwareSerial DeviceUART(1);

// Working credential buffers used by setupWifiAndOTA(). Populated in priority
// order from: UART CONFIG_DATA -> last-known creds in NVS -> compiled default
// below. Start empty so we never connect before a source has been chosen.
static char WIFI_SSID[32]     = "";
static char WIFI_PASSWORD[64] = "";

// Compiled-in default, last-resort only. Used ~2 min after boot if neither the
// main node (UART) nor NVS provided credentials.
static const char DEFAULT_SSID[]     = "ATTNG8pN6a";
static const char DEFAULT_PASSWORD[] = "d4d#ybhz=39n";

static WebServer otaServer(80);
static bool otaWifiUp = false;

static void setupWifiAndOTA();  // defined below; called once creds are chosen

// ── UART framing (identical to the Device/AudioNode link) ───────────────────
// Frame: START_BYTE, type, length, payload[length], XOR checksum.
#define START_BYTE 0xAA

enum PacketType : uint8_t {
    HELLO        = 0x01,
    CONFIG_DATA  = 0x03,
    SENSOR_DATA  = 0x30,
    WIO_STATUS   = 0x31,   // Wio -> Main: MAX30105 init/calibration/smoke insight
};

// Mirror of the Device node's WifiConfig. The Wio only consumes ssid/password
// but the whole struct is received so both sides stay byte-compatible.
struct __attribute__((packed)) WifiConfig {
    char ssid[32];
    char password[64];
    char apiBaseUrl[64];
    uint8_t deviceRole;
    char relayUrl[48];
    char deviceSecret[40];
    int32_t cameraId;
}; // total: 253 bytes — fits in uint8_t UART length field

// Compact real-sensor telemetry received from the main node (SENSOR_DATA packet),
// used to build the LoRa frame in place of the old simulated values. Mirror of the
// Device node's WioTelemetry — keep byte-identical.
struct __attribute__((packed)) WioTelemetry {
    int32_t cameraId;
    int32_t zoneId;
    float   temperatureF;
    float   humidity;
    float   airQ;
    float   gasQ;
    float   gasResistance;
    float   confidence;
    uint8_t alarmSmoke;
    uint8_t alarmFire;
    uint8_t alarmGas;
    uint8_t pumpOn;
    char    classification[24];
};

// MAX30105 status reported back to the main node over UART (WIO_STATUS packet) so
// the calibration/smoke process is visible on Main's serial while the Wio is
// enclosed. Keep byte-identical to the mirror struct in the Device firmware.
struct __attribute__((packed)) WioSmokeStatus {
    uint8_t  sensorOk;       // MAX30105 begin() succeeded
    uint8_t  calibrated;     // clean-air warmup complete / external baseline set
    uint8_t  smokeDetected;  // latched smoke flag
    uint8_t  baselineSource; // 0=unseeded, 1=auto-seed, 2=external
    uint16_t calProgress;    // clean-air reads accumulated toward warmup
    uint16_t calTarget;      // warmup target
    uint32_t ir;             // raw IR count
    uint32_t red;            // raw RED count
    float    baseline;       // rolling clean-air IR baseline
    float    smokeIndex;     // IR - baseline (>= 0)
    float    redIrRatio;     // RED / IR
    float    tempC;          // die temperature
};

// Latest telemetry pushed from the main node; buildTelemetryFrame() reads these.
static WioTelemetry latestTelemetry;
static bool haveTelemetry = false;
static uint32_t lastTelemetryMs = 0;

static void uartSendPacket(HardwareSerial &port, PacketType type, const uint8_t *payload, uint8_t length)
{
    uint8_t checksum = type ^ length;
    for (uint8_t i = 0; i < length; i++)
        checksum ^= payload[i];

    port.write(START_BYTE);
    port.write((uint8_t)type);
    port.write(length);
    if (length)
        port.write(payload, length);
    port.write(checksum);
}

static bool uartReadPacket(HardwareSerial &port, PacketType &typeOut, uint8_t *buffer, uint8_t &lengthOut)
{
    static enum { WAIT_START, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_START;
    static uint8_t type, length, index, checksum;

    while (port.available()) {
        uint8_t b = port.read();
        switch (state) {
            case WAIT_START:
                if (b == START_BYTE) state = WAIT_TYPE;
                break;
            case WAIT_TYPE:
                type = b;
                checksum = b;
                state = WAIT_LEN;
                break;
            case WAIT_LEN:
                length = b;
                checksum ^= b;
                index = 0;
                state = (length == 0) ? WAIT_CHECKSUM : WAIT_PAYLOAD;
                break;
            case WAIT_PAYLOAD:
                buffer[index++] = b;
                checksum ^= b;
                if (index >= length) state = WAIT_CHECKSUM;
                break;
            case WAIT_CHECKSUM:
                if (b == checksum) {
                    typeOut = (PacketType)type;
                    lengthOut = length;
                    state = WAIT_START;
                    return true;
                }
                state = WAIT_START;
                break;
        }
    }
    return false;
}

// ── Credential acquisition (non-blocking, driven from loop) ──────────────────
// Priority: UART CONFIG_DATA is authoritative and always wins (and is persisted
// for next boot). If the main node stays silent we fall back to the last-known
// creds in NVS after 20 s, then to the compiled default after 2 min. HELLO keeps
// going the whole time, so a late-booting main node can still take over.
static Preferences wioPrefs;

enum CredSource { CRED_NONE, CRED_UART, CRED_STORED, CRED_DEFAULT };
static CredSource credSource = CRED_NONE;

static char storedSsid[32]     = "";
static char storedPassword[64] = "";
static bool haveStoredCreds    = false;
static bool storedTried        = false;
static bool defaultTried       = false;
static bool beginIssued        = false;   // WiFi.begin has been called at least once

static char activeSsid[32]     = "";      // creds WiFi is currently using
static char activePassword[64] = "";

static uint32_t credWaitStartMs = 0;
static uint32_t lastHelloMs     = 0;

static const uint32_t HELLO_INTERVAL_MS   = 1000;    // re-send HELLO each second
static const uint32_t STORED_FALLBACK_MS  = 20000;   // 20 s: try last-known creds
static const uint32_t DEFAULT_FALLBACK_MS = 120000;  // 2 min: try compiled default

static void loadStoredCreds()
{
    wioPrefs.begin("wiocfg", true); // read-only
    String s = wioPrefs.getString("ssid", "");
    String p = wioPrefs.getString("pw", "");
    wioPrefs.end();
    if (s.length() > 0) {
        strncpy(storedSsid, s.c_str(), sizeof(storedSsid) - 1);
        storedSsid[sizeof(storedSsid) - 1] = '\0';
        strncpy(storedPassword, p.c_str(), sizeof(storedPassword) - 1);
        storedPassword[sizeof(storedPassword) - 1] = '\0';
        haveStoredCreds = true;
        Serial.printf("Loaded last-known WiFi creds from NVS (ssid=%s)\n", storedSsid);
    }
}

static void saveStoredCreds(const char *ssid, const char *pw)
{
    if (haveStoredCreds && strcmp(ssid, storedSsid) == 0 && strcmp(pw, storedPassword) == 0)
        return; // unchanged; skip the flash write
    wioPrefs.begin("wiocfg", false);
    wioPrefs.putString("ssid", ssid);
    wioPrefs.putString("pw", pw);
    wioPrefs.end();
    strncpy(storedSsid, ssid, sizeof(storedSsid) - 1);
    storedSsid[sizeof(storedSsid) - 1] = '\0';
    strncpy(storedPassword, pw, sizeof(storedPassword) - 1);
    storedPassword[sizeof(storedPassword) - 1] = '\0';
    haveStoredCreds = true;
    Serial.printf("Saved WiFi creds to NVS (ssid=%s)\n", ssid);
}

static void sendHello()
{
    uint8_t hello[1] = { 0x01 };
    uartSendPacket(DeviceUART, HELLO, hello, sizeof(hello));
}

// Copy the working buffers into active* and (re)connect. Returns true if WiFi
// came up. Reconnects cleanly if a prior begin was issued (e.g. UART creds
// arriving after a fallback connect).
static bool applyCredsAndConnect(CredSource src)
{
    credSource = src;
    strncpy(activeSsid, WIFI_SSID, sizeof(activeSsid) - 1);
    activeSsid[sizeof(activeSsid) - 1] = '\0';
    strncpy(activePassword, WIFI_PASSWORD, sizeof(activePassword) - 1);
    activePassword[sizeof(activePassword) - 1] = '\0';
    if (beginIssued) {
        Serial.println("Reconnecting WiFi with updated credentials...");
        WiFi.disconnect();
        delay(50);
    }
    beginIssued = true;
    setupWifiAndOTA();
    return otaWifiUp;
}

// Pull all pending UART frames. CONFIG_DATA updates the working credential
// buffers (returns true so the caller can (re)connect); SENSOR_DATA refreshes the
// latest telemetry used to build the LoRa frame.
static bool pollUartCreds()
{
    uint8_t buffer[256];
    uint8_t len = 0;
    PacketType type;
    bool gotCreds = false;
    while (uartReadPacket(DeviceUART, type, buffer, len)) {
        if (type == CONFIG_DATA) {
            WifiConfig cfg;
            memset(&cfg, 0, sizeof(cfg));
            memcpy(&cfg, buffer, (len < sizeof(cfg)) ? len : sizeof(cfg));
            cfg.ssid[sizeof(cfg.ssid) - 1] = '\0';
            cfg.password[sizeof(cfg.password) - 1] = '\0';
            if (cfg.ssid[0] != '\0') {
                strncpy(WIFI_SSID, cfg.ssid, sizeof(WIFI_SSID) - 1);
                WIFI_SSID[sizeof(WIFI_SSID) - 1] = '\0';
                strncpy(WIFI_PASSWORD, cfg.password, sizeof(WIFI_PASSWORD) - 1);
                WIFI_PASSWORD[sizeof(WIFI_PASSWORD) - 1] = '\0';
                gotCreds = true;
            }
        } else if (type == SENSOR_DATA) {
            memset(&latestTelemetry, 0, sizeof(latestTelemetry));
            memcpy(&latestTelemetry, buffer,
                   (len < sizeof(latestTelemetry)) ? len : sizeof(latestTelemetry));
            latestTelemetry.classification[sizeof(latestTelemetry.classification) - 1] = '\0';
            haveTelemetry = true;
            lastTelemetryMs = millis();
        }
    }
    return gotCreds;
}

// Runs every loop() pass. Keeps nudging the main node with HELLO until it
// answers; UART creds always win and are persisted. Falls back to last-known
// NVS creds after 20 s, then compiled defaults after 2 min.
static void serviceCredentialLink()
{
    if (pollUartCreds()) {
        bool changed = (credSource != CRED_UART) ||
                       strcmp(WIFI_SSID, activeSsid) != 0 ||
                       strcmp(WIFI_PASSWORD, activePassword) != 0;
        saveStoredCreds(WIFI_SSID, WIFI_PASSWORD);
        Serial.printf("WiFi credentials received over UART (ssid=%s)\n", WIFI_SSID);
        if (!beginIssued || changed)
            applyCredsAndConnect(CRED_UART);
        return;
    }

    if (credSource == CRED_UART)
        return; // already locked onto authoritative UART creds

    // Keep announcing ourselves so the main node can answer (or take over) at any
    // time — even after we have connected via a fallback.
    if (millis() - lastHelloMs >= HELLO_INTERVAL_MS) {
        sendHello();
        lastHelloMs = millis();
    }

    if (WiFi.status() == WL_CONNECTED)
        return; // a fallback already got us online; just keep polling UART/HELLO

    uint32_t elapsed = millis() - credWaitStartMs;

    // Fallback 1: last-known creds from NVS after a short wait.
    if (haveStoredCreds && !storedTried && elapsed >= STORED_FALLBACK_MS) {
        storedTried = true;
        strncpy(WIFI_SSID, storedSsid, sizeof(WIFI_SSID) - 1);
        WIFI_SSID[sizeof(WIFI_SSID) - 1] = '\0';
        strncpy(WIFI_PASSWORD, storedPassword, sizeof(WIFI_PASSWORD) - 1);
        WIFI_PASSWORD[sizeof(WIFI_PASSWORD) - 1] = '\0';
        Serial.printf("No UART reply in %lus — trying last-known creds (ssid=%s)\n",
                      (unsigned long)(STORED_FALLBACK_MS / 1000), WIFI_SSID);
        applyCredsAndConnect(CRED_STORED);
        return; // if this fails to connect, the default fires at 2 min
    }

    // Fallback 2: compiled-in default as a last resort.
    if (!defaultTried && elapsed >= DEFAULT_FALLBACK_MS) {
        defaultTried = true;
        strncpy(WIFI_SSID, DEFAULT_SSID, sizeof(WIFI_SSID) - 1);
        WIFI_SSID[sizeof(WIFI_SSID) - 1] = '\0';
        strncpy(WIFI_PASSWORD, DEFAULT_PASSWORD, sizeof(WIFI_PASSWORD) - 1);
        WIFI_PASSWORD[sizeof(WIFI_PASSWORD) - 1] = '\0';
        Serial.printf("No UART reply in %lus — using compiled default creds (ssid=%s)\n",
                      (unsigned long)(DEFAULT_FALLBACK_MS / 1000), WIFI_SSID);
        applyCredsAndConnect(CRED_DEFAULT);
    }
}
#endif

// LoRa control pins
#define PIN_LORA_CS 41
#define PIN_LORA_DIO1 39
#define PIN_LORA_RESET 42
#define PIN_LORA_BUSY 40
#define PIN_LORA_SCK 7
#define PIN_LORA_MISO 8
#define PIN_LORA_MOSI 9

// US 915 LoRa radio settings
static const float FREQUENCY_MHZ = 915.0;
static const float BANDWIDTH_KHZ = 125.0;
static const uint8_t SPREADING_FACTOR = 9;
static const uint8_t CODING_RATE = 7;
static const uint8_t SYNC_WORD = 0x34;
static const int8_t TX_POWER_DBM = 22; // SX1262 max
static const float TCXO_VOLTAGE = 1.8;

// Module(cs, irq/dio1, reset, busy)
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_BUSY);

// ── MAX30105 particle sensor (I2C) ──────────────────────────────────────────
// Wired on the XIAO ESP32S3 default I2C bus: SDA=GPIO5, SCL=GPIO6. INT (GPIO4)
// is the sensor's open-drain, active-low interrupt line; pulled up and read as a
// plain input for now (no ISR needed for basic FIFO polling).
#define PIN_MAX_SDA 5
#define PIN_MAX_SCL 6
#define PIN_MAX_INT 4

static MAX30105 particleSensor;
static bool maxReady = false;

// ── Smoke/particle detection ────────────────────────────────────────────────
// In a dark chamber the MAX30105 reads a low IR baseline in clean air; smoke
// particles scatter the LED light back onto the photodiode and push IR above
// that baseline. We keep a slow rolling baseline (clean air) and flag smoke on a
// sustained rise. RED/IR ratio gives a coarse particle-size cross-check to help
// separate real smoke from nuisance aerosols (steam/dust) later.
//
// Baseline persistence: the Wio has no SD and we avoid frequent SPIFFS writes,
// so we boot from a compiled default and self-seed from the first live reading.
// setMax30105Baseline() lets the main node inject a known-good clean-air baseline
// later (UART/SPIFFS); an injected value always wins over the auto-seed.
static const float    MAX_DEFAULT_IR_BASELINE = 450.0f; // fallback clean-air IR
static const float    MAX_BASELINE_EMA_ALPHA  = 0.01f;  // slow adaptation in clean air
static const uint32_t MAX_SMOKE_RISE_COUNTS   = 1500;   // IR rise above baseline = smoke
static const uint8_t  MAX_SMOKE_CONFIRM        = 3;      // consecutive reads to latch
static const uint16_t MAX_CAL_WARMUP_READS     = 30;     // clean-air reads to trust baseline

static float    irBaseline     = MAX_DEFAULT_IR_BASELINE;
static bool     baselineSeeded = false;   // true once auto-seeded or externally set
static uint8_t  baselineSource = 0;       // 0=unseeded, 1=auto-seed, 2=external
static uint8_t  smokeStreak    = 0;
static bool     smokeDetected  = false;
static bool     isCalibrated   = false;   // clean-air warmup done / baseline injected
static uint16_t calCleanReads  = 0;       // clean-air reads accumulated toward warmup
static float    lastSmokeIndex = 0.0f;    // exposed to buildTelemetryFrame()
static float    lastRedIrRatio = 0.0f;

// Inject a baseline obtained later (UART/SPIFFS). Marks it authoritative so the
// rolling EMA continues from it instead of the compiled default / auto-seed.
static void __attribute__((unused)) setMax30105Baseline(float ir)
{
    irBaseline = ir;
    baselineSeeded = true;
    baselineSource = 2;
    calCleanReads = MAX_CAL_WARMUP_READS;
    isCalibrated = true;
    Serial.printf("[MAX30105] baseline set to %.0f (external) — calibrated\n", ir);
}

// Bring up I2C and the MAX30105. Non-fatal: if the sensor is absent we log and
// keep running so the LoRa/OTA path is unaffected.
static void setupMax30105()
{
    pinMode(PIN_MAX_INT, INPUT_PULLUP);
    Wire.begin(PIN_MAX_SDA, PIN_MAX_SCL);

    Serial.println("[MAX30105] initializing on I2C (sda=5, scl=6)...");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { // 400 kHz
        Serial.println("[MAX30105] NOT found — check wiring/power (3.3V) and address 0x57");
        maxReady = false;
        return;
    }

    // Config tuned for smoke/particle scatter: RED + IR only (ledMode 2), 8x
    // hardware sample-averaging, full 411us pulse width for max sensitivity.
    // Smoke is read on IR; RED gives the RED/IR particle-size cross-check.
    particleSensor.setup(0x1F, 8, 2, 400, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);
    particleSensor.enableDIETEMPRDY(); // allow on-chip temperature reads

    Serial.printf("[MAX30105] init OK — partID=0x%02X rev=0x%02X\n",
                  particleSensor.readPartID(), particleSensor.getRevisionID());
    maxReady = true;
}

// Poll once per second: compute the rolling clean-air baseline, smoke index, and
// RED/IR ratio, latch a sustained rise as smoke, track calibration warmup, print
// locally, and report status to the main node over UART (sender role only).
static void serviceMax30105()
{
#if NODE_ROLE == 0
    // Heartbeat a sensor-down status to the (blind, enclosed) main node so a failed
    // init is still visible on Main's serial even with no Wio serial connection.
    if (!maxReady) {
        static uint32_t lastFailMs = 0;
        if (millis() - lastFailMs >= 10000) {
            lastFailMs = millis();
            WioSmokeStatus s;
            memset(&s, 0, sizeof(s));
            s.calTarget = MAX_CAL_WARMUP_READS;
            uartSendPacket(DeviceUART, WIO_STATUS, (uint8_t *)&s, sizeof(s));
        }
        return;
    }
#else
    if (!maxReady)
        return;
#endif

    static uint32_t nextReadMs = 0;
    if (nextReadMs != 0 && (int32_t)(millis() - nextReadMs) < 0)
        return;
    nextReadMs = millis() + 1000;

    uint32_t red = particleSensor.getRed();
    uint32_t ir  = particleSensor.getIR();

    // Self-seed the baseline from the first live reading unless an external value
    // was already injected — assumes clean air at boot.
    if (!baselineSeeded) {
        irBaseline = (float)ir;
        baselineSeeded = true;
        if (baselineSource == 0)
            baselineSource = 1; // auto-seeded from first live read
    }

    float smokeIndex = (float)ir - irBaseline;
    if (smokeIndex < 0.0f)
        smokeIndex = 0.0f;
    float redIrRatio = (ir > 0) ? (float)red / (float)ir : 0.0f;
    lastSmokeIndex = smokeIndex;
    lastRedIrRatio = redIrRatio;

    if (smokeIndex >= (float)MAX_SMOKE_RISE_COUNTS) {
        if (smokeStreak < 255)
            smokeStreak++;
    } else {
        smokeStreak = 0;
        // Only adapt the baseline in clean air so a real event is never absorbed.
        irBaseline += MAX_BASELINE_EMA_ALPHA * ((float)ir - irBaseline);
        // Clean-air reads accumulate toward the calibration warmup.
        if (!isCalibrated && calCleanReads < MAX_CAL_WARMUP_READS)
            calCleanReads++;
    }

    bool wasCalibrated = isCalibrated;
    if (!isCalibrated && calCleanReads >= MAX_CAL_WARMUP_READS)
        isCalibrated = true;

    bool nowDetected = (smokeStreak >= MAX_SMOKE_CONFIRM);
    bool smokeChanged = (nowDetected != smokeDetected);
    smokeDetected = nowDetected;

    float tempC = particleSensor.readTemperature();

    if (smokeChanged)
        Serial.printf("[MAX30105] *** SMOKE %s ***\n", smokeDetected ? "DETECTED" : "cleared");
    if (!wasCalibrated && isCalibrated)
        Serial.printf("[MAX30105] calibrated — baseline=%.0f\n", irBaseline);

    Serial.printf("[MAX30105] RED=%lu IR=%lu base=%.0f smokeIdx=%.0f R/IR=%.2f cal=%u temp=%.2fC%s\n",
                  (unsigned long)red, (unsigned long)ir, irBaseline, smokeIndex,
                  redIrRatio, (unsigned)isCalibrated, tempC, smokeDetected ? "  [SMOKE]" : "");

#if NODE_ROLE == 0
    // Report to the main node over UART. Steady 5s cadence for smoke-exposure data
    // capture (local wire, no LoRa airtime impact); plus immediate on any
    // calibration or smoke transition.
    static uint32_t lastStatusMs = 0;
    if (smokeChanged || (!wasCalibrated && isCalibrated) ||
        (millis() - lastStatusMs >= 5000)) {
        lastStatusMs = millis();
        WioSmokeStatus s;
        memset(&s, 0, sizeof(s));
        s.sensorOk       = 1;
        s.calibrated     = isCalibrated ? 1 : 0;
        s.smokeDetected  = smokeDetected ? 1 : 0;
        s.baselineSource = baselineSource;
        s.calProgress    = calCleanReads;
        s.calTarget      = MAX_CAL_WARMUP_READS;
        s.ir             = ir;
        s.red            = red;
        s.baseline       = irBaseline;
        s.smokeIndex     = smokeIndex;
        s.redIrRatio     = redIrRatio;
        s.tempC          = tempC;
        uartSendPacket(DeviceUART, WIO_STATUS, (uint8_t *)&s, sizeof(s));
    }
#endif
}

// ── ZE730-CO electrochemical CO sensor (UART) ───────────────────────────────
// Winsen CO module on the Wio's free UART2. Wiring (indoor model only):
//   ZE730 TXD -> Wio D7 / GPIO44 (RX)   ZE730 RXD -> Wio D6 / GPIO43 (TX)
//   VCC = 5V pin, GND = GND. Logic is 3V, compatible with the ESP32-S3.
// The module streams a 9-byte active-upload frame ~once per second at 9600 8N1:
//   [0]=0xFF start, [1]=gas, [2]=unit, [3]=decimals, [4..5]=concentration hi/lo,
//   [6..7]=full range hi/lo, [8]=checksum (two's complement of sum of [1..7]).
// Presence is self-detected: those frames only arrive if a sensor is wired
// (indoor models), so ze730Online gates the co= telemetry field — no indoor/
// outdoor flag needed, and a mid-life disconnect self-heals by dropping the field.
#define PIN_ZE730_RX 44   // D7  <- ZE730 TXD
#define PIN_ZE730_TX 43   // D6  -> ZE730 RXD
static HardwareSerial ZE730Serial(2);

static bool     ze730Ready     = false;   // set once a valid frame is decoded
static bool     ze730Online    = false;   // valid frame within the staleness window
static float    lastCoPpm      = -1.0f;    // -1 => no reading yet (frame sentinel)
static uint32_t lastCoMs       = 0;
static uint16_t ze730FullRange = 0;
static const uint32_t ZE730_STALE_MS = 8000; // ~8 missed 1Hz frames => sensor gone

// Bring up UART2 and nudge the module into active-upload mode. Some units ship
// in Q&A (silent-until-polled) mode; the 0x78/0x40 command forces streaming.
static void setupZE730()
{
    ZE730Serial.begin(9600, SERIAL_8N1, PIN_ZE730_RX, PIN_ZE730_TX);
    static const uint8_t setActive[9] = {0xFF,0x01,0x78,0x40,0x00,0x00,0x00,0x00,0x47};
    ZE730Serial.write(setActive, sizeof(setActive));
    Serial.println("[ZE730] UART init RX=GPIO44(D7) TX=GPIO43(D6) @9600; requested active-upload mode");
}

// Drain the ZE730 stream, sync on the 0xFF start byte, validate the checksum,
// and decode CO ppm. Dumps the first few raw frames so the exact on-wire format
// can be confirmed on the bench during bring-up.
static void serviceZE730()
{
    static uint8_t buf[9];
    static uint8_t idx = 0;
    static uint8_t dbgFrames = 0;

    while (ZE730Serial.available()) {
        uint8_t b = ZE730Serial.read();
        if (idx == 0) {
            if (b != 0xFF) continue;   // hunt for the start byte
            buf[0] = b;
            idx = 1;
            continue;
        }
        buf[idx++] = b;
        if (idx < 9)
            continue;
        idx = 0;

        if (dbgFrames < 5) {
            dbgFrames++;
            Serial.printf("[ZE730] raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]);
        }

        uint8_t sum = 0;
        for (uint8_t i = 1; i <= 7; i++) sum += buf[i];
        uint8_t cksum = (uint8_t)(~sum + 1);
        if (cksum != buf[8]) {
            Serial.printf("[ZE730] checksum mismatch (got %02X want %02X) — dropping\n", buf[8], cksum);
            continue;
        }

        uint8_t  decimals = buf[3];
        uint16_t raw      = ((uint16_t)buf[4] << 8) | buf[5];
        float scale = 1.0f;
        for (uint8_t i = 0; i < decimals; i++) scale *= 10.0f;
        float ppm = raw / scale;

        ze730FullRange = ((uint16_t)buf[6] << 8) | buf[7];
        lastCoPpm = ppm;
        lastCoMs  = millis();
        if (!ze730Ready) {
            ze730Ready = true;
            Serial.printf("[ZE730] first valid frame — CO=%.1f ppm (range=%u)\n", ppm, ze730FullRange);
        }
        // Log on any meaningful change, else heartbeat every 5s (matches MAX cadence).
        static float    lastLoggedPpm = -999.0f;
        static uint32_t lastLogMs     = 0;
        if (fabsf(ppm - lastLoggedPpm) >= 1.0f || millis() - lastLogMs >= 5000) {
            lastLoggedPpm = ppm;
            lastLogMs = millis();
            Serial.printf("[ZE730] CO=%.1f ppm (raw=%u dec=%u range=%u)\n", ppm, raw, decimals, ze730FullRange);
        }
    }

    // Runs every pass (even with no bytes) so a disconnect is noticed: the sensor
    // is "online" only while fresh frames keep arriving.
    bool present = ze730Ready && (millis() - lastCoMs < ZE730_STALE_MS);
    if (present != ze730Online) {
        ze730Online = present;
        Serial.printf("[ZE730] sensor %s\n", present ? "online" : "offline (no frames)");
    }
}

// XOR checksum (NMEA-style) over the frame body between '$' and '*'.
static uint8_t frameChecksum(const char *s)
{
    uint8_t c = 0;
    while (*s)
        c ^= (uint8_t)*s++;
    return c;
}

static void halt(const char *what, int code)
{
    Serial.printf("[FATAL] %s failed, code %d\n", what, code);
    while (true) {
        delay(1000);
    }
}

#if NODE_ROLE != 0
// Set by the DIO1 ISR when a full packet has been received.
static volatile bool rxReady = false;
static void IRAM_ATTR onRxDone()
{
    rxReady = true;
}
#endif

#if NODE_ROLE == 0
// Connect WiFi (blocking, best-effort) and start ElegantOTA. A failed connect is
// non-fatal: the node keeps transmitting LoRa telemetry and simply has no OTA
// until the next boot. Mirrors the AudioNode's sync ElegantOTA setup.
static void setupWifiAndOTA()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);       // keep radio on so inbound OTA connects are not missed
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        Serial.print(".");
        delay(250);
        attempts++;
    }

    otaWifiUp = (WiFi.status() == WL_CONNECTED);
    if (otaWifiUp) {
        static bool otaServiceStarted = false;  // start the OTA server only once
        Serial.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
        if (!otaServiceStarted) {
            otaServer.on("/", []() {
                otaServer.send(200, "text/plain", "LaresAI Wio Sender is running. OTA at /update");
            });
            ElegantOTA.begin(&otaServer);
            otaServer.begin();
            otaServiceStarted = true;
        }
        Serial.printf("ElegantOTA ready: http://%s/update\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connect timeout — continuing without OTA.");
    }
}
#endif

void setup()
{
    Serial.begin(115200);
    delay(1500);

    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

    int st = radio.begin(FREQUENCY_MHZ, BANDWIDTH_KHZ, SPREADING_FACTOR, CODING_RATE, SYNC_WORD, TX_POWER_DBM);
    if (st != RADIOLIB_ERR_NONE)
        halt("radio.begin", st);

    // Board-specific: TCXO on DIO3, RF switch on DIO2. Both are required here.
    radio.setTCXO(TCXO_VOLTAGE);
    radio.setDio2AsRfSwitch(true);

    setupMax30105();
    setupZE730();

#if NODE_ROLE == 0
    Serial.printf("Role: SENDER (node id %d)\n", NODE_ID);
    randomSeed(esp_random());
    DeviceUART.begin(115200, SERIAL_8N1, DEVICE_UART_RX, DEVICE_UART_TX);
    Serial.println("Device UART initialized on GPIO1/2");
    loadStoredCreds();                 // last-known creds for fast/offline fallback
    credWaitStartMs = millis();
    sendHello();                       // first nudge; loop() keeps retrying
    lastHelloMs = millis();
    Serial.println("Requesting WiFi credentials from main node over UART...");
    // WiFi comes up from loop() once a credential source is chosen (see
    // serviceCredentialLink): UART reply, else NVS at 20 s, else default at 2 min.
#else
    Serial.println("Role: RECEIVER / GATEWAY");
    radio.setDio1Action(onRxDone);
    st = radio.startReceive();
    if (st != RADIOLIB_ERR_NONE)
        halt("startReceive", st);
#endif
}

#if NODE_ROLE == 0 // ---------------- SENDER ----------------

// Build one telemetry frame into `out` from the latest values pushed by the main
// node over UART. Returns false until real telemetry has been received, so the
// caller can hold off transmitting instead of broadcasting empty data.
static bool buildTelemetryFrame(char *out, size_t outLen, uint32_t seq)
{
    if (!haveTelemetry)
        return false;

    const WioTelemetry &t = latestTelemetry;
    // MAX30105 optical smoke as an independent second opinion; only trusted once
    // the sensor has calibrated, then OR'd into the BME-derived smoke alarm.
    unsigned maxSmoke = (isCalibrated && smokeDetected) ? 1u : 0u;
    unsigned combinedSmoke = ((unsigned)t.alarmSmoke || maxSmoke) ? 1u : 0u;
    char body[256];
    int n = snprintf(body, sizeof(body),
             "LARES,id=%ld,seq=%lu,typ=SENSOR,zone=%ld,temp=%.1f,hum=%.1f,"
             "airq=%.0f,gasq=%.0f,smoke=%u,fire=%u,gas=%u,pump=%u,cls=%s,conf=%.2f,"
             "msmoke=%u,smokeIdx=%.0f,base=%.0f,ratio=%.2f,cal=%u",
             (long)t.cameraId, (unsigned long)seq,
             (long)t.zoneId,
             t.temperatureF, t.humidity, t.airQ, t.gasQ,
             combinedSmoke, (unsigned)t.alarmFire, (unsigned)t.alarmGas,
             (unsigned)t.pumpOn, t.classification, t.confidence,
             maxSmoke, lastSmokeIndex, irBaseline, lastRedIrRatio, (unsigned)isCalibrated);
    // Fixed schema: co= is always present so RX/Main never handle a missing field.
    // -1 = no sensor / no data (0.0 is a real clean-air reading, so it must not
    // double as "absent"); any value >= 0 is a genuine ppm measurement.
    float coField = ze730Online ? lastCoPpm : -1.0f;
    if (n > 0 && n < (int)sizeof(body))
        snprintf(body + n, sizeof(body) - n, ",co=%.1f", coField);
    snprintf(out, outLen, "$%s*%02X", body, frameChecksum(body));
    return true;
}

void loop()
{
    static uint32_t seq = 0;
    static uint32_t nextTxMs = 0; // 0 => transmit immediately on first pass

    // Keep the credential handshake alive every pass (non-blocking): retry HELLO,
    // accept/persist UART creds, and apply fallbacks on their timers.
    serviceCredentialLink();

    serviceMax30105();
    serviceZE730();

    // Service OTA every loop so an update can land in the gap between transmits.
    if (otaWifiUp) {
        otaServer.handleClient();
        ElegantOTA.loop();
    }

    // Non-blocking cadence: wait until the scheduled transmit time without a long
    // delay() that would starve the OTA web server.
    if (nextTxMs != 0 && (int32_t)(millis() - nextTxMs) < 0)
        return;

    char frame[288];
    if (!buildTelemetryFrame(frame, sizeof(frame), seq)) {
        // No telemetry from the main node yet — retry shortly instead of
        // broadcasting stale/empty data.
        nextTxMs = millis() + 2000;
        return;
    }

    Serial.printf("TX #%lu: %s ... ", (unsigned long)seq, frame);
    int st = radio.transmit(frame);
    Serial.println(st == RADIOLIB_ERR_NONE ? "ok" : "ERR");
    if (st != RADIOLIB_ERR_NONE)
        Serial.printf("  transmit err %d\n", st);

    seq++;

    // ~20-30s with jitter so a fleet of senders never fires in lockstep.
    nextTxMs = millis() + 20000 + random(0, 10000);
}

#else // ---------------- RECEIVER / GATEWAY ----------------

// Validate "$<body>*HH" in place: confirm the '$' prefix and matching XOR
// checksum, then strip "*HH" so `buf` holds just the body. Returns false for
// foreign or corrupted frames (this gateway hears every device in range).
static bool validateFrame(char *buf)
{
    if (buf[0] != '$')
        return false;
    char *star = strrchr(buf, '*');
    if (!star || star[1] == '\0' || star[2] == '\0')
        return false;
    *star = '\0';
    uint8_t want = (uint8_t)strtol(star + 1, nullptr, 16);
    return want == frameChecksum(buf + 1);
}

void loop()
{
    serviceMax30105();
    serviceZE730();

    // Wait for the DIO1 ISR to signal a fully received packet.
    if (!rxReady)
        return;
    rxReady = false;

    String str;
    int st = radio.readData(str);

    if (st == RADIOLIB_ERR_NONE && str.length() > 0) {
        float rssi = radio.getRSSI();
        float snr = radio.getSNR();

        char buf[256];
        strncpy(buf, str.c_str(), sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        if (validateFrame(buf)) {
            // One line per packet: RSSI/SNR followed by the comma-separated body.
            Serial.printf("RX  RSSI=%.1f dBm  SNR=%.1f dB  %s\n", rssi, snr, buf + 1);
        } else {
            Serial.printf("RX  [dropped: not a LARES frame] \"%s\"  RSSI=%.1f dBm\n", str.c_str(), rssi);
        }
    } else if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("RX err %d\n", st);
    }

    radio.startReceive(); // re-arm for the next packet
}

#endif
