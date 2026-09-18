// Minimal one-way LoRa test for the Seeed Wio-SX1262 + XIAO ESP32S3 kit (US 915 MHz).
// Pin map taken from the Seeed variant: CS=41, DIO1=39, RESET=42, BUSY=40,
// SPI SCK=7 MISO=8 MOSI=9. DIO3 drives the TCXO (1.8V), DIO2 is the RF switch.
//
// NODE_ROLE 0 = sender (broadcasts structured telemetry every ~20-30s)
// NODE_ROLE 1 = receiver/gateway (validates + prints each frame with RSSI/SNR)
//
// Over-the-air frame (NMEA-style, kept compact for LoRa):
//   $LARES,id=1,seq=42,typ=SIM,temp=24.3,hum=41.0,gas=0,co=0,batt=3.91*A7
// where *A7 is an XOR checksum of everything between '$' and '*'. The id field
// makes the many-devices -> few-gateways model work; (id,seq) lets the cloud
// dedupe frames heard by overlapping gateways.

#include <RadioLib.h>

#ifndef NODE_ROLE
#define NODE_ROLE 0 // default to sender if not set by build flag
#endif

#ifndef NODE_ID
#define NODE_ID 1 // unique per transmitting device; set via build flag
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

#if NODE_ROLE == 0
    Serial.printf("Role: SENDER (node id %d)\n", NODE_ID);
    randomSeed(esp_random());
#else
    Serial.println("Role: RECEIVER / GATEWAY");
    radio.setDio1Action(onRxDone);
    st = radio.startReceive();
    if (st != RADIOLIB_ERR_NONE)
        halt("startReceive", st);
#endif
}

#if NODE_ROLE == 0 // ---------------- SENDER ----------------

// Build one telemetry frame into `out`. Fields are fake for now; step 2 will
// replace this body with a line read from the main node over UART.
static void buildFakeFrame(char *out, size_t outLen, uint32_t seq)
{
    float temp = 22.0f + (seq % 50) * 0.1f; // drifts 22.0 .. 26.9
    float hum = 40.0f + (seq % 20) * 0.5f;  // drifts 40.0 .. 49.5
    int gas = (seq % 17 == 0) ? 1 : 0;      // occasional gas alarm
    int co = (seq % 29 == 0) ? 1 : 0;       // occasional CO alarm
    float batt = 3.70f + (seq % 30) * 0.01f;

    char body[96];
    snprintf(body, sizeof(body),
             "LARES,id=%d,seq=%lu,typ=SIM,temp=%.1f,hum=%.1f,gas=%d,co=%d,batt=%.2f",
             NODE_ID, (unsigned long)seq, temp, hum, gas, co, batt);
    snprintf(out, outLen, "$%s*%02X", body, frameChecksum(body));
}

void loop()
{
    static uint32_t seq = 0;
    char frame[112];
    buildFakeFrame(frame, sizeof(frame), seq);

    Serial.printf("TX #%lu: %s ... ", (unsigned long)seq, frame);
    int st = radio.transmit(frame);
    Serial.println(st == RADIOLIB_ERR_NONE ? "ok" : "ERR");
    if (st != RADIOLIB_ERR_NONE)
        Serial.printf("  transmit err %d\n", st);

    seq++;

    // ~20-30s with jitter so a fleet of senders never fires in lockstep.
    delay(20000 + random(0, 10000));
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
    // Wait for the DIO1 ISR to signal a fully received packet.
    if (!rxReady)
        return;
    rxReady = false;

    String str;
    int st = radio.readData(str);

    if (st == RADIOLIB_ERR_NONE && str.length() > 0) {
        float rssi = radio.getRSSI();
        float snr = radio.getSNR();

        char buf[128];
        strncpy(buf, str.c_str(), sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        if (validateFrame(buf)) {
            Serial.printf("RX  RSSI=%.1f dBm  SNR=%.1f dB\n", rssi, snr);
            for (char *tok = strtok(buf + 1, ","); tok; tok = strtok(nullptr, ","))
                Serial.printf("    %s\n", tok);
        } else {
            Serial.printf("RX  [dropped: not a LARES frame] \"%s\"  RSSI=%.1f dBm\n", str.c_str(), rssi);
        }
    } else if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("RX err %d\n", st);
    }

    radio.startReceive(); // re-arm for the next packet
}

#endif
