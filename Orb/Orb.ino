#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_system.h"

// ==================== CONFIG ====================
#define NODE_ID 11                // CHANGE per orb: 1..14
static const uint8_t CHANNEL = 1; // MUST MATCH HUB
// IR output + Vibration input
// If your hardware is wired to different GPIOs, change the values here.
static const uint8_t PIN_IR_LED = 7;   // IR LED output pin (GPIO7)
static const uint8_t PIN_VIB    = 4;   // vibration switch -> GND when tapped (GPIO4)

// Debug / bring-up helpers
static const bool     IR_TEST_ON_BOOT = false;   // set true to send one IR code during setup()
static const uint32_t IR_TEST_DELAY_MS = 400;    // delay before boot test send

// Reliability
static const uint32_t RETRY_BASE_MS = 180;
static const uint8_t  RETRY_MAX = 6;

// Timing
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t HELLO_INTERVAL_MS  = 1200; // discovery
static const uint32_t STATUS_INTERVAL_MS = 1200; // status cadence

// Vibration tap detection
static const uint32_t VIB_DEBOUNCE_MS   = 30;
static const uint32_t TAP_WINDOW_MS     = 300;
static const uint8_t  TAP_HITS_REQUIRED = 1;
static const uint32_t TAP_COOLDOWN_MS   = 600;
static const bool     DEBUG_VIB         = true;
static const bool     VIB_ACTIVE_LOW    = true; // true if switch pulls to GND when tapped

// ==================== PROTOCOL ====================
enum MsgType : uint8_t {
  MSG_HELLO     = 1,
  MSG_STATUS    = 2,   // value = reportedColor (1-15)
  MSG_PING      = 3,
  MSG_PONG      = 4,
  MSG_SET       = 5,   // value = set color (1-15)
  MSG_ACK       = 6,
  MSG_LIGHT     = 7,   // value = 1 if pink, 0 otherwise (ONLY on pink transitions)
  MSG_CONSENSUS = 8,   // value = 1 if all 14 orbs are pink, 0 otherwise
  MSG_IR_FUNC   = 9,   // value = 1:ON, 2:OFF, 3:FLASH, 4:STROBE
  MSG_CYCLE_CFG = 10   // value packed as: [count:4][idx0:4][idx1:4][idx2:4][idx3:4][idx4:4][idx5:4]
};

#pragma pack(push, 1)
struct Packet {
  uint8_t  type;
  uint8_t  node_id;
  uint8_t  ack_type;
  uint8_t  flags;
  uint32_t seq;
  uint32_t ack_seq;
  int32_t  value;
};
#pragma pack(pop)

// ==================== STATE ====================
uint32_t orb_seq = 1;

uint8_t NODE_STA_MAC[6] = {0};

bool    hubKnown = false;
uint8_t hubMac[6] = {0};

struct PendingTx {
  bool     active = false;
  uint8_t  type = 0;
  uint32_t seq = 0;
  int32_t  value = 0;
  uint8_t  tries = 0;
  uint32_t next_send_ms = 0;
} pending;

uint32_t lastHelloMs = 0;
uint32_t lastStatusMs = 0;

// This is the "color:<reported color>" number (1-15)
int32_t reportedColor = 1;

// ==================== IR CODES (Flipper-decoded NECext) ====================
// Source: Flipper .ir export (codes.txt)
// Protocol: NECext
// Address:  0xEF00 (16-bit)
// Command:  16-bit per button

struct LightColor {
  const char* name;
  uint16_t addr;
  uint16_t cmd;
  bool isPink;
};

static const uint16_t IR_ADDR = 0xEF00;
static const LightColor LIGHT_FUNC[] = {
  {"ON",     IR_ADDR, 0xFC03, false},
  {"OFF",    IR_ADDR, 0xFD02, false},
  {"FLASH",  IR_ADDR, 0xF40B, false},
  {"STROBE", IR_ADDR, 0xF00F, false},
};
static const LightColor LIGHT_COLORS[] = {
  {"RED",    IR_ADDR, 0xFB04, false},
  {"ORED",   IR_ADDR, 0xF30C, false},
  {"GREEN",  IR_ADDR, 0xFA05, false},
  {"LGREEN", IR_ADDR, 0xF609, false},
  {"BLUE",   IR_ADDR, 0xF906, false},
  {"ORANGE", IR_ADDR, 0xF708, false},
  //{"YELLOW", IR_ADDR, 0xEB14, false},
  {"OYELLOW", IR_ADDR, 0xEF10, false},
  {"AQUA",   IR_ADDR, 0xEE11, false},
  //{"PURPLE", IR_ADDR, 0xF10E, false},
  //{"APURPLE", IR_ADDR, 0xED12, false},
  {"PINK",   IR_ADDR, 0xE916, true}
};
static const uint8_t LIGHT_COLOR_COUNT = sizeof(LIGHT_COLORS) / sizeof(LIGHT_COLORS[0]);
static const uint8_t PINK_LIGHT_INDEX = LIGHT_COLOR_COUNT - 1;
static const uint8_t MAX_CYCLE_COLORS = 7; // up to 6 configured non-pink colors + PINK auto-appended
static const uint32_t PINK_HOLD_MS = 3000;

// Default cycle is exactly 3 colors, then pink auto-appended:
// RED, GREEN, BLUE, PINK
static const uint8_t DEFAULT_CYCLE_NONPINK[3] = {0, 2, 4};

static uint8_t colorsInUse[MAX_CYCLE_COLORS] = {0};
static uint8_t colorsInUseCount = 0;
static uint8_t currentCyclePos = 0; // index into colorsInUse[]
static bool lastIsPink = true;

// STROBE override
static bool strobeActive = false;
static uint32_t strobeEndMs = 0;
static int32_t serverConsensusCmd = 0; // server-driven consensus command (0 idle, 1 trigger strobe)

enum PinkPauseState : uint8_t {
  PINK_PAUSE_IDLE = 0,
  PINK_PAUSE_SEND_BEFORE,
  PINK_PAUSE_HOLD,
  PINK_PAUSE_SEND_AFTER
};
static PinkPauseState pinkPauseState = PINK_PAUSE_IDLE;
static uint32_t pinkPauseUntilMs = 0;
static uint8_t pinkBeforeSendsRemaining = 0;
static uint8_t pinkAfterSendsRemaining = 0;
static uint8_t queuedLightCount = 0;
// ==================== IR SENDER (LEDC hardware 38kHz, NEC/NECext) ====================
// Bit-banging at ~38kHz using digitalWrite can be too slow/inconsistent on some ESP32 variants
// (especially when Wi-Fi/ESP-NOW is active). Use the LEDC hardware PWM for a stable carrier.

static const uint16_t IR_HDR_MARK_US   = 9000;
static const uint16_t IR_HDR_SPACE_US  = 4500;
static const uint16_t IR_BIT_MARK_US   = 560;
static const uint16_t IR_ONE_SPACE_US  = 1690;
static const uint16_t IR_ZERO_SPACE_US = 560;
static const uint16_t IR_TRAILER_MARK_US = 560;

static const uint32_t IR_CARRIER_HZ = 38000;

// LEDC on Arduino-ESP32 core 3.x (your installed core is 3.3.6) uses:
//   ledcAttach(pin, freq, resolutionBits)
//   ledcWrite(pin, duty)
// The old ledcSetup/ledcAttachPin APIs are not available in 3.x.
static const uint8_t  IR_LEDC_BITS = 8;   // duty resolution
static const uint16_t IR_DUTY_ON   = 128; // ~50% duty for 8-bit
static int8_t         IR_LEDC_CH   = -1;  // channel assigned by ledcAttach (kept for reference)

// Arduino-ESP32 core 3.x: ledcWrite() is pin-based (not channel-based).
// Using the channel here can silently write the wrong GPIO (e.g., channel=1 writes GPIO1),
// resulting in no IR output even though ledcAttach() succeeds.
static inline void irCarrierOn()  { if (IR_LEDC_CH >= 0) ledcWrite(PIN_IR_LED, IR_DUTY_ON); }
static inline void irCarrierOff() { if (IR_LEDC_CH >= 0) ledcWrite(PIN_IR_LED, 0); }

static inline void irSpace(uint32_t us) {
  irCarrierOff();
  if (us) delayMicroseconds(us);
}

static inline void irMark(uint32_t us) {
  // Enable 38kHz carrier for the specified duration.
  irCarrierOn();
  if (us) delayMicroseconds(us);
  irCarrierOff();
}

// Raw timing sender: timings are alternating MARK, SPACE, MARK, SPACE... in microseconds.
static void irSendRawTimings(const uint16_t* timings_us, size_t count) {
  // Expect count >= 2 and starts with MARK.
  for (size_t i = 0; i < count; i++) {
    uint16_t d = timings_us[i];
    if ((i & 1) == 0) {
      irMark(d);
    } else {
      irSpace(d);
    }
  }
  // Short gap after frame
  delayMicroseconds(5000);
}

static void irSendRawNECext(uint16_t addr, uint16_t cmd) {
  // Build a raw MARK/SPACE timing buffer for NECext and send it.
  // This avoids any protocol-library/timer behavior differences and uses only raw timings.

  // NEC frame: header (2), 32 bits (64), trailer mark (1) => 67 timing entries
  uint16_t t[67];
  size_t k = 0;

  // Header
  t[k++] = IR_HDR_MARK_US;
  t[k++] = IR_HDR_SPACE_US;

  // LSB-first 32-bit frame = cmd(16) then addr(16)
  uint32_t frame = ((uint32_t)cmd << 16) | (uint32_t)addr;
  for (uint8_t i = 0; i < 32; i++) {
    bool bit = (frame >> i) & 0x1;
    t[k++] = IR_BIT_MARK_US;
    t[k++] = bit ? IR_ONE_SPACE_US : IR_ZERO_SPACE_US;
  }

  // Trailer
  t[k++] = IR_TRAILER_MARK_US;

  irSendRawTimings(t, k);
}

// Back-compat name: this function already emits RAW timings; keep the old name to avoid confusion.
static inline void irSendNECext(uint16_t addr, uint16_t cmd) {
  irSendRawNECext(addr, cmd);
}

static void irInitLedc() {
  // Configure hardware PWM carrier on the IR pin.
  // core 3.x API
  IR_LEDC_CH = ledcAttach(PIN_IR_LED, IR_CARRIER_HZ, IR_LEDC_BITS);
  Serial.print("LEDC attach: pin=");
  Serial.print(PIN_IR_LED);
  Serial.print(" ch=");
  Serial.println(IR_LEDC_CH);
  if (IR_LEDC_CH < 0) {
    Serial.println("LEDC attach failed; IR carrier disabled");
    return;
  }
  Serial.println("IR carrier self-test (120ms)");
  irCarrierOff();

  // Self-test burst: hold a 38kHz carrier long enough that a phone camera will reliably see it.
  irCarrierOn();
  delay(120);
  irCarrierOff();
}

// Vibration state
static uint32_t lastVibChangeMs = 0;
static bool lastVibStable = HIGH; // pullup: HIGH=open, LOW=closed
static uint32_t tapWindowStartMs = 0;
static uint8_t tapHits = 0;
static uint32_t lastTapMs = 0;
static uint32_t lastVibLogMs = 0;
static uint32_t lastHeartbeatMs = 0;

static bool    lightQueued = false;
static int32_t lightQueuedValue = 0;

// ==================== HELPERS ====================
static bool reliableSendToHub(uint8_t type, int32_t value);
static bool sendLightPulseToHub(bool isPink);
static bool trySendLightPulseNow(bool isPink);
static bool cycleIndexExists(uint8_t idx);
static void applyCycleConfigPacked(uint32_t packed);
static void initDefaultCycle();

static void printMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    if (i) Serial.print(":");
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
  }
}

static void readStaMac(uint8_t out[6]) {
  esp_read_mac(out, ESP_MAC_WIFI_STA);
}

static uint8_t getChannelNow() {
  uint8_t ch = 0;
  wifi_second_chan_t ch2;
  esp_wifi_get_channel(&ch, &ch2);
  return ch;
}

// ---- C3 FIX: force Wi-Fi driver init if Arduino didn't ----
static void force_wifi_init_and_start_if_needed() {
  esp_err_t e = esp_wifi_start();
  if (e == ESP_OK) return;

  if (e == ESP_ERR_WIFI_NOT_INIT) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_start();
  }
}

static void force_channel(uint8_t channel) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  delay(20);
}

static void wifi_init_for_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(50);

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  force_wifi_init_and_start_if_needed();
  force_channel(CHANNEL);

  readStaMac(NODE_STA_MAC);

  Serial.print("Orb "); Serial.print(NODE_ID);
  Serial.print(" | STA MAC: ");
  printMac(NODE_STA_MAC);
  Serial.print(" | Channel: ");
  Serial.println(getChannelNow());
}

static bool ensurePeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;

  return esp_now_add_peer(&peer) == ESP_OK;
}

static esp_err_t rawSend(const uint8_t* mac, const Packet& p) {
  return esp_now_send(mac, (const uint8_t*)&p, sizeof(p));
}

static void sendAckTo(const uint8_t* mac, uint8_t ack_type, uint32_t ack_seq) {
  Packet a{};
  a.type = MSG_ACK;
  a.node_id = NODE_ID;
  a.ack_type = ack_type;
  a.flags = 0;
  a.seq = orb_seq++;
  a.ack_seq = ack_seq;
  a.value = 0;
  rawSend(mac, a);
}

static void sendLightStateToHub(bool isPink) {
  sendLightPulseToHub(isPink);
}

static bool cycleIndexExists(uint8_t idx) {
  for (uint8_t i = 0; i < colorsInUseCount; i++) {
    if (colorsInUse[i] == idx) return true;
  }
  return false;
}

static void initDefaultCycle() {
  colorsInUseCount = 0;
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t idx = DEFAULT_CYCLE_NONPINK[i];
    if (idx >= PINK_LIGHT_INDEX) continue;
    colorsInUse[colorsInUseCount++] = idx;
  }
  colorsInUse[colorsInUseCount++] = PINK_LIGHT_INDEX;
  currentCyclePos = colorsInUseCount - 1;
  lastIsPink = true;
  reportedColor = (int32_t)PINK_LIGHT_INDEX + 1;
}

static void applyCycleConfigPacked(uint32_t packed) {
  uint8_t count = (uint8_t)((packed >> 24) & 0x0F);
  if (count > 6) count = 6;

  uint8_t newCycle[MAX_CYCLE_COLORS];
  uint8_t newCount = 0;

  for (uint8_t i = 0; i < count; i++) {
    uint8_t idx = (uint8_t)((packed >> (i * 4)) & 0x0F);
    if (idx >= PINK_LIGHT_INDEX) continue;
    if (newCount > 0 && newCycle[newCount - 1] == idx) continue;
    newCycle[newCount++] = idx;
  }

  // Keep at least one non-pink color.
  if (newCount == 0) {
    initDefaultCycle();
    return;
  }

  for (uint8_t i = 0; i < newCount; i++) colorsInUse[i] = newCycle[i];
  colorsInUseCount = newCount;
  colorsInUse[colorsInUseCount++] = PINK_LIGHT_INDEX; // always force pink as last

  if (!cycleIndexExists((uint8_t)(reportedColor - 1))) {
    currentCyclePos = colorsInUseCount - 1; // reset to pink anchor
    lastIsPink = true;
    reportedColor = (int32_t)PINK_LIGHT_INDEX + 1;
  } else {
    for (uint8_t i = 0; i < colorsInUseCount; i++) {
      if (colorsInUse[i] == (uint8_t)(reportedColor - 1)) {
        currentCyclePos = i;
        break;
      }
    }
    lastIsPink = (colorsInUse[currentCyclePos] == PINK_LIGHT_INDEX);
  }

  Serial.print("Cycle updated (count=");
  Serial.print(colorsInUseCount);
  Serial.print("): ");
  for (uint8_t i = 0; i < colorsInUseCount; i++) {
    Serial.print(LIGHT_COLORS[colorsInUse[i]].name);
    if (i + 1 < colorsInUseCount) Serial.print(",");
  }
  Serial.println();
}

static bool sendLightPulseToHub(bool isPink) {
  int32_t v = isPink ? 1 : 0;
  if (reliableSendToHub(MSG_LIGHT, v)) return true;
  if (queuedLightCount < 10) queuedLightCount++;
  lightQueued = true;
  lightQueuedValue = v;
  return false;
}

static bool trySendLightPulseNow(bool isPink) {
  int32_t v = isPink ? 1 : 0;
  return reliableSendToHub(MSG_LIGHT, v);
}
static void sendStrobeFor3Minutes() {
  const LightColor& fn = LIGHT_FUNC[3]; // STROBE
  Serial.println("CONSENSUS reached -> STROBE for 3 minutes");
  Serial.print("IR RAW send -> ");
  Serial.println(fn.name);
  for (int i = 0; i < 3; i++) {
    irSendRawNECext(fn.addr, fn.cmd);
    delay(20);
  }
  Serial.println("IR RAW send done");

  strobeActive = true;
  strobeEndMs = millis() + 180000UL; // 3 minutes
}

static void resendCurrentColorIR() {
  uint8_t idx = colorsInUse[currentCyclePos];
  const LightColor& lc = LIGHT_COLORS[idx];
  Serial.print("STROBE end -> restore color: ");
  Serial.println(lc.name);
  for (int i = 0; i < 3; i++) {
    irSendRawNECext(lc.addr, lc.cmd);
    delay(20);
  }
}

static void sendIrFunctionByCode(int32_t code) {
  // 1:ON, 2:OFF, 3:FLASH, 4:STROBE
  if (code < 1 || code > 4) return;
  const LightColor& fn = LIGHT_FUNC[code - 1];
  Serial.print("IR RAW send (hub cmd) -> ");
  Serial.println(fn.name);
  for (int i = 0; i < 3; i++) {
    irSendRawNECext(fn.addr, fn.cmd);
    delay(20);
  }
}

static void sendNextLightColor() {
  if (colorsInUseCount == 0) return;
  if (pinkPauseState != PINK_PAUSE_IDLE) {
    Serial.println("Tap ignored: pink hold active");
    return;
  }

  currentCyclePos = (uint8_t)((currentCyclePos + 1) % colorsInUseCount);
  uint8_t colorIdx = colorsInUse[currentCyclePos];
  const LightColor& lc = LIGHT_COLORS[colorIdx];

  // Map the chosen color to a 1..15 value for the hub (use index+1 for now)
  // If the hub expects a different mapping, adjust here.
  reportedColor = (int32_t)colorIdx + 1;

  Serial.print("Color selected -> ");
  Serial.print(lc.name);
  Serial.print(" (reportedColor=");
  Serial.println(reportedColor);
  // Transmit the IR code for this color as RAW mark/space timings (38kHz carrier via LEDC).
  // This is NOT a protocol-library send; we generate a timing buffer and play it out.
  Serial.print("IR RAW send -> ");
  Serial.println(lc.name);
  for (int i = 0; i < 3; i++) {
    irSendRawNECext(lc.addr, lc.cmd);
    delay(20);
  }
  Serial.println("IR RAW send done");
  delay(1);

  // Send the color to the hub (reliable, ACKed).
  // This is the main thing you asked for: send the color, not flash the light.
  if (!reliableSendToHub(MSG_STATUS, reportedColor)) {
    // If we can't send right now (no hub yet / pending busy), it will go out on next status tick.
    if (DEBUG_VIB) Serial.println("STATUS send deferred (hub unknown or pending busy)");
  }

  // Also send the simplified light state (pink=1/other=0) on every color change.
  // This is useful for your panel trigger logic.
  //sendLightStateToHub(lc.isPink);
    // Only send when transitioning into or out of PINK.
  if (lc.isPink != lastIsPink) {
    lastIsPink = lc.isPink;

    if (lc.isPink) {
      // Requirement:
      // 1) send pink active twice BEFORE pause
      // 2) hold 3 seconds
      // 3) send pink active twice AFTER pause
      pinkBeforeSendsRemaining = 2;
      pinkAfterSendsRemaining = 2;
      pinkPauseState = PINK_PAUSE_SEND_BEFORE;
    } else {
      sendLightPulseToHub(false);
    }
  }
}

static void pinkPauseTick() {
  if (pinkPauseState == PINK_PAUSE_IDLE) return;

  if (pinkPauseState == PINK_PAUSE_SEND_BEFORE) {
    if (pinkBeforeSendsRemaining == 0) {
      pinkPauseUntilMs = millis() + PINK_HOLD_MS;
      pinkPauseState = PINK_PAUSE_HOLD;
      Serial.println("Pink hold started (3000ms)");
      return;
    }
    // Always consume two pre-hold send attempts so hold timing never stretches.
    if (!pending.active) {
      (void)trySendLightPulseNow(true);
    }
    pinkBeforeSendsRemaining--;
    Serial.print("Pink pre-hold send remaining=");
    Serial.println(pinkBeforeSendsRemaining);
    return;
  }

  if (pinkPauseState == PINK_PAUSE_HOLD) {
    if ((int32_t)(millis() - pinkPauseUntilMs) < 0) return;
    pinkPauseState = PINK_PAUSE_SEND_AFTER;
    Serial.println("Pink hold ended; sending post-hold pink pulses");
    return;
  }

  if (pinkPauseState == PINK_PAUSE_SEND_AFTER) {
    if (pinkAfterSendsRemaining == 0) {
      pinkPauseState = PINK_PAUSE_IDLE;
      Serial.println("Pink pause sequence complete");
      return;
    }
    // Always consume two post-hold send attempts so lock duration stays 3 seconds.
    if (!pending.active) {
      (void)trySendLightPulseNow(true);
    }
    pinkAfterSendsRemaining--;
    Serial.print("Pink post-hold send remaining=");
    Serial.println(pinkAfterSendsRemaining);
  }
}

// ==================== RELIABLE SEND (one outstanding) ====================
static bool reliableSendToHub(uint8_t type, int32_t value) {
  if (!hubKnown) return false;
  if (pending.active) return false;

  ensurePeer(hubMac);

  Packet p{};
  p.type = type;
  p.node_id = NODE_ID;
  p.ack_type = 0;
  p.flags = 0;
  p.seq = orb_seq++;
  p.ack_seq = 0;
  p.value = value;

  esp_err_t e = rawSend(hubMac, p);
  if (e != ESP_OK) return false;

  pending.active = true;
  pending.type = type;
  pending.seq = p.seq;
  pending.value = value;
  pending.tries = 0;
  pending.next_send_ms = millis() + RETRY_BASE_MS;
  return true;
}

static void retryTick() {
  if (!hubKnown) return;
  if (!pending.active) return;

  uint32_t now = millis();
  if (now < pending.next_send_ms) return;

  if (pending.tries >= RETRY_MAX) {
    pending.active = false;
    return;
  }

  Packet p{};
  p.type = pending.type;
  p.node_id = NODE_ID;
  p.ack_type = 0;
  p.flags = 0;
  p.seq = pending.seq;   // resend same seq
  p.ack_seq = 0;
  p.value = pending.value;

  rawSend(hubMac, p);

  pending.tries++;
  uint32_t backoff = RETRY_BASE_MS << (pending.tries > 4 ? 4 : pending.tries);
  pending.next_send_ms = now + backoff;
}

// ==================== DISCOVERY (broadcast HELLO) ====================
static void broadcastHello() {
  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  ensurePeer(bcast);

  Packet p{};
  p.type = MSG_HELLO;
  p.node_id = NODE_ID;
  p.ack_type = 0;
  p.flags = 0;
  p.seq = orb_seq++;
  p.ack_seq = 0;
  p.value = 0;

  rawSend(bcast, p);
}

// ==================== ESP-NOW callbacks (core v3.x signatures) ====================
void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(Packet)) return;

  Packet p{};
  memcpy(&p, data, sizeof(p));

  uint8_t src_mac[6];
  memcpy(src_mac, info->src_addr, 6);

  // Learn hub from ACK(HELLO)
  if (!hubKnown && p.type == MSG_ACK && p.ack_type == MSG_HELLO) {
    memcpy(hubMac, src_mac, 6);
    hubKnown = true;
    ensurePeer(hubMac);
    // Clear any pending (shouldn't have any yet)
    pending.active = false;
    return;
  }

  // If hub known, ignore non-hub packets
  if (hubKnown) {
    bool fromHub = true;
    for (int i = 0; i < 6; i++) if (hubMac[i] != src_mac[i]) { fromHub = false; break; }
    if (!fromHub) return;
  }

  if (p.type == MSG_ACK) {
    if (pending.active && p.ack_seq == pending.seq && p.ack_type == pending.type) {
      pending.active = false;
    }
    return;
  }

  // ACK any non-ACK message
  sendAckTo(src_mac, p.type, p.seq);

  if (p.type == MSG_PING) {
    reliableSendToHub(MSG_PONG, p.value);
  } else if (p.type == MSG_SET) {
    // Hub sets our reported color (1-15)
    reportedColor = p.value;
    if (reportedColor < 1) reportedColor = 1;
    if (reportedColor > 15) reportedColor = 15;

    // Immediately report
    reliableSendToHub(MSG_STATUS, reportedColor);
  } else if (p.type == MSG_IR_FUNC) {
    sendIrFunctionByCode(p.value);
  } else if (p.type == MSG_CYCLE_CFG) {
    applyCycleConfigPacked((uint32_t)p.value);
  } else if (p.type == MSG_CONSENSUS) {
    // Server is the source of truth for consensus.
    // Keep local state at 0 unless server explicitly sends 1.
    serverConsensusCmd = (p.value == 1) ? 1 : 0;
    if (serverConsensusCmd == 1 && !strobeActive) {
      sendStrobeFor3Minutes();
      // One-shot trigger; return to idle until server sends a new command.
      serverConsensusCmd = 0;
    }
  }
}

static void handleVibration() {
  uint32_t now = millis();

  //bool vibRaw = digitalRead(PIN_VIB);
  //bool vibActive = VIB_ACTIVE_LOW ? (vibRaw == LOW) : (vibRaw == HIGH);
  bool vibRaw = digitalRead(PIN_VIB);
  bool vibActive = false; // computed after debounce from lastVibStable
  if (DEBUG_VIB && (now - lastVibLogMs >= 500)) {
    lastVibLogMs = now;
    //Serial.print("VIB raw=");
    //Serial.println(vibRaw == LOW ? "LOW" : "HIGH");
    Serial.print("VIB GPIO=");
    Serial.print(PIN_VIB);
    Serial.print(" raw=");
    Serial.println(vibRaw == LOW ? "LOW" : "HIGH");
  }

  if (vibRaw != lastVibStable) {
    if (now - lastVibChangeMs >= VIB_DEBOUNCE_MS) {
      lastVibStable = vibRaw;
      lastVibChangeMs = now;
      if (DEBUG_VIB) {
        Serial.print("VIB change -> ");
        Serial.println(lastVibStable == LOW ? "LOW" : "HIGH");
      }
      vibActive = (lastVibStable == LOW);
      if (vibActive) {
        if (tapHits == 0) {
          tapWindowStartMs = now;
          tapHits = 1;
        } else {
          if (now - tapWindowStartMs <= TAP_WINDOW_MS) tapHits++;
          else {
            tapWindowStartMs = now;
            tapHits = 1;
          }
        }

            if (now - lastTapMs >= TAP_COOLDOWN_MS) {

            if (!strobeActive && pinkPauseState == PINK_PAUSE_IDLE) {
              sendNextLightColor();
              lastTapMs = now;
            } else {
              if (strobeActive) Serial.println("Tap ignored: STROBE active");
              else Serial.println("Tap ignored: pink hold active");
            }
          }
      }
    }
  }

  if (tapHits > 0 && (now - tapWindowStartMs > TAP_WINDOW_MS)) {
    if (DEBUG_VIB) {
      Serial.print("Tap window expired hits=");
      Serial.println(tapHits);
    }
    tapHits = 0;
  }
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.print("reset reason: ");
  Serial.println((int)esp_reset_reason());
  Serial.println("setup start");
  Serial.print("Globe ID: ");
  Serial.println(NODE_ID);
 // Vibration switch is active-low (switch to GND when tapped)
pinMode(PIN_VIB, INPUT_PULLUP);

// IR pin bring-up: prove the GPIO can toggle with a DC pulse before attaching PWM.
pinMode(PIN_IR_LED, OUTPUT);
digitalWrite(PIN_IR_LED, LOW);
Serial.print("IR GPIO=");
Serial.println(PIN_IR_LED);
Serial.println("IR DC pulse test (200ms)");
digitalWrite(PIN_IR_LED, HIGH);
delay(200);
digitalWrite(PIN_IR_LED, LOW);

// Now attach LEDC and run the carrier self-test (printed inside irInitLedc())
irInitLedc();

  initDefaultCycle();

  // Optional boot test: sends one IR command so you can confirm the receiver reacts.
  // This is OFF by default to avoid changing behavior unexpectedly.
  if (IR_TEST_ON_BOOT) {
    delay(IR_TEST_DELAY_MS);
    Serial.println("IR boot test: sending PINK");
    // Send a known code (PINK) three times like normal.
    for (int i = 0; i < 3; i++) {
      irSendRawNECext(IR_ADDR, 0xE916);
      delay(20);
    }
    Serial.println("IR boot test done");
  }

  wifi_init_for_espnow();
  Serial.println("wifi init ok");

  esp_err_t e = esp_now_init();
  if (e != ESP_OK) {
    Serial.print("esp_now_init failed: ");
    Serial.println(esp_err_to_name(e));
    return;
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  // Ensure broadcast peer exists for discovery
  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  ensurePeer(bcast);

  Serial.println("Orb ready.");
}

void loop() {
  retryTick();
  pinkPauseTick();
  handleVibration();
  uint32_t now = millis();

  if (strobeActive && (int32_t)(now - strobeEndMs) >= 0) {
    strobeActive = false;
    resendCurrentColorIR();
  }
  // Manual tests (Serial Monitor):
  //  - 't' : send next color (full IR command)
  //  - 'c' : 38kHz carrier ON for 500ms (camera-visible)
  //  - 'd' : DC HIGH for 500ms (NOT IR, just proves GPIO toggles)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 't' || c == 'T') {
      Serial.println("Manual IR test: sendNextLightColor() via serial trigger");
      sendNextLightColor();
    } else if (c == 'c' || c == 'C') {
      Serial.println("Manual IR diag: 38kHz carrier ON for 500ms");
      irCarrierOn();
      delay(500);
      irCarrierOff();
      Serial.println("Manual IR diag: carrier OFF");
    } else if (c == 'd' || c == 'D') {
      Serial.println("Manual GPIO diag: DC HIGH for 500ms (not IR)");
      digitalWrite(PIN_IR_LED, HIGH);
      delay(500);
      digitalWrite(PIN_IR_LED, LOW);
      Serial.println("Manual GPIO diag: DC LOW");
    }
  }

  if (DEBUG_VIB && (now - lastHeartbeatMs >= 1000)) {
    lastHeartbeatMs = now;
    Serial.println("loop alive");
  }

  if (hubKnown && lightQueued && !pending.active && queuedLightCount > 0) {
    if (reliableSendToHub(MSG_LIGHT, lightQueuedValue)) {
      queuedLightCount--;
      if (queuedLightCount == 0) lightQueued = false;
    }
  }

  // Discovery until hub is known
  if (!hubKnown) {
    if (now - lastHelloMs >= HELLO_INTERVAL_MS) {
      lastHelloMs = now;
      broadcastHello();
    }
  }

  // Periodic status: "color:<reportedColor>"
  if (hubKnown && now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    reliableSendToHub(MSG_STATUS, reportedColor);
  }
}
