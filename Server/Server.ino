#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_mac.h"
#include "esp_err.h"

// ==================== CONFIG ====================
static const uint8_t  CHANNEL = 1;            // MUST MATCH ORBS
static const uint8_t  MAX_NODES = 20;         // room for 14+
static const uint32_t SERIAL_BAUD = 115200;

// Reliability
static const uint32_t RETRY_BASE_MS = 180;
static const uint8_t  RETRY_MAX = 6;

// Node online/offline (only used for list display; pins depend on last color)
static const uint32_t NODE_OFFLINE_MS = 8000;
static const uint32_t CONSENSUS_HOLD_MS = 180000; // 3 minutes

// NOTE ON TERMINOLOGY:
// - All values in ORB_PINS[] and ALL_ORBS_PIN are *GPIO numbers* (ESP32 IO numbers), not the printed header pin numbers.
// - The provided IMG_7712 pinout shows both the header pin index (1..19 on the top edge, 20..38 on the bottom edge)
//   and the GPIO number. This code uses the GPIO number.
// - Avoid GPIO6-11 for general I/O (they are connected to the onboard SPI flash on many ESP32-WROOM dev boards).
// - GPIO4 is a strapping/boot-related pin on many ESP32 boards; it is OK as an OUTPUT, but do NOT externally hold it LOW
//   during reset/boot (and avoid heavy loads) or you may get boot issues.

// =============== OUTPUT PIN ASSIGNMENTS (15 TRIGGERS) ===============
// Orb 1..14 trigger outputs (GPIO numbers):
// Mapping verified against IMG_7712 (ESP32-WROOM-32 pinout).
// Format: Orb N -> GPIO (Header pin # from IMG_7712)
static const int ORB_PINS[15] = {
  -1,        // index 0 unused
  13,        // Orb 1  -> GPIO13 (header pin 15)
  14,        // Orb 2  -> GPIO14 (header pin 12)
  16,        // Orb 3  -> GPIO16 (header pin 27)
  17,        // Orb 4  -> GPIO17 (header pin 28)
  18,        // Orb 5  -> GPIO18 (header pin 30)
  19,        // Orb 6  -> GPIO19 (header pin 31)
  21,        // Orb 7  -> GPIO21 (header pin 33)
  22,        // Orb 8  -> GPIO22 (header pin 36)
  23,        // Orb 9  -> GPIO23 (header pin 37)
  25,        // Orb 10 -> GPIO25 (header pin 9)
  26,        // Orb 11 -> GPIO26 (header pin 10)
  27,        // Orb 12 -> GPIO27 (header pin 11)
  32,        // Orb 13 -> GPIO32 (header pin 7)
  33         // Orb 14 -> GPIO33 (header pin 8)
};

// "ALL ORBS == 15" pin (GPIO number):
// Active when all 14 nodes report last_color == TRIGGER_COLOR.
static const int ALL_ORBS_PIN = 4; // GPIO4 (header pin 26)

// Trigger color
static const int32_t TRIGGER_COLOR = 15;

// Outputs are active-high: HIGH = active, LOW = inactive
static const uint8_t PIN_ACTIVE_LEVEL = HIGH;
static const uint8_t PIN_INACTIVE_LEVEL = LOW;

// ==================== PROTOCOL ====================
enum MsgType : uint8_t {
  MSG_HELLO     = 1,
  MSG_STATUS    = 2,
  MSG_PING      = 3,
  MSG_PONG      = 4,
  MSG_SET       = 5,
  MSG_ACK       = 6,
  MSG_LIGHT     = 7,  // value = 1 if pink, 0 otherwise (only on pink transitions)
  MSG_CONSENSUS = 8,  // value = 1 if all 14 orbs are pink, 0 otherwise
  MSG_IR_FUNC   = 9,  // value = 1:ON, 2:OFF, 3:FLASH, 4:STROBE
  MSG_CYCLE_CFG = 10  // value packed as: [count:4][idx0:4][idx1:4][idx2:4][idx3:4][idx4:4][idx5:4]
};

#pragma pack(push, 1)
struct Packet {
  uint8_t  type;
  uint8_t  node_id;   // 1..14
  uint8_t  ack_type;
  uint8_t  flags;
  uint32_t seq;
  uint32_t ack_seq;
  int32_t  value;     // used as "color" for MSG_STATUS; also used for other msg payloads
};
#pragma pack(pop)

// ==================== NODE + RELIABILITY ====================
struct PendingTx {
  bool     active = false;
  uint8_t  to_node_id = 0;
  uint8_t  type = 0;
  uint32_t seq = 0;
  int32_t  value = 0;
  uint8_t  tries = 0;
  uint32_t next_send_ms = 0;
};

struct NodeInfo {
  bool     used = false;
  uint8_t  node_id = 0;
  uint8_t  mac[6] = {0};
  uint32_t last_seen_ms = 0;

  // Store latest color and kv string
  int32_t  last_color = 0;
  bool     is_pink = false;     // latest pink state from MSG_LIGHT
  char     kv[24] = {0};        // e.g., "3{color:15}"

  PendingTx pending;
};

NodeInfo nodes[MAX_NODES];
uint32_t hub_seq = 1;

// Consensus latch (edge-detect when all become pink)
static bool consensusAllPink = false;
static bool consensusHoldActive = false;
static uint32_t consensusHoldUntilMs = 0;

uint8_t HUB_STA_MAC[6] = {0};
uint8_t HUB_CHANNEL = CHANNEL;

// ==================== HELPERS ====================
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

static void wifi_init_fixed_channel() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(50);

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_start();
  delay(30);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  delay(20);

  readStaMac(HUB_STA_MAC);
  HUB_CHANNEL = getChannelNow();

  Serial.print("Hub STA MAC: ");
  printMac(HUB_STA_MAC);
  Serial.print(" | Channel: ");
  Serial.println(HUB_CHANNEL);
}

static int findNodeById(uint8_t node_id) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodes[i].used && nodes[i].node_id == node_id) return i;
  }
  return -1;
}

static int findNodeByMac(const uint8_t* mac) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].used) continue;
    bool same = true;
    for (int j = 0; j < 6; j++) {
      if (nodes[i].mac[j] != mac[j]) { same = false; break; }
    }
    if (same) return i;
  }
  return -1;
}

static bool addPeerIfNeeded(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;

  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) {
    Serial.print("add_peer failed: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }
  return true;
}

static void registerNode(uint8_t node_id, const uint8_t* mac) {
  int idx = findNodeByMac(mac);
  if (idx < 0) idx = findNodeById(node_id);

  if (idx < 0) {
    for (int i = 0; i < MAX_NODES; i++) {
      if (!nodes[i].used) { idx = i; break; }
    }
  }
  if (idx < 0) {
    Serial.println("Node registry full!");
    return;
  }

  nodes[idx].used = true;
  nodes[idx].node_id = node_id;
  memcpy(nodes[idx].mac, mac, 6);
  nodes[idx].last_seen_ms = millis();
  nodes[idx].is_pink = false;

  // init kv string
  snprintf(nodes[idx].kv, sizeof(nodes[idx].kv), "%u{color:%ld}", node_id, (long)nodes[idx].last_color);

  addPeerIfNeeded(mac);
}

static esp_err_t rawSend(const uint8_t* mac, const Packet& p) {
  return esp_now_send(mac, (const uint8_t*)&p, sizeof(p));
}

static void sendAckTo(const uint8_t* mac, uint8_t node_id, uint8_t ack_type, uint32_t ack_seq) {
  Packet a{};
  a.type = MSG_ACK;
  a.node_id = node_id;
  a.ack_type = ack_type;
  a.flags = 0;
  a.seq = hub_seq++;
  a.ack_seq = ack_seq;
  a.value = 0;
  rawSend(mac, a);
}

// ==================== OUTPUT TRIGGERS ====================
static void updatePinsForOrb(uint8_t node_id, int32_t color);
static void updateAllOrbsPin();

static void initTriggerPins() {
  for (int n = 1; n <= 14; n++) {
    pinMode(ORB_PINS[n], OUTPUT);
    digitalWrite(ORB_PINS[n], PIN_INACTIVE_LEVEL);
  }
  pinMode(ALL_ORBS_PIN, OUTPUT);
  digitalWrite(ALL_ORBS_PIN, PIN_INACTIVE_LEVEL);
}

static void setAllPinsActive(bool active) {
  const uint8_t level = active ? PIN_ACTIVE_LEVEL : PIN_INACTIVE_LEVEL;
  for (int n = 1; n <= 14; n++) {
    digitalWrite(ORB_PINS[n], level);
  }
  digitalWrite(ALL_ORBS_PIN, level);
}

static void setTriggerPinActive(uint8_t node_id, bool active) {
  if (node_id < 1 || node_id > 14) return;
  digitalWrite(ORB_PINS[node_id], active ? PIN_ACTIVE_LEVEL : PIN_INACTIVE_LEVEL);
}

static void setTriggerTargetsActive(const String& target, bool active) {
  if (target == "all") {
    setAllPinsActive(active);
    Serial.print("Triggers -> ");
    Serial.println(active ? "ON (ACTIVE HIGH)" : "OFF (INACTIVE LOW)");
    return;
  }

  int id = target.toInt();
  if (id < 1 || id > 14) {
    Serial.println("Usage: trigger <on|off> <id|all>");
    return;
  }

  setTriggerPinActive((uint8_t)id, active);
  Serial.print("Trigger ");
  Serial.print(id);
  Serial.print(" -> ");
  Serial.println(active ? "ON (ACTIVE HIGH)" : "OFF (INACTIVE LOW)");
}

static bool parseColorTokenToIndex(String token, uint8_t& outIdx) {
  token.trim();
  token.toLowerCase();
  if (token == "red")      { outIdx = 0; return true; }
  if (token == "ored")     { outIdx = 1; return true; }
  if (token == "green")    { outIdx = 2; return true; }
  if (token == "lgreen")   { outIdx = 3; return true; }
  if (token == "blue")     { outIdx = 4; return true; }
  if (token == "orange")   { outIdx = 5; return true; }
  if (token == "oyellow")  { outIdx = 6; return true; }
  if (token == "aqua")     { outIdx = 7; return true; }
  if (token == "pink")     { outIdx = 9; return true; } // accepted but ignored (auto-appended on orb)
  return false;
}

static bool packCycleList(const String& csv, int32_t& outPacked) {
  // Packed format (6 colors max):
  // bits 0..23: six 4-bit color indexes, in order
  // bits 24..27: count
  uint8_t idxs[6];
  uint8_t count = 0;

  int start = 0;
  while (start < (int)csv.length()) {
    int comma = csv.indexOf(',', start);
    String tok = (comma < 0) ? csv.substring(start) : csv.substring(start, comma);
    uint8_t idx = 0;
    if (!parseColorTokenToIndex(tok, idx)) return false;
    if (idx != 9) { // pink is auto-added by orb
      if (count < 6) idxs[count++] = idx;
    }
    if (comma < 0) break;
    start = comma + 1;
  }

  if (count == 0) return false;

  uint32_t packed = ((uint32_t)count & 0x0F) << 24;
  for (uint8_t i = 0; i < count; i++) {
    packed |= ((uint32_t)(idxs[i] & 0x0F) << (i * 4));
  }
  outPacked = (int32_t)packed;
  return true;
}

static bool parseIrFuncToken(String token, int32_t& outCode) {
  token.trim();
  token.toLowerCase();
  if (token == "on")     { outCode = 1; return true; }
  if (token == "off")    { outCode = 2; return true; }
  if (token == "flash")  { outCode = 3; return true; }
  if (token == "strobe") { outCode = 4; return true; }
  return false;
}

static void startConsensusHold() {
  consensusHoldActive = true;
  consensusHoldUntilMs = millis() + CONSENSUS_HOLD_MS;
  setAllPinsActive(true);
}

static void consensusHoldTick() {
  if (!consensusHoldActive) return;
  if ((int32_t)(millis() - consensusHoldUntilMs) < 0) return;

  consensusHoldActive = false;
  // Restore normal pin behavior from the latest known node colors.
  for (int n = 1; n <= 14; n++) {
    int idx = findNodeById((uint8_t)n);
    if (idx >= 0) updatePinsForOrb((uint8_t)n, nodes[idx].last_color);
    else digitalWrite(ORB_PINS[n], PIN_INACTIVE_LEVEL);
  }
  updateAllOrbsPin();
}

static void updatePinsForOrb(uint8_t node_id, int32_t color) {
  if (node_id < 1 || node_id > 14) return;
  if (consensusHoldActive) return; // hold overrides per-orb writes
  digitalWrite(
    ORB_PINS[node_id],
    (color == TRIGGER_COLOR) ? PIN_ACTIVE_LEVEL : PIN_INACTIVE_LEVEL
  );
}

static void updateAllOrbsPin() {
  if (consensusHoldActive) return; // hold overrides "ALL" writes
  // "ALL" output is active only if all 14 nodes last_color == 15
  bool all15 = true;

  for (int n = 1; n <= 14; n++) {
    int idx = findNodeById((uint8_t)n);
    if (idx < 0) { all15 = false; break; }
    if (nodes[idx].last_color != TRIGGER_COLOR) { all15 = false; break; }
  }

  digitalWrite(ALL_ORBS_PIN, all15 ? PIN_ACTIVE_LEVEL : PIN_INACTIVE_LEVEL);
}

// ==================== CONSENSUS HELPERS ====================
static bool computeAllOrbsPink() {
  // Consensus is true only if all 14 node IDs exist and each has is_pink == true
  for (int n = 1; n <= 14; n++) {
    int idx = findNodeById((uint8_t)n);
    if (idx < 0) return false;
    if (!nodes[idx].is_pink) return false;
  }
  return true;
}

// ==================== RELIABLE SEND (Hub -> Orb) ====================
static bool reliableSendToNode(uint8_t node_id, uint8_t type, int32_t value) {
  int idx = findNodeById(node_id);
  if (idx < 0) {
    Serial.println("Unknown node id (not registered yet).");
    return false;
  }
  NodeInfo& n = nodes[idx];

  if (n.pending.active) {
    Serial.println("Node busy (pending tx). Try again.");
    return false;
  }

  Packet p{};
  p.type = type;
  p.node_id = node_id;
  p.ack_type = 0;
  p.flags = 0;
  p.seq = hub_seq++;
  p.ack_seq = 0;
  p.value = value;

  esp_err_t e = rawSend(n.mac, p);
  if (e != ESP_OK) {
    Serial.print("send failed: ");
    Serial.println(esp_err_to_name(e));
    return false;
  }

  n.pending.active = true;
  n.pending.to_node_id = node_id;
  n.pending.type = type;
  n.pending.seq = p.seq;
  n.pending.value = value;
  n.pending.tries = 0;
  n.pending.next_send_ms = millis() + RETRY_BASE_MS;

  return true;
}

static void reliableBroadcast(uint8_t type, int32_t value) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].used) continue;
    if (nodes[i].pending.active) continue;
    reliableSendToNode(nodes[i].node_id, type, value);
  }
}

// Send consensus reply without blocking on unrelated pending traffic
static void bestEffortSendConsensusToNode(uint8_t node_id, int32_t value) {
  int idx = findNodeById(node_id);
  if (idx < 0) return;
  NodeInfo& n = nodes[idx];

  // Prefer reliable send if node isn't busy
  if (!n.pending.active) {
    reliableSendToNode(node_id, MSG_CONSENSUS, value);
    return;
  }

  // Otherwise, send a one-shot (no retries) so we don't block on an unrelated pending message.
  Packet p{};
  p.type = MSG_CONSENSUS;
  p.node_id = node_id;
  p.ack_type = 0;
  p.flags = 0;
  p.seq = hub_seq++;
  p.ack_seq = 0;
  p.value = value;
  rawSend(n.mac, p);
}

static void retryTick() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].used) continue;
    PendingTx& pt = nodes[i].pending;
    if (!pt.active) continue;
    if (now < pt.next_send_ms) continue;

    if (pt.tries >= RETRY_MAX) {
      pt.active = false;
      continue;
    }

    Packet p{};
    p.type = pt.type;
    p.node_id = pt.to_node_id;
    p.ack_type = 0;
    p.flags = 0;
    p.seq = pt.seq;       // resend same seq
    p.ack_seq = 0;
    p.value = pt.value;

    rawSend(nodes[i].mac, p);

    pt.tries++;
    uint32_t backoff = RETRY_BASE_MS << (pt.tries > 4 ? 4 : pt.tries);
    pt.next_send_ms = now + backoff;
  }
}

// ==================== ESP-NOW CALLBACKS (core v3.x signatures) ====================
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

  if (p.type == MSG_HELLO) {
    registerNode(p.node_id, src_mac);
    sendAckTo(src_mac, p.node_id, MSG_HELLO, p.seq);
    return;
  }

  int idx = findNodeByMac(src_mac);
  if (idx >= 0) nodes[idx].last_seen_ms = millis();

  if (p.type == MSG_ACK) {
    if (idx >= 0) {
      PendingTx& pt = nodes[idx].pending;
      if (pt.active && p.ack_seq == pt.seq && p.ack_type == pt.type) {
        pt.active = false;
      }
    }
    return;
  }

  // ACK any non-ACK message
  sendAckTo(src_mac, p.node_id, p.type, p.seq);

  if (p.type == MSG_STATUS) {
    // Interpret value as color:<reported color>
    int32_t color = p.value;

    // Store "n{color:x}"
    if (idx >= 0) {
      nodes[idx].last_color = color;
      snprintf(nodes[idx].kv, sizeof(nodes[idx].kv), "%u{color:%ld}", (unsigned)p.node_id, (long)color);
    }

    // Update pins
    updatePinsForOrb(p.node_id, color);
    updateAllOrbsPin();

  } else if (p.type == MSG_LIGHT) {
    // value = 1 if pink, 0 otherwise (only sent when entering/leaving pink)
    if (idx >= 0) {
      nodes[idx].is_pink = (p.value == 1);
    }

    // Compute consensus and reply to the sender (they will listen after their pink transition).
    bool allPink = computeAllOrbsPink();
    bestEffortSendConsensusToNode(p.node_id, allPink ? 1 : 0);

    // Edge-triggered broadcast: when consensus first becomes true, notify everyone.
    if (allPink && !consensusAllPink) {
      consensusAllPink = true;
      startConsensusHold();
      reliableBroadcast(MSG_CONSENSUS, 1);
    } else if (!allPink) {
      consensusAllPink = false;
    }

    Serial.print("{'id':");
    Serial.print((unsigned)p.node_id);
    Serial.print(", 'pink': ");
    Serial.print((int)p.value);
    Serial.print(", 'consensus': ");
    Serial.print(allPink ? 1 : 0);
    Serial.println("}");

  } else if (p.type == MSG_PONG) {
    // nothing needed; ACK already sent
  }
}

// ==================== SERIAL COMMANDS ====================
static String readLine() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = line;
      line = "";
      out.trim();
      return out;
    }
    line += c;
  }
  return "";
}

static void printPinMap() {
  Serial.println();
  Serial.println("=== TRIGGER PIN MAP (GPIO -> Header pin # from ESP32 pinout photo) ===");
  Serial.println("(Remember: code uses GPIO numbers, not header pin numbers.)");
  Serial.println("Orb 1  -> GPIO13 (header pin 15)");
  Serial.println("Orb 2  -> GPIO14 (header pin 12)");
  Serial.println("Orb 3  -> GPIO16 (header pin 27)");
  Serial.println("Orb 4  -> GPIO17 (header pin 28)");
  Serial.println("Orb 5  -> GPIO18 (header pin 30)");
  Serial.println("Orb 6  -> GPIO19 (header pin 31)");
  Serial.println("Orb 7  -> GPIO21 (header pin 33)");
  Serial.println("Orb 8  -> GPIO22 (header pin 36)");
  Serial.println("Orb 9  -> GPIO23 (header pin 37)");
  Serial.println("Orb 10 -> GPIO25 (header pin 9)");
  Serial.println("Orb 11 -> GPIO26 (header pin 10)");
  Serial.println("Orb 12 -> GPIO27 (header pin 11)");
  Serial.println("Orb 13 -> GPIO32 (header pin 7)");
  Serial.println("Orb 14 -> GPIO33 (header pin 8)");
  Serial.println("ALL    -> GPIO4  (header pin 26)  [Active when ALL 14 last_color==TRIGGER_COLOR]");
  Serial.println();
}

static void printHelp() {
  Serial.println();
  Serial.println("=== HUB INFO ===");
  Serial.print("Hub STA MAC: ");
  printMac(HUB_STA_MAC);
  Serial.println();
  Serial.print("Channel: ");
  Serial.println(HUB_CHANNEL);
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  list");
  Serial.println("  pins   (print GPIO/header pin mapping)");
  Serial.println("  ping <id>");
  Serial.println("  set <id> <color 1-15>");
  Serial.println("  broadcast <color 1-15>");
  Serial.println("  on <id|all>");
  Serial.println("  ir <id|all> <on|off|flash|strobe>");
  Serial.println("  trigger <on|off> <id|all>");
  Serial.println("  cycle <id|all> <csv colors>");
  Serial.println("    colors: red,ored,green,lgreen,blue,orange,oyellow,aqua[,pink]");
  Serial.println("    pink is always auto-appended by orb");
  Serial.println("  (Hub sends MSG_CONSENSUS=1 when all 14 orbs report pink)");
  Serial.println();
  Serial.println("Pins:");
  Serial.println("  Orb1..Orb14 GPIOs: 13,14,16,17,18,19,21,22,23,25,26,27,32,33");
  Serial.println("  (Use 'pins' to see the matching header pin numbers from the ESP32 pinout photo.)");
  Serial.print("  ALL GPIO (all 14==TRIGGER_COLOR): ");
  Serial.println(ALL_ORBS_PIN);
  Serial.println();
}

static void listNodes() {
  Serial.println("Known nodes:");
  uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].used) continue;

    bool online = (now - nodes[i].last_seen_ms) <= NODE_OFFLINE_MS;

    Serial.print("  id=");
    Serial.print(nodes[i].node_id);
    Serial.print(" mac=");
    printMac(nodes[i].mac);
    Serial.print(" online=");
    Serial.print(online ? "yes" : "no");
    Serial.print(" lastSeenMsAgo=");
    Serial.print(now - nodes[i].last_seen_ms);
    Serial.print("  ");
    Serial.print(nodes[i].kv); // n{color:x}
    Serial.print(" pin=");
    Serial.print(digitalRead(ORB_PINS[nodes[i].node_id]) == PIN_ACTIVE_LEVEL ? "ACTIVE(HIGH)" : "INACTIVE(LOW)");
    Serial.print(" pending=");
    Serial.print(nodes[i].pending.active ? "yes" : "no");
    Serial.print(" pink=");
    Serial.println(nodes[i].is_pink ? "1" : "0");
  }

  Serial.print("ALL pin=");
  Serial.println(digitalRead(ALL_ORBS_PIN) == PIN_ACTIVE_LEVEL ? "ACTIVE(HIGH)" : "INACTIVE(LOW)");
  Serial.print("CONSENSUS(allPink)=");
  Serial.println(consensusAllPink ? "1" : "0");
  Serial.print("CONSENSUS_HOLD(active)=");
  Serial.println(consensusHoldActive ? "1" : "0");
  Serial.println();
}

// ==================== SETUP / LOOP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  initTriggerPins();
  wifi_init_fixed_channel();

  esp_err_t e = esp_now_init();
  if (e != ESP_OK) {
    Serial.print("esp_now_init failed: ");
    Serial.println(esp_err_to_name(e));
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  Serial.println("Hub ready. Type 'help'.");
  printPinMap();
}

void loop() {
  retryTick();
  consensusHoldTick();

  String cmd = readLine();
  if (!cmd.length()) return;

  if (cmd == "help") {
    printHelp();
  } else if (cmd == "list") {
    listNodes();
  } else if (cmd == "pins") {
    printPinMap();
  } else if (cmd.startsWith("ping ")) {
    int id = cmd.substring(5).toInt();
    reliableSendToNode((uint8_t)id, MSG_PING, (int32_t)millis());
  } else if (cmd.startsWith("set ")) {
    int sp1 = cmd.indexOf(' ', 4);
    if (sp1 < 0) {
      Serial.println("Usage: set <id> <color>");
      return;
    }
    int id = cmd.substring(4, sp1).toInt();
    int color = cmd.substring(sp1 + 1).toInt();
    reliableSendToNode((uint8_t)id, MSG_SET, color);
  } else if (cmd.startsWith("broadcast ")) {
    int color = cmd.substring(10).toInt();
    reliableBroadcast(MSG_SET, color);
  } else if (cmd.startsWith("on ")) {
    String target = cmd.substring(3);
    target.trim();
    if (target == "all") {
      reliableBroadcast(MSG_IR_FUNC, 1);
    } else {
      int id = target.toInt();
      reliableSendToNode((uint8_t)id, MSG_IR_FUNC, 1);
    }
  } else if (cmd.startsWith("ir ")) {
    int sp1 = cmd.indexOf(' ', 3);
    if (sp1 < 0) {
      Serial.println("Usage: ir <id|all> <on|off|flash|strobe>");
      return;
    }
    String target = cmd.substring(3, sp1);
    String action = cmd.substring(sp1 + 1);
    target.trim();
    action.trim();
    target.toLowerCase();
    action.toLowerCase();

    int32_t irCode = 0;
    if (!parseIrFuncToken(action, irCode)) {
      Serial.println("Usage: ir <id|all> <on|off|flash|strobe>");
      return;
    }

    if (target == "all") {
      reliableBroadcast(MSG_IR_FUNC, irCode);
    } else {
      int id = target.toInt();
      if (id < 1 || id > 14) {
        Serial.println("Usage: ir <id|all> <on|off|flash|strobe>");
        return;
      }
      reliableSendToNode((uint8_t)id, MSG_IR_FUNC, irCode);
    }
  } else if (cmd.startsWith("trigger ")) {
    int sp1 = cmd.indexOf(' ', 8);
    if (sp1 < 0) {
      Serial.println("Usage: trigger <on|off> <id|all>");
      return;
    }
    String mode = cmd.substring(8, sp1);
    String target = cmd.substring(sp1 + 1);
    mode.trim();
    target.trim();
    mode.toLowerCase();
    target.toLowerCase();

    if (mode == "on") {
      setTriggerTargetsActive(target, true);
    } else if (mode == "off") {
      setTriggerTargetsActive(target, false);
    } else {
      Serial.println("Usage: trigger <on|off> <id|all>");
    }
  } else if (cmd.startsWith("cycle ")) {
    int sp1 = cmd.indexOf(' ', 6);
    if (sp1 < 0) {
      Serial.println("Usage: cycle <id|all> <csv colors>");
      return;
    }
    String target = cmd.substring(6, sp1);
    String csv = cmd.substring(sp1 + 1);
    target.trim();
    csv.trim();
    target.toLowerCase();

    int32_t packed = 0;
    if (!packCycleList(csv, packed)) {
      Serial.println("Invalid colors. Use: red,ored,green,lgreen,blue,orange,oyellow,aqua[,pink]");
      return;
    }

    if (target == "all") {
      reliableBroadcast(MSG_CYCLE_CFG, packed);
    } else {
      int id = target.toInt();
      if (id < 1 || id > 14) {
        Serial.println("Usage: cycle <id|all> <csv colors>");
        return;
      }
      reliableSendToNode((uint8_t)id, MSG_CYCLE_CFG, packed);
    }
  } else {
    Serial.println("Unknown command. Type 'help'.");
  }
}
