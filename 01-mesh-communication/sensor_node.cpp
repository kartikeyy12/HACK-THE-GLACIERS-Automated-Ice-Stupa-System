/**
 * ============================================================================
 *  MOUNTAIN MESH NETWORK — SENSOR NODE FIRMWARE
 *  File:    sensor_node.cpp
 *  Target:  ESP32 (Arduino framework, ESP-IDF v4.x / Arduino Core v2.x+)
 *  Author:  Kartikey — NIT Goa
 *  License: MIT
 * ============================================================================
 *
 *  ARCHITECTURE OVERVIEW
 *  ─────────────────────
 *  This firmware runs on Sensor Node 1 (SN1) or Sensor Node 2 (SN2).
 *  Node identity is selected at compile time via NODE_ID below.
 *
 *  DUTY CYCLE (simulated field behaviour)
 *  ───────────────────────────────────────
 *  ┌─────────────┐     TX      ┌───────────────┐
 *  │  WAKE (run) │ ──────────► │  Light Sleep  │
 *  │  ~250 ms    │             │  ~9.75 s      │
 *  └─────────────┘◄────────────└───────────────┘
 *                  ACK timeout
 *
 *  Every SAMPLE_INTERVAL_MS milliseconds the node:
 *    1. Reads (or simulates) sensor values.
 *    2. Packs a MeshPacket struct.
 *    3. Transmits via ESP-NOW to the gateway MAC.
 *    4. Waits for a software-level ACK (MeshPacket with type=PKT_ACK).
 *    5. Enters light-sleep for the remainder of the interval to save power.
 *
 *  PACKET RETRY STRATEGY
 *  ──────────────────────
 *  If no ACK arrives within ACK_TIMEOUT_MS, the packet is re-queued.
 *  After MAX_RETRIES failures the packet is dropped and the sequence
 *  number advances (mimics field behaviour where stale data is discarded).
 * ============================================================================
 */

// ─── Arduino / ESP-IDF headers ───────────────────────────────────────────────
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <esp_sleep.h>

// ─── Compile-time node identity ──────────────────────────────────────────────
//  Set to 1 for the first sensor node, 2 for the second.
//  In a real deployment this would be stored in NVS / eFuse.
#ifndef NODE_ID
  #define NODE_ID 1
#endif

// ─── Network topology constants ──────────────────────────────────────────────
// Gateway MAC address — replace with the actual MAC of your gateway ESP32.
// Read the gateway's MAC by running Serial.println(WiFi.macAddress()) on it.
static const uint8_t GATEWAY_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00};

// Wi-Fi channel. All nodes MUST share the same channel for ESP-NOW.
static const uint8_t ESPNOW_CHANNEL = 1;

// ─── Timing constants (milliseconds) ─────────────────────────────────────────
static const uint32_t SAMPLE_INTERVAL_MS = 10000;   // 10 s duty cycle
static const uint32_t ACK_TIMEOUT_MS     = 800;     // wait up to 800 ms for ACK
static const uint8_t  MAX_RETRIES        = 3;       // retransmit up to 3×

// ─── Packet type identifiers ─────────────────────────────────────────────────
// Kept as uint8_t to minimise packet overhead — every byte counts on RF.
static const uint8_t PKT_SENSOR_DATA = 0x01;  // sensor telemetry payload
static const uint8_t PKT_ACK         = 0x02;  // acknowledgement from gateway
static const uint8_t PKT_HEARTBEAT   = 0x03;  // periodic node-alive ping
static const uint8_t PKT_ERROR       = 0xFF;  // error / malformed packet

// ─── Protocol version ────────────────────────────────────────────────────────
// Increment when the struct layout changes. Gateway validates this.
static const uint8_t PROTOCOL_VERSION = 0x01;

// ─── Sensor channels bitmask (which sensors are active on this node) ─────────
// Stored in MeshPacket::sensor_flags. Allows gateway to parse payload selectively.
static const uint8_t SENSOR_FLOW_ACTIVE   = (1 << 0);  // bit 0 — flow meter
static const uint8_t SENSOR_TEMP_ACTIVE   = (1 << 1);  // bit 1 — air temperature
static const uint8_t SENSOR_HUMID_ACTIVE  = (1 << 2);  // bit 2 — relative humidity
static const uint8_t SENSOR_BARO_ACTIVE   = (1 << 3);  // bit 3 — barometric pressure

// ─────────────────────────────────────────────────────────────────────────────
//  MESH PACKET STRUCTURE
//  Total size: 32 bytes (fits within ESP-NOW's 250-byte max payload easily)
//
//  Field layout (packed to avoid padding bytes wasting bandwidth):
//
//  Offset  Size  Field
//  ──────  ────  ──────────────────────────────────────────────────────────
//   0      1     protocol_version   — wire compatibility check
//   1      1     packet_type        — PKT_SENSOR_DATA | PKT_ACK | PKT_HEARTBEAT
//   2      1     node_id            — originating node (1–254; 0 = gateway, 255 = broadcast)
//   3      1     hop_count          — incremented at each relay (future multi-hop use)
//   4      2     seq_number         — rolling counter; gateway detects gaps / duplicates
//   6      4     timestamp_ms       — millis() on originating node at packet creation
//  10      1     sensor_flags       — bitmask of which sensor fields are valid
//  11      1     rssi_last_hop      — RSSI (dBm) of last received packet from gateway (link quality)
//  12      2     flow_rate_mLs      — flow in mL/s × 10 (fixed-point, 1 decimal place)
//  14      2     temperature_c10    — temperature in °C × 10 (signed, e.g. -15.3°C → -153)
//  16      2     humidity_pct10     — relative humidity % × 10 (e.g. 63.4% → 634)
//  18      4     pressure_pa        — absolute barometric pressure in Pascals (uint32 for altitude range)
//  22      2     battery_mv         — supply rail voltage in mV (power budget monitoring)
//  24      1     tx_power_dbm       — current TX power setting (for adaptive power control)
//  25      1     retry_count        — how many times this packet has been retransmitted
//  26      2     crc16              — CRC-16/CCITT of bytes [0..25] for integrity check
//  28      4     reserved           — future use / alignment
//  ──────  ────  ──────────────────────────────────────────────────────────
//  Total:  32 bytes
// ─────────────────────────────────────────────────────────────────────────────
#pragma pack(push, 1)   // disable struct padding — critical for binary serialisation
typedef struct {
    uint8_t  protocol_version;   // wire compatibility guard
    uint8_t  packet_type;        // PKT_* constant
    uint8_t  node_id;            // originating node identifier
    uint8_t  hop_count;          // relay hop counter (for future mesh extension)
    uint16_t seq_number;         // rolling sequence counter (wraps at 65535)
    uint32_t timestamp_ms;       // millis() at packet creation on source node
    uint8_t  sensor_flags;       // bitmask — which sensor readings are valid
    int8_t   rssi_last_hop;      // last received RSSI from gateway (signed dBm)
    uint16_t flow_rate_mLs;      // flow rate: mL/s × 10 (unsigned fixed-point)
    int16_t  temperature_c10;    // temperature: °C × 10 (signed)
    uint16_t humidity_pct10;     // humidity: % × 10 (unsigned)
    uint32_t pressure_pa;        // barometric pressure in Pascals
    uint16_t battery_mv;         // battery/rail voltage in mV
    uint8_t  tx_power_dbm;       // current ESP32 TX power (for log/analysis)
    uint8_t  retry_count;        // retransmission count for this packet
    uint16_t crc16;              // CRC-16/CCITT integrity check
    uint8_t  reserved[4];        // reserved — zero-filled
} MeshPacket;
#pragma pack(pop)

// ─── Static assertions — catch struct size drift at compile time ──────────────
static_assert(sizeof(MeshPacket) == 32, "MeshPacket must be exactly 32 bytes");

// ─────────────────────────────────────────────────────────────────────────────
//  CRC-16/CCITT-FALSE  (polynomial 0x1021, init 0xFFFF, no reflection)
//  Used to detect bit-errors and replay attacks on the sensor data.
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// Compute CRC over all fields EXCEPT the crc16 field itself (bytes 0..25)
static uint16_t compute_packet_crc(const MeshPacket *pkt) {
    // crc16 lives at offset 26; compute over the preceding 26 bytes
    return crc16_ccitt((const uint8_t *)pkt, offsetof(MeshPacket, crc16));
}

// ─── Global state ─────────────────────────────────────────────────────────────
static volatile bool    g_send_success   = false;  // set in onDataSent callback
static volatile bool    g_ack_received   = false;  // set in onDataRecv callback
static volatile int8_t  g_gateway_rssi   = 0;      // RSSI of last gateway message
static uint16_t         g_seq_number     = 0;      // rolling outbound sequence counter
static uint8_t          g_retry_count    = 0;      // retries for current packet
static MeshPacket        g_last_packet;             // keep copy for retry

// ─── ESP-NOW peer registration helper ────────────────────────────────────────
static bool register_peer(const uint8_t *mac) {
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac, 6);
    peer_info.channel   = ESPNOW_CHANNEL;
    peer_info.encrypt   = false;           // no CCMP encryption (PoC stage)
    peer_info.ifidx     = WIFI_IF_STA;
    return esp_now_add_peer(&peer_info) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW CALLBACK: onDataSent
//  Fires in Wi-Fi ISR context after the MAC-layer transmission attempt.
//  status == ESP_NOW_SEND_SUCCESS means the gateway's radio ACK'd the frame
//  at the 802.11 MAC layer. It does NOT mean the application processed it.
//  We use a separate application-level ACK (PKT_ACK) for that guarantee.
// ─────────────────────────────────────────────────────────────────────────────
static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        g_send_success = true;
        Serial.printf("[TX] MAC-layer delivery confirmed (seq=%u)\n", g_last_packet.seq_number);
    } else {
        g_send_success = false;
        Serial.printf("[TX] MAC-layer delivery FAILED (seq=%u) — will retry\n", g_last_packet.seq_number);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW CALLBACK: onDataRecv
//  Fires in Wi-Fi ISR context when an inbound ESP-NOW frame arrives.
//  The sensor node only expects PKT_ACK frames from the gateway.
//  IMPORTANT: Avoid heap allocation or long operations inside this callback.
// ─────────────────────────────────────────────────────────────────────────────
static void onDataRecv(const esp_now_recv_info_t *recv_info,
                       const uint8_t *data, int data_len) {
    // Size guard — reject malformed frames immediately
    if (data_len != sizeof(MeshPacket)) {
        Serial.printf("[RX] Unexpected frame length: %d (expected %zu)\n",
                      data_len, sizeof(MeshPacket));
        return;
    }

    MeshPacket pkt;
    memcpy(&pkt, data, sizeof(MeshPacket));

    // Protocol version check — forward compatibility guard
    if (pkt.protocol_version != PROTOCOL_VERSION) {
        Serial.printf("[RX] Protocol version mismatch: got 0x%02X, expected 0x%02X\n",
                      pkt.protocol_version, PROTOCOL_VERSION);
        return;
    }

    // CRC integrity check
    uint16_t expected_crc = compute_packet_crc(&pkt);
    if (pkt.crc16 != expected_crc) {
        Serial.printf("[RX] CRC mismatch: got 0x%04X, expected 0x%04X — packet discarded\n",
                      pkt.crc16, expected_crc);
        return;
    }

    // Only accept ACKs from gateway (node_id == 0) destined for us
    if (pkt.packet_type == PKT_ACK && pkt.node_id == 0) {
        if (pkt.seq_number == g_last_packet.seq_number) {
            g_ack_received   = true;
            g_gateway_rssi   = recv_info->rx_ctrl->rssi;  // capture link RSSI
            Serial.printf("[RX] ACK received from gateway for seq=%u  RSSI=%d dBm\n",
                          pkt.seq_number, (int)g_gateway_rssi);
        } else {
            Serial.printf("[RX] Stale ACK seq=%u (current=%u) — ignored\n",
                          pkt.seq_number, g_last_packet.seq_number);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SENSOR SIMULATION / READING
//  In a field deployment replace these with actual peripheral reads:
//    - flow_rate : Hall-effect pulse counter (ISR-driven, calculate mL/s)
//    - temperature / humidity : DHT22 or SHT31 via I2C/1-Wire
//    - pressure : BMP280 / MS5611 via SPI/I2C
//    - battery_mv : ADC read on a resistor divider
// ─────────────────────────────────────────────────────────────────────────────
static void read_sensors(MeshPacket *pkt) {
    // Deterministic pseudo-random simulation seeded by node_id and sequence
    // so each node produces a visually distinct data stream on the gateway log.
    float base_flow   = (NODE_ID == 1) ? 12.5f : 8.3f;
    float base_temp   = (NODE_ID == 1) ? -4.2f : -6.8f;   // high-altitude cold
    float base_humid  = (NODE_ID == 1) ? 72.0f : 68.5f;
    uint32_t base_pa  = (NODE_ID == 1) ? 72500UL : 71800UL; // ~2700m ASL equivalent

    // Add a small jitter ±5% to simulate real sensor variance
    float jitter = ((float)(esp_random() % 100) - 50) / 1000.0f;  // ±5%

    // Fixed-point encode for compact wire representation:
    pkt->flow_rate_mLs   = (uint16_t)((base_flow  * (1.0f + jitter)) * 10.0f);
    pkt->temperature_c10 = (int16_t) ((base_temp  * (1.0f + jitter)) * 10.0f);
    pkt->humidity_pct10  = (uint16_t)((base_humid * (1.0f + jitter)) * 10.0f);
    pkt->pressure_pa     = base_pa + (uint32_t)((float)base_pa * jitter * 0.01f);
    pkt->battery_mv      = 3300 - (uint16_t)(g_seq_number % 50);  // simulate slow drain

    // All sensors active on this node
    pkt->sensor_flags = SENSOR_FLOW_ACTIVE | SENSOR_TEMP_ACTIVE |
                        SENSOR_HUMID_ACTIVE | SENSOR_BARO_ACTIVE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  BUILD OUTBOUND PACKET
//  Fills all header and payload fields, then appends the CRC.
// ─────────────────────────────────────────────────────────────────────────────
static void build_packet(MeshPacket *pkt, uint8_t pkt_type) {
    memset(pkt, 0, sizeof(MeshPacket));
    pkt->protocol_version = PROTOCOL_VERSION;
    pkt->packet_type      = pkt_type;
    pkt->node_id          = (uint8_t)NODE_ID;
    pkt->hop_count        = 0;          // direct link — no relay hops yet
    pkt->seq_number       = g_seq_number;
    pkt->timestamp_ms     = (uint32_t)millis();
    pkt->rssi_last_hop    = g_gateway_rssi;
    pkt->tx_power_dbm     = 20;         // default ESP32 max TX power (20 dBm)
    pkt->retry_count      = g_retry_count;

    if (pkt_type == PKT_SENSOR_DATA) {
        read_sensors(pkt);
    }

    // CRC computed LAST, after all other fields are set
    pkt->crc16 = compute_packet_crc(pkt);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TRANSMIT WITH APPLICATION-LEVEL ACK + RETRY
//  Returns true if the packet was acknowledged within the timeout window.
// ─────────────────────────────────────────────────────────────────────────────
static bool transmit_with_retry(MeshPacket *pkt) {
    g_retry_count = 0;

    while (g_retry_count <= MAX_RETRIES) {
        g_send_success = false;
        g_ack_received = false;
        pkt->retry_count = g_retry_count;
        // Recompute CRC because retry_count changed
        pkt->crc16 = compute_packet_crc(pkt);
        memcpy(&g_last_packet, pkt, sizeof(MeshPacket));

        esp_err_t result = esp_now_send(GATEWAY_MAC, (uint8_t *)pkt, sizeof(MeshPacket));
        if (result != ESP_OK) {
            Serial.printf("[TX] esp_now_send error: %s (attempt %u)\n",
                          esp_err_to_name(result), g_retry_count + 1);
            g_retry_count++;
            delay(100);
            continue;
        }

        // Wait for application-level ACK with timeout
        uint32_t t0 = millis();
        while (!g_ack_received && (millis() - t0) < ACK_TIMEOUT_MS) {
            delay(10);  // yield — callbacks fire in Wi-Fi task context
        }

        if (g_ack_received) {
            return true;
        }

        Serial.printf("[TX] No ACK (attempt %u/%u) — retrying in 200 ms\n",
                      g_retry_count + 1, MAX_RETRIES + 1);
        g_retry_count++;
        delay(200);  // back-off before retry
    }

    Serial.printf("[TX] Packet seq=%u DROPPED after %u retries\n",
                  pkt->seq_number, MAX_RETRIES);
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  PRINT PACKET SUMMARY (debug aid)
// ─────────────────────────────────────────────────────────────────────────────
static void print_packet(const MeshPacket *pkt) {
    Serial.println(F("─────────────────────────────────────────"));
    Serial.printf("  [PKT] node=%u  seq=%u  type=0x%02X  hop=%u\n",
                  pkt->node_id, pkt->seq_number, pkt->packet_type, pkt->hop_count);
    Serial.printf("  flow=%.1f mL/s  temp=%.1f°C  humid=%.1f%%  press=%lu Pa\n",
                  pkt->flow_rate_mLs / 10.0f,
                  pkt->temperature_c10 / 10.0f,
                  pkt->humidity_pct10 / 10.0f,
                  (unsigned long)pkt->pressure_pa);
    Serial.printf("  batt=%u mV  RSSI_gw=%d dBm  retries=%u  CRC=0x%04X\n",
                  pkt->battery_mv, (int)pkt->rssi_last_hop,
                  pkt->retry_count, pkt->crc16);
    Serial.println(F("─────────────────────────────────────────"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  ADAPTIVE TX POWER (rudimentary link-quality management)
//  If the gateway RSSI is good (> -60 dBm), reduce TX power to save energy.
//  In a field deployment this mirrors what proper sub-GHz modems do via ADR.
// ─────────────────────────────────────────────────────────────────────────────
static void adapt_tx_power() {
    int8_t rssi = g_gateway_rssi;
    int8_t new_power;
    if (rssi > -50)       new_power = 8;   // excellent link — reduce to 8 dBm
    else if (rssi > -65)  new_power = 14;  // good link — moderate power
    else                  new_power = 20;  // weak link — max power

    esp_wifi_set_max_tx_power(new_power * 4);  // unit: 0.25 dBm steps
    Serial.printf("[PWR] TX power set to %d dBm (RSSI=%d dBm)\n",
                  new_power, (int)rssi);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n╔══════════════════════════════════════╗"));
    Serial.printf( "║  Mountain Mesh — Sensor Node %d       ║\n", NODE_ID);
    Serial.println(F("╚══════════════════════════════════════╝"));

    // ── Wi-Fi: station mode, radio on but not associated ─────────────────────
    // ESP-NOW requires Wi-Fi to be initialised. We use STA mode but never
    // connect to an AP — the radio stays in promiscuous-adjacent raw mode.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.printf("[NET] Node MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[NET] Channel : %u\n", ESPNOW_CHANNEL);

    // Fix the Wi-Fi channel to match the rest of the mesh
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // ── ESP-NOW initialisation ────────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[ERROR] esp_now_init() failed — halting"));
        while (true) { delay(1000); }
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Register gateway as peer
    if (!register_peer(GATEWAY_MAC)) {
        Serial.println(F("[ERROR] Failed to register gateway peer — check MAC"));
        while (true) { delay(1000); }
    }

    Serial.printf("[NET] Gateway peer registered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  GATEWAY_MAC[0], GATEWAY_MAC[1], GATEWAY_MAC[2],
                  GATEWAY_MAC[3], GATEWAY_MAC[4], GATEWAY_MAC[5]);

    // ── Send initial heartbeat ────────────────────────────────────────────────
    MeshPacket hb;
    build_packet(&hb, PKT_HEARTBEAT);
    esp_now_send(GATEWAY_MAC, (uint8_t *)&hb, sizeof(MeshPacket));
    Serial.println(F("[NET] Heartbeat sent — beginning duty cycle"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
//  Runs the full sense → transmit → sleep duty cycle.
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t cycle_start = millis();

    // ── Step 1: Build sensor data packet ─────────────────────────────────────
    MeshPacket pkt;
    build_packet(&pkt, PKT_SENSOR_DATA);
    print_packet(&pkt);

    // ── Step 2: Transmit with retry logic ────────────────────────────────────
    bool delivered = transmit_with_retry(&pkt);
    if (delivered) {
        Serial.printf("[OK]  seq=%u delivered and ACK'd\n", pkt.seq_number);
        adapt_tx_power();   // tune power based on fresh RSSI reading
    } else {
        Serial.printf("[WARN] seq=%u lost — advancing sequence anyway\n", pkt.seq_number);
    }

    // ── Step 3: Advance sequence number ──────────────────────────────────────
    g_seq_number++;  // wraps naturally at uint16 max (65535 → 0)

    // ── Step 4: Light-sleep for the remainder of the duty cycle ──────────────
    // This keeps average current ~1.5–3 mA instead of ~80 mA continuous,
    // directly analogous to LoRa node deep-sleep between transmissions.
    uint32_t elapsed   = millis() - cycle_start;
    uint32_t sleep_ms  = (elapsed < SAMPLE_INTERVAL_MS)
                         ? SAMPLE_INTERVAL_MS - elapsed
                         : 0;

    if (sleep_ms > 50) {
        Serial.printf("[PWR] Light-sleep for %lu ms\n\n", (unsigned long)sleep_ms);
        Serial.flush();
        esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);  // µs
        esp_light_sleep_start();
    }
}
