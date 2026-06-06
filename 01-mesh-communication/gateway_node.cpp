/**
 * ============================================================================
 *  MOUNTAIN MESH NETWORK — GATEWAY NODE FIRMWARE
 *  File:    gateway_node.cpp
 *  Target:  ESP32 (Arduino framework, ESP-IDF v4.x / Arduino Core v2.x+)
 *  Author:  Kartikey — NIT Goa
 *  License: MIT
 * ============================================================================
 *
 *  ARCHITECTURE OVERVIEW
 *  ─────────────────────
 *  The Gateway Node is the central aggregation point of the mesh.
 *  It:
 *    1. Listens passively for inbound MeshPacket frames from any sensor node.
 *    2. Validates each packet (protocol version, CRC, duplicate detection).
 *    3. Sends an application-level PKT_ACK back to the originating node.
 *    4. Decodes and pretty-prints the telemetry on Serial for bench testing.
 *    5. Maintains per-node statistics: packet count, RSSI trend, last seq,
 *       duplicate/out-of-order/drop counters — mirrors what a LoRaWAN NS does.
 *    6. Maintains a watchdog: if a node goes silent for > HEARTBEAT_TIMEOUT_MS,
 *       prints a "node offline" alert (simulates field alarm behaviour).
 *
 *  IN A REAL DEPLOYMENT:
 *    Replace the Serial telemetry dump with:
 *      - MQTT publish over a satellite modem (Iridium/BGAN) or
 *      - SD card append for store-and-forward, or
 *      - BLE to a local Android tablet for ranger inspection.
 *
 *  ROUTING TABLE CONCEPT
 *  ──────────────────────
 *  NodeRegistry[] is a compile-time routing table keyed by node_id.
 *  In a real multi-hop mesh each relay would have one of these per peer
 *  and forward packets whose node_id it doesn't own.
 *  For this 3-node flat topology the gateway is the sole sink, so routing
 *  is implicit — but the data structure is designed for easy extension.
 * ============================================================================
 */

// ─── Arduino / ESP-IDF headers ───────────────────────────────────────────────
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ─── Wi-Fi channel — must match sensor nodes ─────────────────────────────────
static const uint8_t ESPNOW_CHANNEL = 1;

// ─── Known sensor node MAC addresses ─────────────────────────────────────────
// Set these to the actual MAC addresses of your sensor node ESP32 boards.
// Read them by running Serial.println(WiFi.macAddress()) on each sensor node.
static const uint8_t SN1_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
static const uint8_t SN2_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02};

// ─── Protocol constants (must mirror sensor_node.cpp exactly) ─────────────────
static const uint8_t PROTOCOL_VERSION  = 0x01;
static const uint8_t PKT_SENSOR_DATA   = 0x01;
static const uint8_t PKT_ACK           = 0x02;
static const uint8_t PKT_HEARTBEAT     = 0x03;
static const uint8_t PKT_ERROR         = 0xFF;

// ─── Sensor flag bitmask (mirrors sensor_node.cpp) ───────────────────────────
static const uint8_t SENSOR_FLOW_ACTIVE   = (1 << 0);
static const uint8_t SENSOR_TEMP_ACTIVE   = (1 << 1);
static const uint8_t SENSOR_HUMID_ACTIVE  = (1 << 2);
static const uint8_t SENSOR_BARO_ACTIVE   = (1 << 3);

// ─── Watchdog timeout ────────────────────────────────────────────────────────
// If a node sends nothing for this long, emit a "node silent" warning.
static const uint32_t HEARTBEAT_TIMEOUT_MS = 60000;  // 60 s

// ─────────────────────────────────────────────────────────────────────────────
//  MESH PACKET STRUCTURE  (identical layout to sensor_node.cpp)
//  CRITICAL: Both translation units MUST use the same struct definition.
//  In a production system this would live in a shared header (mesh_packet.h).
// ─────────────────────────────────────────────────────────────────────────────
#pragma pack(push, 1)
typedef struct {
    uint8_t  protocol_version;
    uint8_t  packet_type;
    uint8_t  node_id;
    uint8_t  hop_count;
    uint16_t seq_number;
    uint32_t timestamp_ms;
    uint8_t  sensor_flags;
    int8_t   rssi_last_hop;
    uint16_t flow_rate_mLs;
    int16_t  temperature_c10;
    uint16_t humidity_pct10;
    uint32_t pressure_pa;
    uint16_t battery_mv;
    uint8_t  tx_power_dbm;
    uint8_t  retry_count;
    uint16_t crc16;
    uint8_t  reserved[4];
} MeshPacket;
#pragma pack(pop)

static_assert(sizeof(MeshPacket) == 32, "MeshPacket must be exactly 32 bytes");

// ─────────────────────────────────────────────────────────────────────────────
//  PER-NODE STATISTICS STRUCTURE
//  Tracks link quality and data integrity metrics per sensor node.
//  This mirrors the per-device registry a LoRaWAN network server maintains.
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    uint8_t  node_id;               // node identifier (1, 2, ...)
    uint8_t  mac[6];                // node MAC address (for sending ACKs)
    bool     registered;            // is this node in the routing table?
    uint16_t last_seq;              // last received sequence number
    uint32_t last_rx_ms;            // millis() when last packet arrived
    uint32_t packets_rx;            // total valid packets received
    uint32_t packets_dup;           // duplicate packets (same seq repeated)
    uint32_t packets_dropped;       // estimated gaps in sequence number
    uint32_t crc_errors;            // CRC failures (corrupted frames)
    int8_t   rssi_min;              // weakest observed RSSI (dBm)
    int8_t   rssi_max;              // strongest observed RSSI (dBm)
    int16_t  rssi_sum;              // running sum for average (int16 to avoid overflow)
    bool     online;                // currently considered alive
    // Latest sensor readings (decoded from last good packet)
    float    last_flow_mLs;
    float    last_temp_c;
    float    last_humid_pct;
    uint32_t last_pressure_pa;
    uint16_t last_battery_mv;
} NodeStats;

// ─── Routing table: gateway knows about two sensor nodes ─────────────────────
static NodeStats g_nodes[2] = {
    { .node_id = 1, .mac = {SN1_MAC[0], SN1_MAC[1], SN1_MAC[2],
                             SN1_MAC[3], SN1_MAC[4], SN1_MAC[5]},
      .registered = true, .last_seq = 0xFFFF, .online = false,
      .rssi_min = 0, .rssi_max = -127 },
    { .node_id = 2, .mac = {SN2_MAC[0], SN2_MAC[1], SN2_MAC[2],
                             SN2_MAC[3], SN2_MAC[4], SN2_MAC[5]},
      .registered = true, .last_seq = 0xFFFF, .online = false,
      .rssi_min = 0, .rssi_max = -127 }
};
static const uint8_t NUM_NODES = sizeof(g_nodes) / sizeof(g_nodes[0]);

// ─────────────────────────────────────────────────────────────────────────────
//  CRC-16/CCITT-FALSE  (must be identical to sensor_node.cpp)
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

static uint16_t compute_packet_crc(const MeshPacket *pkt) {
    return crc16_ccitt((const uint8_t *)pkt, offsetof(MeshPacket, crc16));
}

// ─── Lookup a NodeStats entry by node_id ─────────────────────────────────────
static NodeStats* find_node(uint8_t node_id) {
    for (uint8_t i = 0; i < NUM_NODES; i++) {
        if (g_nodes[i].node_id == node_id) return &g_nodes[i];
    }
    return nullptr;
}

// ─── Lookup a NodeStats entry by MAC address ─────────────────────────────────
static NodeStats* find_node_by_mac(const uint8_t *mac) {
    for (uint8_t i = 0; i < NUM_NODES; i++) {
        if (memcmp(g_nodes[i].mac, mac, 6) == 0) return &g_nodes[i];
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW peer registration helper (same as sensor_node.cpp)
// ─────────────────────────────────────────────────────────────────────────────
static bool register_peer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;
    return esp_now_add_peer(&peer) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SEND APPLICATION-LEVEL ACK
//  Sends a PKT_ACK back to the originating node, echoing the sequence number
//  so the node can match it to the outstanding transmission.
// ─────────────────────────────────────────────────────────────────────────────
static void send_ack(const uint8_t *dest_mac, uint16_t seq_number, uint8_t dest_node_id) {
    MeshPacket ack;
    memset(&ack, 0, sizeof(MeshPacket));
    ack.protocol_version = PROTOCOL_VERSION;
    ack.packet_type      = PKT_ACK;
    ack.node_id          = 0;           // node_id 0 = gateway
    ack.hop_count        = 0;
    ack.seq_number       = seq_number;  // echo seq so sensor can match
    ack.timestamp_ms     = (uint32_t)millis();
    ack.crc16            = compute_packet_crc(&ack);

    esp_err_t r = esp_now_send(dest_mac, (uint8_t *)&ack, sizeof(MeshPacket));
    if (r == ESP_OK) {
        Serial.printf("[ACK] → Node %u  seq=%u\n", dest_node_id, seq_number);
    } else {
        Serial.printf("[ACK] FAILED to node %u: %s\n",
                      dest_node_id, esp_err_to_name(r));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  DECODE & LOG TELEMETRY
//  Decodes fixed-point sensor values and prints a structured log line.
//  In production this is where you'd write to SD card or push to MQTT.
// ─────────────────────────────────────────────────────────────────────────────
static void log_telemetry(const MeshPacket *pkt, int8_t rssi) {
    // Decode fixed-point representations back to floats for human display
    float flow   = pkt->flow_rate_mLs  / 10.0f;
    float temp   = pkt->temperature_c10 / 10.0f;
    float humid  = pkt->humidity_pct10  / 10.0f;

    // Approximate altitude from pressure using the ISA barometric formula
    // Altitude (m) ≈ 44330 × (1 − (P/P0)^0.1903)
    // P0 = 101325 Pa (sea level ISA)
    float altitude = 44330.0f * (1.0f - powf((float)pkt->pressure_pa / 101325.0f, 0.1903f));

    Serial.println(F("┌─────────────────────────────────────────────────┐"));
    Serial.printf( "│ TELEMETRY  Node:%-3u  Seq:%-5u  Hop:%-3u  RSSI:%d dBm\n",
                   pkt->node_id, pkt->seq_number, pkt->hop_count, (int)rssi);
    Serial.println(F("├─────────────────────────────────────────────────┤"));

    if (pkt->sensor_flags & SENSOR_FLOW_ACTIVE)
        Serial.printf("│  Flow Rate  : %6.1f mL/s\n", flow);
    if (pkt->sensor_flags & SENSOR_TEMP_ACTIVE)
        Serial.printf("│  Temperature: %6.1f °C\n", temp);
    if (pkt->sensor_flags & SENSOR_HUMID_ACTIVE)
        Serial.printf("│  Humidity   : %6.1f %%\n", humid);
    if (pkt->sensor_flags & SENSOR_BARO_ACTIVE)
        Serial.printf("│  Pressure   : %6lu Pa  (~%.0f m ASL)\n",
                      (unsigned long)pkt->pressure_pa, altitude);

    Serial.printf("│  Battery    : %4u mV   TX Power: %u dBm\n",
                  pkt->battery_mv, pkt->tx_power_dbm);
    Serial.printf("│  Retries    : %u        CRC: 0x%04X\n",
                  pkt->retry_count, pkt->crc16);
    Serial.printf("│  Node uptime: %lu ms\n", (unsigned long)pkt->timestamp_ms);
    Serial.println(F("└─────────────────────────────────────────────────┘\n"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  UPDATE NODE STATISTICS
//  Called after each validated inbound packet.
// ─────────────────────────────────────────────────────────────────────────────
static void update_node_stats(NodeStats *node, const MeshPacket *pkt, int8_t rssi) {
    node->online      = true;
    node->last_rx_ms  = millis();

    // Duplicate detection: same seq as last accepted packet
    if (node->last_seq != 0xFFFF && pkt->seq_number == node->last_seq) {
        node->packets_dup++;
        Serial.printf("[STATS] Node %u: DUPLICATE seq=%u\n",
                      node->node_id, pkt->seq_number);
        return;
    }

    // Gap detection: sequence number jumped by more than 1
    // (accounting for uint16 wraparound at 65535)
    if (node->last_seq != 0xFFFF) {
        uint16_t expected = node->last_seq + 1;  // wraps correctly for uint16
        if (pkt->seq_number != expected) {
            uint16_t gap = pkt->seq_number - expected;  // wraps correctly
            // Only treat as drop if gap is small (< 100); larger jump = node restarted
            if (gap < 100) {
                node->packets_dropped += gap;
                Serial.printf("[STATS] Node %u: %u packet(s) dropped (seq %u→%u)\n",
                              node->node_id, gap, node->last_seq, pkt->seq_number);
            }
        }
    }

    // RSSI tracking
    if (rssi < node->rssi_min || node->rssi_min == 0) node->rssi_min = rssi;
    if (rssi > node->rssi_max)                         node->rssi_max = rssi;
    node->rssi_sum = (int16_t)(node->rssi_sum + rssi);

    // Update last known values
    node->last_seq         = pkt->seq_number;
    node->last_flow_mLs    = pkt->flow_rate_mLs  / 10.0f;
    node->last_temp_c      = pkt->temperature_c10 / 10.0f;
    node->last_humid_pct   = pkt->humidity_pct10  / 10.0f;
    node->last_pressure_pa = pkt->pressure_pa;
    node->last_battery_mv  = pkt->battery_mv;
    node->packets_rx++;
}

// ─────────────────────────────────────────────────────────────────────────────
//  PRINT NODE STATUS SUMMARY (called every 30 s from loop)
// ─────────────────────────────────────────────────────────────────────────────
static void print_network_summary() {
    Serial.println(F("\n╔══════════════ NETWORK STATUS SUMMARY ══════════════╗"));
    for (uint8_t i = 0; i < NUM_NODES; i++) {
        NodeStats *n = &g_nodes[i];
        uint32_t age = millis() - n->last_rx_ms;
        float avg_rssi = (n->packets_rx > 0)
                         ? (float)n->rssi_sum / (float)n->packets_rx
                         : 0.0f;

        Serial.printf("║  Node %-2u | %s | RX:%-5lu | DUP:%-3lu | DROP:%-3lu\n",
                      n->node_id,
                      n->online ? "ONLINE " : "OFFLINE",
                      (unsigned long)n->packets_rx,
                      (unsigned long)n->packets_dup,
                      (unsigned long)n->packets_dropped);
        Serial.printf("║           RSSI avg:%.1f  min:%d  max:%d  LastRX:%lus ago\n",
                      avg_rssi, (int)n->rssi_min, (int)n->rssi_max,
                      (unsigned long)(age / 1000));
    }
    Serial.println(F("╚════════════════════════════════════════════════════╝\n"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW CALLBACK: onDataSent  (gateway perspective)
//  The gateway sends ACKs; this tells us if the MAC-layer delivery succeeded.
// ─────────────────────────────────────────────────────────────────────────────
static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println(F("[TX] ACK MAC-layer delivery failed"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW CALLBACK: onDataRecv   ← THIS IS THE HEART OF THE GATEWAY
//  Fires in Wi-Fi ISR context for every inbound ESP-NOW frame.
//
//  Processing pipeline:
//    1. Length guard
//    2. Protocol version check
//    3. CRC integrity check
//    4. Node lookup (routing table)
//    5. Auto-register unknown nodes (dynamic peer discovery)
//    6. Dispatch by packet_type
//    7. Send PKT_ACK
//    8. Update per-node statistics
// ─────────────────────────────────────────────────────────────────────────────
static void onDataRecv(const esp_now_recv_info_t *recv_info,
                       const uint8_t *data, int data_len) {

    // ── Guard 1: Size ─────────────────────────────────────────────────────────
    if (data_len != (int)sizeof(MeshPacket)) {
        Serial.printf("[RX] Bad frame length %d (expected %zu) — discarded\n",
                      data_len, sizeof(MeshPacket));
        return;
    }

    MeshPacket pkt;
    memcpy(&pkt, data, sizeof(MeshPacket));
    int8_t rssi = recv_info->rx_ctrl->rssi;

    // ── Guard 2: Protocol version ─────────────────────────────────────────────
    if (pkt.protocol_version != PROTOCOL_VERSION) {
        Serial.printf("[RX] Protocol mismatch from node %u (v0x%02X vs v0x%02X)\n",
                      pkt.node_id, pkt.protocol_version, PROTOCOL_VERSION);
        return;
    }

    // ── Guard 3: CRC integrity ────────────────────────────────────────────────
    uint16_t expected_crc = compute_packet_crc(&pkt);
    if (pkt.crc16 != expected_crc) {
        // Increment error counter for the relevant node if known
        NodeStats *node = find_node(pkt.node_id);
        if (node) node->crc_errors++;
        Serial.printf("[RX] CRC ERROR node=%u seq=%u  got=0x%04X exp=0x%04X\n",
                      pkt.node_id, pkt.seq_number, pkt.crc16, expected_crc);
        return;
    }

    // ── Guard 4 / 5: Node routing table lookup + dynamic discovery ────────────
    NodeStats *node = find_node(pkt.node_id);
    if (node == nullptr) {
        // Unknown node — dynamic registration (field-deployable, plug-in-play)
        Serial.printf("[NET] Unknown node_id=%u — attempting dynamic registration\n",
                      pkt.node_id);
        // For bench PoC we simply reject unknown nodes to keep routing explicit.
        // In a field build: dynamically allocate a NodeStats entry here.
        return;
    }

    // Ensure the peer is registered in ESP-NOW so we can send the ACK
    if (!register_peer(recv_info->src_addr)) {
        Serial.printf("[NET] Could not register peer for node %u\n", pkt.node_id);
        return;
    }

    // ── Dispatch by packet type ───────────────────────────────────────────────
    switch (pkt.packet_type) {

        case PKT_SENSOR_DATA:
            Serial.printf("[RX] DATA  node=%u  seq=%u  RSSI=%d dBm\n",
                          pkt.node_id, pkt.seq_number, (int)rssi);
            log_telemetry(&pkt, rssi);
            update_node_stats(node, &pkt, rssi);
            send_ack(recv_info->src_addr, pkt.seq_number, pkt.node_id);
            break;

        case PKT_HEARTBEAT:
            Serial.printf("[RX] HEARTBEAT  node=%u  RSSI=%d dBm\n",
                          pkt.node_id, (int)rssi);
            node->online     = true;
            node->last_rx_ms = millis();
            // Reply with ACK so node knows the gateway is alive
            send_ack(recv_info->src_addr, pkt.seq_number, pkt.node_id);
            break;

        case PKT_ACK:
            // Sensor nodes don't typically send ACKs to the gateway,
            // but handle gracefully in case of firmware misconfig
            Serial.printf("[RX] Unexpected PKT_ACK from node=%u — ignored\n",
                          pkt.node_id);
            break;

        default:
            Serial.printf("[RX] Unknown packet type 0x%02X from node=%u\n",
                          pkt.packet_type, pkt.node_id);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  WATCHDOG CHECK
//  Called from loop() to detect silent nodes (simulates field alarm logic).
// ─────────────────────────────────────────────────────────────────────────────
static void check_node_watchdog() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < NUM_NODES; i++) {
        NodeStats *n = &g_nodes[i];
        if (!n->registered) continue;
        if (n->last_rx_ms == 0) continue;  // never heard from this node

        uint32_t age = now - n->last_rx_ms;
        if (age > HEARTBEAT_TIMEOUT_MS && n->online) {
            n->online = false;
            Serial.printf("[ALARM] Node %u went OFFLINE (last seen %lu s ago)\n",
                          n->node_id, (unsigned long)(age / 1000));
            // In a real system: assert alarm GPIO, enqueue telemetry alert to cloud
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n╔══════════════════════════════════════╗"));
    Serial.println(F("║  Mountain Mesh — GATEWAY NODE        ║"));
    Serial.println(F("╚══════════════════════════════════════╝"));

    // ── Wi-Fi initialisation ──────────────────────────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.printf("[NET] Gateway MAC : %s\n", WiFi.macAddress().c_str());
    Serial.printf("[NET] Channel     : %u\n", ESPNOW_CHANNEL);
    Serial.println(F("[NET] Paste the above MAC into GATEWAY_MAC in sensor_node.cpp"));

    // ── ESP-NOW initialisation ────────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[ERROR] esp_now_init() failed — halting"));
        while (true) { delay(1000); }
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Pre-register both sensor nodes as peers so ACKs can be sent immediately
    bool ok1 = register_peer(SN1_MAC);
    bool ok2 = register_peer(SN2_MAC);
    Serial.printf("[NET] Sensor Node 1 peer: %s\n", ok1 ? "registered" : "FAILED");
    Serial.printf("[NET] Sensor Node 2 peer: %s\n", ok2 ? "registered" : "FAILED");

    Serial.println(F("\n[GW] Listening for sensor nodes...\n"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
//  The gateway is event-driven — the heavy lifting happens in onDataRecv().
//  The loop only runs periodic housekeeping tasks.
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    static uint32_t last_summary_ms = 0;

    // ── Periodic network status summary every 30 s ────────────────────────────
    if (millis() - last_summary_ms > 30000UL) {
        print_network_summary();
        last_summary_ms = millis();
    }

    // ── Node watchdog check ───────────────────────────────────────────────────
    check_node_watchdog();

    delay(500);  // loop does not need to be tight; everything is callback-driven
}
