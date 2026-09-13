

// (removed duplicate implementation; only the correct one remains at the end)
/* OpenSprinkler Unified (ESP32-C5) Firmware
 * Zigbee sensor implementation - Gateway/Coordinator mode (internal)
 *
 * This file provides gateway-specific functions called by the runtime
 * dispatcher in sensor_zigbee.cpp. It does NOT define ZigbeeSensor methods
 * (those live in sensor_zigbee.cpp to avoid duplicate symbols).
 *
 * IMPORTANT: sensor_zigbee.cpp and sensor_zigbee_gw.cpp are BOTH compiled.
 * The runtime IEEE 802.15.4 mode determines which set of functions is used:
 *   - IEEE_ZIGBEE_GATEWAY → sensor_zigbee_gw_*() functions (this file)
 *   - IEEE_ZIGBEE_CLIENT  → client functions in sensor_zigbee.cpp
 *   - IEEE_MATTER          → neither (Zigbee disabled)
 *   - IEEE_DISABLED        → neither (radio off)
 */

#include "sensor_zigbee.h"
#include "sensor_zigbee_gw.h"
#include "sensors.h"
#include "OpenSprinkler.h"
#include "opensprinkler_server.h"
#include "ieee802154_config.h"


extern OpenSprinkler os;
extern bool useEth;

#if defined(ESP32C5) && defined(OS_ENABLE_ZIGBEE)

#include <FS.h>
#include <LittleFS.h>
#include "ArduinoJson.hpp"
#include <esp_partition.h>
#include <vector>
#include <cstring>
#include <cctype>
#include <WiFi.h>
#include <esp_err.h>
#include <nvs.h>
extern "C" {

}
#include <esp_ieee802154.h>
#include "esp_zigbee_core.h"
#include "esp_zigbee_secur.h"
#include "nwk/esp_zigbee_nwk.h"
#include "mac/esp_zigbee_mac.h"
#include "Zigbee.h"
#include "ZigbeeEP.h"
#include "sensor_zigbee_common.h"
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/idf_additions.h>
#include "espconnect.h"

// Optional: Restrict Zigbee to a single channel (e.g. 25 = 2475 MHz) to
// minimise radio overlap with WiFi on dual-protocol ESP32-C5.
// If not defined, the Zigbee stack scans all channels 11-26 (default).
// #define ZIGBEE_COEX_CHANNEL_MASK  (1UL << 25)

// High-frequency Tuya/report trace logging. A single chatty Zigbee device can
// emit dozens of frames/sec; logging each one over the 115200 serial console
// saturates the CPU and starves HTTP/UI. Gated OFF by default even in debug
// builds — set to 1 only for targeted Zigbee frame-level debugging.
#ifndef ZB_GW_VERBOSE_LOG
#define ZB_GW_VERBOSE_LOG 0
#endif
#if ZB_GW_VERBOSE_LOG
#define ZB_GW_TRACE(...) DEBUG_PRINTF(__VA_ARGS__)
#else
#define ZB_GW_TRACE(...) do {} while (0)
#endif

// Zigbee Gateway state
static bool gw_zigbee_initialized = false;
static bool gw_zigbee_connected = false;
static unsigned long gw_join_window_end = 0;
static bool gw_zigbee_needs_nvram_reset = false;
// Set once Zigbee.begin() has failed (e.g. internal RAM exhaustion prevents the
// ZBOSS task stack allocation). Retrying begin() is UNSAFE: the report-receiver
// endpoint has already been handed to ZigbeeCore, and a second begin() would
// dereference stale endpoint state → Load access fault panic. Once failed we
// degrade gracefully (gateway disabled until reboot) instead of crashing.
static bool gw_zigbee_begin_failed = false;

// WiFi-off join state machine (Zigbee Gateway on WiFi only).
// Joining a new device fails while WiFi keeps the shared 2.4 GHz radio busy, so
// on WiFi-connected gateways we briefly turn WiFi OFF for the join window and
// automatically reconnect afterwards. On Ethernet gateways this is never used.
enum GwWifiJoinState { GW_WJ_IDLE = 0, GW_WJ_PENDING, GW_WJ_JOINING };
static GwWifiJoinState gw_wj_state = GW_WJ_IDLE;
static uint16_t gw_wj_duration = 0;
static unsigned long gw_wj_timer = 0;   // deadline (millis) for the current phase

// Discovered devices storage
static std::vector<ZigbeeDeviceInfo> gw_discovered_devices;
static constexpr size_t GW_DISCOVERED_MAX = 128;
static constexpr const char* GW_DISCOVERED_DEVICES_FILE = "/zb_gw_devices.json";
static constexpr const char* GW_DISCOVERED_DEVICES_TMP_FILE = "/zb_gw_devices.tmp";
static constexpr const char* GW_DISCOVERED_DEVICES_BAD_FILE = "/zb_gw_devices_bad.json";
static bool gw_discovered_devices_loaded = false;
static bool gw_discovered_devices_dirty = false;

struct GwTuyaScheduledCommand {
    bool used;
    uint16_t short_addr;
    uint8_t endpoint;
    uint8_t command_id;
    uint8_t dp_id;
    uint8_t dp_type;
    uint16_t seq;
    uint8_t payload_len;
    uint8_t payload[32];
    uint8_t sid;          // station this command belongs to (0xFF = none)
};

static constexpr size_t GW_TUYA_SCHEDULE_MAX = 8;
// Station on whose behalf a command is currently being built (0xFF = none);
// set by the switch state machine around its send calls so the ZCL tsn can be
// tied back to the station entry.
static uint8_t gw_ctl_sending_sid = 0xFF;
static void gw_station_ctl_note_sent(uint8_t sid, uint8_t tsn);
// Outgoing Tuya command queue lives in PSRAM to keep it off the internal heap.
static GwTuyaScheduledCommand* gw_tuya_schedule = nullptr;
static uint32_t gw_tuya_next_due_ms = 0;

// Lazily allocate the Tuya command queue in SPIRAM (falls back to internal RAM).
static inline bool ensure_tuya_schedule() {
    if (gw_tuya_schedule) return true;
    gw_tuya_schedule = (GwTuyaScheduledCommand*)heap_caps_calloc(
        GW_TUYA_SCHEDULE_MAX, sizeof(GwTuyaScheduledCommand),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!gw_tuya_schedule) {
        gw_tuya_schedule = (GwTuyaScheduledCommand*)heap_caps_calloc(
            GW_TUYA_SCHEDULE_MAX, sizeof(GwTuyaScheduledCommand),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return gw_tuya_schedule != nullptr;
}

// Minimum spacing between ANY two queued Tuya command sends (global, across
// all devices) to avoid flooding the ZBOSS stack / RF collisions. Urgent sends
// (device just woke up) bypass this; see gw_schedule_tuya_dp_cmd().
#define GW_TUYA_MIN_SEND_SPACING_MS 2000UL

static uint64_t gw_find_ieee_by_short_addr(uint16_t short_addr);
static void gw_tuya_schedule_next();
static bool gw_load_discovered_devices();
static bool gw_save_discovered_devices();
bool sensor_zigbee_gw_query_basic_cluster_by_ieee(uint64_t device_ieee, uint8_t endpoint);
bool sensor_zigbee_gw_query_basic_cluster_by_ieee_attr(uint64_t device_ieee, uint8_t endpoint, uint16_t attr_id);
bool sensor_zigbee_gw_request_dp_query(uint64_t device_ieee, uint8_t endpoint);

static ZigbeeDeviceInfo* gw_find_discovered_device(uint64_t ieee_addr) {
    if (ieee_addr == 0) return nullptr;
    for (auto& dev : gw_discovered_devices) {
        if (dev.ieee_addr == ieee_addr) return &dev;
    }
    return nullptr;
}

static void gw_mark_discovered_devices_dirty() {
    gw_discovered_devices_dirty = true;
}

static void gw_reset_discovered_devices_runtime_fields(ZigbeeDeviceInfo& dev) {
    dev.is_new = false;
    dev.has_responded = true;
    dev.last_rx_at_ms = 0;
    dev.silent_query_count = 0;
    dev.basic_query_attempts = 0;
    dev.logical_lookup_done = false;
    dev.battery = 255;
    dev.lqi = 0;
    dev.rssi = 0;
}

static void gw_clear_discovered_devices_cache(bool remove_persisted_file) {
    gw_discovered_devices.clear();
    gw_discovered_devices_dirty = false;
    gw_discovered_devices_loaded = true;
    if (remove_persisted_file) {
        LittleFS.remove(GW_DISCOVERED_DEVICES_FILE);
        LittleFS.remove(GW_DISCOVERED_DEVICES_TMP_FILE);
    }
}

static bool gw_load_discovered_devices() {
    if (gw_discovered_devices_loaded) return true;
    gw_discovered_devices_loaded = true;
    gw_discovered_devices.clear();

    if (!LittleFS.exists(GW_DISCOVERED_DEVICES_FILE)) return true;

    File file = LittleFS.open(GW_DISCOVERED_DEVICES_FILE, "r");
    if (!file) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Failed to open persisted discovery file"));
        return false;
    }

    ArduinoJson::JsonDocument doc;
    ArduinoJson::DeserializationError err = ArduinoJson::deserializeJson(doc, file);
    file.close();
    if (err) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Discovery file parse error (%s) — renaming to %s\n"),
                     err.c_str(), GW_DISCOVERED_DEVICES_BAD_FILE);
        LittleFS.remove(GW_DISCOVERED_DEVICES_BAD_FILE);
        LittleFS.rename(GW_DISCOVERED_DEVICES_FILE, GW_DISCOVERED_DEVICES_BAD_FILE);
        return false;
    }

    ArduinoJson::JsonArrayConst devices = doc.as<ArduinoJson::JsonArrayConst>();
    if (devices.isNull()) {
        if (doc["devices"].is<ArduinoJson::JsonArrayConst>()) {
            devices = doc["devices"].as<ArduinoJson::JsonArrayConst>();
        }
    }

    if (devices.isNull()) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Discovery file has no device array"));
        return true;
    }

    for (ArduinoJson::JsonVariantConst v : devices) {
        if (!v.is<ArduinoJson::JsonObjectConst>()) continue;
        ArduinoJson::JsonObjectConst obj = v.as<ArduinoJson::JsonObjectConst>();

        ZigbeeDeviceInfo info = {};
        info.ieee_addr = obj["ieee_addr"] | 0ULL;
        info.short_addr = obj["short_addr"] | 0U;
        info.endpoint = obj["endpoint"] | 1U;
        info.device_id = obj["device_id"] | 0U;
        info.app_version = obj["app_version"] | 0xFFU;
        info.stack_version = obj["stack_version"] | 0xFFU;
        info.hw_version = obj["hw_version"] | 0xFFU;
        info.is_tuya = obj["is_tuya"] | false;
        info.discovered_at = obj["discovered_at"] | 0U;
        info.last_seen = obj["last_seen"] | 0U;

        const char* model = obj["model_id"] | "";
        const char* manufacturer = obj["manufacturer"] | "";
        const char* date_code = obj["date_code"] | "";
        const char* sw_build_id = obj["sw_build_id"] | "";
        const char* vendor = obj["vendor"] | "";
        const char* friendly_name = obj["friendly_name"] | "";
        bool is_custom_name = obj["is_custom_name"] | false;
        strncpy(info.model_id, model, sizeof(info.model_id) - 1);
        strncpy(info.manufacturer, manufacturer, sizeof(info.manufacturer) - 1);
        strncpy(info.date_code, date_code, sizeof(info.date_code) - 1);
        strncpy(info.sw_build_id, sw_build_id, sizeof(info.sw_build_id) - 1);
        strncpy(info.vendor, vendor, sizeof(info.vendor) - 1);
        if (friendly_name[0]) {
            strncpy(info.friendly_name, friendly_name, sizeof(info.friendly_name) - 1);
        }
        info.is_custom_name = is_custom_name;

        gw_reset_discovered_devices_runtime_fields(info);
        info.battery = obj["battery"] | 255U;
        info.lqi = obj["lqi"] | 0U;
        info.rssi = (int8_t)(obj["rssi"] | 0);
        if (info.ieee_addr == 0) continue;

        ZigbeeDeviceInfo* existing = gw_find_discovered_device(info.ieee_addr);
        if (existing) {
            *existing = info;
        } else if (gw_discovered_devices.size() < GW_DISCOVERED_MAX) {
            gw_discovered_devices.push_back(info);
        }
    }

    DEBUG_PRINTF(F("[ZIGBEE-GW] Loaded %u persisted Zigbee device(s)\n"),
                 (unsigned)gw_discovered_devices.size());
    return true;
}

static bool gw_save_discovered_devices() {
    if (!gw_discovered_devices_dirty) return true;

    ArduinoJson::JsonDocument doc;
    doc["version"] = 1;
    ArduinoJson::JsonArray arr = doc["devices"].to<ArduinoJson::JsonArray>();
    for (const auto& dev : gw_discovered_devices) {
        ArduinoJson::JsonObject obj = arr.add<ArduinoJson::JsonObject>();
        obj["ieee_addr"] = dev.ieee_addr;
        obj["short_addr"] = dev.short_addr;
        obj["endpoint"] = dev.endpoint;
        obj["device_id"] = dev.device_id;
        obj["app_version"] = dev.app_version;
        obj["stack_version"] = dev.stack_version;
        obj["hw_version"] = dev.hw_version;
        obj["is_tuya"] = dev.is_tuya;
        obj["discovered_at"] = dev.discovered_at;
        if (dev.last_seen != 0) obj["last_seen"] = dev.last_seen;
        if (dev.model_id[0] != '\0') obj["model_id"] = dev.model_id;
        if (dev.manufacturer[0] != '\0') obj["manufacturer"] = dev.manufacturer;
        if (dev.date_code[0] != '\0') obj["date_code"] = dev.date_code;
        if (dev.sw_build_id[0] != '\0') obj["sw_build_id"] = dev.sw_build_id;
        if (dev.vendor[0] != '\0') obj["vendor"] = dev.vendor;
        if (dev.friendly_name[0] != '\0') obj["friendly_name"] = dev.friendly_name;
        if (dev.is_custom_name) obj["is_custom_name"] = true;
        if (dev.battery != 255) obj["battery"] = dev.battery;
        if (dev.lqi != 0) obj["lqi"] = dev.lqi;
        if (dev.rssi != 0) obj["rssi"] = dev.rssi;
    }

    LittleFS.remove(GW_DISCOVERED_DEVICES_TMP_FILE);
    File file = LittleFS.open(GW_DISCOVERED_DEVICES_TMP_FILE, "w");
    if (!file) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Failed to open discovery temp file for writing"));
        return false;
    }

    size_t written = ArduinoJson::serializeJson(doc, file);
    file.close();
    if (written == 0) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Failed to serialize persisted discovery list"));
        LittleFS.remove(GW_DISCOVERED_DEVICES_TMP_FILE);
        return false;
    }

    LittleFS.remove(GW_DISCOVERED_DEVICES_FILE);
    if (!LittleFS.rename(GW_DISCOVERED_DEVICES_TMP_FILE, GW_DISCOVERED_DEVICES_FILE)) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Failed to replace persisted discovery file"));
        return false;
    }

    gw_discovered_devices_dirty = false;
    return true;
}

struct GwDeviceQueryRequest {
    bool used;
    uint64_t ieee_addr;
    uint8_t endpoint;
    bool need_basic;
    bool need_tuya;
    unsigned long next_try_ms;
};

static std::vector<GwDeviceQueryRequest> gw_device_query_queue;
static constexpr unsigned long GW_DEVICE_QUERY_SPACING_MS = 2000UL;

static void gw_queue_device_query(uint64_t ieee_addr, uint8_t endpoint, bool need_basic, bool need_tuya) {
    if (ieee_addr == 0) return;
    if (endpoint == 0) endpoint = 1;

    for (auto& req : gw_device_query_queue) {
        if (!req.used || req.ieee_addr != ieee_addr || req.endpoint != endpoint) continue;
        req.need_basic = req.need_basic || need_basic;
        req.need_tuya = req.need_tuya || need_tuya;
        if (req.next_try_ms == 0) req.next_try_ms = millis();
        DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] merged queue ieee=%016llX ep=%u basic=%u tuya=%u next=%lu\n"),
                     (unsigned long long)ieee_addr, (unsigned)endpoint,
                     req.need_basic ? 1U : 0U, req.need_tuya ? 1U : 0U,
                     (unsigned long)req.next_try_ms);
        return;
    }

    GwDeviceQueryRequest req = {};
    req.used = true;
    req.ieee_addr = ieee_addr;
    req.endpoint = endpoint;
    req.need_basic = need_basic;
    req.need_tuya = need_tuya;
    req.next_try_ms = millis();
    gw_device_query_queue.push_back(req);
    DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] queued ieee=%016llX ep=%u basic=%u tuya=%u\n"),
                 (unsigned long long)ieee_addr, (unsigned)endpoint,
                 need_basic ? 1U : 0U, need_tuya ? 1U : 0U);
}

// True while a station ON/OFF command is outstanding for this device.  A sleepy
// valve fetches ONE pending frame per poll, so discovery traffic (Basic /
// Tuya DP queries) must not compete with the command for that slot.
static bool gw_station_ctl_pending_for(uint64_t ieee);

static void gw_process_device_query_queue() {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) return;
    if (gw_device_query_queue.empty()) return;

    unsigned long now = millis();

    for (auto it = gw_device_query_queue.begin(); it != gw_device_query_queue.end(); ++it) {
        if (!it->used) continue;
        if ((int32_t)(now - it->next_try_ms) < 0) continue;
        if (gw_station_ctl_pending_for(it->ieee_addr)) {
            it->next_try_ms = now + 30000UL;  // station command first
            continue;
        }

        bool sent = false;
        bool tried_any = false;

        if (it->need_tuya) {
            tried_any = true;
            DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] dispatch tuya ieee=%016llX ep=%u\n"),
                         (unsigned long long)it->ieee_addr, (unsigned)it->endpoint);
            bool ok = sensor_zigbee_gw_request_dp_query(it->ieee_addr, it->endpoint);
            if (ok) {
                it->need_tuya = false;
                sent = true;
                it->next_try_ms = now + GW_DEVICE_QUERY_SPACING_MS;
                DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] tuya dispatched ieee=%016llX ep=%u next=%lu\n"),
                             (unsigned long long)it->ieee_addr, (unsigned)it->endpoint,
                             (unsigned long)it->next_try_ms);
            }
        }

        if (it->need_basic && !sent) {
            tried_any = true;
            DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] dispatch basic ieee=%016llX ep=%u\n"),
                         (unsigned long long)it->ieee_addr, (unsigned)it->endpoint);
            bool ok = sensor_zigbee_gw_query_basic_cluster_by_ieee(it->ieee_addr, it->endpoint);
            if (ok) {
                it->need_basic = false;
                sent = true;
                it->next_try_ms = now + GW_DEVICE_QUERY_SPACING_MS;
                DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] basic dispatched ieee=%016llX ep=%u next=%lu\n"),
                             (unsigned long long)it->ieee_addr, (unsigned)it->endpoint,
                             (unsigned long)it->next_try_ms);
            }
        }

        if (tried_any && !sent) {
            DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] deferred ieee=%016llX ep=%u retry=%lu\n"),
                         (unsigned long long)it->ieee_addr, (unsigned)it->endpoint,
                         (unsigned long)it->next_try_ms);
            it->next_try_ms = now + GW_DEVICE_QUERY_SPACING_MS;
            return;
        }

        if (!it->need_basic && !it->need_tuya) {
            gw_device_query_queue.erase(it);
        }
        return;
    }
}

static void gw_tuya_scheduled_send(uint8_t slot) {
    if (!gw_tuya_schedule || slot >= GW_TUYA_SCHEDULE_MAX) return;
    GwTuyaScheduledCommand& cmd = gw_tuya_schedule[slot];
    if (!cmd.used) return;

    esp_zb_zcl_custom_cluster_cmd_req_t req = {};
    req.zcl_basic_cmd.dst_addr_u.addr_short = cmd.short_addr;
    req.zcl_basic_cmd.dst_endpoint = cmd.endpoint;
    req.zcl_basic_cmd.src_endpoint = 10;
    req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_id = 0xEF00;
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    req.dis_default_resp = 1;
    req.custom_cmd_id = cmd.command_id;
    req.data.type = ESP_ZB_ZCL_ATTR_TYPE_SET;
    req.data.size = cmd.payload_len;
    req.data.value = cmd.payload;

    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t tsn = esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();
    cmd.used = false;
    gw_station_ctl_note_sent(cmd.sid, tsn);
    DEBUG_PRINTF(F("[ZIGBEE-GW] Tuya DP cmd dispatched, slot=%u tsn=%u dp=%u seq=%u short=0x%04X\n"),
                 (unsigned)slot, (unsigned)tsn, (unsigned)cmd.dp_id, (unsigned)cmd.seq, cmd.short_addr);
}

struct ZigbeeDeviceAccessTime {
    uint64_t ieee_addr;
    uint16_t short_addr;
    uint32_t last_write_ms;
    uint32_t last_read_ms;
};
static std::vector<ZigbeeDeviceAccessTime> gw_device_access_times;

static uint64_t gw_find_ieee_by_short_addr(uint16_t short_addr) {
    if (short_addr == 0 || short_addr == 0xFFFF || short_addr == 0xFFFE) return 0;
    for (const auto& dev : gw_discovered_devices) {
        if (dev.short_addr == short_addr) return dev.ieee_addr;
    }
    return 0;
}

static uint16_t gw_get_short_addr(uint64_t ieee_addr) {
    if (ieee_addr == 0) return 0xFFFF;
    
    // 1. Authoritative cache lookup first (this has the latest DEVICE_ANNCE address!)
    for (const auto& dev : gw_discovered_devices) {
        if (dev.ieee_addr == ieee_addr && dev.short_addr != 0 && dev.short_addr != 0xFFFF && dev.short_addr != 0xFFFE) {
            return dev.short_addr;
        }
    }
    
    // 2. Stack fallback
    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(ieee_addr >> (i * 8));
    }
    esp_zb_lock_acquire(portMAX_DELAY);
    uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
    esp_zb_lock_release();
    
    return short_addr;
}

static ZigbeeDeviceAccessTime& gw_get_or_create_device_access(uint64_t ieee_addr, uint16_t short_addr) {
    if (ieee_addr == 0 && short_addr != 0 && short_addr != 0xFFFF && short_addr != 0xFFFE) {
        ieee_addr = gw_find_ieee_by_short_addr(short_addr);
    }
    for (auto& entry : gw_device_access_times) {
        if ((ieee_addr != 0 && entry.ieee_addr == ieee_addr) || 
            (short_addr != 0 && short_addr != 0xFFFF && short_addr != 0xFFFE && entry.short_addr == short_addr)) {
            if (ieee_addr != 0 && entry.ieee_addr == 0) entry.ieee_addr = ieee_addr;
            if (short_addr != 0 && short_addr != 0xFFFF && short_addr != 0xFFFE && entry.short_addr == 0) {
                entry.short_addr = short_addr;
            }
            return entry;
        }
    }
    gw_device_access_times.push_back({ieee_addr, short_addr, 0, 0});
    return gw_device_access_times.back();
}

static void gw_record_device_access(uint64_t ieee_addr, uint16_t short_addr, bool is_write) {
    uint32_t now = millis();
    auto& entry = gw_get_or_create_device_access(ieee_addr, short_addr);
    if (is_write) {
        entry.last_write_ms = now;
    } else {
        entry.last_read_ms = now;
    }
}

static bool gw_is_device_access_allowed(uint64_t ieee_addr, uint16_t short_addr, bool is_write, uint32_t cooldown_ms = 5000UL) {
    uint32_t now = millis();
    if (ieee_addr == 0 && short_addr != 0 && short_addr != 0xFFFF && short_addr != 0xFFFE) {
        ieee_addr = gw_find_ieee_by_short_addr(short_addr);
    }
    for (const auto& entry : gw_device_access_times) {
        if ((ieee_addr != 0 && entry.ieee_addr == ieee_addr) || 
            (short_addr != 0 && short_addr != 0xFFFF && short_addr != 0xFFFE && entry.short_addr == short_addr)) {
            if (is_write) {
                // Only space writes against writes. A preceding read (DP query,
                // Basic Cluster) must not delay a valve command: right after a
                // sleepy device answered is the best moment to reach it.
                if (entry.last_write_ms != 0 && (now - entry.last_write_ms < cooldown_ms)) {
                    return false;
                }
            } else {
                if (entry.last_write_ms != 0 && (now - entry.last_write_ms < cooldown_ms)) {
                    return false;
                }
                if (entry.last_read_ms != 0 && (now - entry.last_read_ms < cooldown_ms)) {
                    return false;
                }
            }
            break;
        }
    }
    return true;
}

static void gw_tuya_schedule_next() {
    if (!gw_tuya_schedule) return;
    uint32_t now = millis();
    if (gw_tuya_next_due_ms != 0 && (int32_t)(now - gw_tuya_next_due_ms) < 0) return;

    bool found_any_used = false;
    for (uint8_t slot = 0; slot < GW_TUYA_SCHEDULE_MAX; slot++) {
        if (!gw_tuya_schedule[slot].used) continue;
        found_any_used = true;

        uint16_t short_addr = gw_tuya_schedule[slot].short_addr;
        if (!gw_is_device_access_allowed(0, short_addr, true, 5000UL)) {
            continue; // Space commands/accesses by at least 5s per device
        }

        gw_tuya_scheduled_send(slot);
        gw_record_device_access(0, short_addr, true);
        gw_tuya_next_due_ms = millis() + GW_TUYA_MIN_SEND_SPACING_MS; // min 5s between any two command sends
        return;
    }

    if (found_any_used) {
        // Pending commands exist but are rate-limited per device, recheck in 50ms
        gw_tuya_next_due_ms = millis() + 50UL;
    } else {
        gw_tuya_next_due_ms = 0;
    }
}

static bool gw_schedule_tuya_dp_cmd(uint16_t short_addr, uint8_t endpoint, uint8_t command_id,
                                    uint8_t dp_id, uint8_t dp_type, const uint8_t* payload,
                                    size_t payload_len, uint16_t seq, bool urgent) {
    if (!payload || payload_len == 0 || payload_len > sizeof(gw_tuya_schedule[0].payload)) return false;
    if (!ensure_tuya_schedule()) return false;

    // One pending command per station / per (device, DP, command).  A station
    // retry or a direction change (ON -> OFF) must REPLACE the not-yet-sent
    // command instead of queueing behind it: with the per-device send spacing a
    // sleepy valve otherwise receives the stale opposite command after the new
    // one (seen as "on works, off toggles back, then nothing").
    int8_t reuse = -1;
    for (uint8_t slot = 0; slot < GW_TUYA_SCHEDULE_MAX; slot++) {
        const GwTuyaScheduledCommand& c = gw_tuya_schedule[slot];
        if (!c.used) continue;
        bool same_station = (gw_ctl_sending_sid != 0xFF && c.sid == gw_ctl_sending_sid);
        bool same_cmd     = (gw_ctl_sending_sid == 0xFF && c.sid == 0xFF &&
                             c.short_addr == short_addr && c.endpoint == endpoint &&
                             c.command_id == command_id && c.dp_id == dp_id);
        if (same_station || same_cmd) { reuse = (int8_t)slot; break; }
    }
    if (reuse >= 0) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Tuya DP cmd replaces pending slot=%u (old seq=%u) dp=%u short=0x%04X\n"),
                     (unsigned)reuse, (unsigned)gw_tuya_schedule[reuse].seq, (unsigned)dp_id, short_addr);
    }

    for (uint8_t slot = 0; slot < GW_TUYA_SCHEDULE_MAX; slot++) {
        if (gw_tuya_schedule[slot].used && (int8_t)slot != reuse) continue;
        gw_tuya_schedule[slot].sid = gw_ctl_sending_sid;

        gw_tuya_schedule[slot].used = true;
        gw_tuya_schedule[slot].short_addr = short_addr;
        gw_tuya_schedule[slot].endpoint = endpoint;
        gw_tuya_schedule[slot].command_id = command_id;
        gw_tuya_schedule[slot].dp_id = dp_id;
        gw_tuya_schedule[slot].dp_type = dp_type;
        gw_tuya_schedule[slot].seq = seq;
        gw_tuya_schedule[slot].payload_len = (uint8_t)payload_len;
        memcpy(gw_tuya_schedule[slot].payload, payload, payload_len);

        if (urgent) {
            // The device is awake right now (it just sent us a frame): dispatch
            // immediately, bypassing the per-device cooldown and global spacing.
            gw_tuya_scheduled_send(slot);
            gw_record_device_access(0, short_addr, true);
            gw_tuya_next_due_ms = millis() + 500UL;
            DEBUG_PRINTF(F("[ZIGBEE-GW] Tuya DP cmd sent urgently, dp=%u seq=%u short=0x%04X\n"),
                         (unsigned)dp_id, (unsigned)seq, short_addr);
            return true;
        }
        if (gw_tuya_next_due_ms == 0) gw_tuya_next_due_ms = millis() + 50UL;
        DEBUG_PRINTF(F("[ZIGBEE-GW] Tuya DP cmd queued, slot=%u dp=%u seq=%u short=0x%04X\n"),
                 (unsigned)slot, (unsigned)dp_id, (unsigned)seq, short_addr);
        return true;
    }

    DEBUG_PRINTLN(F("[ZIGBEE-GW] Tuya DP write failed: scheduler queue full"));
    return false;
}

// Try to normalize a possibly stale/truncated station IEEE against runtime
// discovered devices. This handles cases where stored station data contains a
// shifted 64-bit value (e.g. good_ieee >> 8), which would otherwise keep
// short-address resolution in a permanent "unknown" state.
static bool gw_try_fix_ieee(uint64_t* ieee_addr) {
    if (!ieee_addr || *ieee_addr == 0) return false;
    uint64_t in = *ieee_addr;
    for (const auto& dev : gw_discovered_devices) {
        if (dev.ieee_addr == 0) continue;
        if (dev.ieee_addr == in) return false; // already exact

        // Common corruption pattern seen in station payloads: one-byte right
        // shift of the IEEE value. Match both directions defensively.
        if ((dev.ieee_addr >> 8) == in || ((in >> 8) == dev.ieee_addr)) {
            *ieee_addr = dev.ieee_addr;
            DEBUG_PRINTF(F("[ZIGBEE-GW] Corrected station IEEE: %016llX -> %016llX\n"),
                         (unsigned long long)in,
                         (unsigned long long)*ieee_addr);
            return true;
        }
    }
    return false;
}

// Returns true if `ieee_addr` is present in the discovered-devices list.
// Used to distinguish "device is paired but radio hasn't resolved its short
// address yet" (worth queueing/retrying) from "no such device — station
// references an orphan IEEE" (queueing is pointless and just spams the log).
static bool gw_ieee_is_known(uint64_t ieee_addr) {
    if (ieee_addr == 0) return false;
    for (const auto& dev : gw_discovered_devices) {
        if (dev.ieee_addr == ieee_addr) return true;
    }
    return false;
}

// Throttle for the "orphan station" log line: print at most one warning per
// IEEE per boot so a stale station doesn't flood the serial monitor.
static bool gw_orphan_warned(uint64_t ieee_addr) {
    static uint64_t warned[8] = {0};
    static uint8_t  warned_idx = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (warned[i] == ieee_addr) return true;
    }
    warned[warned_idx++ & 7] = ieee_addr;
    return false;
}

// Zigbee Cluster IDs
#define ZB_ZCL_CLUSTER_ID_BASIC                     0x0000
#define ZB_ZCL_CLUSTER_ID_POWER_CONFIG              0x0001
#define ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT   0x0400
#define ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT          0x0402
#define ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT      0x0403
#define ZB_ZCL_CLUSTER_ID_FLOW_MEASUREMENT          0x0404
#define ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT  0x0405
#define ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING         0x0406
#define ZB_ZCL_CLUSTER_ID_LEAF_WETNESS              0x0407
#define ZB_ZCL_CLUSTER_ID_SOIL_MOISTURE             0x0408
#define ZB_ZCL_CLUSTER_ID_METERING                  0x0702

// Basic Cluster attribute IDs
#define ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID     0x0001
#define ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID           0x0002
#define ZB_ZCL_ATTR_BASIC_HW_VERSION_ID              0x0003
#define ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID       0x0004
#define ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID        0x0005
#define ZB_ZCL_ATTR_BASIC_DATE_CODE_ID               0x0006
#define ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID            0x0007
#define ZB_ZCL_ATTR_BASIC_SW_BUILD_ID                0x4000

// Active one-shot read request state (Gateway mode)
static uint16_t gw_read_attr_id = 0;
static bool     gw_read_pending = false;
static unsigned long gw_read_time = 0;
static uint64_t gw_read_pending_ieee = 0;     // IEEE of the pending active read
static uint16_t gw_read_pending_cluster = 0;  // cluster of the pending active read
// Set by zbReadBasicCluster() after a Basic response was handled; the pending
// read slot is released on the next loop tick instead of inside the callback.
// A multi-attribute (batch) Basic read is delivered as several back-to-back
// zbReadBasicCluster() calls in ONE response frame — clearing gw_read_pending
// on the first attribute made every following attribute (manufacturer 0x0004,
// model 0x0005, ...) be dropped, so non-Tuya devices never got their name.
static volatile bool gw_read_clear_after_basic = false;
static uint64_t gw_read_timeout_ieee = 0;     // IEEE that most recently timed out
static unsigned long gw_read_block_until_ms = 0; // per-device cooldown after timeout
// Shorter read window: a reachable device answers a Basic/Tuya read in well
// under 1 s; an out-of-range or sleeping device never answers at all.  The old
// 10 s wall meant a single unreachable device held the one shared read slot for
// 10 s, saturating identification.  4 s recycles the slot ~2.5× faster without
// dropping legitimate slow responders.
#define GW_READ_TIMEOUT_MS 4000

struct GwOneShotThrottleEntry {
    uint64_t ieee_addr;
    unsigned long last_sent_ms;
};

static std::vector<GwOneShotThrottleEntry> gw_oneshot_throttle;
static constexpr unsigned long GW_ONESHOT_MIN_DELAY_MS = 5000UL;

static bool gw_allow_oneshot_for_device(uint64_t ieee_addr, const char* kind) {
    if (ieee_addr == 0) return true;
    uint32_t now = millis();
    for (auto& entry : gw_oneshot_throttle) {
        if (entry.ieee_addr == ieee_addr) {
            if (now - entry.last_sent_ms < GW_ONESHOT_MIN_DELAY_MS) {
                unsigned long remaining = GW_ONESHOT_MIN_DELAY_MS - (now - entry.last_sent_ms);
                DEBUG_PRINTF(F("[ZIGBEE-GW][ONESHOT] BLOCKED: ieee=%016llX kind=%s remaining=%lums\n"),
                             (unsigned long long)ieee_addr, kind, remaining);
                return false;
            }
            entry.last_sent_ms = now;
            return true;
        }
    }
    gw_oneshot_throttle.push_back({ieee_addr, now});
    return true;
}

// Static storage for Basic Cluster read requests.
// attr_field is processed asynchronously by the Zigbee stack and must
// outlive the caller stack frame.
static uint16_t s_gw_basic_query_attrs[] = {
    ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID,
    ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID,
    ZB_ZCL_ATTR_BASIC_HW_VERSION_ID,
    ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
    ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
    ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
    ZB_ZCL_ATTR_BASIC_SW_BUILD_ID
};

// Set when any sensor's comm_mode changes during report processing;
// sensor_save() is then called from sensor_zigbee_gw_loop() (main loop thread).
static bool gw_comm_mode_changed = false;

// Configure Reporting planning — schedule ZCL "Configure Reporting" commands
// to tell sleeping end devices (e.g. AQARA T&H) to proactively push attribute
// reports at the specified interval instead of relying on active reads.
struct GwConfigReportRequest {
    uint64_t ieee_addr;
    uint8_t  endpoint;
    uint16_t cluster_id;
    uint16_t attr_id;
    uint16_t min_interval;  // seconds: device won't report more often than this
    uint16_t max_interval;  // seconds: device reports at least this often
    unsigned long scheduled_time;
};
static std::vector<GwConfigReportRequest> gw_config_report_queue;
static unsigned long gw_last_config_report_ms = 0;
#define GW_DEFAULT_REPORT_INTERVAL 900  // 15 minutes (default reporting interval for sleepy devices)
#define GW_CONFIG_REPORT_STAGGER_MS 600  // ms between successive configure-report sends
struct GwBindRequest {
    uint64_t ieee_addr;
    uint8_t endpoint;
    uint16_t cluster_id;
    unsigned long scheduled_time;
};
static std::vector<GwBindRequest> gw_bind_queue;
static unsigned long gw_last_bind_req_ms = 0;
#define GW_BIND_REQ_STAGGER_MS 600  // ms between successive bind sends
// How long the coordinator keeps a pending unicast for a sleepy child
// (macTransactionPersistenceTime).  0 = leave the 802.15.4 default (7.68 s).
// Longer values were tested (60 s / 300 s): they did not help Third Reality
// sensors (those never poll outside pairing) and 300 s exhausted the ZBOSS
// buffer pool ("zb_bufpool_mult.c:1233" assertion) because every pending
// Bind/ConfigReport/Read/station command occupies a buffer for that long.
#ifndef GW_MAC_PERSISTENCE_S
#define GW_MAC_PERSISTENCE_S 0
#endif
// Keepalive method the coordinator advertises to joining end devices (Parent
// Information in the End Device Timeout Response).  1 = MAC Data Poll only:
// a child then has to keep its parent alive with data polls, which are also
// the moments it can receive queued commands.  2 = ED Timeout Request only,
// 3 = both (ZBOSS default), 0 = none.  Experiment for sleepy GIEX valves that
// otherwise never poll outside their own reports; takes effect at (re)join.
#ifndef GW_KEEPALIVE_MODE
#define GW_KEEPALIVE_MODE 1
#endif
// Default end-device timeout granted to children that do not request one.
#ifndef GW_ED_TIMEOUT
#define GW_ED_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#endif
// 802.15.4 transmit power of the coordinator in dBm.  The C5 PHY allows up to
// 20 dBm (CONFIG_ESP_PHY_MAX_TX_POWER); ETSI EN 300 328 permits 20 dBm EIRP
// at 2.4 GHz, so with a ~0-2 dBi PCB antenna 20 dBm conducted is the limit.
// Only the coordinator -> device direction improves; the device's own TX
// power (what we see as RSSI in the neighbor table) is unaffected.
// 0 = leave the stack default.
#ifndef GW_ZB_TX_POWER_DBM
#define GW_ZB_TX_POWER_DBM 20
#endif
extern "C" esp_err_t esp_zb_nwk_set_keepalive_mode(int mode);
extern "C" esp_err_t esp_zb_nwk_set_ed_timeout(esp_zb_aging_timeout_t timeout);

// Battery reporting (Power Configuration 0x0001 / BatteryPercentageRemaining
// 0x0021).  Standard ZCL sleepy end devices (Third Reality, Sonoff, Aqara, ...)
// only push battery reports after the coordinator has bound the cluster and
// configured reporting — exactly what zigbee2mqtt's `battery()` extend does.
// Bind + ConfigureReporting are queued through the regular queues above; the
// one-time read below fetches the initial value while the device is awake.
#define GW_BATTERY_REPORT_MIN_S 600     // at most one report per 10 min
#define GW_BATTERY_REPORT_MAX_S 43200   // at least one report every 12 h
#define GW_BATTERY_READ_RETRY_MS 3000   // retry spacing when the read slot is busy
#define GW_BATTERY_READ_MAX_ATTEMPTS 3
#define GW_BATTERY_SECOND_READ_MS 45000 // second read after a join (device still awake)
struct GwBatteryReadRequest {
    uint64_t ieee_addr;
    uint8_t  endpoint;
    uint8_t  attempts;
    unsigned long scheduled_time;
};
static std::vector<GwBatteryReadRequest> gw_battery_read_queue;


// Tuya manufacturer-specific cluster (manuSpecificTuya)
#define ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC             0xEF00

struct GwBasicQueryRequest {
    uint64_t ieee_addr;
    uint16_t short_addr;
    uint8_t endpoint;
    uint8_t attempts;
    unsigned long next_query_ms;
    uint16_t target_attr_id; // 0xFFFF for all, otherwise specific attribute ID
    bool tuya_tried;
};
static std::vector<GwBasicQueryRequest> gw_basic_query_queue;
#define GW_BASIC_QUERY_RETRY_MS 15000UL
#define GW_BASIC_QUERY_MAX_ATTEMPTS 20
// Hard per-device cap on how many Basic Cluster requeries we perform while the
// manufacturer/model is still empty.  Reaching this cap stops the requery until
// the device physically re-announces (DEVICE_ANNCE), which resets the budget.
// Without this cap a frequently-reporting Tuya end device (e.g. GIEX GX03
// "_TZE284_") that never answers Basic Cluster reads is bombarded with read
// requests on every DP report, overwhelming the sleepy device so it drops its
// parent and re-enters join mode (blinking LEDs) — and the manufacturer is
// never stored.
#define GW_BASIC_QUERY_ATTEMPT_CAP 12

// Gentle periodic "wake" for devices that never identified themselves. Many
// Tuya valves only report their DPs (and answer Basic Cluster reads) right
// after they receive a frame — e.g. when the valve is physically toggled. To
// mimic that we send ONE payload-less Tuya DP query (0x03, the same lightweight
// "report all DPs" used on announce) to a single unidentified device per
// interval. The long interval + one-device-per-tick keeps sleepy end devices
// from being flooded.
#define GW_WAKE_UNIDENTIFIED_INTERVAL_MS 60000UL

static bool gw_is_basic_query_queued(uint64_t ieee_addr) {
    for (const auto& q : gw_basic_query_queue) {
        if (q.ieee_addr == ieee_addr) return true;
    }
    return false;
}

static bool gw_device_needs_basic_info(const ZigbeeDeviceInfo& dev) {
    return dev.ieee_addr != 0 &&
           (dev.manufacturer[0] == '\0' || strcmp(dev.manufacturer, "unknown") == 0 ||
            dev.model_id[0] == '\0' || strcmp(dev.model_id, "unknown") == 0);
}

static void gw_queue_basic_cluster_query_attr(uint64_t ieee_addr, uint16_t short_addr, uint8_t endpoint, uint16_t attr_id, unsigned long delay_ms) {
    if (ieee_addr == 0) return;
    if (endpoint == 0) endpoint = 1;

    unsigned long next_query_ms = millis() + delay_ms;
    for (auto& q : gw_basic_query_queue) {
        if (q.ieee_addr != ieee_addr || q.target_attr_id != attr_id) continue;
        q.short_addr = short_addr;
        q.endpoint = endpoint;
        if ((int32_t)(next_query_ms - q.next_query_ms) < 0) {
            q.next_query_ms = next_query_ms;
        }
        return;
    }

    GwBasicQueryRequest item = {};
    item.ieee_addr = ieee_addr;
    item.short_addr = short_addr;
    item.endpoint = endpoint;
    item.attempts = 0;
    item.next_query_ms = next_query_ms;
    item.target_attr_id = attr_id;
    item.tuya_tried = false;
    gw_basic_query_queue.push_back(item);
}

static void gw_queue_basic_cluster_query(uint64_t ieee_addr, uint16_t short_addr, uint8_t endpoint, unsigned long delay_ms) {
    gw_queue_basic_cluster_query_attr(ieee_addr, short_addr, endpoint, 0xFFFF, delay_ms);
}

static void gw_process_basic_query_queue() {
    if (!Zigbee.started() || !Zigbee.connected()) return;
    // Release a Basic read slot that was held open across a (possibly
    // multi-attribute) response so the whole batch could be processed.
    if (gw_read_clear_after_basic) {
        gw_read_clear_after_basic = false;
        gw_read_pending = false;
    }
    if (gw_read_pending) return;
    if (gw_basic_query_queue.empty()) return;

    unsigned long now = millis();
    auto& item = gw_basic_query_queue.front();

    // Settling window after another device's Basic read just timed out.
    // A sleepy Tuya device frequently answers its Basic read slightly AFTER our
    // 4 s timeout, and that response arrives via the no-address
    // zbReadBasicCluster() callback which can only attribute it to whatever
    // device is currently the pending read. If we immediately start
    // interviewing a *different* device here, the late response would be
    // stamped onto the wrong device — the reported "registration gets mixed up
    // when several devices join at once". By deferring a different device until
    // the settling window elapses, no read is pending meanwhile, so the stray
    // late response is safely dropped instead of misattributed.
    if (gw_read_timeout_ieee != 0 && item.ieee_addr != gw_read_timeout_ieee &&
        (long)(gw_read_block_until_ms - now) > 0) {
        return;
    }

    if ((int32_t)(now - item.next_query_ms) < 0) return;
    if (gw_station_ctl_pending_for(item.ieee_addr)) {
        item.next_query_ms = now + 30000UL;  // station command first
        return;
    }

    bool is_tuya_dev = false;
    ZigbeeDeviceInfo* dev = gw_find_discovered_device(item.ieee_addr);
    if (dev && dev->is_tuya) {
        is_tuya_dev = true;
    }

    if (item.target_attr_id == 0xFFFF && is_tuya_dev && !item.tuya_tried) {
        item.tuya_tried = true;
        DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Device ieee=%016llX is known to be Tuya. Querying Tuya DP data first!\n"),
                     (unsigned long long)item.ieee_addr);
        if (sensor_zigbee_gw_request_dp_query(item.ieee_addr, item.endpoint)) {
            gw_read_pending = true;
            gw_read_time = millis();
            gw_read_pending_ieee = item.ieee_addr;
            gw_read_pending_cluster = ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC;
            gw_read_attr_id = 0xFFFF;
            item.next_query_ms = now + GW_READ_TIMEOUT_MS + 500;
            return;
        } else {
            DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Tuya DP query send failed for ieee=%016llX. Falling back immediately to ZigBee basic query.\n"),
                         (unsigned long long)item.ieee_addr);
        }
    }

    bool ok = false;
    if (item.target_attr_id == 0xFFFF) {
        if (is_tuya_dev) {
            // Strict Tuya devices reject 7-attribute batch basic queries; query manufacturer and model individually.
            ok = sensor_zigbee_gw_query_basic_cluster_by_ieee_attr(item.ieee_addr, item.endpoint, ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID);
            gw_queue_basic_cluster_query_attr(item.ieee_addr, item.short_addr, item.endpoint, ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 500UL);
        } else {
            ok = sensor_zigbee_gw_query_basic_cluster_by_ieee(item.ieee_addr, item.endpoint);
        }
    } else {
        ok = sensor_zigbee_gw_query_basic_cluster_by_ieee_attr(item.ieee_addr, item.endpoint, item.target_attr_id);
    }

    if (ok) {
        gw_basic_query_queue.erase(gw_basic_query_queue.begin());
        return;
    }

    item.attempts++;
    if (item.attempts >= GW_BASIC_QUERY_MAX_ATTEMPTS) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic Cluster query (attr=0x%04X) dropped after %u attempts: ieee=%016llX\n"),
                     item.target_attr_id, item.attempts, (unsigned long long)item.ieee_addr);
        gw_basic_query_queue.erase(gw_basic_query_queue.begin());
        return;
    }

    item.next_query_ms = now + GW_BASIC_QUERY_RETRY_MS;
}

// Tuya DP command IDs on Zigbee private cluster 0xEF00.
// Reference: Tuya "Zigbee Generic Interfaces" private command table.
#define TUYA_CMD_QUERY_DP_DATA  0x00  // TY_DATA_REQUEST: gateway -> device DP write/request
#define TUYA_CMD_RESPOND_DP     0x01  // TY_DATA_RESPONE: device -> gateway response
#define TUYA_CMD_DP_REPORT      0x02  // TY_DATA_REPORT: device -> gateway proactive report (two-way ACK)
#define TUYA_CMD_QUERY_REQ      0x03  // TY_DATA_QUERY: gateway -> device query all DPs
#define TUYA_CMD_DP_SEND        0x04  // Legacy/vendor variant seen on some devices
#define TUYA_CMD_ACTIVE_REPORT  0x05  // activeStatusReport: device -> gateway DP report variant
#define TUYA_CMD_ACTIVE_REPORT2 0x06  // activeStatusReportAlt: device -> gateway DP report variant
// 0x07: bridged TuyaMCU "report status" (serial CMD 0x07) tunnelled over the
// Zigbee EF00 cluster by some devices (e.g. GX water valves). Same DP payload
// layout as 0x02, so parse it as a report instead of dropping it.
#define TUYA_CMD_MCU_STATUS_REPORT 0x07
// 0x22: TuyaMCU synchronous status report. Per Tuya docs the DP payload format
// is identical to the asynchronous 0x07 report, so parse it the same way.
#define TUYA_CMD_MCU_STATUS_REPORT_SYN 0x22
#define TUYA_CMD_REPORT_DP_DATA 0x02  // Keep alias for historical naming in this file
#define TUYA_CMD_REPORT_NO_LINK 0x2C  // Legacy/vendor extension (not primary path)
#define TUYA_CMD_MCU_VERSION_REQ  0x10  // Device → gateway (MCU version query)
#define TUYA_CMD_MCU_VERSION_RESP 0x11  // Gateway → device (MCU version answer)
#define TUYA_CMD_TIME_SYNC_REQ  0x24  // Device → gateway (time sync request)

// Legacy aliases for compatibility
#define TUYA_CMD_DATA_REQUEST   TUYA_CMD_QUERY_DP_DATA
#define TUYA_CMD_DATA_RESPONSE  TUYA_CMD_RESPOND_DP
#define TUYA_CMD_DATA_REPORT    TUYA_CMD_DP_REPORT
#define TUYA_CMD_DATA_QUERY     TUYA_CMD_QUERY_REQ
#define TUYA_CMD_DATA_SEND      TUYA_CMD_DP_SEND
#define TUYA_CMD_ACTIVE_STATUS  TUYA_CMD_DP_REPORT

// TUYA_TYPE_* and TUYA_REPORT_* live in sensor_zigbee_common.h

static bool gw_is_ignorable_unmatched_tuya_dp(uint16_t dp, uint64_t ieee_addr) {
    // Primary check: if this DP is registered as a secondary channel on any
    // sensor with the same IEEE address (battery, unit, status, consumption),
    // it is handled internally and should not produce a NO MATCH warning.
    if (ieee_addr != 0) {
        SensorIterator it = sensors_iterate_begin();
        SensorBase* sensor;
        while ((sensor = sensors_iterate_next(it)) != NULL) {
            if (!sensor || sensor->type != SENSOR_ZIGBEE) continue;
            ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(sensor);
            if (zb->device_ieee != ieee_addr) continue;
            if ((zb->tuya_dp_battery    >= 0 && dp == (uint16_t)zb->tuya_dp_battery)    ||
                (zb->tuya_dp_unit       >= 0 && dp == (uint16_t)zb->tuya_dp_unit)       ||
                (zb->tuya_dp_status     >= 0 && dp == (uint16_t)zb->tuya_dp_status)     ||
                (zb->tuya_dp_consumption >= 0 && dp == (uint16_t)zb->tuya_dp_consumption)) {
                return true;
            }
        }
    }
    // Fallback: universally ignorable meta/vendor DPs emitted by many devices
    // regardless of configuration (unit selectors, vendor meta channels).
    return (dp == 9   ||   // unit selector (common across Tuya devices)
            dp == 0x65 ||  // vendor meta
            dp == 0x66 ||  // vendor meta
            dp == 0x6F);   // vendor meta/status
}

static uint8_t gw_hex_nibble(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(10 + c - 'A');
    if (c >= 'a' && c <= 'f') return (uint8_t)(10 + c - 'a');
    return 0;
}

static uint64_t gw_parse_ieee_hex(const char* hex16) {
    uint64_t ieee = 0;
    if (!hex16) return 0;
    for (uint8_t i = 0; i < 16; i++) {
        ieee = (ieee << 4) | gw_hex_nibble(hex16[i]);
    }
    return ieee;
}

static uint8_t gw_parse_hex_u8(const char* hex, uint8_t len) {
    uint8_t value = 0;
    if (!hex) return 0;
    while (len--) {
        value = (uint8_t)((value << 4) | gw_hex_nibble(*hex++));
    }
    return value;
}

static bool gw_ieee_matches_station(uint64_t station_ieee, uint64_t report_ieee) {
    if (station_ieee == 0 || report_ieee == 0) return false;
    return station_ieee == report_ieee ||
           (station_ieee >> 8) == report_ieee ||
           (report_ieee >> 8) == station_ieee;
}

// Custom Tuya and standard battery report parser helper

// Lazy-loading report cache
struct ZigbeeAttributeReport {
    uint64_t ieee_addr;
    uint8_t endpoint;
    uint16_t cluster_id;
    uint16_t attr_id;
    int32_t value;
    uint8_t lqi;
    unsigned long timestamp;
    bool consumed;   // true once matched to a sensor — skip on next iteration
};

static constexpr size_t MAX_PENDING_REPORTS = 64;
// Report cache is allocated in PSRAM to keep it off the scarce internal heap.
static ZigbeeAttributeReport* pending_reports = nullptr;
static size_t pending_report_count = 0;
static constexpr unsigned long REPORT_VALIDITY_MS = 60000;

// ---------------------------------------------------------------------------
// RX ingress queue: Zigbee task -> main loop hand-off.
//
// Every ZBOSS/ZigbeeCore callback (attribute reports, Tuya APS frames, Basic
// Cluster answers, device announces, command send status) runs in the Zigbee
// task. Those callbacks used to touch the report cache, the discovered-device
// vector, the station table (LittleFS reads!) and the sensor map directly,
// which (a) raced against the main loop compacting the same structures and
// (b) stalled the Zigbee stack for hundreds of milliseconds per frame, so
// sleepy end devices missed their indirect-transmission window.
// Now the callbacks only copy the raw frame data into this queue; everything
// else happens in gw_rx_drain() from sensor_zigbee_gw_loop().
// ---------------------------------------------------------------------------
enum GwRxKind : uint8_t {
    GW_RX_SEEN = 0,      // device sent a control frame (time sync, MCU version, ...)
    GW_RX_ANNOUNCE,      // DEVICE_ANNCE / findEndpoint
    GW_RX_ATTR,          // numeric ZCL attribute report / read response
    GW_RX_TUYA_DP,       // one Tuya datapoint record
    GW_RX_BASIC,         // Basic Cluster attribute (string or u8) response
    GW_RX_SEND_STATUS,   // ZCL command send status (APS confirm)
    GW_RX_DEFAULT_RESP,  // ZCL default response (cluster + status)
};

struct GwRxEvent {
    uint8_t  kind;
    uint8_t  endpoint;
    uint16_t short_addr;
    uint64_t ieee;
    uint16_t cluster_id;
    uint16_t attr_id;      // ZCL attribute id or Tuya DP number
    int32_t  value;
    uint8_t  lqi;
    uint8_t  dp_type;      // Tuya DP type, or ZCL attribute type for GW_RX_BASIC
    uint8_t  tsn;          // GW_RX_SEND_STATUS
    int32_t  status;       // GW_RX_SEND_STATUS (esp_err_t)
    uint8_t  raw_len;
    uint8_t  raw[34];      // GW_RX_BASIC payload (ZCL string incl. length byte, or u8)
};

static constexpr UBaseType_t GW_RX_QUEUE_LEN = 96;
static QueueHandle_t gw_rx_queue = nullptr;
static uint32_t gw_rx_dropped = 0;

static bool gw_rx_ensure_queue() {
    if (gw_rx_queue) return true;
    gw_rx_queue = xQueueCreateWithCaps(GW_RX_QUEUE_LEN, sizeof(GwRxEvent), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!gw_rx_queue) gw_rx_queue = xQueueCreate(GW_RX_QUEUE_LEN, sizeof(GwRxEvent));
    return gw_rx_queue != nullptr;
}

// Called from Zigbee-task callbacks: never blocks, never touches shared state.
static bool gw_rx_push(const GwRxEvent& ev) {
    if (!gw_rx_queue) return false;
    if (xQueueSend(gw_rx_queue, &ev, 0) != pdTRUE) {
        gw_rx_dropped++;
        return false;
    }
    return true;
}

static uint64_t gw_ieee_from_raw(const esp_zb_ieee_addr_t raw) {
    uint64_t ieee = 0;
    for (int i = 7; i >= 0; i--) ieee = (ieee << 8) | raw[i];
    return ieee;
}

// Resolve the IEEE of a short address (stack call, valid in Zigbee context)
// and queue a "device is awake" event for the main loop.
static void gw_rx_push_seen_from_short(uint16_t short_addr, uint8_t endpoint) {
    esp_zb_ieee_addr_t raw_ieee;
    if (esp_zb_ieee_address_by_short(short_addr, raw_ieee) != ESP_OK) return;
    uint64_t ieee = gw_ieee_from_raw(raw_ieee);
    if (!ieee) return;
    GwRxEvent ev = {};
    ev.kind = GW_RX_SEEN;
    ev.short_addr = short_addr;
    ev.endpoint = endpoint;
    ev.ieee = ieee;
    gw_rx_push(ev);
}

static void gw_rx_push_tuya_dp(uint64_t ieee, uint16_t short_addr, uint8_t endpoint,
                               uint8_t dp_number, uint8_t dp_type, int32_t value, uint8_t lqi) {
    GwRxEvent ev = {};
    ev.kind = GW_RX_TUYA_DP;
    ev.short_addr = short_addr;
    ev.endpoint = endpoint;
    ev.ieee = ieee;
    ev.cluster_id = ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC;
    ev.attr_id = dp_number;
    ev.dp_type = dp_type;
    ev.value = value;
    ev.lqi = lqi;
    gw_rx_push(ev);
}

// Copy a Basic Cluster attribute (string or u8) so it can be processed later.
static void gw_rx_push_basic(uint64_t ieee, uint16_t short_addr, uint8_t endpoint,
                             const esp_zb_zcl_attribute_t *attribute) {
    if (!attribute || !attribute->data.value) return;
    GwRxEvent ev = {};
    ev.kind = GW_RX_BASIC;
    ev.short_addr = short_addr;
    ev.endpoint = endpoint;
    ev.ieee = ieee;
    ev.cluster_id = ZB_ZCL_CLUSTER_ID_BASIC;
    ev.attr_id = attribute->id;
    ev.dp_type = (uint8_t)attribute->data.type;
    const uint8_t* raw = (const uint8_t*)attribute->data.value;
    size_t n = 0;
    if (attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING) {
        uint8_t len = raw[0];
        if (len == 0xFF) len = 0;
        n = (size_t)len + 1;
        if (n > sizeof(ev.raw)) { n = sizeof(ev.raw); }
        memcpy(ev.raw, raw, n);
        ev.raw[0] = (uint8_t)(n - 1);           // clamp the length byte to what was copied
    } else if (attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_LONG_CHAR_STRING) {
        uint16_t len = raw[0] | ((uint16_t)raw[1] << 8);
        if (len == 0xFFFF) len = 0;
        n = (size_t)len + 2;
        if (n > sizeof(ev.raw)) { n = sizeof(ev.raw); }
        memcpy(ev.raw, raw, n);
        uint16_t copied = (uint16_t)(n - 2);
        ev.raw[0] = (uint8_t)(copied & 0xFF);
        ev.raw[1] = (uint8_t)(copied >> 8);
    } else {
        n = 1;
        ev.raw[0] = raw[0];
    }
    ev.raw_len = (uint8_t)n;
    gw_rx_push(ev);
}

// Hook for the station switch state machine: "this device is awake right now".
static void gw_station_ctl_on_device_rx(uint64_t ieee);
static void gw_send_status_cb(esp_zb_zcl_command_send_status_message_t msg);

// Lazily allocate the report cache in SPIRAM (falls back to internal RAM).
static inline bool ensure_report_cache() {
    if (pending_reports) return true;
    pending_reports = (ZigbeeAttributeReport*)heap_caps_calloc(
        MAX_PENDING_REPORTS, sizeof(ZigbeeAttributeReport),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pending_reports) {
        pending_reports = (ZigbeeAttributeReport*)heap_caps_calloc(
            MAX_PENDING_REPORTS, sizeof(ZigbeeAttributeReport),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return pending_reports != nullptr;
}

// Reclaim cache slots occupied by already-consumed or expired reports.
// The regular compaction happens at the end of sensor_zigbee_gw_process_reports(),
// but a chatty device can burst many distinct reports (e.g. a Tuya valve emitting
// dozens of DPs) faster than the loop drains, filling the cache. Freeing stale
// slots on demand keeps fresh reports from being dropped. Returns the new count.
static size_t gw_reclaim_report_slots() {
    if (!pending_reports) return 0;
    unsigned long now = millis();
    size_t write_idx = 0;
    for (size_t read_idx = 0; read_idx < pending_report_count; read_idx++) {
        ZigbeeAttributeReport& r = pending_reports[read_idx];
        if (r.consumed || (now - r.timestamp) > REPORT_VALIDITY_MS) {
            continue;  // drop consumed/expired entry
        }
        if (write_idx != read_idx) pending_reports[write_idx] = r;
        write_idx++;
    }
    pending_report_count = write_idx;
    return pending_report_count;
}

// Throttle the "cache full" warnings so a flooding device can't spam the log.
static bool gw_report_cache_full_should_log() {
    static unsigned long last_full_log = 0;
    unsigned long now = millis();
    if (now - last_full_log > 5000) {
        last_full_log = now;
        return true;
    }
    return false;
}

static bool gw_cache_attribute_report(uint64_t ieee_addr, uint8_t endpoint,
                                      uint16_t cluster_id, uint16_t attr_id,
                                      int32_t value, uint8_t lqi) {
    if (!ensure_report_cache()) return false;
    for (size_t i = 0; i < pending_report_count; i++) {
        ZigbeeAttributeReport& r = pending_reports[i];
        if (r.ieee_addr == ieee_addr && r.cluster_id == cluster_id && r.attr_id == attr_id &&
            (r.endpoint == endpoint || r.endpoint == 0 || endpoint == 0)) {
            r.value = value;
            r.lqi = lqi;
            r.endpoint = endpoint;
            r.timestamp = millis();
            r.consumed = false;
            return true;
        }
    }

    if (pending_report_count >= MAX_PENDING_REPORTS) {
        gw_reclaim_report_slots();  // free consumed/expired slots before giving up
    }

    if (pending_report_count >= MAX_PENDING_REPORTS) {
        if (gw_report_cache_full_should_log()) {
            DEBUG_PRINTF(F("[ZIGBEE-GW] Report cache FULL [%d/%d] - dropping report! cluster=0x%04X attr=0x%04X\n"),
                        (int)pending_report_count, (int)MAX_PENDING_REPORTS, cluster_id, attr_id);
        }
        return false;
    }

    ZigbeeAttributeReport& report = pending_reports[pending_report_count++];
    report.ieee_addr = ieee_addr;
    report.endpoint = endpoint;
    report.cluster_id = cluster_id;
    report.attr_id = attr_id;
    report.value = value;
    report.lqi = lqi;
    report.timestamp = millis();
    report.consumed = false;
    return true;
}

// Forward declarations for functions used in class methods and device management
static void gw_schedule_configure_reporting_for_ieee(uint64_t ieee, unsigned long delay_ms, bool force_battery = false);
static void gw_schedule_default_configure_reporting(uint64_t ieee, uint8_t ep, unsigned long delay_ms);
static bool gw_query_basic_cluster(uint16_t short_addr, uint8_t endpoint);

class GwZigbeeReportReceiver;
static GwZigbeeReportReceiver* gw_reportReceiver = nullptr;

// ========== IEEE address resolution & device management ==========
// Resolve IEEE from short address using the Zigbee stack's internal address
// table. Does NOT auto-add devices - only for already confirmed devices.

static uint64_t gw_resolve_ieee(uint16_t short_addr) {
    // Check our confirmed device list
    for (const auto& dev : gw_discovered_devices) {
        if (dev.short_addr == short_addr) {
            return dev.ieee_addr;
        }
    }
    return 0;  // Device not in confirmed list
}

// Add a device that has confirmed its presence via response to query/report
static void gw_add_responsive_device(uint16_t short_addr, uint64_t ieee_addr, uint8_t endpoint, bool from_response = true) {
    ZigbeeDeviceInfo* dev = gw_find_discovered_device(ieee_addr);
    if (dev) {
        bool changed = false;
        if (dev->short_addr != short_addr) {
            dev->short_addr = short_addr;
            changed = true;
        }
        if (dev->endpoint != endpoint) {
            dev->endpoint = endpoint;
            changed = true;
        }
        dev->has_responded = true;
        dev->last_rx_at_ms = millis();     // stamp last reception
        dev->silent_query_count = 0;       // device is alive — reset silence counter

        // Wall-clock last-seen for the UI status lamp (survives reboots). Only
        // stamp on a genuine incoming frame — callers on the outgoing query path
        // pass from_response=false, so a device that never answers is not shown
        // as "seen". Persist at most every 30 min to avoid flash wear.
        if (from_response) {
            uint32_t nowu = (uint32_t)os.now_tz();
            if (nowu > 1704067200UL) {
                dev->last_seen = nowu;
                static unsigned long s_last_seen_persist_ms = 0;
                if (s_last_seen_persist_ms == 0 || millis() - s_last_seen_persist_ms > 1800000UL) {
                    s_last_seen_persist_ms = millis();
                    changed = true;
                }
            }
        }

        // If the device's manufacturer or model is still empty/unknown, (re)queue
        // a Basic Cluster read — but do NOT reset basic_query_attempts here.
        // Resetting the counter on every DP report defeats the retry cap and
        // lets a frequently-reporting Tuya end device (e.g. GIEX GX03
        // "_TZE284_") be flooded with Basic Cluster reads it can't service,
        // which overwhelms the sleepy device (it drops its parent and rejoins
        // with blinking LEDs) and never actually stores the manufacturer.  The
        // budget is only refreshed on a genuine re-announce (see findEndpoint).
        if (gw_device_needs_basic_info(*dev) &&
            dev->basic_query_attempts < GW_BASIC_QUERY_ATTEMPT_CAP &&
            !gw_is_basic_query_queued(ieee_addr)) {
            gw_queue_basic_cluster_query(ieee_addr, short_addr, endpoint, 1000UL);
        }

        if (changed) gw_mark_discovered_devices_dirty();
        // Device re-announced (e.g. after power cycle / re-join).
        // Re-schedule Configure Reporting so sleeping end-devices
        // resume sending proactive reports on their fresh network slot.
        gw_schedule_configure_reporting_for_ieee(ieee_addr, 1000);
        return;
    }
    
    // Add new confirmed device
    if (gw_discovered_devices.size() >= GW_DISCOVERED_MAX) {
        gw_discovered_devices.erase(gw_discovered_devices.begin());
        gw_mark_discovered_devices_dirty();
    }
    
    ZigbeeDeviceInfo info = {};
    info.ieee_addr = ieee_addr;
    info.short_addr = short_addr;
    info.endpoint = endpoint;
    info.is_new = true;
    info.has_responded = true;
    info.discovered_at = (uint32_t)os.now_tz();
    info.last_rx_at_ms = millis();
    {
        uint32_t nowu = (uint32_t)os.now_tz();
        info.last_seen = (from_response && nowu > 1704067200UL) ? nowu : 0U;
    }
    info.silent_query_count = 0;
    info.basic_query_attempts = 0;
    info.manufacturer[0] = '\0';
    info.model_id[0] = '\0';
    info.date_code[0] = '\0';
    info.sw_build_id[0] = '\0';
    info.app_version = 0xFF;
    info.stack_version = 0xFF;
    info.hw_version = 0xFF;
    info.battery = 255;
    info.lqi = 0;
    info.rssi = 0;
    
    gw_discovered_devices.push_back(info);
    gw_mark_discovered_devices_dirty();
    DEBUG_PRINTF(F("[ZIGBEE-GW] Added responsive device: short=0x%04X ieee=0x%016llX ep=%d\n"),
                 short_addr, (unsigned long long)ieee_addr, endpoint);

    gw_queue_basic_cluster_query(ieee_addr, short_addr, endpoint, 1000UL);

    // Schedule Configure Reporting for all sensors matching this device
    uint64_t resolved_ieee = gw_resolve_ieee(short_addr);
    if (resolved_ieee) {
        gw_schedule_configure_reporting_for_ieee(resolved_ieee, 1000);
    }
    
    // If in join mode and no sensor exists yet for this device, send default
    // Configure Reporting (900s) for common measurement clusters.  The device
    // is still awake right now, so the commands arrive immediately.
    if ((gw_join_window_end != 0)) {
        bool has_sensor = false;
        SensorIterator it = sensors_iterate_begin();
        SensorBase* s;
        while ((s = sensors_iterate_next(it)) != NULL) {
            if (s && s->type == SENSOR_ZIGBEE) {
                ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(s);
                if (zb->device_ieee == ieee_addr) { has_sensor = true; break; }
            }
        }
        if (!has_sensor) {
            gw_schedule_default_configure_reporting(ieee_addr, endpoint, 500);
        }
    }
}

// ========== GW discovered-devices accessors (called from sensor_zigbee.cpp) ==

int sensor_zigbee_gw_get_discovered_devices(ZigbeeDeviceInfo* out, int max_devices) {
    if (!out || max_devices <= 0) return 0;
    if (!gw_discovered_devices_loaded) {
        gw_load_discovered_devices();
    }
    int count = (gw_discovered_devices.size() < (size_t)max_devices)
                    ? (int)gw_discovered_devices.size() : max_devices;
    // Read-only snapshot: reading the device list (every /zg,/zd UI poll) must
    // NOT enqueue Basic Cluster reads.  Doing so amplified the query storm —
    // each UI refresh re-armed reads for every unresolved (often out-of-range)
    // device, saturating the single read slot.  Discovery is driven by the
    // announce path, DP reports and the bounded background wake scanner instead.
    for (int i = 0; i < count; i++) {
        memcpy(&out[i], &gw_discovered_devices[i], sizeof(ZigbeeDeviceInfo));
    }
    return count;
}

void sensor_zigbee_gw_clear_new_device_flags() {
    for (auto& dev : gw_discovered_devices) {
        dev.is_new = false;
    }
}

bool sensor_zigbee_gw_rename_device(uint64_t ieee_addr, const char* new_name) {
    if (ieee_addr == 0) return false;
    if (!gw_discovered_devices_loaded) {
        gw_load_discovered_devices();
    }

    ZigbeeDeviceInfo* dev = gw_find_discovered_device(ieee_addr);
    if (!dev) return false;

    char clean_name[sizeof(dev->friendly_name)] = {0};
    if (new_name) {
        size_t in_len = strlen(new_name);
        size_t start = 0;
        while (start < in_len && isspace((unsigned char)new_name[start])) start++;
        size_t end = in_len;
        while (end > start && isspace((unsigned char)new_name[end - 1])) end--;

        size_t out = 0;
        for (size_t i = start; i < end && out < sizeof(clean_name) - 1; i++) {
            unsigned char ch = (unsigned char)new_name[i];
            if (ch < 32) continue;
            clean_name[out++] = (char)ch;
        }
        clean_name[out] = '\0';
    }

    bool changed = false;
    if (clean_name[0] == '\0') {
        if (dev->friendly_name[0] != '\0') {
            dev->friendly_name[0] = '\0';
            changed = true;
        }
        if (dev->is_custom_name) {
            dev->is_custom_name = false;
            changed = true;
        }
    } else {
        if (strncmp(dev->friendly_name, clean_name, sizeof(dev->friendly_name)) != 0) {
            strncpy(dev->friendly_name, clean_name, sizeof(dev->friendly_name) - 1);
            dev->friendly_name[sizeof(dev->friendly_name) - 1] = '\0';
            changed = true;
        }
        if (!dev->is_custom_name) {
            dev->is_custom_name = true;
            changed = true;
        }
    }

    if (!changed) return true;

    gw_mark_discovered_devices_dirty();
    return gw_save_discovered_devices();
}

// ========== Tuya DP protocol handler (APS layer) ==========

/**
 * @brief Handle Basic Cluster (0x0000) attribute read response in Gateway mode
 * Updates discovered device info and matching sensor configurations.
 */
static void gw_handleBasicClusterResponse(uint16_t short_addr, const esp_zb_zcl_attribute_t *attribute, uint64_t ieee_override = 0) {
    if (!attribute || !attribute->data.value) return;
    
    char str_buf[32] = {0};
    bool has_string = zigbee_extract_string_attribute(attribute, str_buf, sizeof(str_buf));
    bool has_u8 = (attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_U8);
    uint8_t u8_value = has_u8 ? *(uint8_t*)attribute->data.value : 0xFF;
    
    // Find device in discovered list and update.
    // Prefer matching by the explicitly known IEEE address (ieee_override) when
    // available.  The short-address fallback is lossy: for sleepy Tuya devices
    // (GIEX GX02/GX03/GX04) the Basic Cluster response often arrives via the
    // no-address zbReadBasicCluster() callback, and re-deriving the short
    // address from the IEEE can fail (returns 0xFFFF → 0).  That previously
    // caused the manufacturer/model string of one device to be written onto a
    // different device record (e.g. all devices ending up with the GX02's
    // "_TZE200_sh1btabb" manufacturer), which in turn made the UI/database name
    // every device identically.
    uint64_t ieee_addr = 0;
    for (auto& dev : gw_discovered_devices) {
        bool match = (ieee_override != 0) ? (dev.ieee_addr == ieee_override)
                                          : (dev.short_addr == short_addr);
        if (match) {
            ieee_addr = dev.ieee_addr;
            bool changed = false;
            if (attribute->id == ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID && has_u8) {
                if (dev.app_version != u8_value) {
                    dev.app_version = u8_value;
                    changed = true;
                }
            } else if (attribute->id == ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID && has_u8) {
                if (dev.stack_version != u8_value) {
                    dev.stack_version = u8_value;
                    changed = true;
                }
            } else if (attribute->id == ZB_ZCL_ATTR_BASIC_HW_VERSION_ID && has_u8) {
                if (dev.hw_version != u8_value) {
                    dev.hw_version = u8_value;
                    changed = true;
                }
            } else if (attribute->id == ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID && has_string) {
                if (strncmp(dev.manufacturer, str_buf, sizeof(dev.manufacturer)) != 0) {
                    strncpy(dev.manufacturer, str_buf, sizeof(dev.manufacturer) - 1);
                    dev.manufacturer[sizeof(dev.manufacturer) - 1] = '\0';
                    changed = true;
                    // Manufacturer (re)identified. If the device already carries
                    // logical devices that were auto-assigned for an earlier /
                    // wrong identity (e.g. a GX04 soil sensor provisionally
                    // filled with GX03 valve logical devices while its
                    // manufacturer was still unknown, because model "TS0601" is
                    // shared by many Tuya devices), drop them and force a fresh
                    // DB lookup so the correct logical devices for the now-known
                    // manufacturer are fetched by sensor_zigbee_gw_do_lookups().
                    char ieee_hex[17];
                    snprintf(ieee_hex, sizeof(ieee_hex), "%016llX", (unsigned long long)dev.ieee_addr);
                    if (OpenSprinkler::zigbee_logical_count_ieee(ieee_hex) > 0) {
                        OpenSprinkler::zigbee_logical_clear_ieee(ieee_hex);
                        DEBUG_PRINTF(F("[ZIGBEE-GW] Manufacturer resolved to \"%s\" for 0x%016llX — cleared stale logical devices for fresh DB lookup\n"),
                                     dev.manufacturer, (unsigned long long)dev.ieee_addr);
                    }
                    dev.vendor[0] = '\0';
                    dev.logical_lookup_done = false;
                }
            } else if (attribute->id == ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID && has_string) {
                if (strncmp(dev.model_id, str_buf, sizeof(dev.model_id)) != 0) {
                    strncpy(dev.model_id, str_buf, sizeof(dev.model_id) - 1);
                    dev.model_id[sizeof(dev.model_id) - 1] = '\0';
                    changed = true;
                }
            } else if (attribute->id == ZB_ZCL_ATTR_BASIC_DATE_CODE_ID && has_string) {
                if (strncmp(dev.date_code, str_buf, sizeof(dev.date_code)) != 0) {
                    strncpy(dev.date_code, str_buf, sizeof(dev.date_code) - 1);
                    dev.date_code[sizeof(dev.date_code) - 1] = '\0';
                    changed = true;
                }
            } else if (attribute->id == ZB_ZCL_ATTR_BASIC_SW_BUILD_ID && has_string) {
                if (strncmp(dev.sw_build_id, str_buf, sizeof(dev.sw_build_id)) != 0) {
                    strncpy(dev.sw_build_id, str_buf, sizeof(dev.sw_build_id) - 1);
                    dev.sw_build_id[sizeof(dev.sw_build_id) - 1] = '\0';
                    changed = true;
                }
            }
            if (attribute->id == ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID ||
                attribute->id == ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID) {
                dev.basic_query_attempts = 0;
            }
            if (changed) gw_mark_discovered_devices_dirty();
            DEBUG_PRINTF(F("[ZIGBEE-GW] Basic attr 0x%04X for 0x%016llX: app=%u stack=%u hw=%u mfr=\"%s\" model=\"%s\" date=\"%s\" sw=\"%s\"\n"),
                         attribute->id,
                         (unsigned long long)dev.ieee_addr,
                         dev.app_version,
                         dev.stack_version,
                         dev.hw_version,
                         dev.manufacturer,
                         dev.model_id,
                         dev.date_code,
                         dev.sw_build_id);
            break;
        }
    }
    
    // Update matching sensor configurations
    if (ieee_addr != 0 && has_string) {
        const char* mfr = (attribute->id == ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID) ? str_buf : nullptr;
        const char* mdl = (attribute->id == ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID) ? str_buf : nullptr;
        if (mfr || mdl) ZigbeeSensor::updateBasicClusterInfo(ieee_addr, mfr, mdl);
    }
}

// ---------------------------------------------------------------------------
// Station switch state machine: one entry per station.
//
// switch_zigbeestation() only records the desired state (latest command wins,
// so an OFF always replaces a still-pending ON). The machine sends the command,
// re-sends with backoff (6/15/30/60 s), re-sends immediately when the device
// wakes up and talks to us (see gw_station_ctl_on_device_rx), evaluates the
// APS confirm of every send (gw_station_ctl_on_send_status) and treats a Tuya
// DP echo as the final confirmation. An OFF is never abandoned: after the
// backoff ramp it keeps retrying once a minute until the valve confirms.
// ---------------------------------------------------------------------------
// Sleepy GIEX/Tuya valves choke on more than one queued command: every timed
// retry that piles up in the coordinator's indirect queue is delivered in a
// burst on the next poll and the valve MCU drops or misorders them.  So each
// request is sent exactly ONCE; the only re-send is wake-triggered and only if
// the previous frame provably never reached the device (APS send failure).
// The ESP stack itself keeps an indirect frame alive (APS retries) for up to
// ~40-60 s before reporting the send as failed, so a timed retry stacks a
// second frame on top of the first.  Rule: at most ONE frame in flight per
// station.  A re-send happens only after the send-status callback reported
// the previous frame as NOT delivered (or on device wake in that same state).
#define ZB_CTL_MAX_SENDS_TOTAL       20     // hard cap of re-sends per request (then wait for wake only)
#define ZB_CTL_RESEND_AFTER_FAIL_MS 1000UL  // delay between "not delivered" and the re-send
#define ZB_CTL_CONFIRM_TIMEOUT_MS  60000UL  // no confirmation within this -> "switch failed" alert (once)
#define ZB_CTL_INFLIGHT_MAX_MS     60000UL  // after this a frame without send status is considered gone
#define ZB_CTL_SEND_FAIL_RETRY_MS   1500UL  // spacing when the send itself was rejected (cooldown, no short addr)
#define ZB_CTL_WAKE_MIN_GAP_MS      8000UL  // min gap between a send and a wake-triggered re-send (> 7.68 s indirect persistence)
#define ZB_CTL_WAKE_ECHO_GRACE_MS   5000UL  // delivered (APS ack) but no DP echo yet: give the device this long
#define ZB_CTL_ZCL_LEGACY_CONFIRM_MS 2000UL // ZCL without send-status support: assume delivered after this

struct ZbStationCtl {
    bool     active;         // command outstanding, not yet confirmed
    bool     desired_on;
    uint8_t  ctrl_type;      // 0 = standard ZCL, 1 = Tuya DP, 2 = GIEX water valve
    uint8_t  endpoint;
    uint8_t  dp_id;          // DP written (Tuya/GIEX)
    uint8_t  verify_dp;      // DP that echoes the state
    uint8_t  attempts;       // successful sends so far
    uint8_t  send_fails;     // consecutive rejected sends (no short addr, cooldown, queue full)
    uint8_t  last_tsn;
    bool     have_tsn;
    bool     delivered;      // APS confirm OK for the last send
    bool     last_send_failed; // APS confirm reported the last send as NOT delivered
    bool     resend_needed;  // desired state changed while a frame is still in flight
    bool     inflight_on;    // state carried by the frame currently in flight
    bool     error_flagged;  // "switch failed" already raised for this command
    uint64_t ieee;
    uint32_t requested_ms;
    uint32_t last_sent_ms;
    uint32_t next_try_ms;
    uint16_t dur;            // runtime in seconds at request time (0 = unknown)
    // Runtime channel from the device DB (Tuya): written in the same frame as ON
    uint8_t  dp_runtime;     // 0 = none
    uint8_t  runtime_unit;   // ZB_RT_UNIT_*
    uint16_t runtime_max;
    uint8_t  prereq_dp;      // 0 = none
    int16_t  prereq_value;
};

struct GwTuyaDpRecord {
    uint8_t dp_id;
    uint8_t dp_type;
    uint8_t len;        // 1 (bool/enum) or 4 (value)
    uint32_t value;     // big-endian on the wire
};

// Defined further down with the other Tuya senders.
static bool gw_send_tuya_dp_records(uint64_t device_ieee, uint8_t endpoint,
                                    const GwTuyaDpRecord* recs, uint8_t n_recs, bool urgent);
static bool gw_send_tuya_dp_bool_write_cmd(uint64_t device_ieee, uint8_t endpoint, uint8_t dp_id, bool turnon,
                                           uint8_t command_id, bool urgent);
static uint8_t gw_giex_resolve_state_dp(uint64_t device_ieee, uint8_t dp_id);

// Standard-ZCL devices that answered OnWithTimedOff with UNSUP_CLUSTER_COMMAND:
// fall back to plain On for them (the station state machine still retries OFF).
#define GW_ZCL_TIMED_OFF_UNSUP_MAX 16
static PSRAM_BSS_ATTR uint64_t gw_zcl_timed_off_unsupported[GW_ZCL_TIMED_OFF_UNSUP_MAX];
static uint64_t gw_last_timed_off_ieee = 0;
static uint32_t gw_last_timed_off_ms = 0;

static bool gw_zcl_timed_off_is_unsupported(uint64_t ieee) {
    for (int i = 0; i < GW_ZCL_TIMED_OFF_UNSUP_MAX; i++) if (gw_zcl_timed_off_unsupported[i] == ieee) return true;
    return false;
}
static void gw_zcl_timed_off_mark_unsupported(uint64_t ieee) {
    if (!ieee || gw_zcl_timed_off_is_unsupported(ieee)) return;
    for (int i = 0; i < GW_ZCL_TIMED_OFF_UNSUP_MAX; i++) {
        if (gw_zcl_timed_off_unsupported[i] == 0) { gw_zcl_timed_off_unsupported[i] = ieee; return; }
    }
    gw_zcl_timed_off_unsupported[0] = ieee; // table full: overwrite the oldest slot
}

// Remaining runtime of a station command in seconds (0 = unknown / elapsed).
static uint16_t gw_station_ctl_remaining_s(const ZbStationCtl& e) {
    if (e.dur == 0) return 0;
    uint32_t elapsed = (millis() - e.requested_ms) / 1000UL;
    return (elapsed < e.dur) ? (uint16_t)(e.dur - elapsed) : 0;
}

// Convert seconds to the device's runtime unit and clamp to its range.
static uint32_t gw_runtime_to_device_units(uint16_t seconds, uint8_t unit, uint16_t max_units) {
    uint32_t v;
    uint32_t def_max;
    switch (unit) {
        case ZB_RT_UNIT_MIN: v = ((uint32_t)seconds + 59U) / 60U;   def_max = 1440;  break;
        case ZB_RT_UNIT_H:   v = ((uint32_t)seconds + 3599U) / 3600U; def_max = 24;  break;
        default:             v = seconds;                             def_max = 86400; break;
    }
    uint32_t lim = max_units ? max_units : def_max;
    if (v < 1) v = 1;
    if (v > lim) v = lim;
    return v;
}

// Accessed only from task context (main loop), never from an ISR or with the
// flash cache disabled, so the tables can live in PSRAM.
static PSRAM_BSS_ATTR ZbStationCtl zb_station_ctl[MAX_NUM_STATIONS];
static PSRAM_BSS_ATTR uint8_t zb_station_switch_error[MAX_NUM_STATIONS];
static PSRAM_BSS_ATTR uint8_t zb_station_switch_waiting[MAX_NUM_STATIONS];
static PSRAM_BSS_ATTR uint8_t zb_station_physical_on[MAX_NUM_STATIONS];
static bool gw_send_status_supported = false;  // set once the stack delivered its first send-status callback

static uint32_t zb_ctl_backoff_ms(uint8_t attempts) {
    static const uint16_t sched_s[] = {6, 15, 30, 60};
    const uint8_t n = sizeof(sched_s) / sizeof(sched_s[0]);
    uint8_t i = attempts ? (uint8_t)(attempts - 1) : 0;
    if (i >= n) i = n - 1;
    return (uint32_t)sched_s[i] * 1000UL;
}

static void gw_station_switch_error_set(uint8_t sid, bool error) {
    if (sid >= MAX_NUM_STATIONS) return;
    zb_station_switch_error[sid] = error ? 1 : 0;
    if (error) zb_station_switch_waiting[sid] = 0;
}

static void gw_station_switch_waiting_set(uint8_t sid, bool waiting) {
    if (sid >= MAX_NUM_STATIONS) return;
    zb_station_switch_waiting[sid] = waiting ? 1 : 0;
}

static void gw_station_physical_state_set(uint8_t sid, bool actual_on);

static void gw_station_verify_publish_fail(uint8_t sid, bool expected_on, bool timeout) {
    gw_station_switch_error_set(sid, true);
    DEBUG_PRINTF(F("[ZIGBEE-GW] Switch-fail alert: sid=%u expected_on=%d timeout=%d\n"),
                 (unsigned)sid, expected_on ? 1 : 0, timeout ? 1 : 0);
    if (!os.mqtt.enabled()) return;
    if (!os.mqtt.connected()) os.mqtt.reconnect();
    if (!os.mqtt.connected()) return;
    char topic[48];
    char payload[80];
    snprintf_P(topic,   sizeof(topic),   PSTR("station/%u/alert/switch"), (unsigned)sid);
    snprintf_P(payload, sizeof(payload), PSTR("{\"expected_on\":%d,\"timeout\":%d}"),
               expected_on ? 1 : 0, timeout ? 1 : 0);
    os.mqtt.publish(topic, payload);
}

static void gw_station_ctl_note_sent(uint8_t sid, uint8_t tsn) {
    if (sid >= MAX_NUM_STATIONS) return;
    ZbStationCtl &e = zb_station_ctl[sid];
    e.last_tsn = tsn;
    e.have_tsn = true;
    e.last_sent_ms = millis();   // queued Tuya commands: count from the actual dispatch
}

static void gw_station_ctl_confirm(uint8_t sid, const char* how) {
    ZbStationCtl &e = zb_station_ctl[sid];
    e.active = false;
    e.error_flagged = false;
    gw_station_physical_state_set(sid, e.desired_on);
    DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u switch confirmed (%s): on=%d attempts=%u\n"),
                 (unsigned)sid, how, e.desired_on ? 1 : 0, (unsigned)e.attempts);
}

// Send (or re-send) the command for a station entry. `urgent` bypasses the
// per-device cooldown and the global Tuya spacing.
static bool gw_station_ctl_pending_for(uint64_t ieee) {
    if (ieee == 0) return false;
    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        const ZbStationCtl &e = zb_station_ctl[sid];
        if (e.active && e.ieee == ieee) return true;
    }
    return false;
}

static bool gw_station_ctl_send(uint8_t sid, bool urgent) {
    ZbStationCtl &e = zb_station_ctl[sid];
    uint32_t now = millis();
    e.have_tsn = false;
    gw_ctl_sending_sid = sid;
    bool ok = false;
    uint16_t remaining_s = e.desired_on ? gw_station_ctl_remaining_s(e) : 0;
    if (e.ctrl_type == 0) {
        // Standard ZCL: OnWithTimedOff carries the runtime (falls back to On
        // when the device rejected it before).
        ok = sensor_zigbee_send_on_off(e.ieee, e.endpoint, e.desired_on, urgent, remaining_s);
    } else {
        uint8_t state_dp = (e.ctrl_type == 2) ? gw_giex_resolve_state_dp(e.ieee, e.dp_id) : e.dp_id;
        if (e.desired_on && remaining_s > 0 && e.dp_runtime != 0) {
            // ON with runtime: [mode prerequisite] + runtime + state in ONE frame.
            GwTuyaDpRecord recs[3];
            uint8_t n = 0;
            if (e.prereq_dp != 0) {
                recs[n++] = { e.prereq_dp, TUYA_TYPE_ENUM, 1, (uint32_t)(e.prereq_value & 0xFF) };
            }
            uint32_t rt = gw_runtime_to_device_units(remaining_s, e.runtime_unit, e.runtime_max);
            recs[n++] = { e.dp_runtime, TUYA_TYPE_VALUE, 4, rt };
            recs[n++] = { state_dp, TUYA_TYPE_BOOL, 1, 1 };
            DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u ON with runtime %lu (unit %u) on dp=%u\n"),
                         (unsigned)sid, (unsigned long)rt, (unsigned)e.runtime_unit, (unsigned)e.dp_runtime);
            ok = gw_send_tuya_dp_records(e.ieee, e.endpoint, recs, n, urgent);
        } else {
            ok = gw_send_tuya_dp_bool_write_cmd(e.ieee, e.endpoint, state_dp, e.desired_on, TUYA_CMD_DATA_REQUEST, urgent);
        }
    }
    gw_ctl_sending_sid = 0xFF;
    if (ok) {
        if (e.resend_needed) e.attempts = 0;  // deferred direction change: new command, fresh count
        e.attempts++;
        e.last_sent_ms = now;
        e.delivered = false;
        e.last_send_failed = false;
        e.resend_needed = false;
        e.inflight_on = e.desired_on;
        e.next_try_ms = now + zb_ctl_backoff_ms(e.attempts);
        DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u %s sent (attempt %u%s), next check in %lus\n"),
                     (unsigned)sid, e.desired_on ? "ON" : "OFF", (unsigned)e.attempts,
                     urgent ? ", wake-triggered" : "", (unsigned long)(zb_ctl_backoff_ms(e.attempts) / 1000UL));
        e.send_fails = 0;
    } else {
        // Rejected before it left the stack (unknown short address, cooldown,
        // scheduler full): retry with exponential backoff 1.5 s ... 48 s so a
        // station bound to an absent device does not busy-loop. Any frame from
        // the device (wake trigger) short-cuts the wait.
        if (e.send_fails < 16) e.send_fails++;
        uint8_t shift = (e.send_fails > 6) ? 5 : (uint8_t)(e.send_fails - 1);
        e.next_try_ms = now + (ZB_CTL_SEND_FAIL_RETRY_MS << shift);
    }
    return ok;
}

// Called from gw_cache_tuya_dp_report when a Tuya DP is received.
static void gw_station_verify_process(uint64_t ieee, uint8_t dp_id, bool actual_on) {
    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        ZbStationCtl &e = zb_station_ctl[sid];
        if (!e.active || e.ieee != ieee) continue;
        if (e.ctrl_type == 0) continue;  // standard ZCL is confirmed via APS ack / On/Off attribute

        bool match_state = (e.verify_dp == dp_id);

        // GIEX water valves frequently confirm a *stop* via a completion data
        // point instead of echoing the state DP = 0: DP1 (mode/state) = 0, the
        // countdown DP107 reaching 0, or a DP111 last-irrigation report. Accept
        // those as an OFF confirmation so a closed valve does not stay "red".
        bool giex_off = (e.ctrl_type == 2 && !e.desired_on &&
                         ((dp_id == 1   && !actual_on) ||
                          (dp_id == 107 && !actual_on) ||
                          (dp_id == 111)));
        if (!match_state && !giex_off) continue;

        bool confirmed = match_state ? (e.desired_on == actual_on) : true;
        if (!confirmed && e.resend_needed && actual_on == e.inflight_on) {
            // The device applied the frame that was in flight and is awake:
            // send the deferred (opposite) state immediately.
            DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u echoed in-flight state on=%d -> sending deferred %s\n"),
                         (unsigned)sid, actual_on ? 1 : 0, e.desired_on ? "ON" : "OFF");
            gw_station_physical_state_set(sid, actual_on);
            gw_station_ctl_send(sid, true);
            return;
        }
        if (confirmed) {
            gw_station_ctl_confirm(sid, giex_off ? "completion DP" : "DP echo");
        } else {
            // Wrong state reported (or a stale report collided with our
            // command): re-send on the next tick.
            DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u switch mismatch (got on=%d want on=%d) -> re-send\n"),
                         (unsigned)sid, actual_on ? 1 : 0, e.desired_on ? 1 : 0);
            e.next_try_ms = millis();
        }
        return;
    }
}

// Wake trigger: the device just sent us a frame, so it is awake and listening.
// Re-send any outstanding command for it right now.
static void gw_station_ctl_on_device_rx(uint64_t ieee) {
    if (ieee == 0) return;
    uint32_t now = millis();
    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        ZbStationCtl &e = zb_station_ctl[sid];
        if (!e.active || e.ieee != ieee) continue;
        if (e.last_sent_ms != 0 && (now - e.last_sent_ms) < ZB_CTL_WAKE_MIN_GAP_MS) continue;
        if (e.delivered && (now - e.last_sent_ms) < ZB_CTL_WAKE_ECHO_GRACE_MS) continue; // it has the command, echo pending
        // Never push a second copy of a command the device may already hold:
        // re-send on wake only if nothing was sent yet or the APS layer
        // reported the last frame as not delivered.
        if (e.attempts > 0 && !e.last_send_failed) continue;
        gw_station_ctl_send(sid, true);
        break; // one urgent send per frame; the next frame triggers the next station
    }
}

// APS confirm for a command we sent (from the send-status callback).
static void gw_station_ctl_on_send_status(uint8_t tsn, int32_t status, uint16_t short_addr) {
    (void)short_addr;
    gw_send_status_supported = true;
    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        ZbStationCtl &e = zb_station_ctl[sid];
        if (!e.active || !e.have_tsn || e.last_tsn != tsn) continue;
        if (status == ESP_OK) {
            e.delivered = true;
            if (e.resend_needed) {
                // The device just acked the previous frame, so it is awake and
                // the indirect queue is empty: send the deferred state now.
                DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u in-flight frame acked -> sending deferred %s\n"),
                             (unsigned)sid, e.desired_on ? "ON" : "OFF");
                gw_station_ctl_send(sid, true);
                return;
            }
            if (e.ctrl_type == 0) {
                // No DP echo on standard ZCL devices: the APS ack is the confirmation.
                gw_station_ctl_confirm(sid, "APS ack");
            }
        } else {
            // Not delivered (device did not poll within the indirect window).
            // No timed retry: the next frame from the device (wake trigger)
            // re-sends it, see gw_station_ctl_on_device_rx().
            DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u send status 0x%lx (not delivered) -> re-send (stack gave up, nothing in flight)\n"),
                         (unsigned)sid, (unsigned long)status);
            e.delivered = false;
            e.last_send_failed = true;
            e.next_try_ms = millis() + ZB_CTL_RESEND_AFTER_FAIL_MS;
        }
        return;
    }
}

// OnWithTimedOff was rejected: re-send the desired state as a plain On.
static void gw_station_ctl_on_timed_off_rejected(uint64_t ieee) {
    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        ZbStationCtl &e = zb_station_ctl[sid];
        if (e.ieee != ieee || !e.desired_on || e.ctrl_type != 0) continue;
        if ((millis() - e.requested_ms) > 60000UL) continue;
        DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u: OnWithTimedOff unsupported -> plain On\n"), (unsigned)sid);
        e.active = true;
        e.next_try_ms = millis();
    }
}

static void gw_station_physical_state_set(uint8_t sid, bool actual_on) {
    if (sid >= os.nstations) return;

    gw_station_switch_error_set(sid, false);
    gw_station_switch_waiting_set(sid, false);

    bool was_on = zb_station_physical_on[sid] != 0;
    if (was_on == actual_on) return;

    zb_station_physical_on[sid] = actual_on ? 1 : 0;

    DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u physical state from device: on=%d\n"),
                 (unsigned)sid, actual_on ? 1 : 0);
}

static void gw_station_status_process_standard_onoff(uint64_t ieee, uint8_t endpoint, bool actual_on) {
    if (ieee == 0 || os.nstations == 0) return;

    for (uint8_t sid = 0; sid < os.nstations; sid++) {
        if (os.get_station_type(sid) != STN_TYPE_ZIGBEE) continue;

        StationData station = {};
        os.get_station_data(sid, &station);
        ZigbeeStationData* data = (ZigbeeStationData*)station.sped;

        uint64_t station_ieee = gw_parse_ieee_hex(data->device_ieee);
        if (!gw_ieee_matches_station(station_ieee, ieee)) continue;

        uint8_t station_ep = gw_parse_hex_u8(data->endpoint, sizeof(data->endpoint));
        if (station_ep == 0) station_ep = 1;

        bool use_tuya = (data->use_tuya[0] == '1');
        if (use_tuya) continue; // Skip Tuya stations here since they have their own DP processor

        if (endpoint != 0 && station_ep != endpoint) continue;

        DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u matched standard On/Off report: on=%d\n"),
                 (unsigned)sid, actual_on ? 1 : 0);
        gw_station_physical_state_set(sid, actual_on);
    }
}

static void gw_station_status_process_tuya_dp(uint64_t ieee, uint8_t endpoint, uint8_t dp_id, bool actual_on) {
    if (ieee == 0 || dp_id == 0 || os.nstations == 0) return;

    for (uint8_t sid = 0; sid < os.nstations; sid++) {
        if (os.get_station_type(sid) != STN_TYPE_ZIGBEE) continue;

        StationData station = {};
        os.get_station_data(sid, &station);
        ZigbeeStationData* data = (ZigbeeStationData*)station.sped;

        uint64_t station_ieee = gw_parse_ieee_hex(data->device_ieee);
        if (!gw_ieee_matches_station(station_ieee, ieee)) continue;

        uint8_t station_ep = gw_parse_hex_u8(data->endpoint, sizeof(data->endpoint));
        if (station_ep == 0) station_ep = 1;

        bool use_tuya = (data->use_tuya[0] == '1');
        uint8_t station_dp = gw_parse_hex_u8(data->tuya_dp, sizeof(data->tuya_dp));
        if (station_dp == 0) station_dp = 1;
        ZigbeeStationControlConfig cfg = {};
        bool has_cfg = sensor_zigbee_get_station_control_config(ieee, &cfg, station_ep, station_dp) && cfg.found;
        if (has_cfg) {
            station_ep = cfg.endpoint ? cfg.endpoint : station_ep;
            if (cfg.control_mode == ZB_STATION_CTRL_TUYA) {
                use_tuya = true;
            } else if (cfg.control_mode == ZB_STATION_CTRL_STANDARD) {
                use_tuya = false;
            } else if (!use_tuya && (cfg.dp_value != 0 || cfg.dp_status != 0)) {
                // AUTO mode but explicit DP mapping present => treat as Tuya.
                use_tuya = true;
            }
            if (cfg.dp_status != 0) {
                station_dp = cfg.dp_status;
            }
        }
        if (endpoint != 0 && station_ep != endpoint) continue;

        const char* mfr = "";
        const char* mdl = "";
        const char* vnd = "";
        bool dev_is_tuya = false;
        bool matched_device = false;

        for (const auto& dev : gw_discovered_devices) {
            if (dev.ieee_addr != ieee) continue;
            mfr = dev.manufacturer;
            mdl = dev.model_id;
            vnd = dev.vendor;
            dev_is_tuya = dev.is_tuya;
            matched_device = true;
            break;
        }

        if (!matched_device) {
            SensorIterator it = sensors_iterate_begin();
            SensorBase* sensor;
            while ((sensor = sensors_iterate_next(it)) != NULL) {
                if (!sensor || sensor->type != SENSOR_ZIGBEE) continue;
                ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(sensor);
                if (zb->device_ieee != ieee) continue;
                mfr = zb->zb_manufacturer;
                mdl = zb->zb_model;
                vnd = zb->zb_vendor;
                matched_device = true;
                break;
            }
        }

        if (!has_cfg || !use_tuya && dev_is_tuya) use_tuya = true;

        if (!use_tuya) continue;

        char ieee_str[17];
        snprintf(ieee_str, sizeof(ieee_str), "%016llX", (unsigned long long)ieee);

        bool is_match = (station_dp == dp_id);

        if (!is_match) {
            int valve_index = 0;
            // 1. Try to find the valve index for this station's configuration using logical devices
            for (int i = 1; i <= 8; i++) {
                char valve_name[32];
                snprintf(valve_name, sizeof(valve_name), "valve_%d", i);
                ZigBeeLogicalDevice* log_valve = OpenSprinkler::zigbee_logical_lookup(ieee_str, valve_name);
                if (log_valve && (log_valve->tuya_dp_value == station_dp || log_valve->tuya_dp_status == station_dp)) {
                    valve_index = i;
                    break;
                }
                char state_name[32];
                snprintf(state_name, sizeof(state_name), "state_%d", i);
                ZigBeeLogicalDevice* log_state = OpenSprinkler::zigbee_logical_lookup(ieee_str, state_name);
                if (log_state && (log_state->tuya_dp_value == station_dp || log_state->tuya_dp_status == station_dp)) {
                    valve_index = i;
                    break;
                }
            }
            if (valve_index == 0) {
                ZigBeeLogicalDevice* log_valve = OpenSprinkler::zigbee_logical_lookup(ieee_str, "valve");
                if (log_valve && (log_valve->tuya_dp_value == station_dp || log_valve->tuya_dp_status == station_dp)) {
                    valve_index = 1;
                }
                ZigBeeLogicalDevice* log_state = OpenSprinkler::zigbee_logical_lookup(ieee_str, "state");
                if (log_state && (log_state->tuya_dp_value == station_dp || log_state->tuya_dp_status == station_dp)) {
                    valve_index = 1;
                }
            }

            // 2. If found, check if direct DP or status DP matches the incoming dp_id
            if (valve_index > 0) {
                char valve_name[32];
                snprintf(valve_name, sizeof(valve_name), "valve_%d", valve_index);
                ZigBeeLogicalDevice* log_valve = OpenSprinkler::zigbee_logical_lookup(ieee_str, valve_name);
                if (!log_valve && valve_index == 1) {
                    log_valve = OpenSprinkler::zigbee_logical_lookup(ieee_str, "valve");
                }
                if (log_valve && (log_valve->tuya_dp_value == dp_id || log_valve->tuya_dp_status == dp_id)) {
                    is_match = true;
                }

                if (!is_match) {
                    char state_name[32];
                    snprintf(state_name, sizeof(state_name), "state_%d", valve_index);
                    ZigBeeLogicalDevice* log_state = OpenSprinkler::zigbee_logical_lookup(ieee_str, state_name);
                    if (!log_state && valve_index == 1) {
                        log_state = OpenSprinkler::zigbee_logical_lookup(ieee_str, "state");
                    }
                    if (log_state && (log_state->tuya_dp_value == dp_id || log_state->tuya_dp_status == dp_id)) {
                        is_match = true;
                    }
                }
            } else {
                // Symmetrical fallbacks when logical devices map isn't fully initialized / present
                if ((station_dp == 1 || station_dp == 104) && (dp_id == 1 || dp_id == 104)) {
                    is_match = true;
                } else if ((station_dp == 2 || station_dp == 105) && (dp_id == 2 || dp_id == 105)) {
                    is_match = true;
                }
            }
        }

        if (!is_match) continue;

        DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u matched Tuya DP report: dp=%u on=%d\n"),
                 (unsigned)sid, (unsigned)dp_id, actual_on ? 1 : 0);
        gw_station_physical_state_set(sid, actual_on);
    }
}

// ========== Tuya DP protocol handler (APS layer) - continued ==========
// Tuya devices using cluster 0xEF00 send proprietary DataPoint messages
// instead of standard ZCL attribute reports. This APS indication handler
// intercepts those frames, parses the Tuya DP payload, and injects them
// into the existing report cache as if they were standard ZCL reports.
//
// Tuya ZCL frame layout (within asdu):
//   [0] frame_control  (0x09 = cluster-specific, client-to-server, disable default response)
//   [1] seq_number
//   [2] command_id     (0x01 = dataResponse, 0x02 = dataReport)
//   [3..4] tuya_seq    (big-endian, Tuya sequence number — ignored)
//   [5..N] DP records, each:
//       [0] dp_number
//       [1] dp_type
//       [2..3] dp_length (big-endian, length of dp_value)
//       [4..4+dp_length-1] dp_value (big-endian for numeric types)

static void gw_cache_tuya_report(uint64_t ieee_addr, uint8_t src_endpoint,
                                  uint16_t mapped_cluster, uint16_t mapped_attr,
                                  int32_t value, int lqi, uint8_t dp_type) {
    if (!ensure_report_cache()) return;
    uint16_t flagged_attr = tuya_report_attr((uint8_t)mapped_attr, dp_type);

    // Update existing report in-place if we already have one for the same
    // ieee + cluster + attr.  This prevents the cache from filling up with
    // duplicate Tuya reports when no sensor is configured yet.
    for (size_t i = 0; i < pending_report_count; i++) {
        ZigbeeAttributeReport& r = pending_reports[i];
        if (r.cluster_id == mapped_cluster && r.attr_id == flagged_attr &&
            r.ieee_addr == ieee_addr) {
            r.value = value;
            r.lqi = (uint8_t)(lqi & 0xFF);
            r.endpoint = src_endpoint;
            r.timestamp = millis();
            r.consumed = false;  // Allow re-processing with updated value
            return;  // Updated in-place — no new slot needed
        }
    }

    // No existing entry — append a new one
    if (pending_report_count >= MAX_PENDING_REPORTS) {
        gw_reclaim_report_slots();  // free consumed/expired slots before giving up
    }

    if (pending_report_count < MAX_PENDING_REPORTS) {
        ZigbeeAttributeReport& report = pending_reports[pending_report_count++];
        report.ieee_addr = ieee_addr;
        report.endpoint = src_endpoint;
        report.cluster_id = mapped_cluster;
        report.attr_id = flagged_attr;
        report.value = value;
        report.lqi = (uint8_t)(lqi & 0xFF);
        report.timestamp = millis();
        report.consumed = false;

        ZB_GW_TRACE(F("[ZIGBEE-GW][TUYA] Cached DP report: cluster=0x%04X attr=0x%04X value=%ld lqi=%d\n"),
                    mapped_cluster, mapped_attr, value, lqi);
    } else {
        if (gw_report_cache_full_should_log()) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW][TUYA] Report cache full — dropping Tuya DP"));
        }
    }
}

static bool gw_is_tuya_status_on(uint64_t ieee, uint8_t dp_number, int32_t value, uint8_t dp_type) {
    auto value_in_csv = [](const char* csv, int32_t needle) -> bool {
        if (!csv || !csv[0]) return false;
        char buf[32];
        strncpy(buf, csv, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char* token = strtok(buf, ",");
        while (token != NULL) {
            if (atoi(token) == needle) return true;
            token = strtok(NULL, ",");
        }
        return false;
    };

    char ieee_str[17];
    snprintf(ieee_str, sizeof(ieee_str), "%016llX", (unsigned long long)ieee);
    
    if (OpenSprinkler::zigbee_logical_devices_map) {
        for (const auto& entry : *OpenSprinkler::zigbee_logical_devices_map) {
            const ZigBeeLogicalDevice& dev = entry.second.device;
            if (strncmp(dev.ieee, ieee_str, 16) == 0 && dev.tuya_dp_status == dp_number) {
                if (value_in_csv(dev.tuya_status_on, value)) return true;
                if (value_in_csv(dev.tuya_status_off, value)) return false;
                if (dev.tuya_status_on[0] != '\0' || dev.tuya_status_off[0] != '\0') return false;
            }
        }
    }

    if (dp_type == TUYA_TYPE_ENUM) {
        if (dp_number == 104 || dp_number == 105) {
            return (value == 1 || value == 2);
        }
    }
    return (value != 0);
}

static void gw_cache_tuya_dp_report(uint64_t ieee_addr, uint8_t src_endpoint,
                                    uint8_t dp_number, int32_t value, int lqi, uint8_t dp_type) {
    gw_cache_tuya_report(ieee_addr, src_endpoint,
                         ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC,
                         (uint16_t)dp_number,
                         value, lqi, dp_type);
    
    // Check if this DP report satisfies a pending query for Tuya data
    if (gw_read_pending && gw_read_pending_cluster == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC &&
        gw_read_pending_ieee == ieee_addr) {
        gw_read_pending = false;
        ZigbeeDeviceInfo* dev = gw_find_discovered_device(ieee_addr);
        if (dev && gw_device_needs_basic_info(*dev)) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW][TUYA] Received Tuya DP report, but device still needs Basic Cluster info — scheduling single-attribute Basic Cluster queries while device is awake"));
            // Clear batch queries for this IEEE and queue single-attribute 0x0004 & 0x0005 reads while device is active
            for (auto it = gw_basic_query_queue.begin(); it != gw_basic_query_queue.end(); ) {
                if (it->ieee_addr == ieee_addr) {
                    it = gw_basic_query_queue.erase(it);
                } else {
                    ++it;
                }
            }
            gw_queue_basic_cluster_query_attr(ieee_addr, gw_get_short_addr(ieee_addr), src_endpoint, ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, 100UL);
            gw_queue_basic_cluster_query_attr(ieee_addr, gw_get_short_addr(ieee_addr), src_endpoint, ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 600UL);
        } else {
            DEBUG_PRINTLN(F("[ZIGBEE-GW][TUYA] Received Tuya DP report successfully, clearing basic query queue for this device"));
            // Remove basic query requests from the queue for this IEEE
            for (auto it = gw_basic_query_queue.begin(); it != gw_basic_query_queue.end(); ) {
                if (it->ieee_addr == ieee_addr) {
                    it = gw_basic_query_queue.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    // Keep the dashboard's Zigbee physical-state icon in sync when a valve is
    // operated manually on the device and reports its switch DP back.
    bool actual_on = gw_is_tuya_status_on(ieee_addr, dp_number, value, dp_type);
    gw_station_status_process_tuya_dp(ieee_addr, src_endpoint, dp_number, actual_on);
    // Check if this DP echoes a pending station switch command.
    gw_station_verify_process(ieee_addr, dp_number, actual_on);
}

// Tuya sequence counter for outgoing commands. Some Tuya MCU devices reject
// repeated/lower transaction numbers across gateway restarts, so keep it in NVS
// and store the next value before dispatching each command.
static uint16_t gw_tuya_seq = 0;
static bool gw_tuya_seq_loaded = false;

static void gw_save_tuya_seq(uint16_t next_seq) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("zb_gw", NVS_READWRITE, &handle);
    if (err != ESP_OK) return;
    nvs_set_u16(handle, "tuya_seq", next_seq);
    nvs_commit(handle);
    nvs_close(handle);
}

static void gw_load_tuya_seq() {
    if (gw_tuya_seq_loaded) return;
    gw_tuya_seq_loaded = true;

    nvs_handle_t handle;
    esp_err_t err = nvs_open("zb_gw", NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        uint16_t stored = 0;
        // Per Tuya MCU UART Protocol: SEQ cycles from 0 to 0xfff0 (65520)
        if (nvs_get_u16(handle, "tuya_seq", &stored) == ESP_OK && stored <= 0xfff0) {
            gw_tuya_seq = stored;
        } else {
            // Start at 0 per Tuya spec (0-0xfff0 cycling)
            gw_tuya_seq = 0;
            nvs_set_u16(handle, "tuya_seq", gw_tuya_seq);
            nvs_commit(handle);
        }
        nvs_close(handle);
    } else {
        gw_tuya_seq = 0;
    }

    DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Sequence start=%u (0x%04X, max=0xfff0=%u)\n"), 
                 (unsigned)gw_tuya_seq, (unsigned)gw_tuya_seq, 0xfff0);
}

static void gw_zigbee_default_response_cb(zb_cmd_type_t resp_to_cmd, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
    // Zigbee task: only hand the fact over to the main loop.
    (void)resp_to_cmd;
    if (cluster != ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC && cluster != ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) return;
    GwRxEvent ev = {};
    ev.kind = GW_RX_DEFAULT_RESP;
    ev.endpoint = endpoint;
    ev.cluster_id = cluster;
    ev.status = (int32_t)status;
    gw_rx_push(ev);
}

static uint16_t gw_next_tuya_seq() {
    gw_load_tuya_seq();
    // Per Tuya MCU UART Protocol: Sequence number cycles from 0 to 0xfff0 (65520)
    if (gw_tuya_seq > 0xfff0) gw_tuya_seq = 0;
    uint16_t seq = gw_tuya_seq;
    gw_tuya_seq = (uint16_t)(gw_tuya_seq + 1);
    // Wrap at 0xfff1 (65521) back to 0
    if (gw_tuya_seq > 0xfff0) gw_tuya_seq = 0;
    gw_save_tuya_seq(gw_tuya_seq);
    return seq;
}

// Reset Tuya sequence to 0 (called when rejoin is triggered for sync)
static void gw_reset_tuya_seq() {
    gw_tuya_seq = 0;
    gw_tuya_seq_loaded = true;
    gw_save_tuya_seq(0);
    DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Sequence reset to 0 for device rejoin\n"));
}

static size_t gw_build_tuya_dp_payload(uint8_t* payload, size_t payload_size,
                                       uint16_t seq, uint8_t dp_id, uint8_t dp_type,
                                       const uint8_t* value, uint16_t value_len) {
    const size_t required = 6U + value_len;
    if (!payload || !value || payload_size < required) return 0;

    payload[0] = (uint8_t)(seq >> 8);
    payload[1] = (uint8_t)(seq & 0xFF);
    payload[2] = dp_id;
    payload[3] = dp_type;
    payload[4] = (uint8_t)(value_len >> 8);
    payload[5] = (uint8_t)(value_len & 0xFF);
    memcpy(payload + 6, value, value_len);
    return required;
}

/**
 * @brief Send a Tuya time sync response (cmd 0x24) to a device.
 *
 * Tuya devices request time synchronization right after joining the network.
 * If the gateway doesn't respond, many Tuya devices will refuse to report
 * data or will leave the network entirely.
 *
 * Response payload (14 bytes):
 *   [0..3]  UTC seconds since 2000-01-01 (big-endian)
 *   [4]     Local time offset (unused, set to 0)
 *   [5..12] Local time in same format (big-endian), same as UTC for simplicity
 */
static void gw_tuya_send_time_sync(uint16_t short_addr, uint8_t dst_ep, uint8_t seq_number) {
    // Tuya epoch starts 2000-01-01 00:00:00 UTC
    static const uint32_t TUYA_EPOCH_OFFSET = 946684800UL;  // Unix timestamp of 2000-01-01

    time_t now_unix = time(nullptr);
    uint32_t tuya_time = 0;
    if (now_unix > (time_t)TUYA_EPOCH_OFFSET) {
        tuya_time = (uint32_t)(now_unix - TUYA_EPOCH_OFFSET);
    }

    // Build response payload: ZCL header (3 bytes) + Tuya seq (2 bytes) + time data (8 bytes)
    uint8_t payload[8];
    // UTC time (big-endian)
    payload[0] = (tuya_time >> 24) & 0xFF;
    payload[1] = (tuya_time >> 16) & 0xFF;
    payload[2] = (tuya_time >> 8)  & 0xFF;
    payload[3] = (tuya_time)       & 0xFF;
    // Local time (same as UTC)
    payload[4] = payload[0];
    payload[5] = payload[1];
    payload[6] = payload[2];
    payload[7] = payload[3];

    esp_zb_zcl_custom_cluster_cmd_req_t req = {};
    req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    req.zcl_basic_cmd.dst_endpoint = dst_ep;
    req.zcl_basic_cmd.src_endpoint = 10;
    req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_id = ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC;
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    req.dis_default_resp = 1;
    req.custom_cmd_id = TUYA_CMD_TIME_SYNC_REQ;  // Response uses same cmd ID
    req.data.type = ESP_ZB_ZCL_ATTR_TYPE_SET;
    req.data.size = sizeof(payload);
    req.data.value = payload;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();

    // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Time sync response sent to 0x%04X (tuya_time=%lu)\n"),
                // short_addr, (unsigned long)tuya_time);
}

/**
 * @brief Send a Tuya MCU version response (cmd 0x11) to a device.
 *
 * Some Tuya devices query the gateway's MCU version during interview.
 * A minimal response prevents the device from timing out or leaving.
 */
static void gw_tuya_send_mcu_version_resp(uint16_t short_addr, uint8_t dst_ep, uint8_t seq_number) {
    // Response payload: version byte (we report 0x40 = 4.0 like Tuya gateways)
    uint8_t payload[1] = { 0x40 };

    esp_zb_zcl_custom_cluster_cmd_req_t req = {};
    req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    req.zcl_basic_cmd.dst_endpoint = dst_ep;
    req.zcl_basic_cmd.src_endpoint = 10;
    req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_id = ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC;
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    req.dis_default_resp = 1;
    req.custom_cmd_id = TUYA_CMD_MCU_VERSION_RESP;
    req.data.type = ESP_ZB_ZCL_ATTR_TYPE_SET;
    req.data.size = sizeof(payload);
    req.data.value = payload;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();

    // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] MCU version response sent to 0x%04X\n"), short_addr);
}

/**
 * @brief Send a Tuya DP Query (cmd 0x03) to a device to request all datapoints.
 *
 * This is the "interview" that triggers Tuya devices to report their initial
 * sensor values.  Many Tuya devices will NOT spontaneously report data until
 * they receive this query from the gateway.
 */
static void gw_tuya_send_dp_query(uint16_t short_addr, uint8_t dst_ep) {
    // Never query the coordinator itself (0x0000) or an invalid short address.
    // A DP query (TO_SRV) to 0x0000 loops back to our own endpoint as an
    // incoming DATA_QUERY indication, which the handler answers with yet another
    // DP query -> endless self-query flood (tsn keeps incrementing).
    if (short_addr == 0x0000 || short_addr == 0xFFFF || short_addr == 0xFFFE) {
        DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP query skipped: invalid/self short_addr=0x%04X\n"), short_addr);
        return;
    }

    // DP Query has no payload - the Tuya device responds with all its DPs
    esp_zb_zcl_custom_cluster_cmd_req_t req = {};
    req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    req.zcl_basic_cmd.dst_endpoint = dst_ep;
    req.zcl_basic_cmd.src_endpoint = 10;
    req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_id = ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC;
    req.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    req.dis_default_resp = 1;
    req.custom_cmd_id = TUYA_CMD_DATA_QUERY;
    req.data.type = ESP_ZB_ZCL_ATTR_TYPE_SET;
    req.data.size = 0;
    req.data.value = nullptr;

    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t tsn = esp_zb_zcl_custom_cluster_cmd_req(&req);
    esp_zb_lock_release();

    DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP query sent -> short_addr=0x%04X ep=%d cmd=0x%02X dir=TO_SRV tsn=%u\n"),
                 short_addr, dst_ep, TUYA_CMD_DATA_QUERY, (unsigned)tsn);
}

// Send a ZCL Default Response (general command 0x0B, status SUCCESS) for a
// received cluster-specific command. Many Tuya EF00 devices send their data
// reports with the ZCL "disable default response" bit CLEARED, i.e. they expect
// a ZCL Default Response. Because our APS indication handler consumes the frame
// (returns true), the stack's ZCL layer never auto-generates that response, so
// the device keeps RETRANSMITTING the identical frame (same zcl_seq) — the
// flood. Sending the response the device waits for stops the retransmits.
static void gw_tuya_send_default_response(uint16_t short_addr, uint8_t dst_ep,
                                          uint8_t zcl_seq, uint8_t rsp_to_cmd_id) {
    // [0] frame control: general(00) | dir client->server(0) | disable default
    //     response(1) = 0x10; [1] seq echoed; [2] 0x0B Default Response;
    //     [3] response-to command id; [4] status 0x00 SUCCESS.
    uint8_t asdu[5] = { 0x10, zcl_seq, 0x0B, rsp_to_cmd_id, 0x00 };

    esp_zb_apsde_data_req_t req = {};
    req.dst_addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.dst_addr.addr_short = short_addr;
    req.dst_endpoint = dst_ep;
    req.src_endpoint = 10;
    req.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    req.cluster_id = ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC;
    req.asdu_length = sizeof(asdu);
    req.asdu = asdu;
    req.tx_options = ESP_ZB_APSDE_TX_OPT_ACK_TX;
    req.radius = 0;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_aps_data_request(&req);
    esp_zb_lock_release();
}

bool sensor_zigbee_gw_request_dp_query(uint64_t device_ieee, uint8_t endpoint) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) return false;
    if (device_ieee == 0) return false;

    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }

    esp_zb_lock_acquire(portMAX_DELAY);
    uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
    esp_zb_lock_release();

    if (short_addr == 0xFFFF || short_addr == 0xFFFE) {
        DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP query failed: short addr unknown for ieee=%016llX\n"),
                     (unsigned long long)device_ieee);
        return false;
    }

    if (!gw_allow_oneshot_for_device(device_ieee, "tuya_query")) {
        return false;
    }

    if (!gw_is_device_access_allowed(device_ieee, short_addr, false, 5000UL)) {
        DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP query blocked: rate-limited/cooldown (5s) for ieee=%016llX\n"),
                     (unsigned long long)device_ieee);
        return false;
    }

    gw_tuya_send_dp_query(short_addr, endpoint);
    gw_record_device_access(device_ieee, short_addr, false);
    return true;
}

bool sensor_zigbee_gw_query_basic_cluster_queued(uint64_t device_ieee, uint8_t endpoint) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) return false;
    if (device_ieee == 0) return false;

    DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] request basic ieee=%016llX ep=%u\n"),
                 (unsigned long long)device_ieee, (unsigned)endpoint);
    gw_queue_device_query(device_ieee, endpoint, true, false);
    return true;
}

bool sensor_zigbee_gw_query_device_data(uint64_t device_ieee, uint8_t endpoint) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) return false;
    if (device_ieee == 0) return false;

    DEBUG_PRINTF(F("[ZIGBEE-GW][QUERY] request device_data ieee=%016llX ep=%u\n"),
                 (unsigned long long)device_ieee, (unsigned)endpoint);
    gw_queue_device_query(device_ieee, endpoint, true, true);
    return true;
}

// ---------------------------------------------------------------------------
// Tuya frame de-duplication.
// The device (or the mesh) frequently re-delivers the exact same DP report many
// times which otherwise re-runs sensor updates and switch matching for every
// copy. Keying on the ZCL sequence number is unreliable: several Tuya devices
// reuse the SAME zcl_seq for successive, distinct reports (they never increment
// it), so a seq-based window both let retransmits through AND dropped genuine
// value changes that happened to share the seq. Instead we key on a hash of the
// DP-record payload: identical payloads within the window are dropped as
// retransmits, while any value change produces a different hash and is processed
// immediately. Only applied to DP-bearing report frames; control frames
// (time-sync, MCU version, DP query) are always handled.
// ---------------------------------------------------------------------------
#define GW_TUYA_DEDUP_MAX        12
#define GW_TUYA_DEDUP_WINDOW_MS 5000
struct GwTuyaDedup { uint16_t short_addr; uint32_t hash; uint32_t last_ms; bool used; };
static PSRAM_BSS_ATTR GwTuyaDedup gw_tuya_dedup[GW_TUYA_DEDUP_MAX];

// FNV-1a hash of the Tuya DP records (skips the ZCL header and the 2-byte Tuya
// transaction sequence, which change between otherwise-identical retransmits).
static uint32_t gw_tuya_payload_hash(const uint8_t* asdu, uint16_t asdu_length) {
    uint32_t h = 2166136261u;
    // DP records start at offset 5 (ZCL hdr[3] + Tuya tx-seq[2]).
    for (uint16_t i = 5; i < asdu_length; i++) {
        h ^= asdu[i];
        h *= 16777619u;
    }
    return h;
}

static bool gw_tuya_frame_is_duplicate(uint16_t short_addr, uint32_t payload_hash) {
    uint32_t now = millis();
    int free_slot = -1, oldest = 0;
    uint32_t oldest_ms = now;
    bool have_oldest = false;
    for (int i = 0; i < GW_TUYA_DEDUP_MAX; i++) {
        if (gw_tuya_dedup[i].used && gw_tuya_dedup[i].short_addr == short_addr) {
            if (gw_tuya_dedup[i].hash == payload_hash &&
                (uint32_t)(now - gw_tuya_dedup[i].last_ms) < GW_TUYA_DEDUP_WINDOW_MS) {
                return true; // identical DP payload within the window → retransmit
            }
            gw_tuya_dedup[i].hash = payload_hash;
            gw_tuya_dedup[i].last_ms = now;
            return false;
        }
        if (!gw_tuya_dedup[i].used) {
            if (free_slot < 0) free_slot = i;
        } else if (!have_oldest || (int32_t)(gw_tuya_dedup[i].last_ms - oldest_ms) < 0) {
            oldest_ms = gw_tuya_dedup[i].last_ms;
            oldest = i;
            have_oldest = true;
        }
    }
    int use = (free_slot >= 0) ? free_slot : oldest;
    gw_tuya_dedup[use].short_addr = short_addr;
    gw_tuya_dedup[use].hash = payload_hash;
    gw_tuya_dedup[use].last_ms = now;
    gw_tuya_dedup[use].used = true;
    return false;
}


// Mark a device as Tuya (so the loop can send periodic DP queries) and give it
// a generic model when the Basic Cluster interview has not completed yet.
// Runs in the main loop (called from gw_rx_drain()).
static void gw_mark_tuya_device(uint64_t ieee_addr) {
    // Mark this device as a Tuya device (so the loop can send periodic DP queries)
for (auto& dev : gw_discovered_devices) {
    if (dev.ieee_addr == ieee_addr) {
        bool dirty = false;
        if (!dev.is_tuya) {
            dev.is_tuya = true;
            dirty = true;
        }
        // If the model identifier is still unknown, apply a generic Tuya
        // model ("TS0601") so per-DP sensor handling can proceed. We must
        // NOT invent a manufacturer string here: stamping a concrete
        // manufacturer (previously the GX02 valve's "_TZE200_sh1btabb")
        // makes gw_device_needs_basic_info() return false, which suppresses
        // the real Basic Cluster query forever — so EVERY unidentified Tuya
        // device ended up mislabeled as a "GIEX GX02 Water Valve" in /zd and
        // the UI/database name lookup. Leaving the manufacturer empty keeps
        // the Basic Cluster query active so the real manufacturer is filled
        // in (and the database can then provide the correct device name).
        if (dev.model_id[0] == '\0' || strcmp(dev.model_id, "unknown") == 0) {
            DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Received Tuya DP on device with unknown model ieee=%016llX. Applying generic TS0601 model (manufacturer left empty for Basic Cluster resolution).\n"),
                         (unsigned long long)ieee_addr);
            strncpy(dev.model_id, "TS0601", sizeof(dev.model_id) - 1);
            dev.model_id[sizeof(dev.model_id) - 1] = '\0';
            dirty = true;
        }
        if (dirty) gw_mark_discovered_devices_dirty();
        break;
    }
}
}

static bool gw_tuya_aps_indication_handler(esp_zb_apsde_data_ind_t ind) {
    // Log ALL incoming APS frames for debugging
    // DEBUG_PRINTF(F("[ZIGBEE-GW][APS] Indication: cluster=0x%04X src=0x%04X ep=%d len=%u prof=0x%04X\n"),
                // ind.cluster_id, ind.src_short_addr, ind.src_endpoint, ind.asdu_length, ind.profile_id);
    
    // Only intercept Tuya cluster 0xEF00
    if (ind.cluster_id != ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC) {
        return false;  // Let the stack handle non-Tuya clusters normally
    }

    // Minimum ZCL header: 3 bytes (frame_control + seq + command_id)
    if (!ind.asdu || ind.asdu_length < 3) {
        // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Frame too short (%u bytes), ignoring\n"), ind.asdu_length);
        return true;  // Consume anyway — it's on cluster 0xEF00
    }

    uint8_t seq_number = ind.asdu[1];
    uint8_t command_id = ind.asdu[2];

    ZB_GW_TRACE(F("[ZIGBEE-GW][TUYA] cmd=0x%02X zcl_seq=%u len=%u from 0x%04X ep=%u fc=0x%02X\n"),
                 command_id, seq_number, ind.asdu_length, ind.src_short_addr, ind.src_endpoint, ind.asdu[0]);

    // Handle Tuya time sync request (0x24)
    // Many Tuya devices send this right after joining; without a response
    // they refuse to report data or leave the network.
    if (command_id == TUYA_CMD_TIME_SYNC_REQ) {
        // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Time sync request from 0x%04X\n"), ind.src_short_addr);
        gw_tuya_send_time_sync(ind.src_short_addr, ind.src_endpoint, seq_number);
        // Resolve IEEE from stack and add as responsive device
        gw_rx_push_seen_from_short(ind.src_short_addr, ind.src_endpoint);
        return true;
    }

    // Handle Tuya MCU version query (0x10)
    // Some Tuya devices query the gateway MCU version during interview.
    if (command_id == TUYA_CMD_MCU_VERSION_REQ) {
        // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] MCU version request from 0x%04X\n"), ind.src_short_addr);
        gw_tuya_send_mcu_version_resp(ind.src_short_addr, ind.src_endpoint, seq_number);
        // Resolve IEEE from stack and add as responsive device
        gw_rx_push_seen_from_short(ind.src_short_addr, ind.src_endpoint);
        return true;
    }

    // Some Tuya firmwares send MCU version response (0x11) spontaneously
    // during interview/keepalive. Treat it as a known non-DP control frame.
    if (command_id == TUYA_CMD_MCU_VERSION_RESP) {
        gw_rx_push_seen_from_short(ind.src_short_addr, ind.src_endpoint);
        return true;
    }

    // Handle Tuya data query seen from a device defensively: answer with a
    // gateway query, matching the legacy behavior this firmware used before.
    if (command_id == TUYA_CMD_DATA_QUERY) {
        // Ignore self-originated / loopback queries (coordinator short addr
        // 0x0000). Our own TO_SRV DP query loops back here; answering it would
        // create an endless self-query flood.
        if (ind.src_short_addr == 0x0000) {
            return true;
        }
        // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP query from 0x%04X - sending query now\n"), ind.src_short_addr);
        // Resolve IEEE from stack and add as responsive device
        gw_rx_push_seen_from_short(ind.src_short_addr, ind.src_endpoint);
        // Send DP query directly
        gw_tuya_send_dp_query(ind.src_short_addr, ind.src_endpoint);
        return true;
    }

    // Acknowledge the frame the way the device expects. If it left the ZCL
    // "disable default response" bit (frame control bit 4) CLEARED, it wants a
    // ZCL Default Response; without it the device retransmits the same frame
    // repeatedly (the flood). This must run for EVERY command — including
    // unhandled ones like 0x07 — otherwise the device never gets its ACK and
    // keeps resending forever. Respond before the unhandled-command bailout and
    // before the dedup below so every received copy is answered.
    if ((ind.asdu[0] & 0x10) == 0) {
        gw_tuya_send_default_response(ind.src_short_addr, ind.src_endpoint,
                                      seq_number, command_id);
    }

    // Process all known DP-bearing response/report variants.
    if (command_id != TUYA_CMD_DATA_RESPONSE &&
        command_id != TUYA_CMD_DATA_REPORT &&
        command_id != TUYA_CMD_DATA_SEND &&
        command_id != TUYA_CMD_ACTIVE_REPORT &&
        command_id != TUYA_CMD_ACTIVE_REPORT2 &&
        command_id != TUYA_CMD_MCU_STATUS_REPORT &&
        command_id != TUYA_CMD_MCU_STATUS_REPORT_SYN &&
        command_id != TUYA_CMD_ACTIVE_STATUS) {
        DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Unhandled command 0x%02X from 0x%04X (ACKed)\n"),
                command_id, ind.src_short_addr);
        return true;  // Consume — don't let ZCL stack fail on unknown Tuya commands
    }

    // Drop retransmitted duplicates of the same DP report (same source +
    // identical DP payload) so one report isn't processed several times, even
    // when the device reuses the ZCL sequence number across distinct reports.
    if (gw_tuya_frame_is_duplicate(ind.src_short_addr,
                                   gw_tuya_payload_hash(ind.asdu, ind.asdu_length))) {
        return true;
    }

    ZB_GW_TRACE(F("[ZIGBEE-GW][TUYA] APS ind: src=0x%04X ep=%u len=%u\n"),
                 ind.src_short_addr, ind.src_endpoint, ind.asdu_length);

    // Need at least 5 bytes for ZCL header (3) + Tuya seq (2) for DP parsing
    if (ind.asdu_length < 5) {
        // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP frame too short (%u bytes)\n"), ind.asdu_length);
        return true;
    }

    // Resolve IEEE address from short address via the stack's address table.
    // Add device as responsive since it's reporting datapoints.
    esp_zb_ieee_addr_t raw_ieee;
    uint64_t ieee_addr = 0;
    if (esp_zb_ieee_address_by_short(ind.src_short_addr, raw_ieee) == ESP_OK) {
        ieee_addr = gw_ieee_from_raw(raw_ieee);
    }

    if (!ieee_addr) {
        // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Could not resolve IEEE for short=0x%04X\n"), ind.src_short_addr);
        return true;
    }

    // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Processing DP frame: cmd=0x%02X len=%u src=0x%04X\n"),
                // command_id, ind.asdu_length, ind.src_short_addr);

    // Parse DP records starting after ZCL header (3 bytes) + Tuya seq (2 bytes) = offset 5
    uint32_t offset = 5;
    int dps_parsed = 0;
    while (offset + 4 <= ind.asdu_length) {  // Minimum DP record: 4 bytes header
        uint8_t dp_number = ind.asdu[offset];
        uint8_t dp_type = ind.asdu[offset + 1];
        uint16_t dp_len = ((uint16_t)ind.asdu[offset + 2] << 8) | ind.asdu[offset + 3];
        offset += 4;

        if (offset + dp_len > ind.asdu_length) {
            // DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] DP %d truncated (need %u, have %u)\n"),
                        // dp_number, dp_len, ind.asdu_length - offset);
            break;
        }

        // Extract value (big-endian for numeric types)
        int32_t dp_value = 0;
        if (dp_type == TUYA_TYPE_VALUE && dp_len == 4) {
            dp_value = (int32_t)(((uint32_t)ind.asdu[offset] << 24) |
                                 ((uint32_t)ind.asdu[offset + 1] << 16) |
                                 ((uint32_t)ind.asdu[offset + 2] << 8) |
                                 ((uint32_t)ind.asdu[offset + 3]));
        } else if (dp_type == TUYA_TYPE_ENUM || dp_type == TUYA_TYPE_BOOL) {
            dp_value = (int32_t)ind.asdu[offset];
        } else if (dp_len <= 4) {
            // Generic big-endian extraction for short payloads
            for (uint16_t j = 0; j < dp_len; j++) {
                dp_value = (dp_value << 8) | ind.asdu[offset + j];
            }
        }

        ZB_GW_TRACE(F("[ZIGBEE-GW][TUYA] DP %u: type=%u len=%u value=%ld\n"),
                dp_number, dp_type, dp_len, dp_value);

        // Always cache raw Tuya DP reports (cluster 0xEF00, attr=DP id)
        // so per-sensor custom DP mappings can consume them.
        gw_rx_push_tuya_dp(ieee_addr, ind.src_short_addr, ind.src_endpoint, dp_number, dp_type, dp_value, ind.lqi);

        offset += dp_len;
        dps_parsed++;
    }

    // A bare Tuya ACK without DP records only confirms protocol receipt. It is
    // not proof that a valve actually opened or closed, so switch verification
    // remains pending until the expected DP report arrives or times out.

    return true;  // Consumed — do not let ZCL stack process cluster 0xEF00
}

// ========== Configure Reporting helpers ==========

// Return ZCL attribute data type for the MeasuredValue (0x0000) attribute
// of standard measurement clusters. Required by Configure Reporting command.
static uint8_t gw_attr_type(uint16_t cluster_id, uint16_t attr_id) {
    // Power Configuration: BatteryVoltage (0x0020) and
    // BatteryPercentageRemaining (0x0021) are uint8.  A Configure Reporting
    // record with the wrong data type is rejected by the device
    // (INVALID_DATA_TYPE), so the type must be attribute-aware here.
    if (cluster_id == ZB_ZCL_CLUSTER_ID_POWER_CONFIG &&
        (attr_id == 0x0020 || attr_id == 0x0021)) {
        return 0x20; // uint8 (U8)
    }
    switch (cluster_id) {
        case ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:     return 0x29; // int16 (S16)
        case ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT: return 0x29; // int16 (S16)
        default:                                      return 0x21; // uint16 (U16)
    }
}

static bool gw_cluster_supports_config_reporting(uint16_t cluster_id, uint16_t attr_id) {
    // Simple Metering CurrentSummationDelivered is uint48. The current report
    // helper is sized for the common measurement attributes, so water meters
    // use active reads or their own unsolicited reports instead.
    if (cluster_id == ZB_ZCL_CLUSTER_ID_METERING && attr_id == 0x0000) return false;
    return true;
}

static void gw_zdo_bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] ZDO Bind request SUCCESS"));
    } else {
        DEBUG_PRINTF(F("[ZIGBEE-GW] ZDO Bind request FAILED: status %d\n"), zdo_status);
    }
}

bool sensor_zigbee_gw_bind_device(uint64_t device_ieee, uint8_t endpoint, uint16_t cluster_id) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) {
        return false;
    }
    if (device_ieee == 0) return false;

    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }

    uint16_t short_addr = gw_get_short_addr(device_ieee);
    if (short_addr == 0xFFFF || short_addr == 0xFFFE) {
        // We can't bind if we don't know the short address
        return false;
    }

    esp_zb_zdo_bind_req_param_t bind_req;
    memset(&bind_req, 0, sizeof(bind_req));
    bind_req.req_dst_addr = short_addr;
    memcpy(bind_req.src_address, ieee_le, 8);
    bind_req.src_endp = endpoint;
    bind_req.cluster_id = cluster_id;
    bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
    bind_req.dst_endp = 10; // OpenSprinkler coordinator endpoint

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zdo_device_bind_req(&bind_req, gw_zdo_bind_cb, NULL);
    esp_zb_lock_release();

    DEBUG_PRINTF(F("[ZIGBEE-GW] ✓ Bind Req sent: ieee=%016llX ep=%d cluster=0x%04X to coord ep=10\n"),
                 (unsigned long long)device_ieee, endpoint, cluster_id);
    return true;
}

// Send a ZCL Configure Reporting command for one attribute.
// Tells the remote device to push reports every [min_interval..max_interval] seconds.
bool sensor_zigbee_gw_configure_reporting(uint64_t device_ieee, uint8_t endpoint,
                                           uint16_t cluster_id, uint16_t attr_id,
                                           uint16_t min_interval, uint16_t max_interval) {
    if (!gw_cluster_supports_config_reporting(cluster_id, attr_id)) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] ConfigReport skipped: unsupported attr width c=0x%04X a=0x%04X\n"),
                     cluster_id, attr_id);
        return false;
    }
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) {
        // DEBUG_PRINTF("[ZIGBEE-GW] ConfigReport SKIP: not ready. ieee=%016llX\n",
                     // (unsigned long long)device_ieee);
        return false;
    }
    if (device_ieee == 0) return false;

    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }

    uint16_t delta_val = 0;  // report on any change (threshold = 0 raw units)

    esp_zb_zcl_config_report_record_t record;
    memset(&record, 0, sizeof(record));
    record.direction    = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
    record.attributeID  = attr_id;
    record.attrType     = gw_attr_type(cluster_id, attr_id);
    record.min_interval = min_interval;
    record.max_interval = max_interval;
    record.reportable_change = &delta_val;

    esp_zb_zcl_config_report_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.zcl_basic_cmd.src_endpoint = 10;
    cmd.zcl_basic_cmd.dst_endpoint = endpoint;
    cmd.clusterID      = cluster_id;
    cmd.dis_default_resp = 1;
    cmd.record_number  = 1;
    cmd.record_field   = &record;

    esp_zb_lock_acquire(portMAX_DELAY);
    uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
    if (short_addr != 0xFFFF && short_addr != 0xFFFE) {
        cmd.address_mode = (esp_zb_zcl_address_mode_t)ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    } else {
        cmd.address_mode = (esp_zb_zcl_address_mode_t)ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
        memcpy(cmd.zcl_basic_cmd.dst_addr_u.addr_long, ieee_le, sizeof(ieee_le));
    }
    esp_zb_zcl_config_report_cmd_req(&cmd);
    esp_zb_lock_release();

    // DEBUG_PRINTF("[ZIGBEE-GW] \xE2\x9C\x93 Configure Reporting SENT: ieee=%016llX short=0x%04X ep=%d "
                 // "cluster=0x%04X attr=0x%04X min=%ds max=%ds\n",
                 // (unsigned long long)device_ieee, short_addr, endpoint,
                 // cluster_id, attr_id, min_interval, max_interval);
    return true;
}

// Per-device throttle for the battery setup.  The per-IEEE scheduler below is
// invoked on every incoming frame of a known device, so without this gate a
// sleepy sensor would get a Bind + ConfigureReporting + Read on every report.
// A sleepy device only receives the unicast Bind/ConfigureReporting if it
// polls its parent within the indirect-transmission window (~7.7 s), which is
// not guaranteed after a report.  So retry on the device's frames: the first
// GW_BATTERY_FAST_ATTEMPTS at >= GW_BATTERY_FAST_SPACING_MS, afterwards once
// per GW_BATTERY_SLOW_SPACING_MS.  The initial read is sent only once (the
// single active-read slot is too valuable to burn on every retry).
#define GW_BATTERY_FAST_ATTEMPTS      3
#define GW_BATTERY_FAST_SPACING_MS    (120UL * 1000UL)
#define GW_BATTERY_SLOW_SPACING_MS    (60UL * 60UL * 1000UL)
#define GW_BATTERY_MAX_TRACKED 32
struct GwBatteryAttempt {
    uint64_t ieee_addr;
    unsigned long last_ms;
    uint8_t  attempts;
};
static std::vector<GwBatteryAttempt> gw_battery_attempts;

// Returns the attempt number (1 = first) or 0 when throttled.
static uint8_t gw_battery_attempt_allowed(uint64_t ieee, unsigned long now) {
    for (auto& a : gw_battery_attempts) {
        if (a.ieee_addr == ieee) {
            unsigned long spacing = (a.attempts < GW_BATTERY_FAST_ATTEMPTS)
                                        ? GW_BATTERY_FAST_SPACING_MS : GW_BATTERY_SLOW_SPACING_MS;
            if (now - a.last_ms < spacing) return 0;
            a.last_ms = now;
            if (a.attempts < 255) a.attempts++;
            return a.attempts;
        }
    }
    if (gw_battery_attempts.size() >= GW_BATTERY_MAX_TRACKED) {
        gw_battery_attempts.erase(gw_battery_attempts.begin());
    }
    gw_battery_attempts.push_back({ieee, now, 1});
    return 1;
}

// Queue Bind + ConfigureReporting for Power Configuration / battery percentage
// and a one-time initial read.  Returns the delay after the last queued item.
// Tuya (0xEF00) devices deliver battery as a DP and usually do not implement
// cluster 0x0001, so they are skipped.
//   force=true  : (re)join — always queue (device is awake and may have lost
//                 its binding table).
//   force=false : opportunistic — only while the battery is still unknown and
//                 at most once per hour per device.
static unsigned long gw_queue_battery_reporting(uint64_t ieee, uint8_t ep, unsigned long delay_ms, bool force) {
    if (ieee == 0) return delay_ms;
    ZigbeeDeviceInfo* dev = gw_find_discovered_device(ieee);
    if (dev && dev->is_tuya) return delay_ms;
    if (!force && dev && dev->battery != 255) return delay_ms;

    unsigned long now = millis();
    uint8_t attempt = gw_battery_attempt_allowed(ieee, now);
    if (attempt == 0 && !force) return delay_ms;
    bool with_read = force || attempt == 1;
    DEBUG_PRINTF(F("[ZIGBEE-GW] Battery setup (0x0001/0x0021) queued for ieee=%016llX ep=%u force=%d attempt=%u read=%d delay=%lums\n"),
                 (unsigned long long)ieee, (unsigned)ep, force ? 1 : 0, (unsigned)attempt, with_read ? 1 : 0, delay_ms);

    bool bind_found = false;
    for (const auto& ex : gw_bind_queue) {
        if (ex.ieee_addr == ieee && ex.cluster_id == ZB_ZCL_CLUSTER_ID_POWER_CONFIG) {
            bind_found = true;
            break;
        }
    }
    if (!bind_found) {
        GwBindRequest bind_req;
        bind_req.ieee_addr = ieee;
        bind_req.endpoint = ep;
        bind_req.cluster_id = ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
        bind_req.scheduled_time = now + delay_ms;
        gw_bind_queue.push_back(bind_req);
        delay_ms += 150;
    }

    bool cr_found = false;
    for (const auto& ex : gw_config_report_queue) {
        if (ex.ieee_addr == ieee && ex.cluster_id == ZB_ZCL_CLUSTER_ID_POWER_CONFIG && ex.attr_id == 0x0021) {
            cr_found = true;
            break;
        }
    }
    if (!cr_found) {
        GwConfigReportRequest req;
        req.ieee_addr      = ieee;
        req.endpoint       = ep;
        req.cluster_id     = ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
        req.attr_id        = 0x0021;
        req.min_interval   = GW_BATTERY_REPORT_MIN_S;
        req.max_interval   = GW_BATTERY_REPORT_MAX_S;
        req.scheduled_time = now + delay_ms;
        gw_config_report_queue.push_back(req);
        delay_ms += 700;
    }

    if (!with_read) return delay_ms;

    bool rd_found = false;
    for (const auto& ex : gw_battery_read_queue) {
        if (ex.ieee_addr == ieee) {
            rd_found = true;
            break;
        }
    }
    if (!rd_found) {
        GwBatteryReadRequest rd;
        rd.ieee_addr      = ieee;
        rd.endpoint       = ep;
        rd.attempts       = 0;
        rd.scheduled_time = now + delay_ms + 1500;
        gw_battery_read_queue.push_back(rd);
        if (force) {
            // Right after a (re)join many devices answer 0x0021 with 0 (not
            // measured yet) but stay awake for a while.  A second read a bit
            // later usually returns the real level.
            rd.scheduled_time = now + delay_ms + GW_BATTERY_SECOND_READ_MS;
            gw_battery_read_queue.push_back(rd);
        }
    }
    return delay_ms;
}

// Drain the battery read queue: one read per loop tick, only when the single
// active-read slot is free.  Rate-limited sends are retried a few times.
static void gw_process_battery_read_queue() {
    if (gw_battery_read_queue.empty()) return;
    if (gw_read_pending) return;
    unsigned long now = millis();
    for (auto it = gw_battery_read_queue.begin(); it != gw_battery_read_queue.end(); ++it) {
        if ((long)(now - it->scheduled_time) < 0) continue;
        bool sent = sensor_zigbee_gw_read_attribute(it->ieee_addr, it->endpoint,
                                                    ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0021);
        if (sent || ++it->attempts >= GW_BATTERY_READ_MAX_ATTEMPTS) {
            gw_battery_read_queue.erase(it);
        } else {
            it->scheduled_time = now + GW_BATTERY_READ_RETRY_MS;
        }
        break;
    }
}

static void gw_schedule_configure_reporting_for_ieee(uint64_t ieee, unsigned long delay_ms, bool force_battery) {
    if (ieee == 0) return;
    SensorIterator it = sensors_iterate_begin();
    SensorBase* s;
    unsigned long now = millis();

    // Battery setup goes out first and without delay: a sleepy device that
    // just sent a frame only polls its parent for a short moment.
    uint8_t battery_ep = 0;  // endpoint used for the battery bind/report (0 = none)
    while ((s = sensors_iterate_next(it)) != NULL) {
        if (!s || s->type != SENSOR_ZIGBEE) continue;
        ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(s);
        if (zb->device_ieee != ieee) continue;
        if (zb->cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC) {
            battery_ep = 0;  // Tuya device: battery comes via DP, never via 0x0001
            break;
        }
        if (battery_ep == 0) battery_ep = zb->endpoint ? zb->endpoint : 1;
    }
    if (battery_ep != 0) {
        gw_queue_battery_reporting(ieee, battery_ep, 0, force_battery);
    }

    it = sensors_iterate_begin();
    while ((s = sensors_iterate_next(it)) != NULL) {
        if (!s || s->type != SENSOR_ZIGBEE) continue;
        ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(s);
        if (zb->device_ieee != ieee) continue;

        // Queue Bind Request
        bool bind_found = false;
        for (const auto& ex : gw_bind_queue) {
            if (ex.ieee_addr == ieee && ex.cluster_id == zb->cluster_id) {
                bind_found = true;
                break;
            }
        }
        if (!bind_found) {
            GwBindRequest bind_req;
            bind_req.ieee_addr = ieee;
            bind_req.endpoint = zb->endpoint;
            bind_req.cluster_id = zb->cluster_id;
            bind_req.scheduled_time = now + delay_ms;
            gw_bind_queue.push_back(bind_req);
            delay_ms += 150; // Slight stagger before config report
        }

        uint ri = zb->read_interval ? zb->read_interval : 60;
        uint16_t max_interval = (ri >= 15 && ri <= 3600) ? (uint16_t)ri : 120;

        bool found = false;
        for (const auto& ex : gw_config_report_queue) {
            if (ex.ieee_addr == ieee && ex.cluster_id == zb->cluster_id && ex.attr_id == zb->attribute_id) {
                found = true;
                break;
            }
        }
        if (found) continue;

        GwConfigReportRequest req;
        req.ieee_addr      = ieee;
        req.endpoint       = zb->endpoint;
        req.cluster_id     = zb->cluster_id;
        req.attr_id        = zb->attribute_id;
        req.min_interval   = 10;
        req.max_interval   = max_interval;
        req.scheduled_time = now + delay_ms;
        gw_config_report_queue.push_back(req);
        // Update stored interval; clear prediction anchor if the interval changed
        // so it gets re-established on the next confirmed report.
        if (zb->report_interval_s != 0 && zb->report_interval_s != max_interval) {
            zb->join_anchor_ts = 0;
            DEBUG_PRINTF(F("[ZIGBEE-GW] ConfigReport interval changed %u\u2192%u for '%s' \u2014 anchor cleared\n"),
                         zb->report_interval_s, max_interval, zb->getName());
        }
        zb->report_interval_s = max_interval;
        delay_ms += 700;
    }
}

static void gw_schedule_default_configure_reporting(uint64_t ieee, uint8_t ep, unsigned long delay_ms) {
    if (ieee == 0) return;

    // Common ZCL measurement clusters + attribute 0x0000 (MeasuredValue)
    static const uint16_t common_clusters[] = {
        0x0402,  // Temperature Measurement
        0x0405,  // Relative Humidity
        0x0408,  // Soil Moisture (Leaf Wetness)
        0x0400,  // Illuminance Measurement
        0x0403,  // Pressure Measurement
    };

    unsigned long now = millis();
    for (size_t i = 0; i < sizeof(common_clusters) / sizeof(common_clusters[0]); i++) {
        // Queue Bind Request
        bool bind_found = false;
        for (const auto& ex : gw_bind_queue) {
            if (ex.ieee_addr == ieee && ex.cluster_id == common_clusters[i]) {
                bind_found = true;
                break;
            }
        }
        if (!bind_found) {
            GwBindRequest bind_req;
            bind_req.ieee_addr = ieee;
            bind_req.endpoint = ep;
            bind_req.cluster_id = common_clusters[i];
            bind_req.scheduled_time = now + delay_ms;
            gw_bind_queue.push_back(bind_req);
            delay_ms += 150;
        }

        // Check for duplicate entries
        bool found = false;
        for (const auto& ex : gw_config_report_queue) {
            if (ex.ieee_addr == ieee && ex.cluster_id == common_clusters[i] && ex.attr_id == 0x0000) {
                found = true;
                break;
            }
        }
        if (found) continue;

        GwConfigReportRequest req;
        req.ieee_addr      = ieee;
        req.endpoint       = ep;
        req.cluster_id     = common_clusters[i];
        req.attr_id        = 0x0000;
        req.min_interval   = 10;
        req.max_interval   = GW_DEFAULT_REPORT_INTERVAL;
        req.scheduled_time = now + delay_ms;
        gw_config_report_queue.push_back(req);
        delay_ms += 700;
    }
    delay_ms = gw_queue_battery_reporting(ieee, ep, delay_ms, true);
    DEBUG_PRINTF(F("[ZIGBEE-GW] Queued default Bind & ConfigReport (900s) for ieee=%016llX ep=%d (%d clusters)\n"),
                 (unsigned long long)ieee, ep, (int)(sizeof(common_clusters) / sizeof(common_clusters[0])));
}

class GwZigbeeReportReceiver : public ZigbeeEP {
public:
    GwZigbeeReportReceiver(uint8_t endpoint) : ZigbeeEP(endpoint) {
        _cluster_list = esp_zb_zcl_cluster_list_create();
        
        if (_cluster_list) {
            // SERVER-side mandatory clusters
            esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
            if (basic_cluster) {
                esp_zb_cluster_list_add_basic_cluster(_cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            }
            
            // CLIENT-side Basic Cluster - needed to receive Read Attributes Responses
            // when we query remote devices' Basic Cluster (ManufacturerName, ModelIdentifier)
            esp_zb_attribute_list_t *basic_client_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_BASIC);
            if (basic_client_cluster) {
                esp_zb_cluster_list_add_custom_cluster(_cluster_list, basic_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Basic cluster 0x0000 added (CLIENT for remote queries)"));
            }
            
            esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(NULL);
            if (identify_cluster) {
                esp_zb_cluster_list_add_identify_cluster(_cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
            }

            // CLIENT-side measurement clusters — required so the ZCL layer
            // routes incoming attribute reports to our zbAttributeRead() callback.
            // Without these, standard ZCL reports are dropped by the stack.
            esp_zb_attribute_list_t *temp_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
            if (temp_cluster) {
                esp_zb_cluster_list_add_temperature_meas_cluster(_cluster_list, temp_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Temp cluster 0x0402 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *humidity_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
            if (humidity_cluster) {
                esp_zb_cluster_list_add_humidity_meas_cluster(_cluster_list, humidity_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Humidity cluster 0x0405 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *soil_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_SOIL_MOISTURE);
            if (soil_cluster) {
                esp_zb_cluster_list_add_custom_cluster(_cluster_list, soil_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Soil moisture cluster 0x0408 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *pressure_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
            if (pressure_cluster) {
                esp_zb_cluster_list_add_pressure_meas_cluster(_cluster_list, pressure_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Pressure cluster 0x0403 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *light_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);
            if (light_cluster) {
                esp_zb_cluster_list_add_illuminance_meas_cluster(_cluster_list, light_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Illuminance cluster 0x0400 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *power_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
            if (power_cluster) {
                esp_zb_cluster_list_add_power_config_cluster(_cluster_list, power_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Power config cluster 0x0001 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *meter_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_METERING);
            if (meter_cluster) {
                esp_zb_cluster_list_add_custom_cluster(_cluster_list, meter_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Metering cluster 0x0702 added (CLIENT)"));
            }

            esp_zb_attribute_list_t *on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
            if (on_off_client_cluster) {
                esp_zb_cluster_list_add_custom_cluster(_cluster_list, on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] On/Off cluster 0x0006 added (CLIENT)"));
            }

            // Tuya manufacturer-specific cluster (0xEF00) as CLIENT so we receive
            // incoming DP reports from Tuya devices (GIEX GX-04, etc.)
            esp_zb_attribute_list_t *tuya_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC);
            if (tuya_cluster) {
                esp_zb_cluster_list_add_custom_cluster(_cluster_list, tuya_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Tuya cluster 0xEF00 added (CLIENT)"));
            }

            // Tuya cluster 0xEF00 as SERVER — Tuya devices check that the
            // coordinator advertises this cluster as a server during the
            // match descriptor / interview phase.  Without this, many Tuya
            // devices do not complete joining or refuse to send DP reports.
            esp_zb_attribute_list_t *tuya_srv_cluster = esp_zb_zcl_attr_list_create(ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC);
            if (tuya_srv_cluster) {
                esp_zb_cluster_list_add_custom_cluster(_cluster_list, tuya_srv_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
                // DEBUG_PRINTLN(F("[ZIGBEE-GW] Tuya cluster 0xEF00 added (SERVER)"));
            }
        }
        
        _ep_config.endpoint = endpoint;
        _ep_config.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID;
        _ep_config.app_device_id = ESP_ZB_HA_CONFIGURATION_TOOL_DEVICE_ID;
        _ep_config.app_device_version = 0;
        
        // DEBUG_PRINTLN(F("[ZIGBEE-GW] Report receiver endpoint created (full cluster config)"));

        // Allow multiple device bindings so findEndpoint() is called for every
        // new device announcement, not just the first one.
        _allow_multiple_binding = true;
    }
    
    virtual ~GwZigbeeReportReceiver() = default;

    // Called by ZigbeeCore when a new device announces itself (DEVICE_ANNCE).
    // Query the device for basic information and Tuya DPs if applicable.
    // Device is registered immediately so even sleepy/slow responders such as
    // the GIEX GX02 valve show up in the discovered list — we previously waited
    // for the first attribute response which never arrived for some devices
    // (the GX02 typically replies via zbReadBasicCluster which has no address
    // info, and so was effectively lost — the classic "one-shot" miss).
    void findEndpoint(esp_zb_zdo_match_desc_req_param_t *cmd_req) override {
        if (!cmd_req) return;
        uint16_t short_addr = cmd_req->dst_nwk_addr;

        // Resolve IEEE immediately so we can pre-register the device.
        uint64_t ieee_addr = 0;
        esp_zb_ieee_addr_t raw_ieee;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_err_t r = esp_zb_ieee_address_by_short(short_addr, raw_ieee);
        esp_zb_lock_release();
        if (r == ESP_OK) {
            for (int i = 7; i >= 0; i--) ieee_addr = (ieee_addr << 8) | raw_ieee[i];
        }
        DEBUG_PRINTF(F("[ZIGBEE-GW] Device announced: short=0x%04X ieee=0x%016llX (queued)\n"),
                     short_addr, (unsigned long long)ieee_addr);

        // All bookkeeping (pre-register, Basic Cluster budget, DP query) runs
        // in the main loop; see gw_rx_drain(). Never touch shared state here.
        GwRxEvent ev = {};
        ev.kind = GW_RX_ANNOUNCE;
        ev.short_addr = short_addr;
        ev.endpoint = 1;
        ev.ieee = ieee_addr;
        gw_rx_push(ev);
    }

    // Override zbReadBasicCluster to handle Basic Cluster read responses ourselves.
    // The base class implementation calls xSemaphoreGive(lock), but 'lock' may be
    // uninitialised due to an Arduino Zigbee library bug (ZigbeeEP constructor
    // checks 'if (!lock)' before ever assigning it, so heap-poisoned memory
    // 0xfefefefe is treated as valid → crash in xQueueGenericSend).
    // Note: This alternative callback path may not include address  info,
    // so device addition happens primarily via zbAttributeRead.
    void zbReadBasicCluster(const esp_zb_zcl_attribute_t *attribute) override {
        if (!attribute) return;
        // DEBUG_PRINTF(F("[ZIGBEE-GW] Basic Cluster attr 0x%04X received via zbReadBasicCluster\n"),
                     // attribute->id);
        // If a Basic Cluster read is currently pending we know which IEEE was
        // targeted — register the device (or refresh its last_rx) so devices
        // that only respond via this callback (no src address info) still show
        // up.  This used to be the "GX02 invisible after join" path.
        if (gw_read_pending && gw_read_pending_cluster == ZB_ZCL_CLUSTER_ID_BASIC &&
            gw_read_pending_ieee != 0) {
            esp_zb_ieee_addr_t ieee_le;
            for (int i = 0; i < 8; i++) ieee_le[i] = (uint8_t)(gw_read_pending_ieee >> (i * 8));
            esp_zb_lock_acquire(portMAX_DELAY);
            uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
            esp_zb_lock_release();
            if (short_addr == 0xFFFF || short_addr == 0xFFFE) short_addr = 0;
            gw_rx_push_basic(gw_read_pending_ieee, short_addr, 1, attribute);
        }
        // Defer releasing the read slot to the next loop tick. A batch Basic
        // read arrives as several back-to-back zbReadBasicCluster() calls in the
        // same response frame; clearing gw_read_pending here would drop every
        // attribute after the first (including manufacturer/model). All batch
        // attributes are processed synchronously before the loop runs again.
        if (gw_read_pending && gw_read_pending_cluster == ZB_ZCL_CLUSTER_ID_BASIC) {
            gw_read_clear_after_basic = true;
        }
    }

    virtual void zbAttributeRead(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute,
                                  uint8_t src_endpoint, esp_zb_zcl_addr_t src_address) override {
        if (!attribute) {
            // DEBUG_PRINTLN(F("[ZIGBEE-GW] zbAttributeRead called with NULL attribute!"));
            return;
        }

        gw_read_pending = false;

        // Resolve IEEE from short address (stack call, Zigbee context only).
        uint64_t ieee_addr = 0;
        esp_zb_ieee_addr_t raw_ieee;
        if (esp_zb_ieee_address_by_short(src_address.u.short_addr, raw_ieee) == ESP_OK) {
            ieee_addr = gw_ieee_from_raw(raw_ieee);
        }

        // Basic Cluster responses (version, manufacturer, model strings): copy
        // the attribute payload; the discovered-device record is updated in the
        // main loop.
        if (cluster_id == ZB_ZCL_CLUSTER_ID_BASIC &&
            (attribute->id == ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID ||
             attribute->id == ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID ||
             attribute->id == ZB_ZCL_ATTR_BASIC_HW_VERSION_ID ||
             attribute->id == ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID ||
             attribute->id == ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID ||
             attribute->id == ZB_ZCL_ATTR_BASIC_DATE_CODE_ID ||
             attribute->id == ZB_ZCL_ATTR_BASIC_SW_BUILD_ID)) {
            gw_rx_push_basic(ieee_addr, src_address.u.short_addr, src_endpoint, attribute);
            return; // Don't treat the string as a sensor value
        }

        GwRxEvent ev = {};
        ev.kind = GW_RX_ATTR;
        ev.short_addr = src_address.u.short_addr;
        ev.endpoint = src_endpoint;
        ev.ieee = ieee_addr;
        ev.cluster_id = cluster_id;
        ev.attr_id = attribute->id;
        ev.value = extractAttributeValue(attribute);
        ev.lqi = 0;
        gw_rx_push(ev);
    }

private:
    int32_t extractAttributeValue(const esp_zb_zcl_attribute_t *attr) {
        return zigbee_extract_attribute_value(attr);
    }
};

// ========== Helper: update sensor from cached report ==========

static void gw_updateSensorFromReport(ZigbeeSensor* zb_sensor, const ZigbeeAttributeReport& report, bool solicited) {
    if (!zb_sensor) return;

    // ── Battery report short-circuit ─────────────────────────────────────
    // Battery reports (cluster 0x0001/attr 0x0021 or Tuya DP 15) carry a
    // percentage that must ONLY update `last_battery` — never the sensor's
    // `last_data`, otherwise the battery percentage (50/100/…) is logged as
    // the soil-moisture / temperature / etc. value.  This guard is
    // defense-in-depth: even when the dispatch loop or auto-correct routes
    // a battery report to a non-battery sensor by mistake, the measurement
    // payload is preserved.
    {
        bool is_tuya_report = (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0;
        uint16_t raw_attr = zigbee_report_attr_id(report.attr_id);
        bool is_battery_report =
            (report.cluster_id == ZB_ZCL_CLUSTER_ID_POWER_CONFIG && raw_attr == 0x0021) ||
            (is_tuya_report && report.cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC &&
             (zb_sensor->tuya_dp_battery >= 0 && raw_attr == (uint16_t)zb_sensor->tuya_dp_battery));
        if (is_battery_report) {
            uint32_t batt_pct = zigbee_battery_percent_from_report(is_tuya_report, raw_attr, tuya_report_type(report.attr_id), zb_sensor->tuya_dp_battery, report.value);
            if (batt_pct != ZB_BATTERY_UNKNOWN) zb_sensor->last_battery = batt_pct;
            zb_sensor->last_lqi     = report.lqi;
            // Intentionally do NOT touch last_data / last_native_data /
            // flags.data_ok / last_read / last / comm_mode — those belong
            // to the actual measurement channel, not the battery channel.
            DEBUG_PRINTF(F("[ZIGBEE-GW] Sensor '%s' battery update: %u%% (no data overwrite)\n"),
                         zb_sensor->getName(), (unsigned)batt_pct);
            return;
        }
    }

    zb_sensor->last_native_data = report.value;
    double converted_value = (double)report.value;

    // Check if this report came from the Tuya DP parser (values already in natural units)
    bool is_tuya = (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0;
    
    if (is_tuya) {
        // Tuya DP values need conversion to natural units
        if (report.cluster_id == ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
            converted_value = report.value / 10.0;  // Tuya sends tenths of °C (e.g. 227 = 22.7°C)
        } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT) {
            converted_value = report.value / 10.0;  // Tuya sends tenths of %RH
        } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_SOIL_MOISTURE) {
            // Tuya soil moisture is already raw % (0-100)
            converted_value = report.value;
        } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC) {
            uint16_t dp = zigbee_report_attr_id(report.attr_id);
            if (zb_sensor->tuya_dp_value >= 0 && dp == (uint16_t)zb_sensor->tuya_dp_value) {
                converted_value = report.value;
            }
        }
    } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_SOIL_MOISTURE && report.attr_id == 0x0000) {
        converted_value = report.value / 100.0;
    } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT && report.attr_id == 0x0000) {
        converted_value = report.value / 100.0;
    } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT && report.attr_id == 0x0000) {
        converted_value = report.value / 100.0;
    } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT && report.attr_id == 0x0000) {
        converted_value = report.value / 10.0;
    } else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT && report.attr_id == 0x0000) {
        if (report.value > 0 && report.value <= 65534) {
            converted_value = pow(10.0, (report.value - 1.0) / 10000.0);
        } else {
            converted_value = 0.0;
        }
    }
    // NOTE: POWER_CONFIG/0x0021 (battery) is handled by the early-return
    // guard at the top of this function — it must never reach the
    // measurement-update path below.

    converted_value -= (double)zb_sensor->offset_mv / 1000.0;
    if (zb_sensor->factor && zb_sensor->divider)
        converted_value *= (double)zb_sensor->factor / (double)zb_sensor->divider;
    else if (zb_sensor->divider)
        converted_value /= (double)zb_sensor->divider;
    else if (zb_sensor->factor)
        converted_value *= (double)zb_sensor->factor;
    converted_value += zb_sensor->offset2 / 100.0;
    
    zb_sensor->last_data = converted_value;
    zb_sensor->last_lqi = report.lqi;
    zb_sensor->flags.data_ok = true;
    zb_sensor->repeat_read = 1;
    zb_sensor->last_read = os.now_tz();
    zb_sensor->last = zb_sensor->last_read;
    zb_sensor->last_report_at_ms = millis();

    // Update communication mode based on whether this report was solicited
    // (response to our ZCL Read Attributes) or unsolicited (device-pushed report).
    // Battery reports already returned above, so every report reaching here
    // belongs to the measurement channel.
    {
        ZbCommMode new_mode = solicited ? ZB_COMM_ACTIVE : ZB_COMM_REPORT;
        if (zb_sensor->comm_mode == ZB_COMM_UNKNOWN ||
            (new_mode == ZB_COMM_REPORT && zb_sensor->comm_mode == ZB_COMM_ACTIVE)) {
            if (zb_sensor->comm_mode != new_mode) {
                zb_sensor->comm_mode = new_mode;
                gw_comm_mode_changed = true;
                DEBUG_PRINTF(F("[ZIGBEE-GW] '%s' comm_mode → %s\n"), zb_sensor->getName(),
                             new_mode == ZB_COMM_REPORT ? "REPORT" : "ACTIVE");
            }
        }
    }

    DEBUG_PRINTF(F("[ZIGBEE-GW] Sensor updated: cluster=0x%04X raw=%ld conv=%.2f factor=%d div=%d offset=%d\n"),
                report.cluster_id, report.value, converted_value, zb_sensor->factor, zb_sensor->divider, zb_sensor->offset_mv);
}

// ========== NVRAM erase ==========

/**
 * @brief Send a ZCL Read Attributes request for Basic Cluster to a remote device (GW mode)
 * Reads ManufacturerName (0x0004) and ModelIdentifier (0x0005) in one request.
 */
static bool gw_query_basic_cluster_internal(uint16_t short_addr, uint8_t endpoint) {
    if (!gw_zigbee_initialized || !gw_reportReceiver) return false;
    if (!Zigbee.started() || !Zigbee.connected()) return false;
    if (short_addr == 0xFFFF || short_addr == 0xFFFE) return false;
    
    // DEBUG_PRINTF(F("[ZIGBEE-GW] Querying Basic Cluster: short=0x%04X ep=%d\n"),
                 // short_addr, endpoint);
    
    esp_zb_zcl_read_attr_cmd_t read_req;
    memset(&read_req, 0, sizeof(read_req));
    read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    read_req.zcl_basic_cmd.dst_endpoint = endpoint;
    read_req.zcl_basic_cmd.src_endpoint = 10;  // our endpoint
    read_req.clusterID = ZB_ZCL_CLUSTER_ID_BASIC;
    read_req.attr_number = sizeof(s_gw_basic_query_attrs) / sizeof(s_gw_basic_query_attrs[0]);
    read_req.attr_field = s_gw_basic_query_attrs;
    
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    esp_zb_lock_release();
    
    DEBUG_PRINTF(F("[ZIGBEE-GW] Basic Cluster read request sent: short=0x%04X ep=%d\n"), short_addr, endpoint);
    return true;
}

// Alias for direct Basic Cluster query (no queueing—direct send)
static bool gw_query_basic_cluster(uint16_t short_addr, uint8_t endpoint) {
    return gw_query_basic_cluster_internal(short_addr, endpoint);
}

void sensor_zigbee_gw_query_basic_cluster(uint16_t short_addr, uint8_t endpoint) {
    // Directly query the device (no queueing)
    // DEBUG_PRINTF(F("[ZIGBEE-GW] API: Querying Basic Cluster for device 0x%04X\n"), short_addr);
    gw_query_basic_cluster(short_addr, endpoint);
}

bool sensor_zigbee_gw_query_basic_cluster_by_ieee(uint64_t device_ieee, uint8_t endpoint) {
    if (!gw_zigbee_initialized || !gw_reportReceiver) return false;
    if (!Zigbee.started() || !Zigbee.connected()) return false;
    if (device_ieee == 0) return false;

    // Avoid piling requests for the same sleeping/unresponsive device.
    if (gw_read_pending && gw_read_pending_ieee == device_ieee) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic query deferred: read pending for ieee=%016llX cluster=0x%04X\n"),
                     (unsigned long long)device_ieee, gw_read_pending_cluster);
        return false;
    }
    if (gw_read_timeout_ieee == device_ieee &&
        (long)(gw_read_block_until_ms - millis()) > 0) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic query cooldown: ieee=%016llX wait=%lums\n"),
                     (unsigned long long)device_ieee,
                     (unsigned long)(gw_read_block_until_ms - millis()));
        return false;
    }

    if (!gw_allow_oneshot_for_device(device_ieee, "basic_query")) {
        return false;
    }

    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }

    uint16_t short_addr = gw_get_short_addr(device_ieee);

    if (!gw_is_device_access_allowed(device_ieee, short_addr, false, 5000UL)) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic query blocked: rate-limited/cooldown (5s) for ieee=%016llX\n"),
                     (unsigned long long)device_ieee);
        return false;
    }

    esp_zb_zcl_read_attr_cmd_t read_req;
    memset(&read_req, 0, sizeof(read_req));

    if (short_addr != 0xFFFF && short_addr != 0xFFFE) {
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
        gw_add_responsive_device(short_addr, device_ieee, endpoint, false);
    } else {
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
        memcpy(read_req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_le, sizeof(ieee_le));
    }

    read_req.zcl_basic_cmd.dst_endpoint = endpoint;
    read_req.zcl_basic_cmd.src_endpoint = 10;
    read_req.clusterID = ZB_ZCL_CLUSTER_ID_BASIC;
    read_req.attr_number = sizeof(s_gw_basic_query_attrs) / sizeof(s_gw_basic_query_attrs[0]);
    read_req.attr_field = s_gw_basic_query_attrs;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    gw_read_pending = true;
    gw_read_time = millis();
    gw_read_pending_ieee = device_ieee;
    gw_read_pending_cluster = ZB_ZCL_CLUSTER_ID_BASIC;
    // Mark this as a multi-attribute (batch) Basic Cluster read.  The read
    // timeout handler uses attr_id == 0x0000 as the sentinel to fall back to
    // individual 0x0004 (manufacturer) / 0x0005 (model) reads.  Without this,
    // gw_read_attr_id kept its stale value (e.g. 0xFFFF from a preceding Tuya
    // DP query for _TZE284_/_TZE204_/_TZE200_ devices), so the batch timeout
    // never fell back to per-attribute reads.  Strict Tuya devices (e.g. GIEX
    // GX03 "_TZE284_8zizsafo") reject the 7-attribute batch read, so their
    // manufacturer/model strings were never obtained and the device stayed
    // unrecognized.
    gw_read_attr_id = 0x0000;
    esp_zb_lock_release();

    gw_record_device_access(device_ieee, short_addr, false);

    DEBUG_PRINTF(F("[ZIGBEE-GW] Basic query sent: ieee=%016llX short=0x%04X ep=%d\n"),
                 (unsigned long long)device_ieee, short_addr, endpoint);
    return true;
}

bool sensor_zigbee_gw_query_basic_cluster_by_ieee_attr(uint64_t device_ieee, uint8_t endpoint, uint16_t attr_id) {
    if (!gw_zigbee_initialized || !gw_reportReceiver) return false;
    if (!Zigbee.started() || !Zigbee.connected()) return false;
    if (device_ieee == 0) return false;

    // Avoid piling requests for the same sleeping/unresponsive device.
    if (gw_read_pending && gw_read_pending_ieee == device_ieee && gw_read_attr_id == attr_id) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic attribute query 0x%04X deferred: read pending for ieee=%016llX\n"),
                     attr_id, (unsigned long long)device_ieee);
        return false;
    }
    if (gw_read_timeout_ieee == device_ieee &&
        (long)(gw_read_block_until_ms - millis()) > 0) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic attribute query cooldown: ieee=%016llX wait=%lums\n"),
                     (unsigned long long)device_ieee,
                     (unsigned long)(gw_read_block_until_ms - millis()));
        return false;
    }

    if (!gw_allow_oneshot_for_device(device_ieee, "basic_query_attr")) {
        return false;
    }

    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }

    uint16_t short_addr = gw_get_short_addr(device_ieee);

    if (!gw_is_device_access_allowed(device_ieee, short_addr, false, 5000UL)) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic single-attr query blocked: rate-limited/cooldown (5s) for ieee=%016llX\n"),
                     (unsigned long long)device_ieee);
        return false;
    }

    esp_zb_zcl_read_attr_cmd_t read_req;
    memset(&read_req, 0, sizeof(read_req));

    if (short_addr != 0xFFFF && short_addr != 0xFFFE) {
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
        gw_add_responsive_device(short_addr, device_ieee, endpoint, false);
    } else {
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
        memcpy(read_req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_le, sizeof(ieee_le));
    }

    // Allocate single attribute on stack (must persist during sync function call)
    static uint16_t static_field;
    static_field = attr_id;

    read_req.zcl_basic_cmd.dst_endpoint = endpoint;
    read_req.zcl_basic_cmd.src_endpoint = 10;
    read_req.clusterID = ZB_ZCL_CLUSTER_ID_BASIC;
    read_req.attr_number = 1;
    read_req.attr_field = &static_field;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&read_req);
    gw_read_pending = true;
    gw_read_time = millis();
    gw_read_pending_ieee = device_ieee;
    gw_read_pending_cluster = ZB_ZCL_CLUSTER_ID_BASIC;
    gw_read_attr_id = attr_id;
    esp_zb_lock_release();

    gw_record_device_access(device_ieee, short_addr, false);

    DEBUG_PRINTF(F("[ZIGBEE-GW] Basic single-attr query sent: ieee=%016llX short=0x%04X ep=%d attr=0x%04X\n"),
                 (unsigned long long)device_ieee, short_addr, endpoint, attr_id);
    return true;
}

bool sensor_zigbee_gw_read_attribute(uint64_t device_ieee, uint8_t endpoint,
                                     uint16_t cluster_id, uint16_t attribute_id) {
    if (!gw_zigbee_initialized || !gw_reportReceiver) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Read FAILED: GW not initialized"));
        return false;
    }
    if (!Zigbee.started() || !Zigbee.connected()) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Read FAILED: ZB not started=%d or connected=%d\n"),
                    Zigbee.started() ? 1 : 0, Zigbee.connected() ? 1 : 0);
        return false;
    }
    if (device_ieee == 0) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Read FAILED: device_ieee is 0"));
        return false;
    }

    // Tuya cluster 0xEF00 does not support standard ZCL attribute reading!
    // Directly bypass to a non-blocking Tuya DP query command.
    if (cluster_id == 0xEF00) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Bypassing traditional ZCL read to Tuya DP query for ieee=%016llX ep=%u\n"),
                     (unsigned long long)device_ieee, endpoint);
        return sensor_zigbee_gw_request_dp_query(device_ieee, endpoint);
    }

    // Serialize active reads per device. Concurrent reads on different clusters
    // of the same sleepy end device can fill stack pending queues and trip
    // ZB scheduler assertions on ESP32-C5.
    if (gw_read_pending && gw_read_pending_ieee == device_ieee) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Read BLOCKED: request already pending (%.0fs ago). ieee=%016llX pending_cluster=0x%04X cluster=0x%04X attr=0x%04X\n"),
                    (millis() - gw_read_time) / 1000.0,
                    (unsigned long long)device_ieee,
                    gw_read_pending_cluster,
                    cluster_id,
                    attribute_id);
        return false;
    }

    if (gw_read_timeout_ieee == device_ieee &&
        (long)(gw_read_block_until_ms - millis()) > 0) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Read COOLDOWN: ieee=%016llX cluster=0x%04X attr=0x%04X wait=%lums\n"),
                    (unsigned long long)device_ieee,
                    cluster_id,
                    attribute_id,
                    (unsigned long)(gw_read_block_until_ms - millis()));
        return false;
    }

    if (!gw_allow_oneshot_for_device(device_ieee, "read_attr")) {
        return false;
    }

    esp_zb_ieee_addr_t ieee_le = {0};
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }

    esp_zb_lock_acquire(portMAX_DELAY);
    uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
    esp_zb_lock_release();

    if (!gw_is_device_access_allowed(device_ieee, short_addr, false, 5000UL)) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Active read blocked: rate-limited/cooldown (5s) for ieee=%016llX cluster=0x%04X\n"),
                     (unsigned long long)device_ieee, cluster_id);
        return false;
    }

    gw_read_attr_id = attribute_id;

    esp_zb_zcl_read_attr_cmd_t read_req;
    memset(&read_req, 0, sizeof(read_req));

    esp_zb_lock_acquire(portMAX_DELAY);
    if (short_addr != 0xFFFF && short_addr != 0xFFFE) {
        // Short address known: use 16-bit unicast (lower overhead)
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        read_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
    } else {
        // Short address not in coordinator table (e.g. after reboot).
        // Use 64-bit IEEE address mode — the ZigBee stack resolves
        // the route automatically if the device is still on the network.
        // DEBUG_PRINTF(F("[ZIGBEE-GW] Short addr unknown for ieee=%016llX (table may have been cleared after reboot). Using 64-bit IEEE mode.\n"),
                     // (unsigned long long)device_ieee);
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
        memcpy(read_req.zcl_basic_cmd.dst_addr_u.addr_long, ieee_le, sizeof(ieee_le));
    }

    read_req.zcl_basic_cmd.dst_endpoint = endpoint;
    read_req.zcl_basic_cmd.src_endpoint = 10;
    read_req.clusterID = cluster_id;
    read_req.attr_number = 1;
    read_req.attr_field = &gw_read_attr_id;

    esp_zb_zcl_read_attr_cmd_req(&read_req);
    gw_read_pending = true;
    gw_read_time = millis();
    gw_read_pending_ieee = device_ieee;
    gw_read_pending_cluster = cluster_id;
    esp_zb_lock_release();

    gw_record_device_access(device_ieee, short_addr, false);

    DEBUG_PRINTF(F("[ZIGBEE-GW] ✓ Active read SENT: ieee=%016llX short=0x%04X ep=%d cluster=0x%04X attr=0x%04X\n"),
                 (unsigned long long)device_ieee, short_addr, endpoint, cluster_id, attribute_id);
    return true;
}

static bool gw_erase_zigbee_nvram() {
    const esp_partition_t* zb_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_FAT,
        "zb_storage");
    if (!zb_partition) return false;
    esp_err_t err = esp_partition_erase_range(zb_partition, 0, zb_partition->size);
    return err == ESP_OK;
}

// ========== Public Gateway API ==========

// Schedule Configure Reporting for every SENSOR_ZIGBEE that has a known IEEE address.
// Called once when the Zigbee coordinator network forms (or re-forms after reboot).
static void gw_schedule_configure_reporting_all(unsigned long initial_delay_ms = 5000) {
    SensorIterator it = sensors_iterate_begin();
    SensorBase* sensor;
    int count = 0;
    unsigned long now = millis();
    unsigned long delay_ms = initial_delay_ms;

    while ((sensor = sensors_iterate_next(it)) != NULL) {
        if (!sensor || sensor->type != SENSOR_ZIGBEE) continue;
        ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(sensor);
        if (zb->device_ieee == 0) continue;
        if (!gw_cluster_supports_config_reporting(zb->cluster_id, zb->attribute_id)) continue;

        uint ri = zb->read_interval ? zb->read_interval : 60;
        uint16_t max_interval = (ri >= 15 && ri <= 3600) ? (uint16_t)ri : 120;

        bool found = false;
        for (const auto& ex : gw_config_report_queue) {
            if (ex.ieee_addr == zb->device_ieee &&
                ex.cluster_id == zb->cluster_id &&
                ex.attr_id == zb->attribute_id) {
                found = true;
                break;
            }
        }
        if (found) continue;

        GwConfigReportRequest req;
        req.ieee_addr      = zb->device_ieee;
        req.endpoint       = zb->endpoint;
        req.cluster_id     = zb->cluster_id;
        req.attr_id        = zb->attribute_id;
        req.min_interval   = 10;
        req.max_interval   = max_interval;
        req.scheduled_time = now + delay_ms;
        gw_config_report_queue.push_back(req);
        // Store configured interval in sensor for predictive boost
        zb->report_interval_s = max_interval;
        delay_ms += 1000;
        count++;

        // No battery setup here: right after (re)connect the sleepy devices are
        // not polling, the attempt would be lost and would burn the hourly
        // battery budget.  gw_schedule_configure_reporting_for_ieee() queues it
        // on the device's first frame instead (see gw_add_responsive_device).
    }
    if (count > 0) {
        // DEBUG_PRINTF("[ZIGBEE-GW] Scheduled configure reporting for %d sensor(s)\n", count);
    }
}

void sensor_zigbee_gw_factory_reset() {
    gw_erase_zigbee_nvram();
    zigbee_nvram_invalidate('G');
    gw_clear_discovered_devices_cache(true);
}

void sensor_zigbee_gw_stop() {
    // Zigbee Coordinator stays running permanently (non-Matter mode).
    // The Arduino Zigbee library does NOT support stop+restart anyway.
    // This function is now a no-op.
    if (!gw_zigbee_initialized) return;
    // DEBUG_PRINTLN(F("[ZIGBEE-GW] stop() called — Zigbee stays active (permanent mode)"));
}

#if defined(ESP32) && defined(BOARD_HAS_PSRAM)
extern "C" char _ext_ram_bss_start[];
extern "C" char _ext_ram_bss_end[];
#endif

void sensor_zigbee_gw_start() {
    // The station control tables live in PSRAM .bss.  Zero them explicitly:
    // on this build uninitialised entries were observed (active=1 with garbage
    // fields) and would raise "switch failed" alerts for non-Zigbee stations.
    memset(zb_station_ctl, 0, sizeof(zb_station_ctl));
    memset(zb_station_switch_error, 0, sizeof(zb_station_switch_error));
    memset(zb_station_switch_waiting, 0, sizeof(zb_station_switch_waiting));
    memset(zb_station_physical_on, 0, sizeof(zb_station_physical_on));
    memset(gw_zcl_timed_off_unsupported, 0, sizeof(gw_zcl_timed_off_unsupported));
#if defined(ESP32) && defined(BOARD_HAS_PSRAM)
    DEBUG_PRINTF(F("[ZIGBEE-GW] ext_ram_bss=%p..%p zb_station_ctl=%p (%u B) %s\n"),
                 (void*)_ext_ram_bss_start, (void*)_ext_ram_bss_end, (void*)zb_station_ctl,
                 (unsigned)sizeof(zb_station_ctl),
                 ((char*)zb_station_ctl >= _ext_ram_bss_start && (char*)zb_station_ctl < _ext_ram_bss_end) ? "inside" : "OUTSIDE ext_ram_bss!");
#endif
    // NOT POSSIBLE / NOT SENSIBLE over WiFi: a Zigbee Gateway (coordinator) must
    // keep its 802.15.4 receiver on continuously, but the ESP32-C5 shares ONE
    // 2.4 GHz radio between WiFi and 802.15.4. With no Ethernet the coordinator
    // starves the WiFi STA and WiFi disconnects permanently. Verified on
    // hardware that SW coexistence + WiFi modem-sleep + disabling BLE do NOT fix
    // it. Therefore the gateway is only started with Ethernet; over WiFi Zigbee
    // is left completely disabled (use a Zigbee CLIENT/end-device for WiFi, or
    // wire Ethernet for a gateway). See docs/coexistence notes.
    if (!useEth) {
        static bool wifi_gw_warning_shown = false;
        if (!wifi_gw_warning_shown) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Gateway requires Ethernet — Zigbee DISABLED over WiFi (2.4 GHz WiFi/802.15.4 coexistence is not viable)"));
            wifi_gw_warning_shown = true;
        }
        return;
    }
    // DEBUG_PRINTLN(F("[ZIGBEE-GW] sensor_zigbee_gw_start() called"));
    // DEBUG_PRINTF("[ZIGBEE-GW] ieee802154 mode: %d (%s)\n",
                // (int)ieee802154_get_mode(), ieee802154_mode_name(ieee802154_get_mode()));

    if (!ieee802154_is_zigbee_gw()) {
        static bool mode_warning_shown = false;
        if (!mode_warning_shown) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Not in ZIGBEE_GATEWAY mode - Zigbee GW disabled"));
            mode_warning_shown = true;
        }
        return;
    }

    if (gw_zigbee_initialized) {
        // DEBUG_PRINTLN(F("[ZIGBEE-GW] Already initialized, skipping"));
        return;
    }

    // A previous Zigbee.begin() failed (typically internal RAM exhaustion).
    // Do NOT retry: the report-receiver endpoint was already registered with
    // ZigbeeCore, so a second begin() would touch stale state and panic
    // (Load access fault). Degrade gracefully — the gateway stays disabled
    // until the next reboot, when more internal RAM may be available.
    if (gw_zigbee_begin_failed) {
        static bool begin_failed_warning_shown = false;
        if (!begin_failed_warning_shown) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Zigbee.begin() previously failed — gateway disabled until reboot (retry is unsafe)"));
            begin_failed_warning_shown = true;
        }
        return;
    }

    // Zigbee stays active once started (no stop/restart toggling).

    // Load the Gateway dataset into zb_storage (swaps out any Client snapshot).
    // Must run before Zigbee.begin() so ZBOSS reads the correct role's NVRAM.
    zigbee_nvram_prepare_for_role('G');

    if (gw_zigbee_needs_nvram_reset) {
        gw_zigbee_needs_nvram_reset = false;
        gw_erase_zigbee_nvram();
        zigbee_nvram_invalidate('G');
        gw_clear_discovered_devices_cache(true);
    }

    if (!gw_discovered_devices_loaded) {
        gw_load_discovered_devices();
    }

    esp_zb_radio_config_t radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE };
    esp_zb_host_config_t host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE };
    Zigbee.setHostConfig(host_config);
    Zigbee.setRadioConfig(radio_config);

    if (WiFi.getMode() != WIFI_MODE_NULL) {
        // DEBUG_PRINTLN(F("[ZIGBEE-GW] WiFi active - coexistence base already configured"));
        // NOTE: Per-packet PTI must be set AFTER Zigbee.begin() — ieee802154_mac_init()
        // inside esp_zb_start() resets all PTI values to defaults.
    } else {
        // DEBUG_PRINTLN(F("[ZIGBEE-GW] No WiFi - Zigbee has full radio access (Ethernet mode)"));
    }

    gw_reportReceiver = new GwZigbeeReportReceiver(10);
    if (!gw_reportReceiver) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] ERROR: Failed to allocate GwZigbeeReportReceiver!"));
        return;
    }
    Zigbee.addEndpoint(gw_reportReceiver);
    gw_reportReceiver->setManufacturerAndModel("OpenSprinkler", "ZigbeeGateway");

    // Optionally restrict Zigbee to a specific channel to reduce
    // radio contention with WiFi.  When ZIGBEE_COEX_CHANNEL_MASK is
    // not defined, the Zigbee stack uses the default all-channel scan.
#ifdef ZIGBEE_COEX_CHANNEL_MASK
    Zigbee.setPrimaryChannelMask(ZIGBEE_COEX_CHANNEL_MASK);
    // DEBUG_PRINTF("[ZIGBEE-GW] Primary channel mask set to 0x%08X\n",
                 // (unsigned)ZIGBEE_COEX_CHANNEL_MASK);
#else
    {
        uint8_t conf_chan = sensor_zigbee_gw_get_configured_channel();
        if (conf_chan >= 11 && conf_chan <= 26) {
            uint32_t mask = (1UL << conf_chan);
            Zigbee.setPrimaryChannelMask(mask);
            DEBUG_PRINTF("[ZIGBEE-GW] Primary channel mask set to single channel %d (mask: 0x%08X)\n",
                         (int)conf_chan, (unsigned)mask);
        } else {
            // Default: all channels 11-26
            Zigbee.setPrimaryChannelMask(0x07FFF800);
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Using default channel mask (all channels 11-26)"));
        }
    }
#endif

    // [ZBMEM] Instrumentation: measure exact internal vs PSRAM split across ZBOSS
    // init to determine what (if anything) is movable to PSRAM.
    DEBUG_PRINTF("[ZBMEM] before esp_zb config: internal=%u B (largest=%u) | psram=%u B (largest=%u)\n",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

    // Configure ZBOSS memory BEFORE esp_zb_init() (which is called inside Zigbee.begin()).
    // The Coordinator/Router role allocates neighbor, routing, and source-route tables
    // whose sizes are derived from overall_network_size (default=64).  That is too large
    // for the available SRAM on ESP32-C5, triggering:
    //   "No memory for NWK src route table"  /  zb_memconfig.c:271
    //
    // IMPORTANT: ZBOSS uses calloc() for its tables.  With CONFIG_SPIRAM_USE_MALLOC=y
    // the allocator CAN fall back to PSRAM, but the async Zigbee_main task competes
    // with the main loop for internal RAM.  Keeping tables small prevents LittleFS
    // "Unable to allocate FD" failures that occur when internal RAM is exhausted.
    //
    // Size 16 supports up to 16 direct Zigbee children — enough for OpenSprinkler.
    esp_zb_overall_network_size_set(16);
    // Buffers and scheduler queue are expanded (80 each) to fully avoid under-heap 
    // ZBOSS queue exhaustion panics (like zb_bufpool_mult_storage.c:105 assertions).
    esp_zb_io_buffer_size_set(80);
    esp_zb_scheduler_queue_size_set(80);
    // Zigbee 3.0: require joining devices to replace the well-known
    // "ZigBeeAlliance09" trust-center link key with a unique one. Devices that
    // never complete the TCLK update are removed by the stack after the BDB
    // timeout instead of staying on the network with the global key.
    // Build with -D ZIGBEE_ALLOW_WELLKNOWN_TCLK to restore the legacy behaviour
    // for fleets that trigger child-auth asserts on ESP32-C5.
#if defined(ZIGBEE_ALLOW_WELLKNOWN_TCLK)
    esp_zb_secur_link_key_exchange_required_set(false);
#else
    esp_zb_secur_link_key_exchange_required_set(true);
#endif

    gw_rx_ensure_queue();
    DEBUG_PRINTLN(F("[ZIGBEE-GW] Starting as COORDINATOR..."));
    if (!Zigbee.begin(ZIGBEE_COORDINATOR)) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] ERROR: Zigbee.begin(COORDINATOR) FAILED!"));
        // Mark begin as permanently failed for this boot. Do NOT delete
        // gw_reportReceiver here: ZigbeeCore has already taken ownership of the
        // endpoint pointer via addEndpoint(); freeing it would leave a dangling
        // reference inside ZigbeeCore that panics on any later begin() retry.
        // Keeping the (small) object avoids the dangling pointer; the
        // gw_zigbee_begin_failed guard prevents any retry from this boot.
        gw_zigbee_begin_failed = true;
        return;
    }

    gw_zigbee_initialized = true;
    // [ZBMEM] After Zigbee.begin(): shows how much internal vs PSRAM the ZBOSS
    // stack + 802.15.4 driver + Zigbee_main task consumed.
    DEBUG_PRINTF("[ZBMEM] after Zigbee.begin: internal=%u B (largest=%u) | psram=%u B (largest=%u)\n",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    Zigbee.onGlobalDefaultResponse(gw_zigbee_default_response_cb);

    // Unicasts to sleepy end devices sit in the coordinator's indirect queue
    // until the child polls.  The 802.15.4 default (macTransactionPersistenceTime
    // = 7.68 s) is shorter than the poll period of many battery sensors (e.g.
    // Third Reality soil sensors do not poll after a report), so Bind /
    // ConfigureReporting / Read requests were dropped before the device ever
    // asked for them.  Keep them for GW_MAC_PERSISTENCE_S instead.
    {
        int8_t tx_before = 0, tx_after = 0;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_get_tx_power(&tx_before);
        if (GW_ZB_TX_POWER_DBM != 0) esp_zb_set_tx_power((int8_t)GW_ZB_TX_POWER_DBM);
        esp_zb_get_tx_power(&tx_after);
        esp_zb_lock_release();
        DEBUG_PRINTF(F("[ZIGBEE-GW] TX power: default %d dBm -> now %d dBm (requested %d)\n"),
                     (int)tx_before, (int)tx_after, (int)GW_ZB_TX_POWER_DBM);
    }
    {
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_err_t kerr = esp_zb_nwk_set_keepalive_mode(GW_KEEPALIVE_MODE);
        esp_err_t terr = esp_zb_nwk_set_ed_timeout(GW_ED_TIMEOUT);
        esp_zb_lock_release();
        DEBUG_PRINTF(F("[ZIGBEE-GW] Keepalive mode=%d (err=%d), default ED timeout=%d (err=%d)\n"),
                     (int)GW_KEEPALIVE_MODE, (int)kerr, (int)GW_ED_TIMEOUT, (int)terr);
    }
    if (GW_MAC_PERSISTENCE_S > 0) {
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_err_t perr = esp_zb_mac_set_transaction_persistence_time((uint32_t)GW_MAC_PERSISTENCE_S * 1000000UL);
        uint32_t pnow = esp_zb_mac_get_transaction_persistence_time();
        esp_zb_lock_release();
        DEBUG_PRINTF(F("[ZIGBEE-GW] MAC transaction persistence set to %u s (err=%d, readback=%lu us)\n"),
                     (unsigned)GW_MAC_PERSISTENCE_S, (int)perr, (unsigned long)pnow);
    }
    // DEBUG_PRINTLN(F("[ZIGBEE-GW] Zigbee Coordinator started successfully!"));
    // DEBUG_PRINTF("[ZIGBEE-GW] Zigbee.started()=%d, Zigbee.connected()=%d\n",
                // Zigbee.started() ? 1 : 0, Zigbee.connected() ? 1 : 0);

    // Register APS-layer indication handler for Tuya DP protocol (cluster 0xEF00).
    // This intercepts raw Tuya frames before ZCL processing and converts them
    // into standard report-cache entries for sensors like GIEX GX-04.
    esp_zb_aps_data_indication_handler_register(gw_tuya_aps_indication_handler);
    // APS confirm for every ZCL command we send (station switch verification).
    esp_zb_zcl_command_send_status_handler_register(gw_send_status_cb);
    // DEBUG_PRINTLN(F("[ZIGBEE-GW] Tuya APS indication handler registered"));

    // DEBUG_PRINTF(F("[ZIGBEE-GW] Heap AFTER init: internal free=%u, largest=%u, PSRAM free=%u, largest=%u\n"),
                 // heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 // heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                 // heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 // heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

    // Log channel and PAN ID for diagnostics
    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t zb_channel = esp_zb_get_current_channel();
    uint16_t zb_pan_id = esp_zb_get_pan_id();
    esp_zb_ieee_addr_t ext_pan_raw;
    esp_zb_get_extended_pan_id(ext_pan_raw);
    esp_zb_lock_release();
    uint64_t zb_ext_pan = 0;
    for (int i = 7; i >= 0; i--) zb_ext_pan = (zb_ext_pan << 8) | ext_pan_raw[i];
    // DEBUG_PRINTF(F("[ZIGBEE-GW] Network: channel=%d PAN=0x%04X extPAN=%08lX%08lX\n"),
                 // zb_channel, zb_pan_id,
                 // (unsigned long)(zb_ext_pan >> 32), (unsigned long)(zb_ext_pan & 0xFFFFFFFF));

    // Network stays closed after init. Use the HTTP API ("zj" command)
    // to open the network for joining when pairing new devices.
    // DEBUG_PRINTLN(F("[ZIGBEE-GW] Network closed — use API to open for joining"));

}

bool sensor_zigbee_gw_is_active() {
    return gw_zigbee_initialized;
}

bool sensor_zigbee_gw_ensure_started() {
    if (gw_zigbee_initialized) {
        return true;
    }

    // A prior Zigbee.begin() failed this boot — never retry (see note at the
    // gw_zigbee_begin_failed declaration). Return quietly to avoid log spam on
    // every sensor read; sensor_zigbee_gw_start() logs the one-shot warning.
    if (gw_zigbee_begin_failed) {
        return false;
    }

    // Never start Zigbee in SOFTAP mode (RF conflict with 802.15.4)
    wifi_mode_t wmode = WiFi.getMode();
    if (wmode == WIFI_MODE_AP) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Cannot start in SOFTAP mode"));
        return false;
    }

    // Allow Zigbee startup even while WiFi/Ethernet is reconnecting.
    // Discovery/pairing should not be blocked by temporary network reconnects.
    bool is_ethernet = (wmode == WIFI_MODE_NULL);
    if (!is_ethernet && WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] WiFi not connected yet - starting Zigbee anyway"));
    } else if (is_ethernet && !os.network_connected()) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Ethernet not connected yet - starting Zigbee anyway"));
    }

    DEBUG_PRINTLN(F("[ZIGBEE-GW] ensure_started: network ready, starting Zigbee GW..."));
    sensor_zigbee_gw_start();
    DEBUG_PRINTF("[ZIGBEE-GW] ensure_started result: initialized=%d\n", gw_zigbee_initialized ? 1 : 0);
    return gw_zigbee_initialized;
}

static bool is_zigbee_battery_report(const ZigbeeAttributeReport& report, uint64_t ieee, int16_t& out_battery_dp) {
    bool is_tuya_report = (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0;
    uint16_t raw_attr = zigbee_report_attr_id(report.attr_id);

    if (report.cluster_id == ZB_ZCL_CLUSTER_ID_POWER_CONFIG && raw_attr == 0x0021) {
        out_battery_dp = -1;
        return true;
    }

    if (is_tuya_report && report.cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC) {
        // 1. Check if any configured sensor for this IEEE has this DP as tuya_dp_battery
        SensorIterator it = sensors_iterate_begin();
        SensorBase* sensor;
        while ((sensor = sensors_iterate_next(it)) != NULL) {
            if (!sensor || sensor->type != SENSOR_ZIGBEE) continue;
            ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(sensor);
            if (zb->device_ieee == ieee && zb->tuya_dp_battery >= 0 && raw_attr == (uint16_t)zb->tuya_dp_battery) {
                out_battery_dp = zb->tuya_dp_battery;
                return true;
            }
        }

        // 2. Check if any registered logical device for this IEEE has this DP as tuya_dp_battery
        if (OpenSprinkler::zigbee_logical_devices_map) {
            char ieee_str[17];
            snprintf(ieee_str, sizeof(ieee_str), "%016llX", (unsigned long long)ieee);
            for (const auto& entry : *OpenSprinkler::zigbee_logical_devices_map) {
                const ZigBeeLogicalDevice& dev = entry.second.device;
                if (strncmp(dev.ieee, ieee_str, 16) == 0 && dev.is_tuya && dev.tuya_dp_battery >= 0 && raw_attr == (uint16_t)dev.tuya_dp_battery) {
                    out_battery_dp = dev.tuya_dp_battery;
                    return true;
                }
            }
        }

        // 3. Fallback for well-known Tuya battery DPs
        if (raw_attr == 14 || raw_attr == 15 || raw_attr == 59 || raw_attr == 108 || raw_attr == 115 || raw_attr == 18) {
            out_battery_dp = raw_attr;
            return true;
        }
    }

    return false;
}

void sensor_zigbee_gw_process_reports(uint64_t ieee_addr, uint8_t endpoint,
                                       uint16_t cluster_id, uint16_t attr_id,
                                       int32_t value, uint8_t lqi) {

    if (cluster_id != 0 || attr_id != 0) {
        gw_cache_attribute_report(ieee_addr, endpoint, cluster_id, attr_id, value, lqi);
    }
    
    // Log processing status if there are pending reports
    static unsigned long last_report_debug = 0;
    if (pending_report_count > 0 || (millis() - last_report_debug > 30000)) {
        last_report_debug = millis();
        // DEBUG_PRINTF(F("[ZIGBEE-GW] PROCESS: %d pending, now checking sensors...\n"), pending_report_count);
    }
    
    for (size_t i = 0; i < pending_report_count; i++) {
        ZigbeeAttributeReport& report = pending_reports[i];
        
        // Skip already-consumed or expired reports
        if (report.consumed || millis() - report.timestamp > REPORT_VALIDITY_MS) {
            continue;
        }
        
        // DEBUG_PRINTF(F("[ZIGBEE-GW] Report[%d]: ieee=%08lX%08lX ep=%d cluster=0x%04X attr=0x%04X val=%ld lqi=%d\n"),
                    // (int)i,
                    // (unsigned long)(report.ieee_addr >> 32), (unsigned long)(report.ieee_addr & 0xFFFFFFFF),
                    // report.endpoint, report.cluster_id, report.attr_id & ~TUYA_REPORT_FLAG_PRESCALED,
                    // report.value, report.lqi);
        
        // Determine if this report is a response to our own ZCL Read Attributes request
        // (solicited) or a device-initiated unsolicited attribute report.
        bool report_solicited = gw_read_pending &&
                                gw_read_pending_ieee    == report.ieee_addr &&
                                gw_read_pending_cluster == report.cluster_id;

        // ─── Short-circuit battery report processing ───
        int16_t configured_battery_dp = -1;
        if (is_zigbee_battery_report(report, report.ieee_addr, configured_battery_dp)) {
            bool is_tuya_report = (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0;
            uint16_t raw_attr = zigbee_report_attr_id(report.attr_id);
            uint32_t batt_pct = zigbee_battery_percent_from_report(is_tuya_report, raw_attr, tuya_report_type(report.attr_id), configured_battery_dp, report.value);
            if (batt_pct == ZB_BATTERY_UNKNOWN) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] Battery report ignored (raw=%ld = unknown) ieee=%016llX\n"),
                             (long)report.value, (unsigned long long)report.ieee_addr);
                report.consumed = true;
                continue;
            }
            
            // Try to find the device in gw_discovered_devices and update its battery level and LQI
            ZigbeeDeviceInfo* dev = gw_find_discovered_device(report.ieee_addr);
            if (dev) {
                bool changed = false;
                if (dev->battery != batt_pct) {
                    dev->battery = batt_pct;
                    changed = true;
                }
                if (report.lqi != 0 && dev->lqi != report.lqi) {
                    dev->lqi = report.lqi;
                    changed = true;
                }
                if (changed) gw_mark_discovered_devices_dirty();
            }

            SensorIterator it_batt = sensors_iterate_begin();
            SensorBase* s_batt;
            bool updated_any = false;
            while ((s_batt = sensors_iterate_next(it_batt)) != NULL) {
                if (!s_batt || s_batt->type != SENSOR_ZIGBEE) continue;
                ZigbeeSensor* zb_s = static_cast<ZigbeeSensor*>(s_batt);
                if (zb_s->device_ieee == report.ieee_addr) {
                    zb_s->last_battery = batt_pct;
                    zb_s->last_lqi = report.lqi;
                    updated_any = true;
                    DEBUG_PRINTF(F("[ZIGBEE-GW] Short-circuit battery update for '%s': %u%%\n"),
                                 zb_s->getName(), (unsigned)batt_pct);
                }
            }
            // Always consume the battery report so it doesn't trigger "✗ NO MATCH" warnings
            report.consumed = true;
            continue;
        }

        SensorIterator it = sensors_iterate_begin();
        SensorBase* sensor;
        bool found = false;
        int checked_count = 0;
        
        while ((sensor = sensors_iterate_next(it)) != NULL) {
            if (!sensor || sensor->type != SENSOR_ZIGBEE) continue;
            checked_count++;
            ZigbeeSensor* zb_sensor = static_cast<ZigbeeSensor*>(sensor);
            
            uint16_t report_attr_unmasked = zigbee_report_attr_id(report.attr_id);
            bool cluster_match = (zb_sensor->cluster_id == report.cluster_id);
            bool attr_match = (zb_sensor->attribute_id == report_attr_unmasked);
            bool ieee_match = true;
            if (zb_sensor->device_ieee != 0 && report.ieee_addr != 0) {
                ieee_match = (zb_sensor->device_ieee == report.ieee_addr);
            }
            bool ep_match = true;
            // Endpoint 10 is the coordinator's local endpoint, not a remote one.
            // Treat ep=10 or ep=1 in sensor config as "match any endpoint".
            if (zb_sensor->endpoint != 1 && zb_sensor->endpoint != 10 && report.endpoint != 0) {
                ep_match = (zb_sensor->endpoint == report.endpoint);
            }
            bool matches = cluster_match && attr_match && ieee_match && ep_match;

            // Optional Tuya DP override per sensor: allow matching raw EF00/DP
            // reports directly, independent of the configured cluster/attribute.
            bool is_tuya_dp_report = (report.cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC) &&
                                     ((report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0);
            uint16_t report_dp = zigbee_report_attr_id(report.attr_id);
            if (!matches && is_tuya_dp_report && zb_sensor->tuya_dp_value >= 0 &&
                ieee_match && ep_match && report_dp == (uint16_t)zb_sensor->tuya_dp_value) {
                matches = true;
            }
            
            if (matches) {
                ZB_GW_TRACE(F("[ZIGBEE-GW]   ✓ Matched '%s': c=0x%04X a=0x%04X ieee=%08lX%08lX → raw=%ld\n"),
                            zb_sensor->getName(), report.cluster_id, report_attr_unmasked,
                            (unsigned long)(report.ieee_addr >> 32), (unsigned long)(report.ieee_addr & 0xFFFFFFFF),
                            report.value);
                gw_updateSensorFromReport(zb_sensor, report, report_solicited);
                found = true;
                // Don't break — multiple logical sensors may reference the
                // same physical device (same IEEE/cluster/attr).  Continue
                // iterating so every matching sensor is updated.
            }
        }
        
        // Mark consumed after iterating ALL sensors so every match is serviced
        report.consumed = true;
        
        if (checked_count == 0) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] ✗ WARNING: No ZigBee sensors configured! (checked empty list)"));
        } else if (!found) {
            bool suppress_nomatch = false;
            if (report.cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC &&
                ((report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0)) {
                suppress_nomatch = gw_is_ignorable_unmatched_tuya_dp(zigbee_report_attr_id(report.attr_id), report.ieee_addr);
            }

            if (!suppress_nomatch) {
#if ZB_GW_VERBOSE_LOG
                DEBUG_PRINTF(F("[ZIGBEE-GW] ✗ NO MATCH (checked %d ZigBee sensor%s)\n"), checked_count, checked_count == 1 ? "" : "s");
                DEBUG_PRINTF(F("[ZIGBEE-GW]   Report detail: ieee=0x%016llX ep=%u cluster=0x%04X attr=0x%04X value=%ld%s solicited=%d\n"),
                             (unsigned long long)report.ieee_addr,
                             report.endpoint,
                             report.cluster_id,
                             zigbee_report_attr_id(report.attr_id),
                             report.value,
                             (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) ? " (Tuya)" : "",
                             report_solicited ? 1 : 0);

                SensorIterator dbg_it = sensors_iterate_begin();
                SensorBase* dbg_sensor;
                bool same_ieee_found = false;
                while ((dbg_sensor = sensors_iterate_next(dbg_it)) != NULL) {
                    if (!dbg_sensor || dbg_sensor->type != SENSOR_ZIGBEE) continue;
                    ZigbeeSensor* dbg_zb = static_cast<ZigbeeSensor*>(dbg_sensor);
                    if (report.ieee_addr == 0 || dbg_zb->device_ieee != report.ieee_addr) continue;
                    same_ieee_found = true;
                    DEBUG_PRINTF(F("[ZIGBEE-GW]   Candidate '%s': ep=%u cluster=0x%04X attr=0x%04X mfr=\"%s\" model=\"%s\" vendor=\"%s\" data_ok=%d\n"),
                                 dbg_sensor->getName(),
                                 dbg_zb->endpoint,
                                 dbg_zb->cluster_id,
                                 dbg_zb->attribute_id,
                                 dbg_zb->zb_manufacturer,
                                 dbg_zb->zb_model,
                                 dbg_zb->zb_vendor,
                                 dbg_zb->flags.data_ok ? 1 : 0);
                }
                if (!same_ieee_found && report.ieee_addr != 0) {
                    DEBUG_PRINTF(F("[ZIGBEE-GW]   No configured ZigBee sensor bound to ieee=0x%016llX\n"),
                                 (unsigned long long)report.ieee_addr);
                }
#endif
            }
        }
        
        bool handled_custom_tuya_batt = false;
        bool handled_custom_tuya_unit = false;
        if (!found) {

            // Custom Tuya battery DP mapping per sensor
            if (report.cluster_id == ZB_ZCL_CLUSTER_ID_TUYA_SPECIFIC && report.ieee_addr != 0 &&
                ((report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0)) {
                uint16_t report_dp = zigbee_report_attr_id(report.attr_id);
                SensorIterator batt_it = sensors_iterate_begin();
                SensorBase* batt_sensor;
                while ((batt_sensor = sensors_iterate_next(batt_it)) != NULL) {
                    if (!batt_sensor || batt_sensor->type != SENSOR_ZIGBEE) continue;
                    ZigbeeSensor* zb_batt = static_cast<ZigbeeSensor*>(batt_sensor);
                    if (zb_batt->device_ieee != report.ieee_addr) continue;
                    bool explicit_battery_dp = (zb_batt->tuya_dp_battery >= 0 && (uint16_t)zb_batt->tuya_dp_battery == report_dp);
                    if (!explicit_battery_dp) continue;

                    uint32_t battery_pct = zigbee_battery_percent_from_report(true, report_dp, tuya_report_type(report.attr_id), zb_batt->tuya_dp_battery, report.value);
                    zb_batt->last_battery = battery_pct;
                    zb_batt->last_lqi = report.lqi;
                    handled_custom_tuya_batt = true;
                }
                if (handled_custom_tuya_batt) {
                    DEBUG_PRINTF(F("[ZIGBEE-GW] Custom Tuya battery DP %u applied: ieee=%08lX%08lX value=%ld\n"),
                                (unsigned)report_dp,
                                (unsigned long)(report.ieee_addr >> 32), (unsigned long)(report.ieee_addr & 0xFFFFFFFF),
                                report.value);
                }

                SensorIterator unit_it = sensors_iterate_begin();
                SensorBase* unit_sensor;
                while ((unit_sensor = sensors_iterate_next(unit_it)) != NULL) {
                    if (!unit_sensor || unit_sensor->type != SENSOR_ZIGBEE) continue;
                    ZigbeeSensor* zb_unit = static_cast<ZigbeeSensor*>(unit_sensor);
                    if (zb_unit->device_ieee != report.ieee_addr) continue;
                    if (zb_unit->tuya_dp_unit < 0 || (uint16_t)zb_unit->tuya_dp_unit != report_dp) continue;

                    zb_unit->tuya_unit = (report.value < 0) ? 0xFF : (uint8_t)report.value;
                    zb_unit->last_lqi = report.lqi;
                    handled_custom_tuya_unit = true;
                }
                if (handled_custom_tuya_unit) {
                    DEBUG_PRINTF(F("[ZIGBEE-GW] Custom Tuya unit DP %u applied: ieee=%08lX%08lX unit=%ld\n"),
                                (unsigned)report_dp,
                                (unsigned long)(report.ieee_addr >> 32), (unsigned long)(report.ieee_addr & 0xFFFFFFFF),
                                report.value);
                }
            }
            
            // Auto-correct misconfigured sensors: if a known standard ZCL cluster report comes in
            // for a sensor that has the correct IEEE address but wrong cluster/attribute configured,
            // correct the stored cluster_id/attribute_id and re-save.
            // This handles cases where sensors were saved with wrong values (e.g. 0x0401 instead of 0x0405).
            uint16_t report_attr_raw = zigbee_report_attr_id(report.attr_id);
            bool is_known_standard_cluster = (
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT && report_attr_raw == 0x0000) ||
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT         && report_attr_raw == 0x0000) ||
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_SOIL_MOISTURE            && report_attr_raw == 0x0000) ||
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT     && report_attr_raw == 0x0000) ||
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT  && report_attr_raw == 0x0000) ||
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_FLOW_MEASUREMENT         && report_attr_raw == 0x0000) ||
                (report.cluster_id == ZB_ZCL_CLUSTER_ID_METERING                 && report_attr_raw == 0x0000)
            );
            if (is_known_standard_cluster && report.ieee_addr != 0) {
                SensorIterator ac_it = sensors_iterate_begin();
                SensorBase* ac_sensor;
                while ((ac_sensor = sensors_iterate_next(ac_it)) != NULL) {
                    if (!ac_sensor || ac_sensor->type != SENSOR_ZIGBEE) continue;
                    ZigbeeSensor* zb_ac = static_cast<ZigbeeSensor*>(ac_sensor);
                    if (zb_ac->device_ieee != report.ieee_addr) continue;
                    if (zb_ac->flags.data_ok) continue;  // already receiving data correctly
                    if (zb_ac->cluster_id == report.cluster_id && zb_ac->attribute_id == report_attr_raw) continue;  // already correct
                    // Correct the misconfigured cluster/attribute
                    // DEBUG_PRINTF(F("[ZIGBEE-GW] Auto-correcting sensor '%s': cluster 0x%04X→0x%04X attr 0x%04X→0x%04X\n"),
                                // zb_ac->name, zb_ac->cluster_id, report.cluster_id, zb_ac->attribute_id, report_attr_raw);
                    zb_ac->cluster_id = report.cluster_id;
                    zb_ac->attribute_id = report_attr_raw;
                    sensor_save();
                    // Now apply the report to this newly-corrected sensor
                    // Auto-correct reports are always unsolicited (we didn't ask for them)
                    gw_updateSensorFromReport(zb_ac, report, false);
                }
            }

            // If custom Tuya battery/unit handlers consumed this report, skip
            // generic battery/unmatched handling.
            if (handled_custom_tuya_batt || handled_custom_tuya_unit) {
                // handled
            }
            // If this is a battery report (cluster 0x0001, attr 0x0021), update
            // battery on any sensor matching the IEEE address
            else if (report.cluster_id == ZB_ZCL_CLUSTER_ID_POWER_CONFIG &&
                zigbee_report_attr_id(report.attr_id) == 0x0021 && report.ieee_addr != 0) {
                bool is_tuya_battery = (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) != 0;
                uint32_t battery_pct = zigbee_battery_percent_from_report(is_tuya_battery, zigbee_report_attr_id(report.attr_id), tuya_report_type(report.attr_id), -1, report.value);
                SensorIterator it2 = sensors_iterate_begin();
                SensorBase* s2;
                while ((s2 = sensors_iterate_next(it2)) != NULL) {
                    if (s2 && s2->type == SENSOR_ZIGBEE) {
                        ZigbeeSensor* zb2 = static_cast<ZigbeeSensor*>(s2);
                        if (zb2->device_ieee == report.ieee_addr && battery_pct != ZB_BATTERY_UNKNOWN) {
                            zb2->last_battery = battery_pct;
                        }
                    }
                }
                DEBUG_PRINTF(F("[ZIGBEE-GW] Battery report: ieee=%08lX%08lX battery=%d%%\n"),
                            (unsigned long)(report.ieee_addr >> 32), (unsigned long)(report.ieee_addr & 0xFFFFFFFF),
                            battery_pct);
            } else {
                // Throttle unmatched (non-battery) report logging
                static unsigned long last_unmatched_log = 0;
                static uint16_t last_unmatched_cluster = 0;
                static uint16_t last_unmatched_attr = 0;
                static int unmatched_count = 0;
                
                if (report.cluster_id != last_unmatched_cluster || 
                    zigbee_report_attr_id(report.attr_id) != last_unmatched_attr ||
                    millis() - last_unmatched_log > 30000) {
                    if (unmatched_count > 1) {
                        // DEBUG_PRINTF(F("[ZIGBEE-GW]   (%d identical unmatched reports suppressed)\n"), unmatched_count - 1);
                    }
                    // DEBUG_PRINTF(F("[ZIGBEE-GW] Unmatched report: ieee=%08lX%08lX cluster=0x%04X attr=0x%04X value=%ld%s\n"),
                                // (unsigned long)(report.ieee_addr >> 32), (unsigned long)(report.ieee_addr & 0xFFFFFFFF),
                                // report.cluster_id, report.attr_id & ~TUYA_REPORT_FLAG_PRESCALED, report.value,
                                // (report.attr_id & TUYA_REPORT_FLAG_PRESCALED) ? " (Tuya)" : "");
                    last_unmatched_log = millis();
                    last_unmatched_cluster = report.cluster_id;
                    last_unmatched_attr = zigbee_report_attr_id(report.attr_id);
                    unmatched_count = 1;
                } else {
                    unmatched_count++;
                }
            }
        }
    }
    
    // Clear consumed and expired reports
    size_t write_idx = 0;
    for (size_t read_idx = 0; read_idx < pending_report_count; read_idx++) {
        if (!pending_reports[read_idx].consumed &&
            millis() - pending_reports[read_idx].timestamp <= REPORT_VALIDITY_MS) {
            pending_reports[write_idx++] = pending_reports[read_idx];
        }
    }
    pending_report_count = write_idx;
}

// Track when the join window closes so radio lock can be released

void sensor_zigbee_gw_open_network(uint16_t duration) {
    if (!gw_zigbee_initialized) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] open_network: Zigbee not initialized!"));
        sensor_zigbee_gw_ensure_started();
        if (!gw_zigbee_initialized) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] open_network: Failed to start Zigbee!"));
            return;
        }
    }
    uint8_t dur = (duration > 254) ? 254 : (uint8_t)duration;

    if (dur == 0) {
        gw_join_window_end = 0;
        esp_zb_lock_acquire(portMAX_DELAY);
        Zigbee.openNetwork(0);
        esp_zb_lock_release();
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Join window CLOSED by request"));
        return;
    }
    // DEBUG_PRINTF("[ZIGBEE-GW] Opening network for %d seconds (permit join), started=%d connected=%d\n",
                 // dur, Zigbee.started() ? 1 : 0, Zigbee.connected() ? 1 : 0);

    // Acquire exclusive radio ownership for the scan window
    unsigned long window_ms = (unsigned long)dur * 1000UL;
    unsigned long now_ms = millis();
    unsigned long requested_end = now_ms + window_ms;
    if (gw_join_window_end != 0 && (long)(gw_join_window_end - now_ms) > 0) {
        if (requested_end > gw_join_window_end) {
            // DEBUG_PRINTF("[ZIGBEE-GW] Join window extended: remaining=%lu ms -> %lu ms\n",
                         // (unsigned long)(gw_join_window_end - now_ms),
                         // (unsigned long)(requested_end - now_ms));
            gw_join_window_end = requested_end;
        } else {
            // DEBUG_PRINTF("[ZIGBEE-GW] Join window already open: remaining=%lu ms\n",
                         // (unsigned long)(gw_join_window_end - now_ms));
        }
    } else {
        gw_join_window_end = requested_end;
    }

    DEBUG_PRINTF(F("[ZIGBEE-GW] Join window OPENED for %u s\n"), (unsigned)dur);

    // Enter join mode: applies maximum radio priority (txrx=HIGH) for the full
    // join window duration.  On 2.4 GHz this also disconnects WiFi for up to 10s.
    // The coex manager auto-exits join mode when duration_ms elapses, and

    esp_zb_lock_acquire(portMAX_DELAY);
    bool zb_started = Zigbee.started();
    bool zb_connected = Zigbee.connected();
    uint8_t cur_channel = esp_zb_get_current_channel();
    uint16_t cur_pan = esp_zb_get_pan_id();
    esp_err_t open_err = esp_zb_bdb_open_network(dur);
    esp_zb_lock_release();
    DEBUG_PRINTF(F("[ZIGBEE-GW] open_network: started=%d connected=%d ch=%d pan=0x%04X esp_zb_bdb_open_network=0x%x (%s)\n"),
                 zb_started ? 1 : 0, zb_connected ? 1 : 0,
                 cur_channel, cur_pan,
                 (unsigned)open_err, esp_err_to_name(open_err));
}

// Clear a device's cached identity WITHOUT forcing a physical leave/rejoin.
// Preserves the user's custom friendly name. Used to repair a cross-contaminated
// manufacturer (e.g. a standard-ZCL soil sensor wrongly stamped with a GX03
// "_TZE284_..." Tuya code): resets the identity fields so the next Basic read /
// DB lookup re-identifies the device correctly.
bool sensor_zigbee_gw_clear_device_identity(uint64_t device_ieee) {
    if (device_ieee == 0) return false;
    if (!gw_discovered_devices_loaded) {
        gw_load_discovered_devices();
    }
    ZigbeeDeviceInfo* dev = gw_find_discovered_device(device_ieee);
    if (!dev) return false;

    char ieee_buf[17] = "";
    snprintf(ieee_buf, sizeof(ieee_buf), "%016llX", (unsigned long long)device_ieee);
    OpenSprinkler::zigbee_logical_clear_ieee(ieee_buf);

    dev->manufacturer[0] = '\0';
    dev->model_id[0] = '\0';
    dev->vendor[0] = '\0';
    dev->logical_lookup_done = false;
    dev->is_tuya = false;
    dev->basic_query_attempts = 0;
    dev->silent_query_count = 0;
    gw_mark_discovered_devices_dirty();
    gw_save_discovered_devices();
    DEBUG_PRINTF(F("[ZIGBEE-GW] Cleared cached identity (no rejoin) for ieee=%s\n"), ieee_buf);
    return true;
}

// Trigger a forced rejoin for a device and reset Tuya sequence counter.
// This helps when a device loses sync with the gateway's DP sequence numbering.
bool sensor_zigbee_gw_rejoin_device(uint64_t device_ieee) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Rejoin FAILED for ieee=%016llX: GW not ready\n"),
                     (unsigned long long)device_ieee);
        return false;
    }
    if (device_ieee == 0) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Rejoin FAILED: device_ieee is 0"));
        return false;
    }

    // Reset the global Tuya sequence counter so the device starts fresh
    gw_reset_tuya_seq();
    DEBUG_PRINTF(F("[ZIGBEE-GW] Sequence reset to 0 for device ieee=%016llX\n"),
                 (unsigned long long)device_ieee);

    // Clear local logical devices of this IEEE and reset cached discovery info to restore defaults
    char ieee_buf[17] = "";
    snprintf(ieee_buf, sizeof(ieee_buf), "%016llX", (unsigned long long)device_ieee);
    OpenSprinkler::zigbee_logical_clear_ieee(ieee_buf);

    ZigbeeDeviceInfo* dev = gw_find_discovered_device(device_ieee);
    if (dev) {
        dev->manufacturer[0] = '\0';
        dev->model_id[0] = '\0';
        dev->vendor[0] = '\0';
        dev->logical_lookup_done = false;
        dev->is_tuya = false;
        dev->basic_query_attempts = 0;
        dev->silent_query_count = 0;
        gw_mark_discovered_devices_dirty();
        gw_save_discovered_devices();
        DEBUG_PRINTF(F("[ZIGBEE-GW] Cleared cached info and logical devices for rejoin: ieee=%s\n"), ieee_buf);
    }

    // Force the physical device to leave and rejoin by sending a ZDO leave request
    uint8_t ieee_le[8];
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }
    esp_zb_lock_acquire(portMAX_DELAY);
    uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
    esp_zb_lock_release();

    if (short_addr != 0xFFFF && short_addr != 0xFFFE) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Sending ZDO Leave Request (with Rejoin) to short_addr=0x%04X ...\n"),
                     (unsigned)short_addr);
        esp_zb_zdo_mgmt_leave_req_param_t leave_req = {};
        leave_req.dst_nwk_addr = short_addr;
        memcpy(leave_req.device_address, ieee_le, 8);
        leave_req.rejoin = 1; // Bitfield rejoin = 1
        leave_req.remove_children = 0;

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zdo_device_leave_req(&leave_req, [](esp_zb_zdp_status_t zdp_status, void* user_ctx) {
            DEBUG_PRINTF(F("[ZIGBEE-GW] ZDO Leave Response callback status: 0x%02X\n"), (unsigned)zdp_status);
        }, NULL);
        esp_zb_lock_release();
    } else {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Device not connected; opening network for manual pairing window."));
    }

    // Open network to make sure it can successfully complete the joining window
    sensor_zigbee_gw_open_network(60);  // 60 seconds for rejoin window

    DEBUG_PRINTF(F("[ZIGBEE-GW] ✓ Rejoin + seq reset initiated for ieee=%016llX (60s window)\n"),
                 (unsigned long long)device_ieee);
    return true;
}

// Permanently remove device from Zigbee gateway stack and discovery list
bool sensor_zigbee_gw_remove_device_from_stack(uint64_t device_ieee) {
    if (device_ieee == 0) return false;

    bool removed = false;
    // 1. Remove from gw_discovered_devices discovery list so it is no longer listed in UI
    for (auto it = gw_discovered_devices.begin(); it != gw_discovered_devices.end(); ) {
        if (it->ieee_addr == device_ieee) {
            DEBUG_PRINTF(F("[ZIGBEE-GW] Removing device ieee=%016llX from discovery list\n"),
                         (unsigned long long)device_ieee);
            it = gw_discovered_devices.erase(it);
            removed = true;
        } else {
            ++it;
        }
    }
    if (removed) {
        gw_mark_discovered_devices_dirty();
        gw_save_discovered_devices();
    }

    // 2. Clear pending state
    sensor_zigbee_gw_clear_device_runtime_state(device_ieee);

    // 3. Send ZDO Mgmt Leave request with rejoin=0 to unpair and reset the device to factory defaults
    uint8_t ieee_le[8];
    for (int i = 0; i < 8; i++) {
        ieee_le[i] = (uint8_t)(device_ieee >> (i * 8));
    }
    esp_zb_lock_acquire(portMAX_DELAY);
    uint16_t short_addr = esp_zb_address_short_by_ieee(ieee_le);
    esp_zb_lock_release();

    if (short_addr != 0xFFFF && short_addr != 0xFFFE) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Sending ZDO Permanent Leave Request to short_addr=0x%04X ...\n"),
                     (unsigned)short_addr);
        esp_zb_zdo_mgmt_leave_req_param_t leave_req = {};
        leave_req.dst_nwk_addr = short_addr;
        memcpy(leave_req.device_address, ieee_le, 8);
        leave_req.rejoin = 0; // Permanent leave (rejoin = 0)
        leave_req.remove_children = 0;

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zdo_device_leave_req(&leave_req, [](esp_zb_zdp_status_t zdp_status, void* user_ctx) {
            DEBUG_PRINTF(F("[ZIGBEE-GW] ZDO Permanent Leave Response callback status: 0x%02X\n"), (unsigned)zdp_status);
        }, NULL);
        esp_zb_lock_release();
    } else {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Device not connected; removed from local RAM list only."));
    }

    return true;
}

// Wrapper: Reset Tuya sequence (called when device is removed or manual resync needed)
void sensor_zigbee_gw_reset_tuya_seq() {
    gw_reset_tuya_seq();
}

int sensor_zigbee_gw_clear_device_runtime_state(uint64_t device_ieee) {
    if (device_ieee == 0) return 0;

    int cleared_verify = 0;

    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        ZbStationCtl &e = zb_station_ctl[sid];
        if (!e.active || e.ieee != device_ieee) continue;
        gw_station_switch_waiting_set(sid, false);
        e.active = false;
        cleared_verify++;
    }

    DEBUG_PRINTF(F("[ZIGBEE-GW] Cleared runtime state for ieee=%016llX (verify_entries=%d)\n"),
                 (unsigned long long)device_ieee, cleared_verify);
    return cleared_verify;
}

// Bounded-retry tracking for the device-DB logical-device lookup.  The HTTPS
// request to opensprinklershop.de can fail transiently under memory pressure
// (BLE + MQTT + RainMaker leave little internal RAM for the TLS handshake), so
// a single failure must NOT permanently mark the device "looked up".
struct GwLookupFail { uint64_t ieee; uint8_t count; };
static std::vector<GwLookupFail> gw_lookup_fails;
#define GW_LOOKUP_MAX_FAILS 20

static void gw_clear_lookup_failed(uint64_t ieee) {
    for (auto it = gw_lookup_fails.begin(); it != gw_lookup_fails.end(); ++it) {
        if (it->ieee == ieee) { gw_lookup_fails.erase(it); return; }
    }
}

static void gw_note_lookup_failed(uint64_t ieee) {
    for (auto& f : gw_lookup_fails) {
        if (f.ieee == ieee) {
            if (f.count < 255) f.count++;
            if (f.count >= GW_LOOKUP_MAX_FAILS) {
                // Give up after repeated failures (device likely absent from
                // the DB, or persistent connectivity/memory issue) so we stop
                // retrying forever. Re-find the device by IEEE: the vector may
                // have been reallocated by a concurrent announce during the
                // blocking HTTP call, so a cached reference could be stale.
                ZigbeeDeviceInfo* dp = gw_find_discovered_device(ieee);
                if (dp) {
                    dp->logical_lookup_done = true;
                    gw_mark_discovered_devices_dirty();
                }
            }
            return;
        }
    }
    gw_lookup_fails.push_back({ieee, 1});
}

// On ESP32, OpenSprinkler::send_http_request() delivers the HTTP response ONLY
// through the callback and then frees its internal buffer.  With a NULL callback
// the response is discarded and the caller's request buffer (ether_buffer) still
// holds the *request* — so a subsequent parse of ether_buffer sees only the
// request headers (nothing after the final CRLF) and fails with "EmptyInput".
// This callback copies the raw response (headers + body) into a dedicated buffer
// so sensor_zigbee_gw_do_lookups() can locate the JSON body after "\r\n\r\n".
// A dedicated (PSRAM) buffer is used instead of ether_buffer because the DB JSON
// for multi-sensor devices (e.g. GIEX GX03, 11 sensors) exceeds ETHER_BUFFER_SIZE
// and would otherwise be truncated → ArduinoJson "IncompleteInput" → the device
// name never resolves.
#define GW_LOOKUP_RESP_SIZE 8192
static char* gw_lookup_resp = nullptr;

static void gw_lookup_http_response_cb(char* response) {
    if (!response || !gw_lookup_resp) return;
    size_t n = strlen(response);
    if (n >= (size_t)GW_LOOKUP_RESP_SIZE) n = GW_LOOKUP_RESP_SIZE - 1;
    memcpy(gw_lookup_resp, response, n);
    gw_lookup_resp[n] = '\0';
}

static void sensor_zigbee_gw_do_lookups() {
    // Cover Ethernet AND WiFi: a WiFi-only check disabled the entire device-DB
    // logical-device lookup on Ethernet-only gateways (the common setup).
    if (!os.network_connected()) return;
    static unsigned long s_last_attempt_ms = 0;
    // Don't hammer the API — try at most once every 10 s
    if (s_last_attempt_ms != 0 && millis() - s_last_attempt_ms < 10000UL) return;

    // Scan through gw_discovered_devices
    for (auto& dev : gw_discovered_devices) {
        // logical_lookup_done is a runtime flag reset to false on every boot
        // (gw_reset_discovered_devices_runtime_fields), so each device is
        // re-evaluated once per session and fully-resolved ones short-circuit
        // below without an HTTP request.
        if (dev.logical_lookup_done) continue;
        if (dev.ieee_addr == 0) continue;
        if (!dev.manufacturer[0] || !dev.model_id[0]) continue;
        if (strcmp(dev.manufacturer, "unknown") == 0 || strcmp(dev.model_id, "unknown") == 0) continue;

        char ieee_buf[17];
        snprintf(ieee_buf, sizeof(ieee_buf), "%016llX", (unsigned long long)dev.ieee_addr);
        bool name_resolved = (dev.friendly_name[0] != '\0') || dev.is_custom_name;
        bool have_logicals = OpenSprinkler::zigbee_logical_count_ieee(ieee_buf) > 0;
        // Fully resolved: logical devices exist AND a name is set → done.
        if (have_logicals && name_resolved) {
            dev.logical_lookup_done = true;
            gw_mark_discovered_devices_dirty();
            continue;
        }
        // If logical devices already exist we still query the DB to fetch the
        // vendor + aliased model name for friendly_name, but must NOT re-register
        // the logical devices (would duplicate them).
        bool skip_register = have_logicals;

        // We found a device that needs logical and vendor name lookup!
        // Do NOT mark logical_lookup_done here — only after a SUCCESSFUL lookup
        // below.  The HTTPS request can fail transiently under memory pressure,
        // and marking done prematurely permanently skips the device.
        s_last_attempt_ms = millis();
        uint64_t lookup_ieee = dev.ieee_addr;  // stable key across the blocking HTTP call below

        DEBUG_PRINTF(F("[ZIGBEE-GW] Triggering background lookups for %s|%s (ieee=%s)\n"),
                     dev.manufacturer, dev.model_id, ieee_buf);

        // Percent-encode manufacturer and model
        char mfr_enc[64], mdl_enc[64];
        auto pct_encode = [](const char* src, char* dst, size_t dsz) {
            size_t j = 0;
            for (size_t i = 0; src[i] && j + 4 < dsz; i++) {
                unsigned char c = (unsigned char)src[i];
                if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.') {
                    dst[j++] = (char)c;
                } else {
                    j += snprintf(dst + j, dsz - j, "%%%02X", c);
                }
            }
            dst[j] = '\0';
        };
        pct_encode(dev.manufacturer, mfr_enc, sizeof(mfr_enc));
        pct_encode(dev.model_id,     mdl_enc, sizeof(mdl_enc));

        snprintf(ether_buffer, ETHER_BUFFER_SIZE - 1,
            "GET /zigbee/devices_api.php?manufacturer=%s&model=%s&firmware=1"
            " HTTP/1.0\r\nHost: opensprinklershop.de\r\n"
            "User-Agent: %s\r\nConnection: close\r\n\r\n",
            mfr_enc, mdl_enc, user_agent_string);

        // Use plain HTTP (port 80) rather than HTTPS: this gateway runs with
        // very little free internal RAM (BLE + MQTT + RainMaker), and the TLS
        // handshake for an HTTPS request frequently fails with out-of-memory,
        // which left devices without their logical-device profile.  The device
        // DB endpoint serves the same JSON over HTTP with no redirect.
        // A callback is REQUIRED on ESP32 (see gw_lookup_http_response_cb): the
        // response is otherwise discarded and ether_buffer keeps the request.
        // Read into a dedicated GW_LOOKUP_RESP_SIZE buffer (PSRAM) so large
        // multi-sensor DB entries (e.g. GX03) are not truncated to
        // ETHER_BUFFER_SIZE (which caused ArduinoJson "IncompleteInput").
        if (!gw_lookup_resp) {
            gw_lookup_resp = (char*)heap_caps_malloc(GW_LOOKUP_RESP_SIZE, MALLOC_CAP_SPIRAM);
            if (!gw_lookup_resp) gw_lookup_resp = (char*)malloc(GW_LOOKUP_RESP_SIZE);
        }
        if (!gw_lookup_resp) { gw_note_lookup_failed(lookup_ieee); return; }
        gw_lookup_resp[0] = '\0';
        int ret = os.send_http_request("opensprinklershop.de", 80, ether_buffer, gw_lookup_http_response_cb, false, 8000, true, GW_LOOKUP_RESP_SIZE);

        // The blocking request above can run for several seconds. During that
        // window a concurrent device announce may push_back into
        // gw_discovered_devices and REALLOCATE it, leaving the loop's `dev`
        // reference dangling. Writing the retry-cap flag to that stale memory
        // previously turned a failing lookup into an ENDLESS retry loop.
        // Re-acquire the device by IEEE and stop using the loop reference; we
        // return after handling this one device, so iterator invalidation of
        // the range-for is harmless.
        ZigbeeDeviceInfo* devp = gw_find_discovered_device(lookup_ieee);
        if (!devp) return;
        ZigbeeDeviceInfo& devr = *devp;  // use this, not the possibly-stale loop `dev`

        if (ret == HTTP_RQT_SUCCESS) {
            // Let's parse the HTTP response using ArduinoJson!
            // First locate the JSON start by skipping HTTP headers.
            const char* json_start = strstr(gw_lookup_resp, "\r\n\r\n");
            if (json_start) {
                json_start += 4;
            } else {
                json_start = gw_lookup_resp;
            }

            ArduinoJson::JsonDocument doc;
            ArduinoJson::DeserializationError err = ArduinoJson::deserializeJson(doc, json_start);
            if (err) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] JSON parsing failed: %s\n"), err.c_str());
                gw_note_lookup_failed(lookup_ieee);
                return;
            }

            // Extract vendor name if available
            const char* vnd = doc["vendor"];
            if (vnd && vnd[0]) {
                strncpy(devr.vendor, vnd, sizeof(devr.vendor) - 1);
                devr.vendor[sizeof(devr.vendor) - 1] = '\0';
                DEBUG_PRINTF(F("[ZIGBEE-GW] Found vendor: %s\n"), devr.vendor);
            }

            // Build a default friendly_name from the DB. Prefer the full
            // marketing description ("GIEX GX03 2-zone watering timer"); fall
            // back to "<vendor> <model_name>" ("GIEX GX03"). model_id is left
            // untouched (technical Zigbee model for fingerprint matching). Never
            // override a name the user set manually (is_custom_name).
            const char* model_name  = doc["model_name"];
            const char* description = doc["description"];
            if (!devr.is_custom_name) {
                char default_name[sizeof(devr.friendly_name)] = {0};
                if (description && description[0]) {
                    snprintf(default_name, sizeof(default_name), "%s", description);
                } else if (model_name && model_name[0]) {
                    if (devr.vendor[0]) {
                        snprintf(default_name, sizeof(default_name), "%s %s", devr.vendor, model_name);
                    } else {
                        snprintf(default_name, sizeof(default_name), "%s", model_name);
                    }
                }
                if (default_name[0] &&
                    strncmp(devr.friendly_name, default_name, sizeof(devr.friendly_name)) != 0) {
                    strncpy(devr.friendly_name, default_name, sizeof(devr.friendly_name) - 1);
                    devr.friendly_name[sizeof(devr.friendly_name) - 1] = '\0';
                    DEBUG_PRINTF(F("[ZIGBEE-GW] Default friendly name: %s\n"), devr.friendly_name);
                }
            }

            // Extract sensors array
            ArduinoJson::JsonArrayConst sensors = doc["sensors"].as<ArduinoJson::JsonArrayConst>();
            if (!skip_register && !sensors.isNull() && sensors.size() > 0) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] Parsing %u sensor definitions\n"), (unsigned int)sensors.size());

                // Pass 1: find battery DP and unit selector DP
                int16_t battery_dp = -1;
                int16_t unit_dp = -1;

                // 1a. Try to find exact battery name matches first (to prefer exact '%' level over 'battery_state')
                for (ArduinoJson::JsonVariantConst val : sensors) {
                    if (!val.is<ArduinoJson::JsonObjectConst>()) continue;
                    ArduinoJson::JsonObjectConst s = val.as<ArduinoJson::JsonObjectConst>();

                    const char* s_name = s["name"] | s["description"] | "";
                    int dp_val = -1;
                    if (s["dp"].is<int>()) {
                        dp_val = s["dp"].as<int>();
                    } else if (s["tuya_dp"].is<int>()) {
                        dp_val = s["tuya_dp"].as<int>();
                    }

                    if (dp_val >= 0) {
                        char name_lower[64];
                        strncpy(name_lower, s_name, sizeof(name_lower) - 1);
                        name_lower[sizeof(name_lower) - 1] = '\0';
                        for (int idx = 0; name_lower[idx]; idx++) {
                            name_lower[idx] = tolower((unsigned char)name_lower[idx]);
                        }

                        if (strcmp(name_lower, "battery") == 0 || strcmp(name_lower, "battery_percentage_remaining") == 0) {
                            battery_dp = dp_val;
                            break;
                        }
                    }
                }

                // 1b. Fallback to substring and unit searches if not found yet
                for (ArduinoJson::JsonVariantConst val : sensors) {
                    if (!val.is<ArduinoJson::JsonObjectConst>()) continue;
                    ArduinoJson::JsonObjectConst s = val.as<ArduinoJson::JsonObjectConst>();

                    const char* s_name = s["name"] | s["description"] | "";
                    int dp_val = -1;
                    if (s["dp"].is<int>()) {
                        dp_val = s["dp"].as<int>();
                    } else if (s["tuya_dp"].is<int>()) {
                        dp_val = s["tuya_dp"].as<int>();
                    }

                    if (dp_val >= 0) {
                        char name_lower[64];
                        strncpy(name_lower, s_name, sizeof(name_lower) - 1);
                        name_lower[sizeof(name_lower) - 1] = '\0';
                        for (int idx = 0; name_lower[idx]; idx++) {
                            name_lower[idx] = tolower((unsigned char)name_lower[idx]);
                        }

                        if (battery_dp == -1 && (strcmp(name_lower, "battery_state") == 0 || strstr(name_lower, "battery"))) {
                            battery_dp = dp_val;
                        }
                        if (strcmp(name_lower, "temperature_unit") == 0 || strstr(name_lower, "unit") || strstr(name_lower, "quantity")) {
                            if (unit_dp == -1) {
                                unit_dp = dp_val;
                            }
                        }
                    }
                }

                DEBUG_PRINTF(F("[ZIGBEE-GW] Metadata found: battery_dp=%d, unit_dp=%d\n"), battery_dp, unit_dp);

                // Pass 2: register each non-metadata sensor as a logical device
                for (ArduinoJson::JsonVariantConst val : sensors) {
                    if (!val.is<ArduinoJson::JsonObjectConst>()) continue;
                    ArduinoJson::JsonObjectConst s = val.as<ArduinoJson::JsonObjectConst>();

                    const char* s_name = s["name"] | s["description"] | "Logical Device";
                    char name_lower[64];
                    strncpy(name_lower, s_name, sizeof(name_lower) - 1);
                    name_lower[sizeof(name_lower) - 1] = '\0';
                    for (int idx = 0; name_lower[idx]; idx++) {
                        name_lower[idx] = tolower((unsigned char)name_lower[idx]);
                    }

                    // Skip metadata sensors! They are mapped as attributes on actual measured logical devices.
                    if (strcmp(name_lower, "battery") == 0 || strcmp(name_lower, "battery_state") == 0 ||
                        strcmp(name_lower, "temperature_unit") == 0 ||
                        strstr(name_lower, "battery") ||
                        strstr(name_lower, "unit") ||
                        strstr(name_lower, "quantity")) {
                        continue;
                    }

                    // Build and populate the logical device struct
                    ZigBeeLogicalDevice logdev = {};
                    strncpy(logdev.ieee, ieee_buf, sizeof(logdev.ieee) - 1);
                    strncpy(logdev.name, s_name, sizeof(logdev.name) - 1);
                    const char* s_desc = s["description"] | s["desc"] | "";
                    strncpy(logdev.desc, s_desc, sizeof(logdev.desc) - 1);
                    logdev.desc[sizeof(logdev.desc) - 1] = '\0';

                    logdev.endpoint = s["endpoint"] | 1;

                    // Parse cluster_id
                    const char* cluster_str = s["cluster_id"] | "";
                    uint16_t cluster_id = 0;
                    if (cluster_str[0] == '0' && (cluster_str[1] == 'x' || cluster_str[1] == 'X')) {
                        cluster_id = strtol(cluster_str, NULL, 16);
                    } else {
                        cluster_id = atoi(cluster_str);
                    }
                    logdev.cluster_id = cluster_id;

                    // Parse attr_id
                    const char* attr_str = s["attr_id"] | s["attribute_id"] | "";
                    uint16_t attr_id = 0;
                    if (attr_str[0] == '0' && (attr_str[1] == 'x' || attr_str[1] == 'X')) {
                        attr_id = strtol(attr_str, NULL, 16);
                    } else {
                        attr_id = atoi(attr_str);
                    }
                    logdev.attr_id = attr_id;

                    // Check if is_tuya
                    bool is_tuya_dp = s["is_tuya_dp"] | false;
                    if (cluster_id == 0xEF00 || is_tuya_dp) {
                        logdev.is_tuya = true;
                    } else {
                        logdev.is_tuya = false;
                    }

                    // Tuya DP values
                    int dp_val = -1;
                    if (s["dp"].is<int>()) {
                        dp_val = s["dp"].as<int>();
                    } else if (s["tuya_dp"].is<int>()) {
                        dp_val = s["tuya_dp"].as<int>();
                    }
                    if (logdev.is_tuya && dp_val >= 0) {
                        logdev.tuya_dp_value = dp_val;
                    } else {
                        logdev.tuya_dp_value = -1;
                    }

                    logdev.tuya_dp_battery = battery_dp;
                    logdev.tuya_dp_unit = unit_dp;

                    // Other secondary DP fields
                    logdev.tuya_dp_status = s["tuya_dp_status"] | s["status_dp"] | -1;
                    logdev.tuya_dp_consumption = s["tuya_dp_consumption"] | s["consumption_dp"] | -1;

                    const char* status_on_str = s["tuya_status_on"] | s["status_on"] | "";
                    strncpy(logdev.tuya_status_on, status_on_str, sizeof(logdev.tuya_status_on) - 1);
                    logdev.tuya_status_on[sizeof(logdev.tuya_status_on) - 1] = '\0';

                    const char* status_off_str = s["tuya_status_off"] | s["status_off"] | s["tuya_status_of"] | s["status_of"] | "";
                    strncpy(logdev.tuya_status_off, status_off_str, sizeof(logdev.tuya_status_off) - 1);
                    logdev.tuya_status_off[sizeof(logdev.tuya_status_off) - 1] = '\0';

                    // Factor / Divider / Offset
                    logdev.factor = s["factor"] | 1;
                    logdev.divider = s["divider"] | 1;
                    logdev.offset = s["offset"] | 0;

                    // Unit info
                    const char* unit_str = s["unit"] | "";
                    strncpy(logdev.unit, unit_str, sizeof(logdev.unit) - 1);
                    logdev.unitid = s["unitid"] | 0;

                    // Runtime role (see runtime_roles.php in the device DB)
                    const char* role_str = s["role"] | "";
                    if (strcmp(role_str, "runtime") == 0)   logdev.role = ZB_LD_ROLE_RUNTIME;
                    else if (strcmp(role_str, "mode") == 0) logdev.role = ZB_LD_ROLE_MODE;
                    else                                     logdev.role = ZB_LD_ROLE_NONE;
                    logdev.channel = s["channel"] | 0;
                    const char* rt_unit = s["runtime_unit"] | "";
                    if (strcmp(rt_unit, "min") == 0)    logdev.runtime_unit = ZB_RT_UNIT_MIN;
                    else if (strcmp(rt_unit, "h") == 0) logdev.runtime_unit = ZB_RT_UNIT_H;
                    else                                logdev.runtime_unit = ZB_RT_UNIT_S;
                    logdev.runtime_max  = s["runtime_max"] | 0;
                    logdev.prereq_dp    = s["prereq_dp"] | 0;
                    logdev.prereq_value = s["prereq_value"] | 0;

                    // Register it!
                    OpenSprinkler::zigbee_logical_register(logdev);
                }
            }
            // Lookup succeeded (device found in DB).  Mark done so we don't
            // re-query it and clear the failure backoff.
            devr.logical_lookup_done = true;
            gw_mark_discovered_devices_dirty();
            gw_clear_lookup_failed(devr.ieee_addr);
        } else {
            // Transient failure (often TLS/HTTPS out-of-memory on this busy
            // gateway).  Leave logical_lookup_done=false so the next cycle
            // retries, bounded by gw_note_lookup_failed().
            DEBUG_PRINTF(F("[ZIGBEE-GW] HTTP request to devices_api failed: %d\n"), ret);
            gw_note_lookup_failed(lookup_ieee);
        }
        return; // handle only one device lookup per call
    }
}

uint16_t sensor_zigbee_gw_get_join_window_remaining() {
    if (gw_join_window_end == 0) return 0;
    unsigned long now = millis();
    if ((long)(gw_join_window_end - now) <= 0) return 0;
    unsigned long remaining_ms = gw_join_window_end - now;
    return (uint16_t)((remaining_ms + 999UL) / 1000UL);
}

uint8_t sensor_zigbee_gw_get_channel() {
    if (!gw_zigbee_initialized) return 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    uint8_t chan = esp_zb_get_current_channel();
    esp_zb_lock_release();
    return chan;
}

uint8_t sensor_zigbee_gw_get_configured_channel() {
    uint8_t chan = 0; // 0 means default / all channels
    if (LittleFS.exists("/zb_channel.txt")) {
        File file = LittleFS.open("/zb_channel.txt", "r");
        if (file) {
            String s = file.readStringUntil('\n');
            file.close();
            s.trim();
            int val = s.toInt();
            if (val >= 11 && val <= 26) {
                chan = (uint8_t)val;
            }
        }
    }
    return chan;
}

bool sensor_zigbee_gw_set_configured_channel(uint8_t channel) {
    if (channel != 0 && (channel < 11 || channel > 26)) {
        return false;
    }
    File file = LittleFS.open("/zb_channel.txt", "w");
    if (file) {
        file.printf("%d\n", (int)channel);
        file.close();
        return true;
    }
    return false;
}

// Deferred Zigbee startup tuning (reduce post-restart boot spike).
// At boot the network also brings up RainMaker/MQTT TLS, the HTTPS server and
// BLE, all heavy users of scarce internal RAM. Delaying the Zigbee rejoin
// auto-open + Tuya DP query burst by a few seconds avoids overlapping those
// peaks (which could push free internal heap low enough to stall the web UI on
// an "hourglass").
#define GW_STARTUP_DEFER_MS      20000UL  // wait after network forms before startup burst
#define GW_STARTUP_OPEN_SECONDS  60       // rejoin permit-join window (was 180s)

// Schedule a "WiFi-off" join: turn WiFi off for the join window so Zigbee gets
// the shared 2.4 GHz radio to itself, then reconnect WiFi automatically. The
// actual WiFi shutdown is deferred slightly by the loop state machine so the
// HTTP response can flush over WiFi before it drops.
void sensor_zigbee_gw_start_wifi_off_join(uint16_t duration) {
    if (duration == 0) duration = 30;
    if (duration > 180) duration = 180;
    if (gw_wj_state != GW_WJ_IDLE) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] WiFi-off join already in progress — ignoring"));
        return;
    }
    gw_wj_duration = duration;
    gw_wj_timer = millis() + 1500;   // let the /zo HTTP reply flush first
    gw_wj_state = GW_WJ_PENDING;
    DEBUG_PRINTF(F("[ZIGBEE-GW] WiFi-off join scheduled: %us window\n"), (unsigned)duration);
}

bool sensor_zigbee_gw_wifi_off_join_active() {
    return gw_wj_state != GW_WJ_IDLE;
}

// Drive the WiFi-off join phases. Called from sensor_zigbee_gw_loop().
static void gw_wifi_off_join_service() {
    if (gw_wj_state == GW_WJ_IDLE) return;
    unsigned long now = millis();
    switch (gw_wj_state) {
    case GW_WJ_PENDING:
        if ((long)(now - gw_wj_timer) >= 0) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] WiFi-off join: turning WiFi OFF for join window"));
            // Free the 2.4 GHz radio entirely for Zigbee joining.
            WiFi.disconnect(true, false);
            WiFi.mode(WIFI_OFF);
            // Open the Zigbee join window now that WiFi is out of the way.
            sensor_zigbee_gw_open_network(gw_wj_duration);
            gw_wj_timer = now + (unsigned long)gw_wj_duration * 1000UL;
            gw_wj_state = GW_WJ_JOINING;
        }
        break;
    case GW_WJ_JOINING:
        if ((long)(now - gw_wj_timer) >= 0) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] WiFi-off join: closing window, re-enabling WiFi"));
            sensor_zigbee_gw_open_network(0);   // close the join window
            // Reconnect WiFi using the stored credentials. Device-DB lookups for
            // any newly joined device are gated on os.network_connected() and so
            // resume automatically once WiFi is back up.
            start_network_sta(os.wifi_ssid.c_str(), os.wifi_pass.c_str(),
                              (int32_t)os.wifi_channel, os.wifi_bssid);
            WiFi.setAutoReconnect(true);
            gw_wj_state = GW_WJ_IDLE;
        }
        break;
    default:
        gw_wj_state = GW_WJ_IDLE;
        break;
    }
}

// Gently prompt one still-unidentified device per interval with a payload-less
// Tuya DP query so devices that only report after receiving a frame (e.g. Tuya
// valves woken by a physical toggle) eventually deliver their DPs and Basic
// Cluster info. Bounded to one send per GW_WAKE_UNIDENTIFIED_INTERVAL_MS to
// avoid flooding sleepy end devices.
static void gw_wake_unidentified_devices() {
    if (!Zigbee.started() || !Zigbee.connected()) return;
    if (gw_read_pending) return;  // don't collide with an in-flight read
    static unsigned long s_last_wake_ms = 0;
    static size_t s_wake_idx = 0;
    unsigned long now = millis();
    if (s_last_wake_ms != 0 && now - s_last_wake_ms < GW_WAKE_UNIDENTIFIED_INTERVAL_MS) return;

    size_t n = gw_discovered_devices.size();
    if (n == 0) { s_last_wake_ms = now; return; }

    for (size_t scanned = 0; scanned < n; scanned++) {
        size_t idx = (s_wake_idx + scanned) % n;
        ZigbeeDeviceInfo& dev = gw_discovered_devices[idx];
        if (dev.ieee_addr == 0) continue;
        if (!gw_device_needs_basic_info(dev)) continue;  // already identified
        if (gw_station_ctl_pending_for(dev.ieee_addr)) continue;  // station command first
        // Skip devices that reported recently — the report path re-queries them.
        if (dev.last_rx_at_ms != 0 && now - dev.last_rx_at_ms < GW_WAKE_UNIDENTIFIED_INTERVAL_MS) continue;
        uint16_t sa = dev.short_addr;
        if (sa == 0xFFFF || sa == 0xFFFE) continue;  // need a usable short address
        uint8_t ep = dev.endpoint ? dev.endpoint : 1;
        DEBUG_PRINTF(F("[ZIGBEE-GW] Waking unidentified device short=0x%04X ep=%u with Tuya DP query\n"), sa, ep);
        gw_tuya_send_dp_query(sa, ep);
        s_wake_idx = idx + 1;
        s_last_wake_ms = now;
        return;  // one device per interval
    }
    s_last_wake_ms = now;  // nothing to wake this round
}

// Send-status callback (Zigbee task): queue the APS confirm for the station
// switch state machine.
static void gw_send_status_cb(esp_zb_zcl_command_send_status_message_t msg) {
    GwRxEvent ev = {};
    ev.kind = GW_RX_SEND_STATUS;
    ev.tsn = msg.tsn;
    ev.status = (int32_t)msg.status;
    ev.endpoint = msg.dst_endpoint;
    ev.short_addr = (msg.dst_addr.addr_type == ESP_ZB_ZCL_ADDR_TYPE_SHORT) ? msg.dst_addr.u.short_addr : 0xFFFF;
    gw_rx_push(ev);
}

static void gw_station_ctl_on_send_status(uint8_t tsn, int32_t status, uint16_t short_addr);
static void gw_station_ctl_on_timed_off_rejected(uint64_t ieee);

// Drain the RX queue in the main loop and run all application-level work that
// used to live in the Zigbee callbacks.
static void gw_rx_drain() {
    if (!gw_rx_queue) return;
    GwRxEvent ev;
    uint64_t last_rx_ieee = 0;
    int processed = 0;
    while (processed < 64 && xQueueReceive(gw_rx_queue, &ev, 0) == pdTRUE) {
        processed++;
        switch (ev.kind) {
            case GW_RX_SEEN:
                if (ev.ieee) gw_add_responsive_device(ev.short_addr, ev.ieee, ev.endpoint);
                break;

            case GW_RX_ANNOUNCE:
                if (ev.ieee) {
                    // Pre-register so the device is visible in /zd / /zg even if
                    // it never answers the immediate Basic Cluster query.
                    gw_add_responsive_device(ev.short_addr, ev.ieee, 1);
                    // Genuine (re)join: refresh the Basic Cluster retry budget
                    // (this is the only place the attempt cap is reset) and
                    // re-queue one query.
                    ZigbeeDeviceInfo* joined = gw_find_discovered_device(ev.ieee);
                    if (joined && gw_device_needs_basic_info(*joined)) {
                        joined->basic_query_attempts = 0;
                        if (!gw_is_basic_query_queued(ev.ieee)) {
                            gw_queue_basic_cluster_query(ev.ieee, ev.short_addr, 1, 1000UL);
                        }
                    }
                    // A (re)joined device may have lost its bindings/reporting
                    // configuration: re-schedule it for all sensors on this IEEE.
                    gw_schedule_configure_reporting_for_ieee(ev.ieee, 8000UL, true);
                }
                // Tuya DP query first (fast, unblocks TS0601/GIEX immediately).
                gw_tuya_send_dp_query(ev.short_addr, 1);
                break;

            case GW_RX_ATTR:
                if (ev.ieee) gw_add_responsive_device(ev.short_addr, ev.ieee, ev.endpoint);
                if (ev.cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && ev.attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
                    gw_station_status_process_standard_onoff(ev.ieee, ev.endpoint, ev.value != 0);
                }
                if (gw_cache_attribute_report(ev.ieee, ev.endpoint, ev.cluster_id, ev.attr_id, ev.value, ev.lqi)) {
                    ZB_GW_TRACE(F("[ZIGBEE-GW] Report cached [%d/%d]: ieee=%016llX cluster=0x%04X attr=0x%04X value=%ld ep=%d\n"),
                                (int)pending_report_count, (int)MAX_PENDING_REPORTS,
                                (unsigned long long)ev.ieee, ev.cluster_id, ev.attr_id, (long)ev.value, ev.endpoint);
                }
                break;

            case GW_RX_TUYA_DP:
                if (ev.ieee) {
                    gw_add_responsive_device(ev.short_addr, ev.ieee, ev.endpoint);
                    gw_mark_tuya_device(ev.ieee);
                }
                gw_cache_tuya_dp_report(ev.ieee, ev.endpoint, (uint8_t)ev.attr_id, ev.value, ev.lqi, ev.dp_type);
                break;

            case GW_RX_BASIC: {
                if (ev.ieee && ev.short_addr != 0) gw_add_responsive_device(ev.short_addr, ev.ieee, ev.endpoint);
                esp_zb_zcl_attribute_t attr = {};
                attr.id = ev.attr_id;
                attr.data.type = (esp_zb_zcl_attr_type_t)ev.dp_type;
                attr.data.size = ev.raw_len;
                attr.data.value = ev.raw;
                gw_handleBasicClusterResponse(ev.short_addr, &attr, ev.ieee);
                break;
            }

            case GW_RX_SEND_STATUS:
                gw_station_ctl_on_send_status(ev.tsn, ev.status, ev.short_addr);
                continue; // no device RX

            case GW_RX_DEFAULT_RESP:
                DEBUG_PRINTF(F("[ZIGBEE-GW][ZCL] Default response: cluster=0x%04X ep=%u status=0x%02X\n"),
                             ev.cluster_id, ev.endpoint, (unsigned)ev.status);
                // A device that does not implement OnWithTimedOff answers with
                // UNSUP_CLUSTER_COMMAND: remember it and re-issue a plain On.
                if (ev.cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
                    ev.status == (int32_t)ESP_ZB_ZCL_STATUS_UNSUP_CLUST_CMD &&
                    gw_last_timed_off_ieee != 0 && (millis() - gw_last_timed_off_ms) < 3000UL) {
                    gw_zcl_timed_off_mark_unsupported(gw_last_timed_off_ieee);
                    gw_station_ctl_on_timed_off_rejected(gw_last_timed_off_ieee);
                }
                continue; // no device RX

            default:
                break;
        }
        // Wake trigger: the device just talked to us, so it is listening now.
        if (ev.ieee && ev.ieee != last_rx_ieee) {
            last_rx_ieee = ev.ieee;
            gw_station_ctl_on_device_rx(ev.ieee);
        }
    }
    static uint32_t s_last_drop_report = 0;
    if (gw_rx_dropped != s_last_drop_report) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] RX queue overflow: %lu events dropped so far\n"), (unsigned long)gw_rx_dropped);
        s_last_drop_report = gw_rx_dropped;
    }
}

// Pull RSSI / LQI of every child out of the stack's neighbor table into the
// discovered-device records (shown in the UI as a signal-quality lamp).  The
// neighbor RSSI is the level at which the coordinator last heard the child;
// sleepy children rarely carry a usable LQI there, so LQI is only taken when
// non-zero.  In debug builds the full table is dumped as well (device type,
// rx-on-when-idle, end-device timeout and aging countdown, which resets on
// every poll of the child).
static void gw_refresh_link_quality() {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) return;
    esp_zb_nwk_info_iterator_t nit = ESP_ZB_NWK_INFO_ITERATOR_INIT;
    esp_zb_nwk_neighbor_info_t nb;
    bool changed = false;
    esp_zb_lock_acquire(portMAX_DELAY);
    while (esp_zb_nwk_get_next_neighbor(&nit, &nb) == ESP_OK) {
        uint64_t ieee = gw_ieee_from_raw(nb.ieee_addr);
        DEBUG_PRINTF(F("[ZIGBEE-GW][NBR] short=0x%04X ieee=%016llX type=%u rxidle=%u rel=%u lqi=%u rssi=%d edto=%lus age=%lums\n"),
                     nb.short_addr, (unsigned long long)ieee,
                     (unsigned)nb.device_type, (unsigned)nb.rx_on_when_idle, (unsigned)nb.relationship,
                     (unsigned)nb.lqi, (int)nb.rssi, (unsigned long)nb.device_timeout, (unsigned long)nb.timeout_counter);
        ZigbeeDeviceInfo* dev = gw_find_discovered_device(ieee);
        if (!dev) continue;
        if (nb.rssi != 127 && nb.rssi != 0 && dev->rssi != nb.rssi) { dev->rssi = nb.rssi; changed = true; }
        if (nb.lqi != 0 && dev->lqi != nb.lqi) { dev->lqi = nb.lqi; changed = true; }
    }
    esp_zb_lock_release();
    if (changed) gw_mark_discovered_devices_dirty();
}

void sensor_zigbee_gw_loop() {
    if (!gw_zigbee_initialized) return;

    // Hand-off from the Zigbee task: run all callback follow-up work here.
    gw_rx_drain();

    // Service the WiFi-off join state machine first (WiFi may be down).
    gw_wifi_off_join_service();

    // Deferred startup actions (auto-open for rejoin + Tuya DP query burst).
    static bool gw_startup_actions_pending = false;
    static unsigned long gw_network_formed_ms = 0;

    // Use os.network_connected() (covers Ethernet AND WiFi) rather than a
    // WiFi-only check: the Zigbee gateway commonly runs Ethernet-only, and a
    // WiFi.status() gate would prevent the device-DB lookup that fetches the
    // correct logical devices from ever running.
    if (os.network_connected()) {
        sensor_zigbee_gw_do_lookups();
    }

    // Check for timed-out station switch verifications.
    sensor_zigbee_station_verify_tick();

    // Continue serialized Tuya writes from the normal gateway loop. Scheduling
    // the next alarm from inside the Zigbee scheduler callback can be dropped
    // by ZBOSS on ESP32-C5, so the callback only dispatches one command.
    gw_tuya_schedule_next();

    // One-shot device query queue: Basic Cluster first, then Tuya datapoints,
    // with a hard 5 second gap between Zigbee commands.
    gw_process_device_query_queue();

    // Persist comm_mode changes that were set during report processing.
    if (gw_comm_mode_changed) {
        gw_comm_mode_changed = false;
        sensor_save();
    }

    if (gw_discovered_devices_dirty) {
        gw_save_discovered_devices();
    }

    // Timeout stale active-read requests
    if (gw_read_pending && millis() - gw_read_time > GW_READ_TIMEOUT_MS) {
        gw_read_timeout_ieee = gw_read_pending_ieee;
        gw_read_block_until_ms = millis() + GW_DEVICE_QUERY_SPACING_MS;
        
        uint16_t timed_out_cluster = gw_read_pending_cluster;
        uint64_t timed_out_ieee = gw_read_pending_ieee;
        uint16_t timed_out_attr = gw_read_attr_id;

        gw_read_pending = false;
        DEBUG_PRINTF(F("[ZIGBEE-GW] ⚠ Read TIMEOUT: attr_id=0x%04X waited %lu ms (GW_READ_TIMEOUT_MS=%lu). Clearing flag.\n"),
                    gw_read_attr_id, millis() - gw_read_time, GW_READ_TIMEOUT_MS);

        if (timed_out_cluster == ZB_ZCL_CLUSTER_ID_BASIC) {
            ZigbeeDeviceInfo* dev = gw_find_discovered_device(timed_out_ieee);
            if (dev) {
                if (timed_out_attr == 0x0000) {
                    DEBUG_PRINTF(F("[ZIGBEE-GW] Multi-attribute Basic Cluster query timed out. Retrying with individual queries (0x0004 & 0x0005) for ieee=%016llX...\n"),
                                 (unsigned long long)timed_out_ieee);
                    dev->basic_query_attempts++;
                    gw_queue_basic_cluster_query_attr(dev->ieee_addr, dev->short_addr, dev->endpoint, ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, 1500UL);
                    gw_queue_basic_cluster_query_attr(dev->ieee_addr, dev->short_addr, dev->endpoint, ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 3000UL);
                } else {
                    dev->basic_query_attempts++;
                    if (dev->basic_query_attempts < 4) {
                        DEBUG_PRINTF(F("[ZIGBEE-GW] Single attribute basic query timed out for attr=0x%04X ieee=%016llX (attempt %u/4). Retrying...\n"),
                                     timed_out_attr, (unsigned long long)timed_out_ieee, dev->basic_query_attempts);
                        gw_queue_basic_cluster_query_attr(dev->ieee_addr, dev->short_addr, dev->endpoint, timed_out_attr, 2000UL);
                    } else if (dev->is_tuya) {
                        // Do NOT invent a concrete manufacturer here. Stamping
                        // the GX02 valve's "_TZE200_sh1btabb" mislabeled every
                        // sleepy Tuya device that never answers Basic Cluster as
                        // a "GIEX GX02 Water Valve". Only apply a generic TS0601
                        // model so the device stays usable; leave manufacturer
                        // empty so the database name lookup is not poisoned.
                        DEBUG_PRINTF(F("[ZIGBEE-GW] ⚠ Sleepy Tuya device ieee=%016llX reached query retry limit (%d). Applying generic TS0601 model (manufacturer left empty to avoid mislabeling).\n"),
                                     (unsigned long long)timed_out_ieee, dev->basic_query_attempts);
                        if (dev->model_id[0] == '\0' || strcmp(dev->model_id, "unknown") == 0) {
                            strncpy(dev->model_id, "TS0601", sizeof(dev->model_id) - 1);
                            dev->model_id[sizeof(dev->model_id) - 1] = '\0';
                            gw_mark_discovered_devices_dirty();
                        }
                    } else {
                        DEBUG_PRINTF(F("[ZIGBEE-GW] Basic read timeout on non-Tuya device ieee=%016llX after %u attempts. Leaving unknown.\n"),
                                     (unsigned long long)timed_out_ieee, dev->basic_query_attempts);
                    }
                }
            }
        }
    }

    // Always process pending reports regardless of web priority.
    // Zigbee ZCL reports are lightweight and must not be starved.
    // During join mode the radio is already dedicated to ZigBee — skip the
    // lock acquire/release cycle to avoid noisy strategy reapply calls.
    if (pending_report_count > 0) {
        // DEBUG_PRINTF(F("[ZIGBEE-GW] LOOP: %d reports waiting → processing now...\n"), pending_report_count);
        
        sensor_zigbee_gw_process_reports(0, 0, 0, 0, 0, 0);
        
        // DEBUG_PRINTF(F("[ZIGBEE-GW] LOOP: processing done, %d reports remaining\n"), pending_report_count);
    }
    
    static bool last_connected = false;
    bool connected = Zigbee.started() && Zigbee.connected();
    if (connected != last_connected) {
        if (connected) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Coordinator network FORMED"));
            // Schedule Configure Reporting for all sensors with known IEEE addresses.
            // Sleeping end devices (AQARA, etc.) will receive this on their next wake
            // and start pushing reports proactively.
            gw_schedule_configure_reporting_all(5000);

            // Defer the startup auto-open + Tuya DP query burst instead of
            // firing them the instant the network forms. At boot the network
            // also stands up RainMaker/MQTT TLS, the HTTPS server and BLE — all
            // heavy consumers of scarce internal RAM. Running the Zigbee rejoin
            // flood at the same moment drove free internal heap to its minimum
            // and could starve the web server (UI "hourglass"). The deferred
            // block in the loop runs these once, GW_STARTUP_DEFER_MS later.
            static bool gw_startup_scheduled = false;
            if (!gw_startup_scheduled) {
                gw_startup_scheduled = true;
                gw_startup_actions_pending = true;
                gw_network_formed_ms = millis();
            }
        } else {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Coordinator network LOST"));
        }
        last_connected = connected;
        gw_zigbee_connected = connected;
    }

    // Deferred Zigbee startup actions: auto-open the network for rejoin and send
    // the Tuya DP query burst, but only once the boot-time TLS/MQTT/BLE
    // allocations have had GW_STARTUP_DEFER_MS to settle. This decouples the
    // Zigbee rejoin flood from the internal-RAM boot peak (see note above).
    if (gw_startup_actions_pending && connected &&
        millis() - gw_network_formed_ms >= GW_STARTUP_DEFER_MS) {
        gw_startup_actions_pending = false;

        DEBUG_PRINTF(F("[ZIGBEE-GW] Deferred startup: auto-open %us for rejoin + Tuya DP queries\n"),
                     (unsigned)GW_STARTUP_OPEN_SECONDS);
        sensor_zigbee_gw_open_network(GW_STARTUP_OPEN_SECONDS);

        // Proactively query known Tuya sensors via ZBOSS address table.
        // If ZBOSS NVRAM is intact after a restart, esp_zb_address_short_by_ieee()
        // resolves the short address and we send a DP query, so data flows again.
        SensorIterator it_sq = sensors_iterate_begin();
        SensorBase* s_sq;
        while ((s_sq = sensors_iterate_next(it_sq)) != NULL) {
            if (!s_sq || s_sq->type != SENSOR_ZIGBEE) continue;
            ZigbeeSensor* zb_sq = static_cast<ZigbeeSensor*>(s_sq);
            if (zb_sq->device_ieee == 0) continue;
            // Tuya devices have manufacturer names starting with "_TZ"
            if (zb_sq->zb_manufacturer[0] != '_' || zb_sq->zb_manufacturer[1] != 'T') continue;
            bool sent = sensor_zigbee_gw_request_dp_query(
                zb_sq->device_ieee, zb_sq->endpoint ? zb_sq->endpoint : 1);
            if (sent) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] Startup DP query → '%s' (ieee=%016llX)\n"),
                             s_sq->getName(), (unsigned long long)zb_sq->device_ieee);
            }
        }
    }

    // NOTE: No idle timeout - Arduino Zigbee library does NOT support
    // Zigbee.stop() + Zigbee.begin() restart (causes Load access fault).
    // Once started, Zigbee stays running until reboot.

    // Release radio lock and restore WiFi after join window expires.
    if (gw_join_window_end != 0 && millis() > gw_join_window_end) {
        gw_join_window_end = 0;
        // DEBUG_PRINTLN(F("[ZIGBEE-GW] Join window closed, radio released"));
    }

    // Boot-settle guard: wait 90s after first ZigBee connection before
    // sending Tuya DP refresh queries (avoids hammering devices during startup).
    static unsigned long gw_first_connected_ms = 0;
    if (connected && gw_first_connected_ms == 0) gw_first_connected_ms = millis();
    bool boot_settled = gw_first_connected_ms > 0 && (millis() - gw_first_connected_ms) > 90000;

    // Drain queued Basic Cluster requests for devices that still need
    // manufacturer/model identification.
    gw_process_basic_query_queue();

    // Gently prompt still-unidentified devices to report (mimics a valve toggle).
    gw_wake_unidentified_devices();

    // Periodic Tuya DP refresh: send a "get all datapoints" query (cmd 0x00) to
    // every confirmed Tuya device that has at least one stale REPORT-mode sensor.
    // This mirrors the Z2M behaviour (dp: true in tuyaBase) and ensures sensors
    // that only send DP 3 (soil moisture) on significant changes still get polled.
    // Rate-limited to once per 90 s globally; this is also the earliest we will
    // fire after boot-settle, so the first query arrives very shortly after WiFi
    // has stabilised (boot_settled = 90 s after first connection).
    static unsigned long gw_tuya_refresh_ms = 0;
    if (connected && boot_settled && !(gw_join_window_end != 0)) {
        unsigned long now_tr = millis();
        if (now_tr - gw_tuya_refresh_ms >= 90000UL) {
            gw_tuya_refresh_ms = now_tr;
            for (auto& dev : gw_discovered_devices) {  // non-const: we update silent_query_count
                if (!dev.is_tuya || !dev.has_responded) continue;
                // Check if any REPORT-mode sensor for this device is stale
                bool needs_refresh = false;
                SensorIterator it_tr = sensors_iterate_begin();
                SensorBase* s_tr;
                while ((s_tr = sensors_iterate_next(it_tr)) != NULL) {
                    if (!s_tr || s_tr->type != SENSOR_ZIGBEE) continue;
                    ZigbeeSensor* zb_tr = static_cast<ZigbeeSensor*>(s_tr);
                    if (zb_tr->device_ieee != dev.ieee_addr) continue;
                    if (zb_tr->comm_mode == ZB_COMM_REPORT) {
                        uint32_t intv_tr = zb_tr->read_interval ? zb_tr->read_interval : 60;
                        // Stale = no report this boot, or last report > 2× read_interval ago
                        if (zb_tr->last_report_at_ms == 0 ||
                            (now_tr - zb_tr->last_report_at_ms) > (unsigned long)intv_tr * 2000UL) {
                            needs_refresh = true;
                            break;
                        }
                    }
                }
                if (needs_refresh) {
                    bool sent = sensor_zigbee_gw_request_dp_query(dev.ieee_addr, dev.endpoint);
                    if (sent) {
                        dev.silent_query_count++;
                        unsigned long silent_s = dev.last_rx_at_ms > 0
                            ? (now_tr - dev.last_rx_at_ms) / 1000UL : 9999UL;
                        DEBUG_PRINTF(F("[ZIGBEE-GW][TUYA] Refresh query → ieee=%016llX ep=%d (silent=%lus, count=%d)\n"),
                                     (unsigned long long)dev.ieee_addr, dev.endpoint,
                                     silent_s, dev.silent_query_count);

                        // After 5 unanswered queries (~7.5 min), open network so the device
                        // can rejoin if it lost its network slot after a coordinator restart.
                        // Reset counter after opening to wait another 5 cycles before retrying.
                        if (dev.silent_query_count >= 5 && !(gw_join_window_end != 0)) {
                            DEBUG_PRINTF(F("[ZIGBEE-GW] ⚠ Device %016llX silent %lus — auto-opening network for rejoin (60s)\n"),
                                         (unsigned long long)dev.ieee_addr, silent_s);
                            sensor_zigbee_gw_open_network(60);
                            dev.silent_query_count = 0;  // avoid re-opening every cycle
                        }
                    }
                }
            }
        }
    }

    // Process queued Basic Cluster requests only for devices that still have
    // missing manufacturer/model data. This keeps scan enrichment in firmware
    // without turning it into a periodic radio hammer.

    // Process pending Configure Reporting requests (one per stagger window)
    if (connected && !gw_config_report_queue.empty()) {
        unsigned long now_ms = millis();
        if (now_ms - gw_last_config_report_ms >= GW_CONFIG_REPORT_STAGGER_MS) {
            for (auto it_cr = gw_config_report_queue.begin(); it_cr != gw_config_report_queue.end(); ++it_cr) {
                if (now_ms >= it_cr->scheduled_time) {
                    sensor_zigbee_gw_configure_reporting(
                        it_cr->ieee_addr, it_cr->endpoint,
                        it_cr->cluster_id, it_cr->attr_id,
                        it_cr->min_interval, it_cr->max_interval);
                    gw_last_config_report_ms = now_ms;
                    gw_config_report_queue.erase(it_cr);
                    break;  // send one at a time; restart next loop
                }
            }
        }
    }

    // Process pending Bind Requests (one per stagger window)
    if (connected && !gw_bind_queue.empty()) {
        unsigned long now_ms = millis();
        if (now_ms - gw_last_bind_req_ms >= GW_BIND_REQ_STAGGER_MS) {
            for (auto it_br = gw_bind_queue.begin(); it_br != gw_bind_queue.end(); ++it_br) {
                if (now_ms >= it_br->scheduled_time) {
                    sensor_zigbee_gw_bind_device(it_br->ieee_addr, it_br->endpoint, it_br->cluster_id);
                    gw_last_bind_req_ms = now_ms;
                    gw_bind_queue.erase(it_br);
                    break;
                }
            }
        }
    }

    // Initial battery read after bind/configure-reporting (one per tick)
    if (connected) {
        gw_process_battery_read_queue();
    }

    static unsigned long last_status_print = 0;
    if (millis() - last_status_print > 60000) {
        last_status_print = millis();
        gw_refresh_link_quality();
        // DEBUG_PRINTF("[ZIGBEE-GW] Status: started=%d connected=%d devices=%d pending_reports=%d\n",
                    // Zigbee.started() ? 1 : 0, Zigbee.connected() ? 1 : 0,
                    // (int)gw_discovered_devices.size(), (int)pending_report_count);
        
        // Log registered sensors and their config for debugging
        SensorIterator it = sensors_iterate_begin();
        SensorBase* sensor;
        int zb_count = 0;
        while ((sensor = sensors_iterate_next(it)) != NULL) {
            if (sensor && sensor->type == SENSOR_ZIGBEE) {
                ZigbeeSensor* zb = static_cast<ZigbeeSensor*>(sensor);
                // DEBUG_PRINTF("[ZIGBEE-GW]   Sensor '%s': ieee=%08lX%08lX ep=%d cluster=0x%04X attr=0x%04X data_ok=%d last=%.2f\n",
                            // sensor->name,
                            // (unsigned long)(zb->device_ieee >> 32), (unsigned long)(zb->device_ieee & 0xFFFFFFFF),
                            // zb->endpoint, zb->cluster_id, zb->attribute_id,
                            // zb->flags.data_ok ? 1 : 0, zb->last_data);
                zb_count++;
            }
        }
        if (zb_count == 0) {
            // DEBUG_PRINTLN(F("[ZIGBEE-GW]   No Zigbee sensors registered!"));
        }
        
        // Dump pending reports that haven't been consumed
        for (size_t i = 0; i < pending_report_count; i++) {
            ZigbeeAttributeReport& r = pending_reports[i];
            if (!r.consumed) {
                // DEBUG_PRINTF("[ZIGBEE-GW]   Pending[%d]: ieee=%08lX%08lX cluster=0x%04X attr=0x%04X value=%ld age=%lums\n",
                            // (int)i,
                            // (unsigned long)(r.ieee_addr >> 32), (unsigned long)(r.ieee_addr & 0xFFFFFFFF),
                            // r.cluster_id, r.attr_id & ~TUYA_REPORT_FLAG_PRESCALED, r.value,
                            // millis() - r.timestamp);
            }
        }
    }
}

bool sensor_zigbee_send_on_off(uint64_t device_ieee, uint8_t endpoint, bool turnon, bool urgent, uint16_t on_time_s) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) {
        if (gw_zigbee_initialized) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Standard On/Off command failed: Zigbee not initialized or not connected"));
        }
        return false;
    }
    if (device_ieee == 0) return false;

    // Correct stale/truncated station IEEE against discovered devices before
    // attempting short-address lookup.
    gw_try_fix_ieee(&device_ieee);

    uint16_t short_addr = gw_get_short_addr(device_ieee);

    if (short_addr == 0xFFFF || short_addr == 0xFFFE) {
        if (!gw_ieee_is_known(device_ieee)) {
            if (!gw_orphan_warned(device_ieee)) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] On/Off skipped: orphan station references unknown device ieee=%016llX (remove or reassign the station)\n"),
                            (unsigned long long)device_ieee);
            }
            return false;
        }
        DEBUG_PRINTF(F("[ZIGBEE-GW] Standard On/Off command failed: short addr unknown for ieee=%016llX\n"),
                    (unsigned long long)device_ieee);
        return false;
    }

    if (!urgent && !gw_is_device_access_allowed(device_ieee, short_addr, true, 5000UL)) {
        DEBUG_PRINTF(F("[ZIGBEE-GW] Standard On/Off rate-limited (5s) for ieee=%016llX; station state machine retries.\n"),
                     (unsigned long long)device_ieee);
        return false;
    }

    uint8_t tsn;
    if (turnon && on_time_s > 0 && !gw_zcl_timed_off_is_unsupported(device_ieee)) {
        // OnWithTimedOff (ZCL On/Off 0x42): the device switches itself off after
        // on_time (1/10 s, max 6553.5 s). Longer runs are refreshed by the
        // periodic special-station refresh with the remaining time.
        uint32_t tenths = (uint32_t)on_time_s * 10UL;
        if (tenths > 0xFFFEUL) tenths = 0xFFFEUL;
        esp_zb_zcl_on_off_on_with_timed_off_cmd_t req = {};
        req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
        req.zcl_basic_cmd.dst_endpoint = endpoint;
        req.zcl_basic_cmd.src_endpoint = 10;
        req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        req.on_off_control = 0;
        req.on_time = (uint16_t)tenths;
        req.off_wait_time = 0;
        DEBUG_PRINTF(F("[ZIGBEE-GW] Sending OnWithTimedOff -> short_addr=0x%04X ep=%d on_time=%us\n"),
                     short_addr, endpoint, (unsigned)on_time_s);
        esp_zb_lock_acquire(portMAX_DELAY);
        tsn = esp_zb_zcl_on_off_on_with_timed_off_cmd_req(&req);
        esp_zb_lock_release();
        gw_last_timed_off_ieee = device_ieee;
        gw_last_timed_off_ms = millis();
    } else {
        esp_zb_zcl_on_off_cmd_t req = {};
        req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
        req.zcl_basic_cmd.dst_endpoint = endpoint;
        req.zcl_basic_cmd.src_endpoint = 10;
        req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        req.on_off_cmd_id = turnon ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;

        DEBUG_PRINTF(F("[ZIGBEE-GW] Sending On/Off command -> short_addr=0x%04X ep=%d state=%d\n"),
                     short_addr, endpoint, turnon);

        esp_zb_lock_acquire(portMAX_DELAY);
        tsn = esp_zb_zcl_on_off_cmd_req(&req);
        esp_zb_lock_release();
    }
    gw_station_ctl_note_sent(gw_ctl_sending_sid, tsn);

    gw_record_device_access(device_ieee, short_addr, true);
    return true;
}

static bool gw_resolve_tuya_short_addr(uint64_t* device_ieee, uint16_t* short_addr, const char* context) {
    if (!gw_zigbee_initialized || !Zigbee.started() || !Zigbee.connected()) {
        if (gw_zigbee_initialized) {
            DEBUG_PRINTF(F("[ZIGBEE-GW] %s failed: Zigbee not initialized or not connected\n"), context);
        }
        return false;
    }
    if (!device_ieee || *device_ieee == 0 || !short_addr) return false;

    // Correct stale/truncated station IEEE against discovered devices before
    // attempting short-address lookup.
    gw_try_fix_ieee(device_ieee);

    // Persisted device cache first (survives a reboot, the ZBOSS address table
    // may not resolve a sleepy device until it talks to us), then the stack.
    *short_addr = gw_get_short_addr(*device_ieee);

    if (*short_addr == 0xFFFF || *short_addr == 0xFFFE) {
        // Distinguish a transient lookup miss (device paired but short addr
        // not yet resolved after a fresh boot/rejoin) from an orphan IEEE
        // that refers to a device which is no longer paired. The latter is
        // typically a station entry referencing a deleted Zigbee device — in
        // that case there's nothing to wait for, so don't queue retries.
        if (!gw_ieee_is_known(*device_ieee)) {
            if (!gw_orphan_warned(*device_ieee)) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] %s skipped: orphan station references unknown device ieee=%016llX (remove or reassign the station)\n"),
                            context, (unsigned long long)*device_ieee);
            }
            return false;
        }
        // Throttled: the station state machine retries with backoff and would
        // otherwise repeat this line every few seconds per station.
        static uint64_t s_last_unknown_ieee = 0;
        static uint32_t s_last_unknown_ms = 0;
        if (*device_ieee != s_last_unknown_ieee || (millis() - s_last_unknown_ms) > 60000UL) {
            s_last_unknown_ieee = *device_ieee;
            s_last_unknown_ms = millis();
            DEBUG_PRINTF(F("[ZIGBEE-GW] %s deferred: short addr unknown for ieee=%016llX (retrying with backoff)\n"),
                        context, (unsigned long long)*device_ieee);
        }
        return false;
    }

    return true;
}

static bool gw_send_tuya_dp_raw_cmd(uint64_t device_ieee, uint8_t endpoint, uint8_t dp_id, uint8_t dp_type,
                                    const uint8_t* value, uint16_t value_len, uint8_t command_id,
                                    bool urgent) {
    uint16_t short_addr = 0;
    if (!gw_resolve_tuya_short_addr(&device_ieee, &short_addr, "Tuya DP write")) {
        return false;
    }

    // Prepare a generic Tuya dataRequest payload. This is the same logical
    // shape used by Zigbee2MQTT's sendDataPoint* helpers and works for valves,
    // switches, lights, and other Tuya MCU devices once their DP map is known.
    uint8_t payload[32];
    uint16_t tuya_seq = gw_next_tuya_seq();
    size_t payload_len = gw_build_tuya_dp_payload(payload, sizeof(payload),
                                                  tuya_seq, dp_id, dp_type,
                                                  value, value_len);
    if (payload_len == 0) {
        DEBUG_PRINTLN(F("[ZIGBEE-GW] Tuya DP write failed: payload build error"));
        return false;
    }

    DEBUG_PRINTF(F("[ZIGBEE-GW] Queueing Tuya DP set -> short_addr=0x%04X ep=%d cmd=0x%02X dp=%d type=%d len=%u seq=%u dir=TO_SRV defresp=off\n"),
                 short_addr, endpoint, command_id, dp_id, dp_type, (unsigned)value_len, (unsigned)tuya_seq);
    DEBUG_PRINTF(F("[ZIGBEE-GW] Tuya payload (%u bytes): %02X %02X %02X %02X %02X %02X %02X%s\n"),
                 (unsigned)payload_len,
                 payload[0], payload[1], payload[2], payload[3],
                 payload[4], payload[5], payload[6], payload_len > 7 ? " ..." : "");

    return gw_schedule_tuya_dp_cmd(short_addr, endpoint, command_id, dp_id, dp_type,
                                   payload, payload_len, tuya_seq, urgent);
}

// Send several datapoints in ONE Tuya dataRequest frame (what zigbee2mqtt's
// sendDataPoints does). One frame = one radio transaction, so mode, runtime
// and switch state reach a sleepy valve together instead of colliding as
// separate commands.
static bool gw_send_tuya_dp_records(uint64_t device_ieee, uint8_t endpoint,
                                    const GwTuyaDpRecord* recs, uint8_t n_recs, bool urgent) {
    if (!recs || n_recs == 0) return false;
    uint16_t short_addr = 0;
    if (!gw_resolve_tuya_short_addr(&device_ieee, &short_addr, "Tuya DP multi-write")) return false;

    uint8_t payload[32];
    uint16_t tuya_seq = gw_next_tuya_seq();
    size_t len = 0;
    payload[len++] = (uint8_t)(tuya_seq >> 8);
    payload[len++] = (uint8_t)(tuya_seq & 0xFF);
    for (uint8_t i = 0; i < n_recs; i++) {
        const GwTuyaDpRecord& r = recs[i];
        if (r.len != 1 && r.len != 4) return false;
        if (len + 4U + r.len > sizeof(payload)) {
            DEBUG_PRINTLN(F("[ZIGBEE-GW] Tuya multi-DP frame too large"));
            return false;
        }
        payload[len++] = r.dp_id;
        payload[len++] = r.dp_type;
        payload[len++] = 0;
        payload[len++] = r.len;
        if (r.len == 4) {
            payload[len++] = (uint8_t)(r.value >> 24);
            payload[len++] = (uint8_t)(r.value >> 16);
            payload[len++] = (uint8_t)(r.value >> 8);
        }
        payload[len++] = (uint8_t)(r.value & 0xFF);
    }
    DEBUG_PRINTF(F("[ZIGBEE-GW] Queueing Tuya multi-DP frame -> short_addr=0x%04X ep=%d records=%u len=%u seq=%u\n"),
                 short_addr, endpoint, (unsigned)n_recs, (unsigned)len, (unsigned)tuya_seq);
    return gw_schedule_tuya_dp_cmd(short_addr, endpoint, TUYA_CMD_DATA_REQUEST,
                                   recs[n_recs - 1].dp_id, recs[n_recs - 1].dp_type,
                                   payload, len, tuya_seq, urgent);
}

static bool gw_send_tuya_dp_bool_write_cmd(uint64_t device_ieee, uint8_t endpoint, uint8_t dp_id, bool turnon,
                                           uint8_t command_id, bool urgent) {
    uint8_t bool_value[1] = { (uint8_t)(turnon ? 0x01 : 0x00) };
    return gw_send_tuya_dp_raw_cmd(device_ieee, endpoint, dp_id, TUYA_TYPE_BOOL,
                                   bool_value, sizeof(bool_value), command_id, urgent);
}

static bool gw_send_tuya_dp_value_write_cmd(uint64_t device_ieee, uint8_t endpoint, uint8_t dp_id, uint32_t value,
                                            uint8_t command_id, bool urgent) {
    uint8_t value_bytes[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF) 
    };
    return gw_send_tuya_dp_raw_cmd(device_ieee, endpoint, dp_id, TUYA_TYPE_VALUE,
                                   value_bytes, sizeof(value_bytes), command_id, urgent);
}

bool sensor_zigbee_send_tuya_dp_write(uint64_t device_ieee, uint8_t endpoint, uint8_t dp_id, bool turnon, bool urgent) {
    return gw_send_tuya_dp_bool_write_cmd(device_ieee, endpoint, dp_id, turnon, TUYA_CMD_DATA_REQUEST, urgent);
}

bool sensor_zigbee_send_tuya_dp_value_write(uint64_t device_ieee, uint8_t endpoint, uint8_t dp_id, uint32_t value, bool urgent) {
    return gw_send_tuya_dp_value_write_cmd(device_ieee, endpoint, dp_id, value, TUYA_CMD_DATA_REQUEST, urgent);
}

// Resolve the state DP of a GIEX valve from its logical devices: dp_id 0/1
// means "the first valve" (valve_1 / valve), otherwise the DP is used as is.
static uint8_t gw_giex_resolve_state_dp(uint64_t device_ieee, uint8_t dp_id) {
    if (dp_id != 0 && dp_id != 1) return dp_id;
    char ieee_str[17];
    snprintf(ieee_str, sizeof(ieee_str), "%016llX", (unsigned long long)device_ieee);
    ZigBeeLogicalDevice* log_valve = OpenSprinkler::zigbee_logical_lookup(ieee_str, "valve_1");
    if (!log_valve) log_valve = OpenSprinkler::zigbee_logical_lookup(ieee_str, "valve");
    if (log_valve && log_valve->tuya_dp_value > 0) return (uint8_t)log_valve->tuya_dp_value;
    return 2; // default for single-zone GIEX valves
}

bool sensor_zigbee_send_giex_water_valve_state_with_dur(uint64_t device_ieee, uint8_t endpoint, bool turnon, uint16_t dur, uint8_t dp_id, bool urgent) {
    // The runtime (dur) is written by the station state machine, which knows
    // the device's runtime DP from the DB template; this plain entry point
    // only switches the valve.
    (void)dur;
    uint8_t active_dp = gw_giex_resolve_state_dp(device_ieee, dp_id);
    return gw_send_tuya_dp_bool_write_cmd(device_ieee, endpoint, active_dp, turnon, TUYA_CMD_DATA_REQUEST, urgent);
}

bool sensor_zigbee_send_giex_water_valve_state(uint64_t device_ieee, uint8_t endpoint, bool turnon) {
    return sensor_zigbee_send_giex_water_valve_state_with_dur(device_ieee, endpoint, turnon, 0, 0, false);
}

void sensor_zigbee_station_command(uint8_t sid, uint64_t ieee, uint8_t endpoint, uint8_t dp_id, uint8_t verify_dp,
                                   bool turnon, uint8_t ctrl_type, uint16_t dur,
                                   const ZigbeeStationControlConfig* cfg) {
    if (sid >= MAX_NUM_STATIONS) return;
    ZbStationCtl &e = zb_station_ctl[sid];
    uint32_t now = millis();
    e.dp_runtime   = cfg ? cfg->dp_runtime : 0;
    e.runtime_unit = cfg ? cfg->runtime_unit : 0;
    e.runtime_max  = cfg ? cfg->runtime_max : 0;
    e.prereq_dp    = cfg ? cfg->prereq_dp : 0;
    e.prereq_value = cfg ? cfg->prereq_value : 0;

    // A frame for this station may still be in flight inside the stack (no
    // send status yet, up to ~60 s for a sleepy child).  Sending the new state
    // now would put two commands into the indirect queue and the valve would
    // receive both in a burst on its next poll.  Defer instead: the new state
    // goes out as soon as the in-flight frame is acked, reported failed or
    // timed out.  Nothing is lost by waiting: while the frame is pending the
    // device has not polled, so it could not have taken the new command either.
    bool in_flight = e.active && e.ieee == ieee && e.attempts > 0 && !e.delivered &&
                     !e.last_send_failed && (now - e.last_sent_ms) < ZB_CTL_INFLIGHT_MAX_MS;
    if (in_flight) {
        e.desired_on = turnon;
        e.dur        = dur;
        e.requested_ms = now;
        if (e.inflight_on == turnon) {
            e.resend_needed = false;   // identical command already on its way
        } else {
            e.resend_needed = true;
            e.next_try_ms  = now + 1000UL;
        }
        DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u %s deferred: frame (on=%d) still in flight for %lums\n"),
                     (unsigned)sid, turnon ? "ON" : "OFF", e.inflight_on ? 1 : 0,
                     (unsigned long)(now - e.last_sent_ms));
        return;
    }

    // Latest command wins: an OFF always replaces a still-pending ON (and vice
    // versa), so a command can never be lost behind an older one.
    e.active        = true;
    e.desired_on    = turnon;
    e.ctrl_type     = ctrl_type;
    e.endpoint      = endpoint ? endpoint : 1;
    e.dp_id         = dp_id;
    e.verify_dp     = verify_dp;
    e.ieee          = ieee;
    e.dur           = dur;
    e.attempts      = 0;
    e.send_fails    = 0;
    e.have_tsn      = false;
    e.delivered     = false;
    e.last_send_failed = false;
    e.resend_needed = false;
    e.error_flagged = false;
    e.requested_ms  = millis();
    e.last_sent_ms  = 0;
    e.next_try_ms   = millis();

    // Status semantics for the UI (zst): 1 = command pending, 3/4 = state
    // confirmed by the device (DP echo / APS ack), 2 = failed, 0 = idle.
    // physical_on is therefore NOT set optimistically here; only
    // gw_station_ctl_confirm() / a device report changes it.
    zb_station_switch_error[sid] = 0;

    gw_station_ctl_send(sid, false);
}

uint8_t sensor_zigbee_station_status_code(uint8_t sid) {
    if (sid >= MAX_NUM_STATIONS) return 0;
    if (zb_station_switch_error[sid]) return 2;
    if (zb_station_switch_waiting[sid] || zb_station_ctl[sid].active) {
        return zb_station_physical_on[sid] ? 4 : 1;
    }
    return zb_station_physical_on[sid] ? 3 : 0;
}

void sensor_zigbee_station_clear_error(uint8_t sid) {
    gw_station_switch_error_set(sid, false);
    gw_station_switch_waiting_set(sid, false);
}

void sensor_zigbee_station_verify_tick() {
    uint32_t now = millis();
    for (uint8_t sid = 0; sid < MAX_NUM_STATIONS; sid++) {
        ZbStationCtl &e = zb_station_ctl[sid];
        if (!e.active) continue;
        if (sid >= os.nstations || os.get_station_type(sid) != STN_TYPE_ZIGBEE || e.ieee == 0) {
            e.active = false;  // stale / foreign entry: never act on it
            continue;
        }
        if ((int32_t)(now - e.next_try_ms) < 0) continue;

        // Standard ZCL device on a stack that never reports send status:
        // fall back to the former optimistic behaviour after a short grace.
        if (e.ctrl_type == 0 && e.attempts > 0 && !gw_send_status_supported &&
            (now - e.last_sent_ms) >= ZB_CTL_ZCL_LEGACY_CONFIRM_MS) {
            gw_station_ctl_confirm(sid, "legacy/no send status");
            continue;
        }

        // Alert once when nothing confirmed the switch within the timeout
        // (measured from the request).  ON gives up; OFF keeps trying.
        if (!e.error_flagged && (now - e.requested_ms) >= ZB_CTL_CONFIRM_TIMEOUT_MS) {
            e.error_flagged = true;
            DEBUG_PRINTF(F("[ZIGBEE-GW] Switch not confirmed within %lus: sid=%u dp=%u expected_on=%d sends=%u delivered=%d\n"),
                         (unsigned long)(ZB_CTL_CONFIRM_TIMEOUT_MS / 1000UL), (unsigned)sid,
                         (unsigned)e.verify_dp, e.desired_on ? 1 : 0, (unsigned)e.attempts, e.delivered ? 1 : 0);
            gw_station_verify_publish_fail(sid, e.desired_on, true);
            if (e.desired_on) {
                e.active = false;
                continue;
            }
        }

        // One frame in flight at a time: re-send only after the stack reported
        // the previous one as not delivered (or nothing was sent yet).
        if (e.attempts > 0 && !e.last_send_failed) {
            if (e.resend_needed && (now - e.last_sent_ms) >= ZB_CTL_INFLIGHT_MAX_MS) {
                DEBUG_PRINTF(F("[ZIGBEE-GW] Station sid=%u in-flight frame timed out -> sending deferred %s\n"),
                             (unsigned)sid, e.desired_on ? "ON" : "OFF");
                gw_station_ctl_send(sid, false);
                continue;
            }
            e.next_try_ms = now + 5000UL;  // re-evaluate timeout / wait for send status
            continue;
        }
        if (e.attempts >= ZB_CTL_MAX_SENDS_TOTAL) {
            e.next_try_ms = now + 60000UL; // cap reached: wake trigger only
            continue;
        }

        gw_station_ctl_send(sid, false);
    }
}

void sensor_zigbee_gw_refresh_reporting(uint64_t ieee) {
    if (ieee == 0 || !gw_zigbee_initialized) return;
    gw_schedule_configure_reporting_for_ieee(ieee, 500UL);
}

void sensor_zigbee_gw_force_off_all_stations() {
    if (os.nstations == 0) return;

    for (uint8_t sid = 0; sid < os.nstations; sid++) {
        uint8_t bid = sid >> 3;
        uint8_t s = sid & 0x07;
        if (!(os.attrib_spe[bid] & (1 << s))) continue;
        if (os.get_station_type(sid) != STN_TYPE_ZIGBEE) continue;

        StationData* pdata = (StationData*)tmp_buffer;
        os.get_station_data(sid, pdata);
        os.switch_zigbeestation((ZigbeeStationData*)pdata->sped, false, sid);
    }
}

#endif // ESP32C5 && OS_ENABLE_ZIGBEE
