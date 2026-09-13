/* sensor_zigbee_common.h - helpers shared by the Zigbee client (sensor_zigbee.cpp)
 * and the Zigbee gateway/coordinator (sensor_zigbee_gw.cpp).
 *
 * Header-only (static inline) so both translation units share one definition.
 * Include it after the ESP Zigbee headers (esp_zb_zcl_attribute_t) and after
 * the TUYA_* / TUYA_REPORT_* defines of the including file.
 */
#ifndef _SENSOR_ZIGBEE_COMMON_H
#define _SENSOR_ZIGBEE_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Tuya DP data types (cluster 0xEF00 payload)
#define TUYA_TYPE_RAW    0x00
#define TUYA_TYPE_BOOL   0x01
#define TUYA_TYPE_VALUE  0x02  // 4-byte big-endian integer
#define TUYA_TYPE_STRING 0x03
#define TUYA_TYPE_ENUM   0x04
#define TUYA_TYPE_BITMAP 0x05

// Reports originating from Tuya DP parsing carry the DP number in the low byte,
// the DP type in bits 8..11 and this flag (value is already scaled).
#define TUYA_REPORT_FLAG_PRESCALED     0x8000
#define TUYA_REPORT_TYPE_SHIFT         8
#define TUYA_REPORT_TYPE_MASK          0x0F00
#define TUYA_REPORT_DP_MASK            0x00FF

static inline uint16_t tuya_report_attr(uint8_t dp_number, uint8_t dp_type) {
    return TUYA_REPORT_FLAG_PRESCALED |
           (((uint16_t)dp_type << TUYA_REPORT_TYPE_SHIFT) & TUYA_REPORT_TYPE_MASK) |
           (uint16_t)dp_number;
}

static inline uint16_t zigbee_report_attr_id(uint16_t attr_id) {
    return (attr_id & TUYA_REPORT_FLAG_PRESCALED) ? (attr_id & TUYA_REPORT_DP_MASK) : attr_id;
}

static inline uint8_t tuya_report_type(uint16_t attr_id) {
    return (attr_id & TUYA_REPORT_FLAG_PRESCALED) ? (uint8_t)((attr_id & TUYA_REPORT_TYPE_MASK) >> TUYA_REPORT_TYPE_SHIFT) : 0;
}

#define ZB_BATTERY_UNKNOWN 255U  // sentinel: no usable battery level in this report

static inline uint32_t zigbee_battery_percent_from_report(bool is_tuya_report, uint16_t raw_attr_id, uint8_t tuya_type, int16_t configured_tuya_battery_dp, int32_t value) {
    if (is_tuya_report) {
        // DP 14 is specifically "battery_state" (ENUM): 0=normal/full, 1=low
        if (raw_attr_id == 14) {
            if (value == 0) return 100;
            return 15;
        }

        // DP 59 is GX03 battery (typically percentage 0-100 or enum/steps)
        if (raw_attr_id == 59) {
            if (value > 4 && value <= 100) return (uint32_t)value;
            if (value == 4) return 100;
            if (value == 3) return 75;
            if (value == 2) return 50;
            if (value == 1) return 25;
            if (value == 0) return 100; // default to 100 on communicating device
        }

        if (raw_attr_id == 15 || raw_attr_id == 108 || raw_attr_id == 115 || raw_attr_id == 18 || 
            (configured_tuya_battery_dp >= 0 && raw_attr_id == (uint16_t)configured_tuya_battery_dp)) {
            
            if (tuya_type == TUYA_TYPE_ENUM) {
                // 3-state enum: 0=high/normal, 1=medium, 2=low
                // OR 3-state: 0=low, 1=medium, 2=high.
                if (value == 0) return 100;
                if (value == 1) return 50;
                if (value == 2 || value == 3) return 100;
                return 100;
            }

            if (value <= 0) {
                return 100; // active device with 0 is uninitialized or inverted full
            }
            return (value > 100) ? 100 : (uint32_t)value;
        }

        if (value < 0) return 0;
        return (value > 100) ? 100 : (uint32_t)value;
    }

    // ZCL BatteryPercentageRemaining (0x0021): half-percent units, 0xFF =
    // invalid.  A raw 0 is what devices answer right after (re)joining before
    // their first measurement (a device at a real 0 % could not transmit), so
    // both are reported as "unknown" and must not overwrite a known level.
    if (value <= 0 || value == 0xFF) return ZB_BATTERY_UNKNOWN;
    uint32_t batt_pct = (uint32_t)(value / 2);
    return (batt_pct > 100) ? 100 : batt_pct;
}

/** Extract a ZCL CHAR_STRING / LONG_CHAR_STRING attribute into a C string.
 *  Returns true if a non-empty string was copied. */
static inline bool zigbee_extract_string_attribute(const esp_zb_zcl_attribute_t *attr, char *buf, size_t buf_size) {
    if (!attr || !attr->data.value || buf_size == 0) return false;
    
    if (attr->data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING) {
        // ZCL CHAR_STRING: [length_byte][chars...] (NOT null-terminated)
        uint8_t *raw = (uint8_t*)attr->data.value;
        uint8_t len = raw[0];
        if (len == 0xFF) { buf[0] = '\0'; return false; }  // 0xFF = invalid
        if (len >= buf_size) len = buf_size - 1;
        memcpy(buf, raw + 1, len);
        buf[len] = '\0';
        return len > 0;
    } else if (attr->data.type == ESP_ZB_ZCL_ATTR_TYPE_LONG_CHAR_STRING) {
        // ZCL LONG_CHAR_STRING: [length_u16_le][chars...]
        uint8_t *raw = (uint8_t*)attr->data.value;
        uint16_t len = raw[0] | ((uint16_t)raw[1] << 8);
        if (len == 0xFFFF) { buf[0] = '\0'; return false; }
        if (len >= buf_size) len = buf_size - 1;
        memcpy(buf, raw + 2, len);
        buf[len] = '\0';
        return len > 0;
    }
    return false;
}

/** Read a numeric ZCL attribute value as int32 (0 for unsupported types). */
static inline int32_t zigbee_extract_attribute_value(const esp_zb_zcl_attribute_t *attr) {
    if (!attr || !attr->data.value) return 0;
    switch (attr->data.type) {
        case ESP_ZB_ZCL_ATTR_TYPE_S8:  return (int32_t)(*(int8_t*)attr->data.value);
        case ESP_ZB_ZCL_ATTR_TYPE_S16: return (int32_t)(*(int16_t*)attr->data.value);
        case ESP_ZB_ZCL_ATTR_TYPE_S32: return *(int32_t*)attr->data.value;
        case ESP_ZB_ZCL_ATTR_TYPE_U8:  return (int32_t)(*(uint8_t*)attr->data.value);
        case ESP_ZB_ZCL_ATTR_TYPE_U16: return (int32_t)(*(uint16_t*)attr->data.value);
        case ESP_ZB_ZCL_ATTR_TYPE_U32: return (int32_t)(*(uint32_t*)attr->data.value);
        case 0x24:  // uint40
        case 0x25: { // uint48
            uint8_t len = (attr->data.type == 0x24) ? 5 : 6;
            const uint8_t* raw = (const uint8_t*)attr->data.value;
            uint32_t value = 0;
            for (uint8_t i = 0; i < len && i < 4; i++) {
                value |= ((uint32_t)raw[i]) << (8 * i);
            }
            if (value > 0x7FFFFFFFUL) value = 0x7FFFFFFFUL;
            return (int32_t)value;
        }
        default:
            // DEBUG_PRINTF(F("[ZIGBEE-CLIENT] Unknown attribute type: 0x%02X\n"), attr->data.type);
            return 0;
    }
}

#endif // _SENSOR_ZIGBEE_COMMON_H
