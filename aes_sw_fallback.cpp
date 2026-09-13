/**
 * AES Software Fallback Wrapper
 * 
 * This file provides a wrapper that intercepts mbedTLS AES functions
 * and forces software implementation when internal RAM is low.
 * 
 * Problem: ESP32-C5 Hardware-AES uses DMA which requires internal RAM (MALLOC_CAP_DMA).
 * With Matter enabled, internal heap gets too low and AES fails with:
 * "E (17451) esp-aes: Failed to allocate memory"
 * 
 * Solution: Check available internal heap BEFORE calling hardware AES.
 * If less than threshold, use mbedTLS software AES instead.
 * 
 * Build: Compile this file and link BEFORE libmbedcrypto.a
 * 
 * Author: Copilot for OpenSprinkler ESP32-C5
 */

#if defined(ESP32) || defined(ESP_PLATFORM)

#include "sdkconfig.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

/**
 * Unfortunately we cannot easily override the esp_aes functions as they are
 * not declared weak in the ESP-IDF mbedTLS port.
 * 
 * The best solution is to rebuild libmbedcrypto.a with:
 * CONFIG_MBEDTLS_HARDWARE_AES=n
 * 
 * Or use CONFIG_MBEDTLS_AES_HW_SMALL_DATA_LEN_OPTIM=y with a very high threshold
 * 
 * This file serves as documentation of the issue.
 * 
 * WORKAROUND: Modify esp-idf/components/mbedtls/port/aes/dma/esp_aes_dma_core.c
 * to check heap before allocation and return error if insufficient.
 * The caller (Matter) should then fall back to software AES.
 */

#endif // ESP32 || ESP_PLATFORM
