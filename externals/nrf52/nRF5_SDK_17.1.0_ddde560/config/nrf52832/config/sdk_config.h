/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
 * Copyright 2022 Sony Semiconductor Solutions Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */



#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

#include "nrf_porting.h"

// <q> NRF_STRERROR_ENABLED  - nrf_strerror - Library for converting error code to string.

#ifndef NRF_STRERROR_ENABLED
#define NRF_STRERROR_ENABLED 1
#endif

// <h> nRF_SoftDevice 

//==========================================================
// <e> NRF_SDH_BLE_ENABLED - nrf_sdh_ble - SoftDevice BLE event handler
//==========================================================
#ifndef NRF_SDH_BLE_ENABLED
#define NRF_SDH_BLE_ENABLED 1
#endif
// <h> BLE Stack configuration - Stack configuration parameters

// <i> The SoftDevice handler will configure the stack with these parameters when calling @ref nrf_sdh_ble_default_cfg_set.
// <i> Other libraries might depend on these values; keep them up-to-date even if you are not explicitely calling @ref nrf_sdh_ble_default_cfg_set.
//==========================================================
// <o> NRF_SDH_BLE_GAP_DATA_LENGTH   <27-251> 


// <i> Requested BLE GAP data length to be negotiated.

//#ifndef NRF_SDH_BLE_GAP_DATA_LENGTH
//#define NRF_SDH_BLE_GAP_DATA_LENGTH 27
//#endif

// <o> NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - Maximum number of peripheral links. 
#ifndef NRF_SDH_BLE_PERIPHERAL_LINK_COUNT
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 5 //TBD
#endif

// <o> NRF_SDH_BLE_CENTRAL_LINK_COUNT - Maximum number of central links. 
#ifndef NRF_SDH_BLE_CENTRAL_LINK_COUNT
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT 15 //TBD
#endif

// <o> NRF_SDH_BLE_TOTAL_LINK_COUNT - Total link count. 
// <i> Maximum number of total concurrent connections using the default configuration.

#ifndef NRF_SDH_BLE_TOTAL_LINK_COUNT
#define NRF_SDH_BLE_TOTAL_LINK_COUNT 1 //TBD
#endif

// <o> NRF_SDH_BLE_GAP_EVENT_LENGTH - GAP event length. 
// <i> The time set aside for this connection on every connection interval in 1.25 ms units.

#ifndef NRF_SDH_BLE_GAP_EVENT_LENGTH
#define NRF_SDH_BLE_GAP_EVENT_LENGTH 6
#endif

// <o> NRF_SDH_BLE_GATT_MAX_MTU_SIZE - Static maximum MTU size. 
#ifndef NRF_SDH_BLE_GATT_MAX_MTU_SIZE
#define NRF_SDH_BLE_GATT_MAX_MTU_SIZE 247
#endif

// <o> NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE - Attribute Table size in bytes. The size must be a multiple of 4. 
#ifndef NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE
#define NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE 1408
#endif

// <o> NRF_SDH_BLE_VS_UUID_COUNT - The number of vendor-specific UUIDs. 
#ifndef NRF_SDH_BLE_VS_UUID_COUNT
#define NRF_SDH_BLE_VS_UUID_COUNT 20
#endif

// <q> NRF_SDH_BLE_SERVICE_CHANGED  - Include the Service Changed characteristic in the Attribute Table.
 

#ifndef NRF_SDH_BLE_SERVICE_CHANGED
#define NRF_SDH_BLE_SERVICE_CHANGED 0
#endif

// </h>
//==========================================================

// <h> BLE Observers - Observers and priority levels

//==========================================================
// <o> NRF_SDH_BLE_OBSERVER_PRIO_LEVELS - Total number of priority levels for BLE observers. 
// <i> This setting configures the number of priority levels available for BLE event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_BLE_OBSERVER_PRIO_LEVELS
#define NRF_SDH_BLE_OBSERVER_PRIO_LEVELS 4
#endif
//==========================================================

// </h>
//==========================================================

// </e>

// <e> NRF_SDH_ENABLED - nrf_sdh - SoftDevice handler
//==========================================================
#ifndef NRF_SDH_ENABLED
#define NRF_SDH_ENABLED 1
#endif

// </e>

// <q> NRF_QUEUE_ENABLED  - nrf_queue - Queue module

#ifndef NRF_QUEUE_ENABLED
#define NRF_QUEUE_ENABLED 1
#endif

// Build warnings

#ifndef NRF_LOG_ENABLED
#define NRF_LOG_ENABLED 0
#endif

#ifndef NRF_LOG_FILTERS_ENABLED
#define NRF_LOG_FILTERS_ENABLED 0
#endif

#ifndef NRF_QUEUE_CLI_CMDS
#define NRF_QUEUE_CLI_CMDS 0
#endif


// <q> NRF_SECTION_ITER_ENABLED  - nrf_section_iter - Section iterator
 

#ifndef NRF_SECTION_ITER_ENABLED
#define NRF_SECTION_ITER_ENABLED 1
#endif

// <o> NRF_LOG_DEFAULT_LEVEL  - Default Severity level

// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug

#ifndef NRF_LOG_DEFAULT_LEVEL
#define NRF_LOG_DEFAULT_LEVEL 3
#endif

// <e> NRF_QUEUE_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRF_QUEUE_CONFIG_LOG_ENABLED
#define NRF_QUEUE_CONFIG_LOG_ENABLED 0
#endif
// <o> NRF_QUEUE_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_QUEUE_CONFIG_LOG_LEVEL
#define NRF_QUEUE_CONFIG_LOG_LEVEL 3
#endif

// <o> NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL
#define NRF_QUEUE_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NRF_QUEUE_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_QUEUE_CONFIG_INFO_COLOR
#define NRF_QUEUE_CONFIG_INFO_COLOR 0
#endif

// <o> NRF_QUEUE_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_QUEUE_CONFIG_DEBUG_COLOR
#define NRF_QUEUE_CONFIG_DEBUG_COLOR 0
#endif

// <e> NRF_SDH_BLE_LOG_ENABLED - Enable logging in SoftDevice handler (BLE) module.
//==========================================================
#ifndef NRF_SDH_BLE_LOG_ENABLED
#define NRF_SDH_BLE_LOG_ENABLED 1
#endif
// <o> NRF_SDH_BLE_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRF_SDH_BLE_LOG_LEVEL
#define NRF_SDH_BLE_LOG_LEVEL 3
#endif

// <o> NRF_SDH_BLE_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_BLE_INFO_COLOR
#define NRF_SDH_BLE_INFO_COLOR 0
#endif

// <o> NRF_SDH_BLE_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRF_SDH_BLE_DEBUG_COLOR
#define NRF_SDH_BLE_DEBUG_COLOR 0
#endif

//==========================================================
// <e> SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED
#define SER_HAL_TRANSPORT_CONFIG_LOG_ENABLED 0
#endif
// <o> SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL
#define SER_HAL_TRANSPORT_CONFIG_LOG_LEVEL 3
#endif

// <o> SER_HAL_TRANSPORT_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SER_HAL_TRANSPORT_CONFIG_INFO_COLOR
#define SER_HAL_TRANSPORT_CONFIG_INFO_COLOR 0
#endif

// <o> SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR
#define SER_HAL_TRANSPORT_CONFIG_DEBUG_COLOR 0
#endif

//==========================================================
// <e> SER_SD_TRANSPORT_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SER_SD_TRANSPORT_CONFIG_LOG_ENABLED
#define SER_SD_TRANSPORT_CONFIG_LOG_ENABLED 0
#endif
// <o> SER_SD_TRANSPORT_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SER_SD_TRANSPORT_CONFIG_LOG_LEVEL
#define SER_SD_TRANSPORT_CONFIG_LOG_LEVEL 3
#endif

//==========================================================
// <e> SER_SD_HANDLER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SER_SD_HANDLER_CONFIG_LOG_ENABLED
#define SER_SD_HANDLER_CONFIG_LOG_ENABLED 0
#endif
// <o> SER_SD_HANDLER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SER_SD_HANDLER_CONFIG_LOG_LEVEL
#define SER_SD_HANDLER_CONFIG_LOG_LEVEL 3
#endif

//==========================================================
// <e> SER_SD_HANDLER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef SER_PHY_UART_CONFIG_LOG_ENABLED
#define SER_PHY_UART_CONFIG_LOG_ENABLED 0
#endif
// <o> SER_PHY_UART_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef SER_PHY_UART_CONFIG_LOG_LEVEL
#define SER_PHY_UART_CONFIG_LOG_LEVEL 3
#endif

// </h> 
//==========================================================

// <h> SDH Observers - Observers and priority levels

//==========================================================
// <o> NRF_SDH_REQ_OBSERVER_PRIO_LEVELS - Total number of priority levels for request observers. 
// <i> This setting configures the number of priority levels available for the SoftDevice request event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_REQ_OBSERVER_PRIO_LEVELS
#define NRF_SDH_REQ_OBSERVER_PRIO_LEVELS 2
#endif

// <o> NRF_SDH_STATE_OBSERVER_PRIO_LEVELS - Total number of priority levels for state observers. 
// <i> This setting configures the number of priority levels available for the SoftDevice state event handlers.
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_STATE_OBSERVER_PRIO_LEVELS
#define NRF_SDH_STATE_OBSERVER_PRIO_LEVELS 2
#endif

// <o> NRF_SDH_STACK_OBSERVER_PRIO_LEVELS - Total number of priority levels for stack event observers. 
// <i> This setting configures the number of priority levels available for the SoftDevice stack event handlers (ANT, BLE, SoC).
// <i> The priority level of a handler determines the order in which it receives events, with respect to other handlers.

#ifndef NRF_SDH_STACK_OBSERVER_PRIO_LEVELS
#define NRF_SDH_STACK_OBSERVER_PRIO_LEVELS 2
#endif

//==========================================================

// <h> Stack Event Observers priorities - Invididual priorities

//==========================================================
// <o> NRF_SDH_ANT_STACK_OBSERVER_PRIO  
// <i> This setting configures the priority with which ANT events are processed with respect to other events coming from the stack.
// <i> Modify this setting if you need to have ANT events dispatched before or after other stack events, such as BLE or SoC.
// <i> Zero is the highest priority.

#ifndef NRF_SDH_ANT_STACK_OBSERVER_PRIO
#define NRF_SDH_ANT_STACK_OBSERVER_PRIO 0
#endif

// <o> NRF_SDH_BLE_STACK_OBSERVER_PRIO  
// <i> This setting configures the priority with which BLE events are processed with respect to other events coming from the stack.
// <i> Modify this setting if you need to have BLE events dispatched before or after other stack events, such as ANT or SoC.
// <i> Zero is the highest priority.

#ifndef NRF_SDH_BLE_STACK_OBSERVER_PRIO
#define NRF_SDH_BLE_STACK_OBSERVER_PRIO 0
#endif

// <o> NRF_SDH_SOC_STACK_OBSERVER_PRIO  
// <i> This setting configures the priority with which SoC events are processed with respect to other events coming from the stack.
// <i> Modify this setting if you need to have SoC events dispatched before or after other stack events, such as ANT or BLE.
// <i> Zero is the highest priority.

#ifndef NRF_SDH_SOC_STACK_OBSERVER_PRIO
#define NRF_SDH_SOC_STACK_OBSERVER_PRIO 0
#endif

// <h> nRF_Crypto

//==========================================================
// <e> NRF_CRYPTO_ENABLED - nrf_crypto - Cryptography library.
//==========================================================
#ifndef NRF_CRYPTO_ENABLED
#define NRF_CRYPTO_ENABLED 1
#endif
// <o> NRF_CRYPTO_ALLOCATOR  - Memory allocator

// <i> Choose memory allocator used by nrf_crypto. Default is alloca if possible or nrf_malloc otherwise. If 'User macros' are selected, the user has to create 'nrf_crypto_allocator.h' file that contains NRF_CRYPTO_ALLOC, NRF_CRYPTO_FREE, and NRF_CRYPTO_ALLOC_ON_STACK.
// <0=> Default
// <1=> User macros
// <2=> On stack (alloca)
// <3=> C dynamic memory (malloc)
// <4=> SDK Memory Manager (nrf_malloc)

#ifndef NRF_CRYPTO_ALLOCATOR
#define NRF_CRYPTO_ALLOCATOR 3
#endif

// </e>

// <e> NRF_CRYPTO_BACKEND_MBEDTLS_ENABLED - Enable the mbed TLS backend.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ENABLED NRF52_CRYPTO_BACKEND_MBEDTLS
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192R1_ENABLED  - Enable secp192r1 (NIST 192-bit) curve

// <i> Enable this setting if you need secp192r1 (NIST 192-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224R1_ENABLED  - Enable secp224r1 (NIST 224-bit) curve

// <i> Enable this setting if you need secp224r1 (NIST 224-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256R1_ENABLED  - Enable secp256r1 (NIST 256-bit) curve

// <i> Enable this setting if you need secp256r1 (NIST 256-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256R1_ENABLED NRF52_CRYPTO_BACKEND_MBEDTLS
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP384R1_ENABLED  - Enable secp384r1 (NIST 384-bit) curve

// <i> Enable this setting if you need secp384r1 (NIST 384-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP384R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP384R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP521R1_ENABLED  - Enable secp521r1 (NIST 521-bit) curve

// <i> Enable this setting if you need secp521r1 (NIST 521-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP521R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP521R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192K1_ENABLED  - Enable secp192k1 (Koblitz 192-bit) curve

// <i> Enable this setting if you need secp192k1 (Koblitz 192-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192K1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP192K1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224K1_ENABLED  - Enable secp224k1 (Koblitz 224-bit) curve

// <i> Enable this setting if you need secp224k1 (Koblitz 224-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224K1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP224K1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256K1_ENABLED  - Enable secp256k1 (Koblitz 256-bit) curve

// <i> Enable this setting if you need secp256k1 (Koblitz 256-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256K1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_SECP256K1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP256R1_ENABLED  - Enable bp256r1 (Brainpool 256-bit) curve

// <i> Enable this setting if you need bp256r1 (Brainpool 256-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP256R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP384R1_ENABLED  - Enable bp384r1 (Brainpool 384-bit) curve

// <i> Enable this setting if you need bp384r1 (Brainpool 384-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP384R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP384R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP512R1_ENABLED  - Enable bp512r1 (Brainpool 512-bit) curve

// <i> Enable this setting if you need bp512r1 (Brainpool 512-bit) support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP512R1_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_BP512R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_ECC_CURVE25519_ENABLED  - Enable Curve25519 curve

// <i> Enable this setting if you need Curve25519 support using MBEDTLS

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_ECC_CURVE25519_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_ECC_CURVE25519_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA256_ENABLED  - Enable mbed TLS SHA-256 hash functionality.

// <i> mbed TLS backend implementation for SHA-256.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA256_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA512_ENABLED  - Enable mbed TLS SHA-512 hash functionality.

// <i> mbed TLS backend implementation for SHA-512.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HASH_SHA512_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA256_ENABLED  - Enable mbed TLS HMAC using SHA-256.

// <i> mbed TLS backend implementation for HMAC using SHA-256.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA256_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA256_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA512_ENABLED  - Enable mbed TLS HMAC using SHA-512.

// <i> mbed TLS backend implementation for HMAC using SHA-512.

#ifndef NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA512_ENABLED
#define NRF_CRYPTO_BACKEND_MBEDTLS_HMAC_SHA512_ENABLED 0
#endif

// <e> NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED - Enable the micro-ecc backend.
//==========================================================
#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED NRF52_CRYPTO_BACKEND_MICRO_ECC
#endif
// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED  - Enable secp192r1 (NIST 192-bit) curve

// <i> Enable this setting if you need secp192r1 (NIST 192-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP224R1_ENABLED  - Enable secp224r1 (NIST 224-bit) curve

// <i> Enable this setting if you need secp224r1 (NIST 224-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP224R1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP224R1_ENABLED 0
#endif

// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED  - Enable secp256r1 (NIST 256-bit) curve

// <i> Enable this setting if you need secp256r1 (NIST 256-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED NRF52_CRYPTO_BACKEND_MICRO_ECC
#endif

// <q> NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256K1_ENABLED  - Enable secp256k1 (Koblitz 256-bit) curve

// <i> Enable this setting if you need secp256k1 (Koblitz 256-bit) support using micro-ecc

#ifndef NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256K1_ENABLED
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256K1_ENABLED 0
#endif
// </e>

// <<< end of configuration section >>>
#endif //SDK_CONFIG_H

