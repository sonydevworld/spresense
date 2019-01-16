/****************************************************************************
 * modules/include/bluetooth/hal/bt_if.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bt_if.h
 * @author Sony Semiconductor Solutions Corporation
 * @brief Bluetooth HAL I/F.
 * @details This header file includes bluetooth HAL I/F definitions.
 *           - HAL I/F
 *           - HAL callback function
 */

#ifndef __MODULES_INCLUDE_BLUETOOTH_HAL_BT_IF_H
#define __MODULES_INCLUDE_BLUETOOTH_HAL_BT_IF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bt_a2dp_codecs.h>
#include <bluetooth/bt_avrcp_cmds.h>
#include <bluetooth/bt_hfp_features.h>
#include <bluetooth/ble_gatt.h>
#include <bluetooth/ble_params.h>
#include <bluetooth/hal/bt_event.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * @struct bt_hal_common_ops_s
 * @brief Bluetooth Common HAL callbacks
 */
struct bt_hal_common_ops_s
{
  int (*init)(void);                              /**< HAL initialization */
  int (*finalize)(void);                          /**< HAL finalization */
  int (*enable)(bool enable);                     /**< Turn ON/OFF */
  int (*setDevAddr)(BT_ADDR *addr);               /**< Set device address */
  int (*getDevAddr)(BT_ADDR *addr);               /**< Get current device address */
  int (*setDevName)(char *name);                  /**< Set device name */
  int (*getDevName)(char *name);                  /**< Get current device name */
  int (*paringEnable)(bool enable);               /**< Set pairing mode enable/disable */
  int (*getBondList)(BT_ADDR *addrs, int *num);   /**< Get bonding list */
  int (*unBond)(BT_ADDR *addr);                   /**< Un-bond device */
  int (*setVisibility)(BT_VISIBILITY visibility); /**< Set visibility */
  int (*inquiryStart)(void);                      /**< Start inquiry */
  int (*inquiryCancel)(void);                     /**< Cancel inquiry */
};

/**
 * @struct bt_hal_a2dp_ops_s
 * @brief Bluetooth A2DP HAL callbacks
 */
struct bt_hal_a2dp_ops_s
{
  int (*connect)(BT_ADDR *addr, uint16_t handle, bool connect); /**< Connect/Disconnect A2DP by BT_ADDR */
  int (*aacEnable)(bool enable);                                /**< Enable/Disable AAC */
  int (*vendorCodecEnable)(bool enable);                        /**< Enable/Disable vendor codec */
  int (*set_codec)(BT_AUDIO_CODEC_INFO *codec_info);            /**< Set codec parameters */
};

/**
 * @struct bt_hal_avrcp_ops_s
 * @brief Bluetooth AVRCP HAL callbacks
 */
struct bt_hal_avrcp_ops_s
{
  int (*avrcc_connect)(BT_ADDR *addr, uint16_t handle, bool connect);                            /**< Connect/Disconnect AVRCP controller by BT_ADDR */
  int (*avrct_connect)(BT_ADDR *addr, uint16_t handle, bool connect);                            /**< Connect/Disconnect AVRCP target by BT_ADDR */
  int (*send_avrcp_command)(BT_ADDR *addr, BT_AVRCP_CMD_ID cmd_id, bool press, uint16_t handle); /**< Send AVRCP command @ref BT_AVRCP_CMD_ID */
  int (*configure_notification)(BT_AVRC_SUPPORT_NOTIFY_EVENT *notification_list);                /**< Configure notification event @ref BT_AVRC_SUPPORT_NOTIFY_EVENT */
};

/**
 * @struct bt_hal_hfp_ops_s
 * @brief Bluetooth HFP HAL callbacks
 */
struct bt_hal_hfp_ops_s
{
  int (*connect)(BT_ADDR *addr, uint16_t handle, bool connect);         /**< Connect/Disconnect HFP by BT_ADDR */
  int (*audio_connect)(BT_ADDR *addr, uint16_t handle, bool connect);   /**< Connect/Disconnect HFP audio by BT_ADDR */
  int (*set_hf_feature)(BT_HFP_HF_FEATURE_FLAG hf_heature);             /**< Setup HFP HF feature @ref BT_HFP_HF_FEATURE_FLAG */
  int (*send_at_command)(BT_ADDR *addr, char *at_str, uint16_t handle); /**< Send AT comand */
  int (*press_button)(BT_ADDR *addr, uint16_t handle);                  /**< Send pressing button comand */
};

/**
 * @struct bt_hal_spp_ops_s
 * @brief Bluetooth SPP HAL callbacks
 */
struct bt_hal_spp_ops_s
{
  int (*connect)(BT_ADDR *addr, uint16_t handle, bool connect);              /**< Connect/Disconnect SPP by BT_ADDR */
  int (*setUuid)(BT_UUID *uuid);                                             /**< Setup UUID @ref BT_UUID */
  int (*sendTxData)(BT_ADDR *addr, uint8_t *data, int len, uint16_t handle); /**< Send SPP Tx data */
};

/**
 * @struct ble_hal_common_ops_s
 * @brief Bluetooth LE common HAL callbacks
 */
struct ble_hal_common_ops_s
{
  int (*setDevAddr)(BT_ADDR *addr);                /**< Set BLE device address */
  int (*setDevName)(char *name);                   /**< Set BLE device name */
  int (*setAppearance)(BLE_APPEARANCE appearance); /**< Set BLE appearance ID @ref BLE_APPEARANCE */
  int (*setPPCP)(BLE_CONN_PARAMS ppcp);            /**< Set PPCP connection parameter */
  int (*advertise)(bool enable);                   /**< Advertisement start/stop */
  int (*scan)(bool enable);                        /**< Scan start/stop */
  int (*connect)(const BT_ADDR *addr);             /**< Create a connection */
  int (*disconnect)(const uint16_t conn_handle);   /**< Destroy a connection */
};

/**
 * @struct ble_hal_gatts_ops_s
 * @brief Bluetooth LE GATTS HAL callbacks
 */
struct ble_hal_gatts_ops_s
{
  int (*addService)(struct ble_gatt_service_s *ble_gatt_service);              /**< Add service to HAL */
  int (*addChar)(uint16_t serv_handle, struct ble_gatt_char_s *ble_gatt_char); /**< Add characteristic to service */
  int (*write)(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle);        /**< Write characteristic response(Peripheral) */
  int (*read)(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle);         /**< Read characteristic response(Peripheral) */
  int (*notify)(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle);       /**< Notify characteristic request(Central)/response(Peripheral) */
};

/**
 * @struct ble_hal_gattc_ops_s
 * @brief Bluetooth LE GATTC HAL callbacks
 */
struct ble_hal_gattc_ops_s
{
  int (*startDbDiscovery)(uint16_t conn_handle);                           /**< GATT client start attribute database discovery */
  int (*continueDbDiscovery)(uint16_t start_handle, uint16_t conn_handle); /**< GATT client start attribute database discovery */
  int (*write)(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle);    /**< Write characteristic request(Central)/response(Peripheral) */
  int (*read)(struct ble_gatt_char_s *ble_gatt_char, uint16_t handle);     /**< Read characteristic request(Central)/response(Peripheral) */
};

/**
 * @struct ble_hal_gatt_ops_s
 * @brief Bluetooth LE GATT HAL
 */
struct ble_hal_gatt_ops_s
{
  struct ble_hal_gatts_ops_s gatts; /**< GATT server HAL */
  struct ble_hal_gattc_ops_s gattc; /**< GATT client HAL */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Bluetooth common function HAL register
 *
 * @param[in] bt_hal_common_ops: HAL callback functions @ref bt_hal_common_ops_s
 *
 * @retval error code
 */

int bt_common_register_hal(struct bt_hal_common_ops_s *bt_hal_common_ops);

/**
 * @brief Common event handler
 *        HAL should call this function if receive common event(@ref BT_COMMON_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int bt_common_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth A2DP function HAL register
 *
 * @param[in] bt_hal_a2dp_ops: HAL callback functions @ref bt_hal_a2dp_ops_s
 *
 * @retval error code
 */

int bt_a2dp_register_hal(struct bt_hal_a2dp_ops_s *bt_hal_a2dp_ops);

/**
 * @brief A2DP event handler
 *        HAL should call this function if receive A2DP event(@ref BT_A2DP_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int bt_a2dp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth AVRCP function HAL register
 *
 * @param[in] bt_hal_avrcp_ops: HAL callback functions @ref bt_hal_avrcp_ops_s
 *
 * @retval error code
 */

int bt_avrcp_register_hal(struct bt_hal_avrcp_ops_s *bt_hal_avrcp_ops);

/**
 * @brief AVRCP event handler
 *        HAL should call this function if receive AVRCP event(@ref BT_AVRCP_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int bt_avrcp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth HFP function HAL register
 *
 * @param[in] bt_hal_hfp_ops: HAL callback functions @ref bt_hal_hfp_ops_s
 *
 * @retval error code
 */

int bt_hfp_register_hal(struct bt_hal_hfp_ops_s *bt_hal_hfp_ops);

/**
 * @brief HFP event handler
 *        HAL should call this function if receive HFP event(@ref BT_HFP_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int bt_hfp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth SPP function HAL register
 *
 * @param[in] bt_hal_spp_ops: HAL callback functions @ref bt_hal_spp_ops_s
 *
 * @retval error code
 */

int bt_spp_register_hal(struct bt_hal_spp_ops_s *bt_hal_spp_ops);

/**
 * @brief SPP event handler
 *        HAL should call this function if receive SPP event(@ref BT_SPP_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int bt_spp_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth LE common function HAL register
 *
 * @param[in] ble_hal_common_ops: HAL callback functions @ref ble_hal_common_ops_s
 *
 * @retval error code
 */

int ble_common_register_hal(struct ble_hal_common_ops_s *ble_hal_common_ops);

/**
 * @brief BLE common event handler
 *        HAL should call this function if receive BLE common event(@ref BLE_COMMON_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int ble_common_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Bluetooth LE GATT function HAL register
 *
 * @param[in] ble_hal_gatt_ops: HAL callback functions @ref ble_hal_gatt_ops_s
 *
 * @retval error code
 */

int ble_gatt_register_hal(struct ble_hal_gatt_ops_s *ble_hal_gatt_ops);

/**
 * @brief BLE GATT event handler
 *        HAL should call this function if receive BLE GATT event(@ref BLE_GATT_EVENT_ID).
 *
 * @param[in] bt_event: Event data @ref bt_event_t
 *
 * @retval error code
 */

int ble_gatt_event_handler(struct bt_event_t *bt_event);

/**
 * @brief Temporary I/F helps app hook central callbacks to framework stored state
 *        Should be removed after consider overall design about how to manage app state
 *
 * @param[in] central_ops: gatt central role callback operation functions
 *
 * @retval error code
 */

int ble_register_gatt_central_cb(struct ble_gatt_central_ops_s *central_ops);

#endif /* __MODULES_INCLUDE_BLUETOOTH_HAL_BT_IF_H */
