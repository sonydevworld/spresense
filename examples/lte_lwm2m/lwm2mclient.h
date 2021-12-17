/*******************************************************************************
 *
 * Copyright (c) 2014 Bosch Software Innovations GmbH, Germany.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v2.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v20.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Bosch Software Innovations GmbH - Please refer to git log
 *
 *******************************************************************************/
/*
 * lwm2mclient.h
 *
 *  General functions of lwm2m test client.
 *
 *  Created on: 22.01.2015
 *  Author: Achim Kraus
 *  Copyright (c) 2015 Bosch Software Innovations GmbH, Germany. All rights reserved.
 */

#ifndef LWM2MCLIENT_H_
#define LWM2MCLIENT_H_

#include "liblwm2m.h"

extern int g_reboot;

/*
 * object_device.c
 */
lwm2m_object_t * get_object_device(void);
void free_object_device(lwm2m_object_t * objectP);
uint8_t device_change(lwm2m_data_t * dataArray, lwm2m_object_t * objectP);
void display_device_object(lwm2m_object_t * objectP);
/*
 * object_firmware.c
 */
lwm2m_object_t * get_object_firmware(void);
void free_object_firmware(lwm2m_object_t * objectP);
void display_firmware_object(lwm2m_object_t * objectP);
/*
 * object_location.c
 */
lwm2m_object_t * get_object_location(void);
void free_object_location(lwm2m_object_t * object);
void display_location_object(lwm2m_object_t * objectP);
void location_setVelocity(lwm2m_object_t* object,
                          uint16_t bearing,
                          uint16_t horizontalSpeed,
                          uint8_t speedUncertainty);
void location_setLocationAtTime(lwm2m_object_t* object,
                             float latitude,
                             float longitude,
                             float altitude,
                             float radius,
                             float speed,
                             uint64_t timestamp);
/*
 * system_gnss.c
 */
int gnss_start(void *arg);
int gnss_stop(void);
/*
 * object_test.c
 */
#define TEST_OBJECT_ID 31024
lwm2m_object_t * get_test_object(void);
void free_test_object(lwm2m_object_t * object);
void display_test_object(lwm2m_object_t * objectP);
/*
 * object_server.c
 */
lwm2m_object_t * get_server_object(int serverId, const char* binding, int lifetime, bool storing);
void clean_server_object(lwm2m_object_t * object);
void display_server_object(lwm2m_object_t * objectP);
void copy_server_object(lwm2m_object_t * objectDest, lwm2m_object_t * objectSrc);

/*
 * object_connectivity_moni.c
 */
lwm2m_object_t * get_object_conn_m(void);
void free_object_conn_m(lwm2m_object_t * objectP);
uint8_t connectivity_moni_change(lwm2m_data_t * dataArray, lwm2m_object_t * objectP);

/*
 * object_connectivity_stat.c
 */
extern lwm2m_object_t * get_object_conn_s(void);
void free_object_conn_s(lwm2m_object_t * objectP);
extern void conn_s_updateTxStatistic(lwm2m_object_t * objectP, uint16_t txDataByte, bool smsBased);
extern void conn_s_updateRxStatistic(lwm2m_object_t * objectP, uint16_t rxDataByte, bool smsBased);

/*
 * object_access_control.c
 */
lwm2m_object_t* acc_ctrl_create_object(void);
void acl_ctrl_free_object(lwm2m_object_t * objectP);
bool  acc_ctrl_obj_add_inst (lwm2m_object_t* accCtrlObjP, uint16_t instId,
                 uint16_t acObjectId, uint16_t acObjInstId, uint16_t acOwner);
bool  acc_ctrl_oi_add_ac_val(lwm2m_object_t* accCtrlObjP, uint16_t instId,
                 uint16_t aclResId, uint16_t acValue);
/*
 * lwm2mclient.c
 */
void handle_value_changed(lwm2m_context_t* lwm2mH, lwm2m_uri_t* uri, const char * value, size_t valueLength);
/*
 * system_api.c
 */
void init_value_change(lwm2m_context_t * lwm2m);
void system_reboot(void);
/*
 * system_device.c
 */
const char * get_manufacture(void);
const char * get_model_number(void);
const char * get_serial_number(void);
const char * get_firmware_version(void);
void device_reboot(void);
int get_free_memory(void);
int get_total_memory(void);
void set_utc_offset_sec(char * timeoffset);
int get_utc_offset_sec(void);

/*
 * system_fwupdate.c
 */
int get_package_info(char *pkg_name, size_t pkg_name_len,
                     char *pkg_version, size_t pkg_ver_len);
int save_package(void *buffer, size_t length);
int execute_fwupdate(void);

/*
 * object_security.c
 */
lwm2m_object_t * get_security_object(int serverId, const char* serverUri, char * bsPskId, char * psk, uint16_t pskLen, bool isBootstrap);
void clean_security_object(lwm2m_object_t * objectP);
char * get_server_uri(lwm2m_object_t * objectP, uint16_t secObjInstID);
void display_security_object(lwm2m_object_t * objectP);
void copy_security_object(lwm2m_object_t * objectDest, lwm2m_object_t * objectSrc);

/*
 * object_digital_input.c
 */
#define LWM2M_DIGITAL_INPUT_OBJECT_ID     3200
lwm2m_object_t * get_digital_input_object(void);
void free_digital_input_object(lwm2m_object_t * objectP);
void display_digital_input_object(lwm2m_object_t * object);
void digital_input_setValue(uint16_t id, bool state);
void digital_input_setCounter(uint16_t id);

/*
 * object_digital_output.c
 */
#define LWM2M_DIGITAL_OUTPUT_OBJECT_ID    3201
lwm2m_object_t * get_digital_output_object(void);
void free_digital_output_object(lwm2m_object_t * objectP);
void display_digital_output_object(lwm2m_object_t * object);

/*
 * system_gpio.c
 */
int gpio_input_config(uint16_t id, bool polarity);
int gpio_output_config(uint16_t id);
int gpio_read(uint16_t id, bool polarity);
int gpio_interrupt(uint16_t id, int selection);
int gpio_write(uint16_t id, bool state, bool polarity);

/*
 * object_analog_input.c
 */
#define LWM2M_ANALOG_INPUT_OBJECT_ID      3202
lwm2m_object_t * get_analog_input_object(void);
void free_analog_input_object(lwm2m_object_t * objectP);
void display_analog_input_object(lwm2m_object_t * object);
void analog_input_setValue(uint16_t id, float value,
                           float range_min, float range_max);

/*
 * system_adc.c
 */
int adc_start(int ch);
int adc_stop(int ch);

#endif /* LWM2MCLIENT_H_ */
