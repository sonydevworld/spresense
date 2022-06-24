/****************************************************************************
 * modules/bluetooth/hal/nrf52/ble_gatts.c
 *
 *   Copyright 2022 Sony Semiconductor Solutions Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <ble/ble_comm.h>
#include <ble/ble_gatts.h>
#include "ble.h"
#include "ble_comm_internal.h"
#include <bluetooth/ble_gatt.h>

// #define BLE_DBGPRT_ENABLE
#ifdef BLE_DBGPRT_ENABLE
#include <stdio.h>
#define BLE_PRT printf
#else
#define BLE_PRT(...)
#endif

/******************************************************************************
 * externs
 *****************************************************************************/
extern int bleConvertErrorCode(uint32_t errCode);
extern bleCommMem commMem;
/******************************************************************************
 * Define
 *****************************************************************************/

 /******************************************************************************
 * Structre define
 *****************************************************************************/

 /******************************************************************************
 * Function prototype declaration
 *****************************************************************************/

/******************************************************************************
 * Function
 *****************************************************************************/
static int setPermission(ble_gap_conn_sec_mode_t *perm, BLE_SEC_MODE secMode)
{
	int ret = 0;
	switch( secMode ) {
	case BLE_SEC_MODE_NO_ACCESS:
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(perm);
		break;
	case BLE_SEC_MODE1LV1_NO_SEC:
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(perm);
		break;
	case BLE_SEC_MODE1LV2_NO_MITM_ENC:
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(perm);
		break;
	case BLE_SEC_MODE1LV3_MITM_ENC:
		BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(perm);
		break;
	case BLE_SEC_MODE2LV1_NO_MITM_DATA_SGN:
		BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(perm);
		break;
	case BLE_SEC_MODE2LV2__MITM_DATA_SGN:
		BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(perm);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

int BLE_GattsAddService(BLE_GATT_TYPE type, BLE_Uuid const* uuid, uint16_t* serviceHandle)
{
	int ret      = BLE_SUCCESS;
	int errCode = 0;
	ble_uuid_t        uuidBaseAlias = {0};
	ble_uuid128_t     *uuidBase = NULL;
	uint8_t          uuid16[2] = {0,0};

	if((uuid == NULL) || (serviceHandle == NULL)) {
		return -EINVAL;
	}

	memset(&uuidBaseAlias, 0, sizeof(ble_uuid_t));
	switch( uuid->type ) {
	case BLE_UUID_TYPE_BASEALIAS_BTSIG:
	{
		uuidBaseAlias.type = BLE_UUID_TYPE_BLE;
		uuidBaseAlias.uuid = uuid->value.baseAlias.uuidAlias;
		break;
	}
	case BLE_UUID_TYPE_BASEALIAS_VENDOR:
	{
		uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
		uuidBaseAlias.uuid = uuid->value.baseAlias.uuidAlias;
		uuidBase           = (ble_uuid128_t*)&uuid->value.baseAlias.uuidBase;
		break;
	}
	case BLE_UUID_TYPE_UUID128:
	{
		uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
		//octecs 12-13 of 128-bit UUID,16bits
		memcpy(uuid16, (uuid->value.uuid128.uuid128 + 12), sizeof(uuid16));
		uuidBaseAlias.uuid = (uuid16[1]<<8)|uuid16[0];
		uuidBase = (ble_uuid128_t*)&uuid->value.uuid128;
		break;
	}
	default:
		break;
	}
	if(uuidBase != NULL) {
		errCode  = sd_ble_uuid_vs_add(uuidBase, &uuidBaseAlias.type);
		ret = bleConvertErrorCode((uint32_t)errCode);
		if(ret) {
			BLE_PRT("sd_ble_uuid_vs_add NG %d\n", ret);
			return ret;
		}
	}
	// Add the service.
	errCode = sd_ble_gatts_service_add(type, &uuidBaseAlias, serviceHandle);
	ret = bleConvertErrorCode((uint32_t)errCode);
	if(ret) {
		BLE_PRT("sd_ble_gatts_service_add NG %d\n", ret);
	}
	return ret;
}

int BLE_GattsAddCharacteristic(uint16_t serviceHandle, BLE_CharMeta const* charMeta, BLE_GattsAttr const* attrCharValue, BLE_GattsCharHandles* handles)
{
	int ret      = BLE_SUCCESS;
	int errCode = 0;
	uint8_t             uuid16[2] = {0,0};
	ble_uuid_t          uuidBaseAlias = {0};
	ble_uuid128_t       *uuidBase = NULL;
	ble_gatts_char_md_t charMd = {0};
	ble_gatts_attr_t    localAttrCharValue = {0};
	ble_uuid_t          bleUuid = {0};
	ble_gatts_attr_md_t attrMd = {0};
	ble_gatts_attr_md_t cccdMd = {0};
	ble_gatts_attr_md_t sccdMd = {0};
	BLE_SEC_MODE         noAccessMode = BLE_SEC_MODE_NO_ACCESS;
	ble_gatts_char_handles_t charHandles = {0};

	if( (charMeta == NULL) || (attrCharValue == NULL) || (attrCharValue->attrValue == NULL)) {
		return -EINVAL;
	}

	memset(&uuidBaseAlias, 0, sizeof(ble_uuid_t));
	memset(&cccdMd, 0, sizeof(cccdMd));
	memset(&sccdMd, 0, sizeof(sccdMd));
	memset(&attrMd, 0, sizeof(attrMd));
	memset(&charMd, 0, sizeof(charMd));
	memset(&localAttrCharValue, 0, sizeof(localAttrCharValue));
	memset(&bleUuid, 0, sizeof(bleUuid));
	memcpy(&charMd.char_props, &charMeta->charPrope, sizeof(ble_gatt_char_props_t));

	charMd.p_sccd_md        = NULL;
	charMd.p_char_pf        = NULL;
	charMd.p_sccd_md        = NULL;
	charMd.p_char_user_desc = NULL;
	charMd.p_user_desc_md   = NULL;

	//to be tested readPerm not like sm lv
	if((charMeta->clientCharCfgDpr.readPerm == noAccessMode )||(charMeta->clientCharCfgDpr.writePerm == noAccessMode )){
		charMd.p_cccd_md  = NULL;
	} else {
		ret = setPermission(&cccdMd.read_perm, charMeta->clientCharCfgDpr.readPerm);
		if (ret) {
			return ret;
		}
		ret = setPermission(&cccdMd.write_perm, charMeta->clientCharCfgDpr.writePerm);
		if (ret) {
			return ret;
		}
		cccdMd.vloc       = BLE_GATTS_VLOC_STACK;
		charMd.p_cccd_md  = &cccdMd;
	}

	attrMd.rd_auth = 0;
	attrMd.vlen    = 1;
	attrMd.vloc    = BLE_GATTS_VLOC_STACK;
	ret = setPermission(&attrMd.read_perm, attrCharValue->attrPerm.readPerm);
	if (ret) {
		return ret;
	}
	ret = setPermission(&attrMd.write_perm, attrCharValue->attrPerm.writePerm);
	if (ret) {
		return ret;
	}

	localAttrCharValue.p_value   = attrCharValue->attrValue;
	localAttrCharValue.init_len  = attrCharValue->valueLen;
	localAttrCharValue.max_len   = attrCharValue->valueLen;
	localAttrCharValue.init_offs = 0;
	localAttrCharValue.p_attr_md = &attrMd;
	switch(attrCharValue->valueUuid.type){
	case BLE_UUID_TYPE_BASEALIAS_BTSIG:
	{
		uuidBaseAlias.type = BLE_UUID_TYPE_BLE;
		uuidBaseAlias.uuid = attrCharValue->valueUuid.value.baseAlias.uuidAlias;
		break;
	}
	case BLE_UUID_TYPE_BASEALIAS_VENDOR:
	{
		uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
		uuidBaseAlias.uuid = attrCharValue->valueUuid.value.baseAlias.uuidAlias;
		uuidBase           = (ble_uuid128_t*)&attrCharValue->valueUuid.value.baseAlias.uuidBase;
		break;
	}
	case BLE_UUID_TYPE_UUID128:
	{
		uuidBaseAlias.type = BLE_UUID_TYPE_VENDOR_BEGIN;
		//octecs 12-13 of 128-bit UUID,16bits
		memcpy(uuid16,(attrCharValue->valueUuid.value.uuid128.uuid128+12),sizeof(uuid16));
		uuidBaseAlias.uuid = (uuid16[1]<<8)|uuid16[0];
		uuidBase = (ble_uuid128_t*)&attrCharValue->valueUuid.value.uuid128;
		break;
	}
	default:
		break;
	}
	if(uuidBase != NULL) {
		errCode  = sd_ble_uuid_vs_add(uuidBase, &uuidBaseAlias.type);
		ret = bleConvertErrorCode((uint32_t)errCode);
	}
	if(ret != BLE_SUCCESS) {
		return ret;
	}
	localAttrCharValue.p_uuid = &uuidBaseAlias;
	errCode = sd_ble_gatts_characteristic_add(serviceHandle, &charMd, &localAttrCharValue, &charHandles);
	ret = bleConvertErrorCode((uint32_t)errCode);
	if(ret == BLE_SUCCESS)
	{
		handles->charHandle               = charHandles.value_handle;
		handles->dprHandle.cccdHandle     = charHandles.cccd_handle;
		handles->dprHandle.sccdHandle     = charHandles.sccd_handle;
		handles->dprHandle.userDescHandle = charHandles.user_desc_handle;
	}
	return ret;
}

int BLE_GattsHandleValueNfyInd(uint16_t connHandle, BLE_GattsHandleValueNfyIndParams const* handleValueNfyInd)
{
#define BLE_GATTS_TX_BUFFER_CHECK_MAX 5
	int ret      = BLE_SUCCESS;
	int errCode = 0;
	uint16_t len = 0;
	ble_gatts_hvx_params_t hvxParams = {0};

	BLE_PRT("%s:connHandle :%d.\n", __func__, connHandle);
	if( handleValueNfyInd == NULL) {
		BLE_PRT("%s:handleValueNfyInd null.\n", __func__);
		return -EINVAL;
	}

	memset(&hvxParams, 0, sizeof(hvxParams));
	len = handleValueNfyInd->attrValLen;
	hvxParams.handle = handleValueNfyInd->attrHandle;
	hvxParams.type   = handleValueNfyInd->type;
	hvxParams.p_len  = &len;
	hvxParams.p_data = handleValueNfyInd->attrValData;
	for (int i=0; i<BLE_GATTS_TX_BUFFER_CHECK_MAX; i++) {
		errCode = sd_ble_gatts_hvx(connHandle, &hvxParams);
		if (errCode != NRF_ERROR_RESOURCES) {
			ret = bleConvertErrorCode((uint32_t)errCode);
			return ret;
		}
		int msec = (commMem.gapMem->connParams.max_conn_interval * 1250 + (1000 - 1)) / 1000;
		BLE_PRT("BLE_GattsHandleValueNfyInd: delay %d\n", msec);
		usleep(msec * 1000);
	}
	return -EBUSY;
}

int BLE_GattsUpdateAttrValue(BLE_GattsAttrValueParam *attrValueParam)
{
	int ret      = BLE_SUCCESS;
	int errCode = 0;
	ble_gatts_value_t gattsValue = {0};

	if( attrValueParam == NULL) {
		return -EINVAL;
	}

	memset(&gattsValue, 0, sizeof(gattsValue));
	gattsValue.offset   = 0;
	gattsValue.len      = attrValueParam->attrValLen;
	gattsValue.p_value  = attrValueParam->attrValData;

	errCode = sd_ble_gatts_value_set(attrValueParam->connHandle, attrValueParam->attrHandle, &gattsValue);
	ret = bleConvertErrorCode((uint32_t)errCode);
	return ret;
}
