/****************************************************************************
 * modules/bluetooth/hal/bcm20706/bt_util.h
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

#ifndef __MODULES_BLUETOOTH_HAL_BCM20706_BT_UTIL_H
#define __MODULES_BLUETOOTH_HAL_BCM20706_BT_UTIL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <bt/bt_comm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PACKET_CONTROL  (0x19)
#define PACKET_HCI		(0x04)
#define PACKET_MEDIA	(0x02)

#define BT_SHORT_COMMAND_LEN 20
#define BT_MID_COMMAND_LEN   80
#define BT_LONG_COMMAND_LEN  550

#define BT_LOCK_ROSC_UART_BAUD_RATE 115200
#define BT_LOCK_LV_UART_BAUD_RATE   460800
#define BT_UART_BAUD_RATE_APP       921600

#define BITS8 8
#define BITS16 16
#define BITS24 24
#define BITS32 32

#define MIN(x, y) ((x) < (y) ? (x) : (y))

/*
 * General purpose commands
 */
#define BT_CONTROL_COMMAND_RESET                           ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x01 )    /* Restart controller */
#define BT_CONTROL_COMMAND_TRACE_ENABLE                    ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x02 )    /* Enable or disable WICED traces */
#define BT_CONTROL_COMMAND_SET_LOCAL_BDA                   ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x03 )    /* Set local device addrsss */
#define BT_CONTROL_COMMAND_SET_BAUD_RATE                   ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x04 )    /* Change UART baud rate */
#define BT_CONTROL_COMMAND_PUSH_NVRAM_DATA                 ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x05 )    /* Download previously saved NVRAM chunk */
#define BT_CONTROL_COMMAND_DELETE_NVRAM_DATA               ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x06 )    /* Delete NVRAM chunk currently stored in RAM */
#define BT_CONTROL_COMMAND_INQUIRY                         ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x07 )    /* Start/stop inquiry */
#define BT_CONTROL_COMMAND_SET_VISIBILITY                  ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x08 )    /* Set BR/EDR connectability and discoverability of the device */
#define BT_CONTROL_COMMAND_SET_PAIRING_MODE                ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x09 )    /* Set Pairing Mode for the device 0 = Not pairable 1 = Pairable */
#define BT_CONTROL_COMMAND_UNBOND                          ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x0A )    /* Delete bond with specified BDADDR */
#define BT_CONTROL_COMMAND_REPLY_CONFIRMATION              ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x0B )    /* User Confirmation during pairing, TRUE/FALSE passed as parameter */
#define BT_CONTROL_COMMAND_ENABLE_COEX                     ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x0C )    /* Enable coex functionality */
#define BT_CONTROL_COMMAND_DISABLE_COEX                    ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x0D )    /* Disable coex functionality */
#define BT_CONTROL_COMMAND_STARTBOND                       ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x10 )    /* Start bond with remote device */
#define BT_CONTROL_COMMAND_SET_BT_NAME                     ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x12 )    /* Set bt name*/
#define BT_CONTROL_COMMAND_SET_PAIR_MODE                   ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x13 )    /* Set pairing mode*/
#define BT_CONTROL_COMMAND_REPLY_PASSKEY                   ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x15 )    /* User Passkey during pairing, TRUE/FALSE passed as parameter */
#define BT_CONTROL_COMMAND_REPLY_SECURITY                  ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x16 )    /* Reply security*/
#define BT_CONTROL_COMMAND_REPLY_FEATURE                   ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x17 )    /* Reply feature*/
#define BT_CONTROL_COMMAND_SET_PINCODE                     ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x18 )    /* Set pincode*/
#define BT_CONTROL_COMMAND_SET_SNIFF_MODE_PARAM            ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x19 )    /* Set sniff mode param*/
#define BT_CONTROL_COMMAND_SET_VISIBILITY_PARAM            ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x20 )    /* Set discoverable and connectable param*/
#define BT_CONTROL_COMMAND_SET_PCM_I2S_ROLE                ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x21 )    /* Set pcm i2s role*/
#define BT_CONTROL_COMMAND_SET_SNIFF_TIMER_VALUE           ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x22 )    /* Set sniff mode timer timeout value*/
#define BT_CONTROL_COMMAND_ENABLE_SNIFF_MODE               ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x23 )    /* Enable/Disable sniff mode*/
#define BT_CONTROL_COMMAND_GET_BT_VERSION                  ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x24 )    /* Get the BT FW version */
#define BT_CONTROL_COMMAND_SET_EXT_INQR_DATA               ( ( BT_CONTROL_GROUP_DEVICE << 8 ) | 0x25 )    /* Set extended inquiry data */

/*
 * LE Commands
 * Define commands sent to the GAP/GATT implementation on 20706
 */
#define BT_CONTROL_LE_COMMAND_SCAN                         ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x01 )    /* start scan */
#define BT_CONTROL_LE_COMMAND_ADVERTISE                    ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x02 )    /* start advertisements */
#define BT_CONTROL_LE_COMMAND_CONNECT                      ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x03 )    /* connect to peer */
#define BT_CONTROL_LE_COMMAND_CANCEL_CONNECT               ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x04 )    /* cancel connect */
#define BT_CONTROL_LE_COMMAND_DISCONNECT                   ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x05 )    /* disconnect */
#define BT_CONTROL_LE_RE_PAIR                              ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x06 )    /* delete keys and then re-pair */
#define BT_CONTROL_LE_COMMAND_GET_IDENTITY_ADDRESS         ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x07 )    /* get identity address */
#define BT_CONTROL_LE_COMMAND_SET_CHANNEL_CLASSIFICATION   ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x08 )    /* set channel classification for the available 40 channels */
#define BT_CONTROL_LE_COMMAND_SET_CONN_PARAMS              ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x09 )    /* set connection parameters */
#define BT_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA       ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x0a )    /* set raw advertisement data */
#define BT_CONTROL_LE_COMMAND_SECURITY_GRANT               ( ( BT_CONTROL_GROUP_LE << 8 ) | 0x0b )    /* grant or deny access */

/*
 * GATT Commands
 * Define commands to perform various GATT procedures
 */
#define BT_CONTROL_GATT_COMMAND_DISCOVER_SERVICES          ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x01 )    /* discover services */
#define BT_CONTROL_GATT_COMMAND_DISCOVER_CHARACTERISTICS   ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x02 )    /* discover characteristics */
#define BT_CONTROL_GATT_COMMAND_DISCOVER_DESCRIPTORS       ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x03 )    /* discover descriptors */
#define BT_CONTROL_GATT_COMMAND_READ_REQUEST               ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x04 )    /* send read request */
#define BT_CONTROL_GATT_COMMAND_READ_RESPONSE              ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x05 )    /* send read response */
#define BT_CONTROL_GATT_COMMAND_WRITE_COMMAND              ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x06 )    /* send write command */
#define BT_CONTROL_GATT_COMMAND_WRITE_REQUEST              ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x07 )    /* send write request */
#define BT_CONTROL_GATT_COMMAND_WRITE_RESPONSE             ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x08 )    /* send write response */
#define BT_CONTROL_GATT_COMMAND_NOTIFY                     ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x09 )    /* send notification */
#define BT_CONTROL_GATT_COMMAND_INDICATE                   ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x0a )    /* send indication */
#define BT_CONTROL_GATT_COMMAND_INDICATE_CONFIRM           ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x0b )    /* send indication confirmation */
#define BT_CONTROL_GATT_COMMAND_REGISTER                   ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x0c )    /* register GATT callback */
#define BT_CONTROL_GATT_COMMAND_DB_INIT                    ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x0d )	 /* initialize GATT database */
#define BT_CONTROL_GATT_COMMAND_ADD_SERVICE                ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x0e )    /* add Service */
#define BT_CONTROL_GATT_COMMAND_ADD_CHAR                   ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x0f )    /* add Charateristic */
#define BT_CONTROL_GATT_COMMAND_UPDATE_ATTR                ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x10 )    /* update AttrValue */
#define BT_CONTROL_GATT_COMMAND_SET_BLE_NAME_PERMISSION    ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x11 )    /* set ble name permission */
#define BT_CONTROL_GATT_COMMAND_ENABLE_SERVICE_CHANGED     ( ( BT_CONTROL_GROUP_GATT << 8 ) | 0x12 )    /* set service changed charateristic */

/*
 * Serial Port Profile Commands
 * Define commands sent to the SPP profile
 */
#define BT_CONTROL_SPP_COMMAND_CONNECT                     ( ( BT_CONTROL_GROUP_SPP << 8 ) | 0x01 )    /* establish connection to SPP server */
#define BT_CONTROL_SPP_COMMAND_DISCONNECT                  ( ( BT_CONTROL_GROUP_SPP << 8 ) | 0x02 )    /* release SPP connection */
#define BT_CONTROL_SPP_COMMAND_DATA                        ( ( BT_CONTROL_GROUP_SPP << 8 ) | 0x03 )    /* send data */
#define BT_CONTROL_SPP_COMMAND_UUID                        ( ( BT_CONTROL_GROUP_SPP << 8 ) | 0x04 )    /* set uuid */


/*
* Audio Profile Commands
* Define commands sent to the Audio profile
*/
#define BT_CONTROL_AUDIO_COMMAND_CONNECT                   ( ( BT_CONTROL_GROUP_A2DP_SRC << 8 ) | 0x01 )    /* Audio connect to sink */
#define BT_CONTROL_AUDIO_COMMAND_DISCONNECT                ( ( BT_CONTROL_GROUP_A2DP_SRC << 8 ) | 0x02 )    /* Audio disconnect  */
#define BT_CONTROL_AUDIO_START                             ( ( BT_CONTROL_GROUP_A2DP_SRC << 8 ) | 0x03 )    /* start audio with speciifc sample rate/mode */
#define BT_CONTROL_AUDIO_STOP                              ( ( BT_CONTROL_GROUP_A2DP_SRC << 8 ) | 0x04 )    /* stop audio */
#define BT_CONTROL_AUDIO_PACKET_COUNT                      ( ( BT_CONTROL_GROUP_A2DP_SRC << 8 ) | 0x05 )    /* debug packet counter sent from host */

/*
* Audio Sink Profile Commands
* Define commands sent to the Audio profile
*/
#define BT_CONTROL_AUDIO_SINK_COMMAND_CONNECT                   ( ( BT_CONTROL_GROUP_A2DP_SINK << 8 ) | 0x01 )    /* Audio Sink connect to source */
#define BT_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT                ( ( BT_CONTROL_GROUP_A2DP_SINK << 8 ) | 0x02 )    /* Audio Sink disconnect */
#define BT_CONTROL_AUDIO_SINK_COMMAND_SET_CODEC                 ( ( BT_CONTROL_GROUP_A2DP_SINK << 8 ) | 0x06 )    /* Audio Sink set codec */
#define BT_CONTROL_AUDIO_SINK_COMMAND_AAC_ENABLE                ( ( BT_CONTROL_GROUP_A2DP_SINK << 8 ) | 0x07 )    /* Audio Sink aac enable/disable */

/*
* Avrc Controller Profile Commands
* Define commands sent to the Audio profile
*/
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_CONNECT                   ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x01 )    /* Avrc Controller connect  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_DISCONNECT                ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x02 )    /* Avrc Controller disconnect  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY                      ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x03 )    /* Avrc Controller play  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_STOP                      ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x04 )    /* Avrc Controller stop  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE                     ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x05 )    /* Avrc Controller pause  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD        ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x06 )    /* Avrc Controller begin fast forward  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_END_FAST_FORWARD          ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x07 )    /* Avrc Controller end fast forward  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND              ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x08 )    /* Avrc Controller begin rewind  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_END_REWIND                ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x09 )    /* Avrc Controller end rewind  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK                ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0A )    /* Avrc Controller next trask  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK            ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0B )    /* Avrc Controller privious track  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP                 ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0C )    /* Avrc Controller volume up  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN               ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0D )    /* Avrc Controller volume down  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_GET_TRACK_INFORMATION     ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x0E )    /* Avrc Controller get track information  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE           ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x10 )    /* Avrc Controller set repeat mode  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE          ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x11 )    /* Avrc Controller set shuffle mode  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_SET_VOLUME                ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x13 )    /* Avrc Controller set volume  */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_NOTIFY_VOLUME             ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x20 )    /* Avrc Controller notify volume change event */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_INIT_NOTIFY_VOLUME        ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x21 )    /* Avrc Controller init notify volume change event */
#define BT_CONTROL_AVRC_CONTROLLER_COMMAND_SUPPORT_EVENT             ( ( BT_CONTROL_GROUP_AVRC_CONTROLLER << 8 ) | 0x22 )    /* Avrc Controller setting supported notify event */

/*
* Avrc Target Profile Commands
* Define commands sent to the Audio profile
*/
#define BT_CONTROL_AVRC_TARGET_COMMAND_CONNECT                       ( ( BT_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x01 )    /* Avrc Target connect  */
#define BT_CONTROL_AVRC_TARGET_COMMAND_DISCONNECT                    ( ( BT_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x02 )    /* Avrc Target disconnect  */
#define BT_CONTROL_AVRC_TARGET_COMMAND_PLAYER_STATUS                 ( ( BT_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x06 )    /* Avrc Target player status  */
#define BT_CONTROL_AVRC_TARGET_COMMAND_VOLUME_UP                     ( ( BT_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0B )    /* Avrc Target volume up  */
#define BT_CONTROL_AVRC_TARGET_COMMAND_VOLUME_DOWN                   ( ( BT_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x0C )    /* Avrc Target volume down  */
#define BT_CONTROL_AVRC_TARGET_COMMAND_SET_VOLUME                    ( ( BT_CONTROL_GROUP_AVRC_TARGET << 8 ) | 0x13 )    /* Avrc Target set volume  */

/*
* Handsfree Commands
* Define commands sent to the HFP profile
*/
#define BT_CONTROL_HF_COMMAND_CONNECT                                ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x01 )    /* establish connection to HF Audio Gateway */
#define BT_CONTROL_HF_COMMAND_DISCONNECT                             ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x02 )    /* release HF connection */
#define BT_CONTROL_HF_COMMAND_OPEN_AUDIO                             ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x03 )    /* create audio connection on existing service level connection */
#define BT_CONTROL_HF_COMMAND_CLOSE_AUDIO                            ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x04 )    /* disconnect audio */
#define BT_CONTROL_HF_COMMAND_BUTTON_PRESS                           ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x07 )    /* headset button press */
#define BT_CONTROL_HF_COMMAND_SET_FEATURE                            ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x10 )    /* set hf supported feature */

/*
* Sub commands to send various AT Commands
*/
#define BT_CONTROL_HF_AT_COMMAND_BASE                                ( ( BT_CONTROL_GROUP_HF << 8 ) | 0x20 )    /* send AT command and supporting data */
#define BT_CONTROL_HF_AT_COMMAND_SPK                                 0x00    /* Update speaker volume */
#define BT_CONTROL_HF_AT_COMMAND_MIC                                 0x01    /* Update microphone volume */
#define BT_CONTROL_HF_AT_COMMAND_A                                   0x02    /* Answer incoming call */
#define BT_CONTROL_HF_AT_COMMAND_BINP                                0x03    /* Retrieve number from voice tag */
#define BT_CONTROL_HF_AT_COMMAND_BVRA                                0x04    /* Enable/Disable voice recognition */
#define BT_CONTROL_HF_AT_COMMAND_BLDN                                0x05    /* Last Number redial */
#define BT_CONTROL_HF_AT_COMMAND_CHLD                                0x06    /* Call hold command */
#define BT_CONTROL_HF_AT_COMMAND_CHUP                                0x07    /* Call hang up command */
#define BT_CONTROL_HF_AT_COMMAND_CIND                                0x08    /* Read Indicator Status */
#define BT_CONTROL_HF_AT_COMMAND_CNUM                                0x09    /* Retrieve Subscriber number */
#define BT_CONTROL_HF_AT_COMMAND_D                                   0x0A    /* Place a call using a number or memory dial */
#define BT_CONTROL_HF_AT_COMMAND_NREC                                0x0B    /* Disable Noise reduction and echo canceling in AG */
#define BT_CONTROL_HF_AT_COMMAND_VTS                                 0x0C    /* Transmit DTMF tone */
#define BT_CONTROL_HF_AT_COMMAND_BTRH                                0x0D    /* CCAP incoming call hold */
#define BT_CONTROL_HF_AT_COMMAND_COPS                                0x0E    /* Query operator selection */
#define BT_CONTROL_HF_AT_COMMAND_CMEE                                0x0F    /* Enable/disable extended AG result codes */
#define BT_CONTROL_HF_AT_COMMAND_CLCC                                0x10    /* Query list of current calls in AG */
#define BT_CONTROL_HF_AT_COMMAND_BIA                                 0x11    /* Activate/Deactivate indicators */
#define BT_CONTROL_HF_AT_COMMAND_BIEV                                0x12    /* Send HF indicator value to peer */
#define BT_CONTROL_HF_AT_COMMAND_UNAT                                0x13    /* Transmit AT command not in the spec  */
#define BT_CONTROL_HF_AT_COMMAND_MAX                                 0x13    /* For command validation */

/*
* Handsfree AG Commands
* Define commands sent to the HF-AG profile
*/
#define BT_CONTROL_AG_COMMAND_CONNECT                                ( ( BT_CONTROL_GROUP_AG << 8 ) | 0x01 )    /* establish connection to HF Device */
#define BT_CONTROL_AG_COMMAND_DISCONNECT                             ( ( BT_CONTROL_GROUP_AG << 8 ) | 0x02 )    /* release HF connection */
#define BT_CONTROL_AG_COMMAND_OPEN_AUDIO                             ( ( BT_CONTROL_GROUP_AG << 8 ) | 0x03 )    /* create audio connection on existing service level connection */
#define BT_CONTROL_AG_COMMAND_CLOSE_AUDIO                            ( ( BT_CONTROL_GROUP_AG << 8 ) | 0x04 )    /* disconnect audio */

/*
 * Define general events that controller can send
 */
#define BT_CONTROL_EVENT_COMMAND_STATUS                    (  0x01 )    /* Command status event for the requested operation */
#define BT_CONTROL_EVENT_WICED_TRACE                       (  0x02 )    /* WICED trace packet */
#define BT_CONTROL_EVENT_HCI_TRACE                         (  0x03 )    /* Bluetooth protocol trace */
#define BT_CONTROL_EVENT_NVRAM_DATA                        (  0x04 )    /* Request to MCU to save NVRAM chunk */
#define BT_CONTROL_EVENT_DEVICE_STARTED                    (  0x05 )    /* Device completed power up initialization */
#define BT_CONTROL_EVENT_INQUIRY_RESULT                    (  0x06 )    /* Inquiry result */
#define BT_CONTROL_EVENT_INQUIRY_COMPLETE                  (  0x07 )    /* Inquiry completed event */
#define BT_CONTROL_EVENT_PAIRING_COMPLETE                  (  0x08 )    /* Pairing Completed */
#define BT_CONTROL_EVENT_ENCRYPTION_CHANGED                (  0x09 )    /* Encryption changed event */
#define BT_CONTROL_EVENT_CONNECTED_DEVICE_NAME             (  0x0A )    /* Device name event */
#define BT_CONTROL_EVENT_USER_CONFIRMATION                 (  0x0B )    /* User Confirmation during pairing */
#define BT_CONTROL_EVENT_SECURITY_REQUEST                  (  0x14 )    /* security*/
#define BT_CONTROL_EVENT_USER_PASSKEY                      (  0x17 )    /* User Passkey during pairing */
#define BT_CONTROL_EVENT_PASSKEY_NOTIFICATION              (  0x19 )    /* Passkey notify*/
#define BT_CONTROL_EVENT_REQUEST_FEATURE                   (  0x1A )    /* request pairing feature*/
#define BT_CONTROL_EVENT_POWER_MANAGER_STATUS              (  0x20 )    /* power manager status*/
#define BT_CONTROL_EVENT_REPLY_BAUD_RATE                   (  0x21 )    /* rely baud rate*/
#define BT_CONTROL_EVENT_REPLY_I2S_ROLE                    (  0x22 )    /* rely I2S role*/
#define BT_CONTROL_EVENT_REPLY_VENDORID                    ( 0x23 )    /* Reply the vendor Id */
#define BT_CONTROL_EVENT_REPLY_BT_VERSION                  ( 0x24 )    /* Reply the bt version */

/*
 * Define LE events from the BLE GATT/GAP
 */
#define BT_CONTROL_LE_EVENT_COMMAND_STATUS                 ( 0x01 )    /* Command status event for the requested operation */
#define BT_CONTROL_LE_EVENT_SCAN_STATUS                    ( 0x02 )    /* LE scanning state change notification */
#define BT_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT           ( 0x03 )    /* Advertisement report */
#define BT_CONTROL_LE_EVENT_ADVERTISEMENT_STATE            ( 0x04 )    /* LE Advertisement state change notification */
#define BT_CONTROL_LE_EVENT_CONNECTED                      ( 0x05 )    /* LE Connection established */
#define BT_CONTROL_LE_EVENT_DISCONNECTED                   ( 0x06 )    /* Le Connection Terminated */
#define BT_CONTROL_LE_EVENT_IDENTITY_ADDRESS               ( 0x07 )    /* Identity address */
#define BT_CONTROL_LE_EVENT_PEER_MTU                       ( 0x08 )    /* Client MTU Request */
#define BT_CONTROL_LE_EVENT_CONN_PARAMS                    ( 0x09 )    /* BLE connection parameter update event */
#define BT_CONTROL_LE_EVENT_CONNECT_TIMEOUT                ( 0x10 )    /* Connection time out */

/*
 * Define GATT events
 */
#define BT_CONTROL_GATT_EVENT_COMMAND_STATUS               ( 0x01 )    /* Command status event for the requested operation */
#define BT_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE           ( 0x02 )    /* Discovery requested by host completed */
#define BT_CONTROL_GATT_EVENT_SERVICE_DISCOVERED           ( 0x03 )    /* Service discovered */
#define BT_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED    ( 0x04 )    /* Characteristic discovered */
#define BT_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED        ( 0x05 )    /* Characteristic descriptor discovered */
#define BT_CONTROL_GATT_EVENT_READ_REQUEST                 ( 0x06 )    /* Peer sent Read Request */
#define BT_CONTROL_GATT_EVENT_READ_RESPONSE                ( 0x07 )    /* Read response */
#define BT_CONTROL_GATT_EVENT_WRITE_REQUEST                ( 0x08 )    /* Peer sent Write Request */
#define BT_CONTROL_GATT_EVENT_WRITE_RESPONSE               ( 0x09 )    /* Write operation completed */
#define BT_CONTROL_GATT_EVENT_INDICATION                   ( 0x0a )    /* indication from peer */
#define BT_CONTROL_GATT_EVENT_NOTIFICATION                 ( 0x0b )    /* notification from peer */
#define BT_CONTROL_GATT_EVENT_READ_ERROR                   ( 0x0c )    /* GATT Read operation error */
#define BT_CONTROL_GATT_EVENT_WRITE_ERROR                  ( 0x0d )    /* GATT Write operation error */
#define BT_CONTROL_GATT_EVENT_WRITE_COMPLETE               ( 0x10 )    /* GATT Write operation completed */


/*
 * Define events from the SPP profile
 */
#define BT_CONTROL_SPP_EVENT_CONNECTED                     (  0x01 )    /* SPP connection opened */
#define BT_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND             (  0x02 )    /* SDP record with SPP service not found */
#define BT_CONTROL_SPP_EVENT_CONNECTION_FAILED             (  0x03 )    /* Connection attempt failed  */
#define BT_CONTROL_SPP_EVENT_DISCONNECTED                  (  0x04 )    /* SPP connection closed */
#define BT_CONTROL_SPP_EVENT_TX_COMPLETE                   (  0x05 )    /* Data packet has been queued for transmission */
#define BT_CONTROL_SPP_EVENT_RX_DATA                       (  0x06 )    /* Data packet has been queued for transmission */
#define BT_CONTROL_SPP_EVENT_STATUS                        (  0x07 )    /* SPP connect status*/


/*
* Define events from the Audio profile
*/
#define BT_CONTROL_AUDIO_EVENT_COMMAND_COMPLETE            (  0x00 )    /* Command complete event for the requested operation */
#define BT_CONTROL_AUDIO_EVENT_COMMAND_STATUS              (  0x01 )    /* Command status event for the requested operation */
#define BT_CONTROL_AUDIO_EVENT_CONNECTED                   (  0x02 )    /* Audio connection opened */
#define BT_CONTROL_AUDIO_EVENT_SERVICE_NOT_FOUND           (  0x03 )    /* SDP record with audio service not found */
#define BT_CONTROL_AUDIO_EVENT_CONNECTION_FAILED           (  0x04 )    /* Connection attempt failed  */
#define BT_CONTROL_AUDIO_EVENT_DISCONNECTED                (  0x05 )    /* Audio connection closed */
#define BT_CONTROL_AUDIO_EVENT_REQUEST_DATA                (  0x06 )    /* Request for audio pcm sample data */
#define BT_CONTROL_AUDIO_EVENT_STARTED                     (  0x07 )    /* Command for audio start succeeded */
#define BT_CONTROL_AUDIO_EVENT_STOPPED                     (  0x08 )    /* Command for audio stop completed */

/*
* Define events from the Sink profile
*/
#define BT_CONTROL_SINK_EVENT_COMMAND_COMPLETE            (  0x00 )    /* Command complete event for the requested operation */
#define BT_CONTROL_SINK_EVENT_COMMAND_STATUS              (  0x01 )    /* Command status event for the requested operation */
#define BT_CONTROL_SINK_EVENT_CONNECTED                   (  0x02 )    /* Audio connection opened */
#define BT_CONTROL_SINK_EVENT_SERVICE_NOT_FOUND           (  0x03 )    /* SDP record with audio service not found */
#define BT_CONTROL_SINK_EVENT_CONNECTION_FAILED           (  0x04 )    /* Connection attempt failed  */
#define BT_CONTROL_SINK_EVENT_DISCONNECTED                (  0x05 )    /* Audio connection closed */
#define BT_CONTROL_SINK_EVENT_STARTED                     (  0x06 )    /* Command for audio start succeeded */
#define BT_CONTROL_SINK_EVENT_STOPPED                     (  0x07 )    /* Command for audio stop completed */
#define BT_CONTROL_SINK_EVENT_RECEIVE_DATA                (  0x0A )    /* Receive audio data request */

/*
* Define events from the Avrc controller profile
*/
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED         0x01    /* AVRC controller connected event */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED      0x02    /* AVRC controller disconnected event */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_INFO        0x03    /* AVRC controller track information event */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS       0x04    /* AVRC controller play status change event */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION     0x05    /* AVRC controller play position change event */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE    0x0A    /* AVRC controller player setting changed */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_VOLUME_LEVEL      0x20    /* AVRC contoller volume level command received event */
#define BT_CONTROL_AVRC_CONTROLLER_EVENT_CMD_STATUS        0xFF    /* Results status for AVRC commands */

/*
* Define events from the Avrc target profile
*/
#define BT_CONTROL_AVRC_TARGET_EVENT_CONNECTED             0x01    /* AVRC target connected event */
#define BT_CONTROL_AVRC_TARGET_EVENT_DISCONNECTED          0x02    /* AVRC target disconnected event */
#define BT_CONTROL_AVRC_TARGET_EVENT_PLAY                  0x03    /* Play command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_STOP                  0x04    /* Stop command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_PAUSE                 0x05    /* Pause command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_NEXT_TRACK            0x06    /* Next track command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_PREVIOUS_TRACK        0x07    /* Previous command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_BEGIN_FAST_FORWARD    0x08    /* Begin fast forward command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_END_FAST_FORWARD      0x09    /* End fast forward command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_BEGIN_REWIND          0x0A    /* Begin rewind command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_END_REWIND            0x0B    /* End rewind command received event */
#define BT_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL          0x0C    /* Volume level command received event */

/*
* Define events from the HFP profile
*/
#define BT_CONTROL_HF_EVENT_COMMAND_STATUS                 0x00    /* HFP command status */
#define BT_CONTROL_HF_EVENT_OPEN                           0x01    /* HS connection opened or connection attempt failed  */
#define BT_CONTROL_HF_EVENT_CLOSE                          0x02    /* HS connection closed */
#define BT_CONTROL_HF_AUDIO_EVENT_OPEN                     0x04    /* Audio connection open */
#define BT_CONTROL_HF_AUDIO_EVENT_CLOSE                    0x05    /* Audio connection closed */
#define BT_CONTROL_HF_AUDIO_EVENT_REQUESTED                0x06    /* Audio connection requested */
#define BT_CONTROL_HF_EVENT_CONNECTED                      0x07    /* HS connection connected */
#define BT_CONTROL_HF_EVENT_AG_FEATURE                     0x08    /* HS peer device AG feature */
#define BT_CONTROL_HF_EVENT_AG_INDICATOR                   0x2A    /* AG indicator event */

/*
* Subcommands AT resoponses defined with AT Commands
*/
#define BT_CONTROL_HF_AT_EVENT_BASE                        0x20    /* send AT command and supporting data */
#define BT_CONTROL_HF_AT_EVENT_OK                          0x20    /* OK response received to previous AT command */
#define BT_CONTROL_HF_AT_EVENT_ERROR                       0x21    /* ERROR response received */
#define BT_CONTROL_HF_AT_EVENT_CMEE                        0x22    /* Extended error codes response */
#define BT_CONTROL_HF_AT_EVENT_RING                        0x23    /* RING indicator */
#define BT_CONTROL_HF_AT_EVENT_VGS                         0x24
#define BT_CONTROL_HF_AT_EVENT_VGM                         0x25
#define BT_CONTROL_HF_AT_EVENT_CCWA                        0x26
#define BT_CONTROL_HF_AT_EVENT_CHLD                        0x27
#define BT_CONTROL_HF_AT_EVENT_CIND                        0x28
#define BT_CONTROL_HF_AT_EVENT_CLIP                        0x29
#define BT_CONTROL_HF_AT_EVENT_CIEV                        0x2A
#define BT_CONTROL_HF_AT_EVENT_BINP                        0x2B
#define BT_CONTROL_HF_AT_EVENT_BVRA                        0x2C
#define BT_CONTROL_HF_AT_EVENT_BSIR                        0x2D
#define BT_CONTROL_HF_AT_EVENT_CNUM                        0x2E
#define BT_CONTROL_HF_AT_EVENT_BTRH                        0x2F
#define BT_CONTROL_HF_AT_EVENT_COPS                        0x30
#define BT_CONTROL_HF_AT_EVENT_CLCC                        0x31
#define BT_CONTROL_HF_AT_EVENT_BIND                        0x32
#define BT_CONTROL_HF_AT_EVENT_UNAT                        0x33
#define BT_CONTROL_HF_AT_EVENT_MAX                         0x33    /* Maximum AT event value */

/*
* Define event for Handsfree AG implementation
*/
#define BT_CONTROL_AG_EVENT_OPEN                           0x01
#define BT_CONTROL_AG_EVENT_CLOSE                          0x02
#define BT_CONTROL_AG_EVENT_CONNECTED                      0x03
#define BT_CONTROL_AG_EVENT_AUDIO_OPEN                     0x04
#define BT_CONTROL_AG_EVENT_AUDIO_CLOSE                    0x05
#define BT_CONTROL_AG_EVENT_RECEIVE_AT                     0x06


#define BT_LE_ADV_STATE_NO_DISCOVERABLE                    0
#define BT_LE_ADV_STATE_HIGH_DISCOVERABLE                  1
#define BT_LE_ADV_STATE_LOW_DISCOVERABLE                   2

#define BT_LE_SCAN_STATE_NO_SCAN                           0
#define BT_LE_SCAN_STATE_HIGH_SCAN                         1
#define BT_LE_SCAN_STATE_LOW_SCAN                          2

//Little Endian format
#define STREAM_TO_UINT8(u8, p)   {u8 = (uint8_t)(*(p)); (p) += 1;}
#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define STREAM_TO_UINT24(u32, p) {u32 = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) + ((((uint32_t)(*((p) + 2)))) << 16) ); (p) += 3;}
#define STREAM_TO_UINT32(u32, p) {u32 = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) + ((((uint32_t)(*((p) + 2)))) << 16) + ((((uint32_t)(*((p) + 3)))) << 24)); (p) += 4;}


#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}
#define UINT24_TO_STREAM(p, u24) {*(p)++ = (uint8_t)(u24); *(p)++ = (uint8_t)((u24) >> 8); *(p)++ = (uint8_t)((u24) >> 16);}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}


//Big Endian format
#define BE_STREAM_TO_UINT8(u8, p)   {u8 = (uint8_t)(*(p)); (p) += 1;}
#define BE_STREAM_TO_UINT16(u16, p) {u16 = (uint16_t)(((uint16_t)(*(p)) << 8) + (uint16_t)(*((p) + 1))); (p) += 2;}

#define UINT32_TO_BE_STREAM(p, u32) {*(p)++ = (uint8_t)((u32) >> 24);  *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)(u32); }
#define UINT24_TO_BE_STREAM(p, u24) {*(p)++ = (uint8_t)((u24) >> 16); *(p)++ = (uint8_t)((u24) >> 8); *(p)++ = (uint8_t)(u24);}
#define UINT16_TO_BE_STREAM(p, u16) {*(p)++ = (uint8_t)((u16) >> 8); *(p)++ = (uint8_t)(u16);}
#define UINT8_TO_BE_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}

#define STREAM_TO_BDADDR(a, p)   {register int ijk; register uint8_t *pbda = (uint8_t *)a + 6 - 1; for (ijk = 0; ijk < 6; ijk++) *pbda-- = *p++;}

#endif/* __MODULES_BLUETOOTH_HAL_BCM20706_BT_UTIL_H */


