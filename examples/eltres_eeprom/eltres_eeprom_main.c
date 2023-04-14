/****************************************************************************
 * examples/eltres_eeprom/eltres_eeprom_main.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <CXM150x_APITypeDef.h>
#include <CXM150x_SYS.h>
#include <CXM150x_GNSS.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EEPROM Lfour ID address */

#define LFROUID_UPPER 0x0008
#define LFROUID_LOWER 0x000C

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct eeprom_addr_name_s
{
  int32_t     addr;
  const char *name;
};

struct eeprom_addr_value_s
{
  int32_t     addr;
  int32_t     value;
};

struct subcmd_s
{
  const char *name;
  int (*func)(int argc, char *argv[]);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The list of EEPROM address-name pairs. */

static struct eeprom_addr_name_s s_eeprom_map[] =
{
  { 0x0000, "DEVICE ID UPPER" },
  { 0x0004, "DEVICE ID_LOWER" },
  { 0x0008, "LFOUR ID UPPER" },
  { 0x000C, "LFOUR ID LOWER" },
  { 0x0020, "MODULE_SELECT" },
  { 0x0028, "MODULE_BRANCH" },
  { 0x0204, "SM_TOUT" },
  { 0x0208, "FET_TOUT" },
  { 0x020C, "FWAT_TOUT" },
  { 0x0218, "POW_ENABLE_REMAIN_OFFSET" },
  { 0x021C, "INT_OUT1" },
  { 0x0220, "INT_OUT2" },
  { 0x0224, "INT_OUT1_EVT" },
  { 0x0230, "AUTO_PERIODIC_SELECT" },
  { 0x0234, "WAKEUP_CTRL" },
  { 0x023C, "GNSS_STT_TOUT" },
  { 0x0250, "dataGridPositionOffset" },
  { 0x0254, "dataGridPositionRange" },
  { 0x0258, "arGridPositionOffset" },
  { 0x025C, "arGridPositionRange" },
  { 0x0260, "AR_POS_ENABLE" },
  { 0x0264, "PROFILE_SELECT" },
  { 0x0274, "regionalParameter" },
  { 0x0400, "p1Enabled" },
  { 0x0404, "p1StartTime" },
  { 0x0408, "p1EndTime" },
  { 0x040C, "p1BurstInterval" },
  { 0x0410, "p1TxWindowSize" },
  { 0x0414, "p1TxWindowOffset" },
  { 0x0424, "p1ArBurstNumber" },
  { 0x0428, "p1ArDataBurstDelay" },
  { 0x042C, "p1ArRatio" },
  { 0x0434, "p1DT_RX_SETUP" },
  { 0x0438, "p1AR_RX_SETUP" },
  { 0x043C, "p1EPM_INIC" },
  { 0x0440, "p1EPM_MINC" },
  { 0x0444, "p1EPM_MAXC" },
  { 0x0450, "p1POW_MODE" },
  { 0x0454, "p1INT_OUT1" },
  { 0x0500, "p2Enabled" },
  { 0x0504, "p2StartTime" },
  { 0x0508, "p2EndTime" },
  { 0x050C, "p2BurstInterval" },
  { 0x0510, "p2TxWindowSize" },
  { 0x0514, "p2TxWindowOffset" },
  { 0x0524, "p2ArBurstNumber" },
  { 0x0528, "p2ArDataBurstDelay" },
  { 0x052C, "p2ArRatio" },
  { 0x0534, "p2DT_RX_SETUP" },
  { 0x0538, "p2AR_RX_SETUP" },
  { 0x053C, "p2EPM_INIC" },
  { 0x0540, "p2EPM_MINC" },
  { 0x0544, "p2EPM_MAXC" },
  { 0x0550, "p2POW_MODE" },
  { 0x0554, "p2INT_OUT1" },
  { 0x0600, "evEnabled" },
  { 0x060C, "evBurstInterval" },
  { 0x0610, "evTxWindowSize" },
  { 0x0614, "evTxWindowOffset" },
  { 0x061C, "evDataBurstNumber" },
  { 0x0624, "evArBurstNumber" },
  { 0x0628, "evArDataBurstDelay" },
  { 0x062C, "evArRatio" },
  { 0x0634, "evDT_RX_SETUP" },
  { 0x0638, "evAR_RX_SETUP" },
  { 0x063C, "evEPM_INIC" },
  { 0x0640, "evEPM_MINC" },
  { 0x0644, "evEPM_MAXC" },
  { 0x0650, "evPOW_MODE" },
  { 0x0654, "evINT_OUT1" },
  { 0x0750, "MIN_DSLP_TIME" },
  { 0x0754, "FET_DSLP_TOUT" },
  { 0x0758, "DSLP_BUP" },
  { 0x07A0, "AUTOPLD_SRC_SELECT" },
  { 0x07A4, "AUTOPLD_COLLECT" },
  { 0x07A8, "AUTOPLD_LAT_BASE" },
  { 0x07AC, "AUTOPLD_LON_BASE" },
  { 0x07B0, "AUTOPLD_LAT_RANGE" },
  { 0x07B4, "AUTOPLD_LON_RANGE" },
  { 0x07B8, "AUTOPLD_LAT_RES" },
  { 0x07BC, "AUTOPLD_LAT_RES" },
  { 0x07C8, "AUTOPLD_HEIGHT_OFFSET" },
  { 0x07CC, "AUTOPLD_USER_DATA" },
  { 0x0804, "AUTOPLD_SRC1_BIT_POS" },
  { 0x080C, "AUTOPLD_SRC2_BIT_POS" },
  { 0x0814, "AUTOPLD_SRC3_BIT_POS" },
  { 0x081C, "AUTOPLD_SRC4_BIT_POS" },
  { 0x0820, "AUTOPLD_SRC5_BIT_WIDTH" },
  { 0x0824, "AUTOPLD_SRC5_BIT_POS" },
  { 0x0828, "AUTOPLD_SRC6_BIT_WIDTH" },
  { 0x082C, "AUTOPLD_SRC6_BIT_POS" },
  { 0x0830, "AUTOPLD_SRC7_BIT_WIDTH" },
  { 0x0834, "AUTOPLD_SRC7_BIT_POS" },
  { 0x083C, "AUTOPLD_SRC8_BIT_POS" },
  { 0x0840, "AUTOPLD_SRC9_BIT_WIDTH" },
  { 0x0844, "AUTOPLD_SRC9_BIT_POS" },
};

/* The user-customized list of EEPROM address-value pairs.
 * The following table is an example and should be overwritten according to
 * your requirements.
 */

static struct eeprom_addr_value_s s_userlist[] =
{
  { 0x07A0, 0x000003FE },
  { 0x07A4, 0x00000005 },
  { 0x07A8, 0xECB02700 },
  { 0x07AC, 0xD9604E00 },
  { 0x07B0, 0x269FB200 },
  { 0x07B4, 0x4D3F6400 },
  { 0x07B8, 0x00000020 },
  { 0x07BC, 0x00000020 },
  { 0x07C8, 0x000003E8 },
  { 0x07CC, 0x000003FF },
  { 0x0800, 0x00000008 },
  { 0x0804, 0x0000000B },
  { 0x0808, 0x0000000A },
  { 0x080C, 0x00000013 },
  { 0x0810, 0x0000000E },
  { 0x0814, 0x00000029 },
  { 0x0818, 0x0000000C },
  { 0x081C, 0x0000001D },
  { 0x0820, 0x0000000E },
  { 0x0824, 0x00000037 },
  { 0x0828, 0x00000019 },
  { 0x082C, 0x0000005F },
  { 0x0830, 0x0000001A },
  { 0x0834, 0x00000045 },
  { 0x0838, 0x00000008 },
  { 0x083C, 0x00000078 },
  { 0x0840, 0x0000000B },
  { 0x0844, 0x00000000 },
};

static CXM150xFATALMessage g_fatalmessage_info;
static CXM150xEventBufferOverflow g_buffer_overflow_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * fatal_message_event_callback
 ****************************************************************************/

static void fatal_message_event_callback(void *info, uint32_t id)
{
  CXM150xFATALMessage *fatal_info = (CXM150xFATALMessage *)info;
  printf("FATAL Message Event: %s\n", fatal_info->m_str);
}

/****************************************************************************
 * event_buffer_overflow_callback
 ****************************************************************************/

static void event_buffer_overflow_callback(void *info, uint32_t id)
{
  printf("event buffer overflow\n");
}

/****************************************************************************
 * show_usage
 ****************************************************************************/

static void show_usage(const char *progname)
{
  printf("Usage: %s command\n", progname);
  printf(" dump                  : Dump all EEPROM data\n");
  printf(" get <address>         : Read data from an EEPROM address\n");
  printf(" put <address> <value> : Write any value to an EEPROM address\n");
  printf(" setlist               : Write EEPROM data pre-set by the user\n");
  printf(" version               : Show version information\n");
}

/****************************************************************************
 * eeprom_dump
 ****************************************************************************/

static int eeprom_dump(int argc, char *argv[])
{
  int i;
  uint64_t                   lfour_id = 0ull;
  CXM150x_return_code        retcode;
  CmdResGetCXM150xEEPROMData eeprom_data;

  /* Get Lfour ID */

  retcode = get_CXM150x_EEPROM_data(LFROUID_UPPER, &eeprom_data, NULL);
  if (retcode != RETURN_OK)
    {
      goto errout;
    }

  lfour_id = (uint64_t)eeprom_data.m_num << 32;

  retcode = get_CXM150x_EEPROM_data(LFROUID_LOWER, &eeprom_data, NULL);
  if (retcode != RETURN_OK)
    {
      goto errout;
    }

  lfour_id |= eeprom_data.m_num;

  printf("LfourID = %010llX\n", lfour_id);

  /* Dump EEPROM address map */

  printf("Name, Address, Value(HEX), Value(DEC)\n");

  for (i = 0; i < sizeof(s_eeprom_map) / sizeof(s_eeprom_map[0]); i++)
    {
      retcode = get_CXM150x_EEPROM_data(s_eeprom_map[i].addr,
                                        &eeprom_data, NULL);
      if (retcode != RETURN_OK)
        {
          goto errout;
        }

      printf("%s, 0x%04lX, 0x%lX, %ld\n", s_eeprom_map[i].name,
                                          s_eeprom_map[i].addr,
                                          eeprom_data.m_num,
                                          eeprom_data.m_num);
    }

  return OK;

errout:
  printf("ERROR: get_CXM150x_EEPROM_data (%d)\n", retcode);
  return ERROR;
}

/****************************************************************************
 * eeprom_get
 ****************************************************************************/

static int eeprom_get(int argc, char *argv[])
{
  CXM150x_return_code        retcode;
  CmdResGetCXM150xEEPROMData eeprom_data;
  int32_t                    eeprom_addr = 0;

  if (argc < 3)
    {
      show_usage(argv[0]);
      return ERROR;
    }

  eeprom_addr = strtol(argv[2], NULL, 0);

  retcode = get_CXM150x_EEPROM_data(eeprom_addr, &eeprom_data, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: get_CXM150x_EEPROM_data (%d)\n", retcode);
      return ERROR;
    }

  printf("EEPROM 0x%04lx -> 0x%lX (%ld)\n", eeprom_addr,
                                            eeprom_data.m_num,
                                            eeprom_data.m_num);

  return OK;
}

/****************************************************************************
 * eeprom_put
 ****************************************************************************/

static int eeprom_put(int argc, char *argv[])
{
  CXM150x_return_code        retcode;
  CmdResSetCXM150xMode       res_set_mode;
  CmdResSetCXM150xEEPROMData res_set_eep;
  CXM150xEEPROMSetData       eeprom_param;

  if (argc < 4)
    {
      show_usage(argv[0]);
      return ERROR;
    }

  /* Transit to EEPROM writable mode */

  retcode = set_CXM150x_mode(CXM150x_MODE_EEPROM_WRITE, &res_set_mode, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: set_CXM150x_mode (%d)\n", retcode);
    }

  /* Write to EEPROM */

  eeprom_param.m_offset_address = strtol(argv[2], NULL, 0);
  eeprom_param.m_val            = strtol(argv[3], NULL, 0);

  retcode = set_CXM150x_EEPROM_data(&eeprom_param, &res_set_eep, NULL);

  if ((retcode != RETURN_OK) ||
      (res_set_eep.m_result != CXM150x_RESPONSE_OK))
    {
      printf("ERROR: set_CXM150x_EEPROM_data (ret=%d, res=%ld)\n",
             retcode,
             res_set_eep.m_result);
      return ERROR;
    }

  printf("EEPROM 0x%04lx <- 0x%lX (%ld)\n", eeprom_param.m_offset_address,
                                            eeprom_param.m_val,
                                            eeprom_param.m_val);

  return OK;
}

/****************************************************************************
 * eeprom_setlist
 ****************************************************************************/

static int eeprom_setlist(int argc, char *argv[])
{
  int i;
  CXM150x_return_code        retcode;
  CmdResSetCXM150xMode       res_set_mode;
  CmdResSetCXM150xEEPROMData res_set_eep;
  CXM150xEEPROMSetData       eeprom_param;

  /* Transit to EEPROM writable mode */

  retcode = set_CXM150x_mode(CXM150x_MODE_EEPROM_WRITE, &res_set_mode, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: set_CXM150x_mode (%d)\n", retcode);
    }

  /* Write to EEPROM */

  for (i = 0; i < sizeof(s_userlist) / sizeof(s_userlist[0]); i++)
    {
      eeprom_param.m_offset_address = s_userlist[i].addr;
      eeprom_param.m_val            = s_userlist[i].value;

      retcode = set_CXM150x_EEPROM_data(&eeprom_param, &res_set_eep, NULL);
      if ((retcode != RETURN_OK) ||
          (res_set_eep.m_result != CXM150x_RESPONSE_OK))
        {
          printf("ERROR: set_CXM150x_EEPROM_data (ret=%d, res=%ld)\n",
                 retcode,
                 res_set_eep.m_result);
          return ERROR;
        }

      printf("EEPROM 0x%04lx <- 0x%lX (%ld)\n",
             eeprom_param.m_offset_address,
             eeprom_param.m_val,
             eeprom_param.m_val);
    }

  return OK;
}

/****************************************************************************
 * get_version
 ****************************************************************************/

static int get_version(int argc, char *argv[])
{
  int i;
  CXM150x_return_code                 retcode;
  CmdResSetCXM150xMode                res_set_mode;
  CmdResGetCXM150xBootloaderVersion   bootloader_ver;
  CmdResGetCXM150xFirmwareVersion     fw_ver;
  CmdResGetCXM150xGNSSFirmwareVersion gnss_ver;
  CmdResGetCXM150xAPIVersion          api_ver;

  /* Set normal mode */

  retcode = set_CXM150x_mode(CXM150x_MODE_NORMAL, &res_set_mode, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: set_CXM150x_mode (%d)\n", retcode);
    }

  /* Get version information */

  retcode = get_CXM150x_Bootloader_version(NULL, &bootloader_ver, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: get_CXM150x_Bootloader_version (%d)\n", retcode);
    }
  else
    {
      printf("Bootloader : %s / %s\n", bootloader_ver.m_version,
                                       bootloader_ver.m_build_date);
    }

  retcode = get_CXM150x_firmware_version(NULL, &fw_ver, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: get_CXM150x_firmware_version (%d)\n", retcode);
    }
  else
    {
      printf("Firmware : ");
      for (i = 0; i < CXM150x_FIRMWARE_VERSION_LEN_ID; i++)
        {
          printf("%02X", fw_ver.m_id[i]);
        }

      printf(" / %s / %lX / %s\n", fw_ver.m_version,
                                   fw_ver.m_commit_id,
                                   fw_ver.m_build_date);
    }

  retcode = get_CXM150x_GNSS_firmware_version(NULL, &gnss_ver, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: get_CXM150x_GNSS_firmware_version (%d)\n", retcode);
    }
  else
    {
      printf("GNSS FW : %s / %s / %s\n", gnss_ver.m_version,
                                         gnss_ver.m_id1,
                                         gnss_ver.m_id2);
    }

  retcode = get_CXM150x_api_version(NULL, &api_ver, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: get_CXM150x_api_version (%d)\n", retcode);
    }
  else
    {
      printf("API version : %s\n", api_ver.m_version);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * eltres_eeprom_main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  int i;
  int ret = OK;
  struct subcmd_s subcmd[] = {
    { "dump",    eeprom_dump },
    { "get",     eeprom_get },
    { "put",     eeprom_put },
    { "setlist", eeprom_setlist },
    { "version", get_version }
  };

  CXM150x_return_code   retcode;
  CmdResSetCXM150xPower res_set_power;

  /* Check argument */

  if (argc < 2)
    {
      show_usage(argv[0]);
      return ERROR;
    }

  /* FATAL message event callback setting */

  register_CXM150x_FATAL_message_event(&g_fatalmessage_info,
                                       fatal_message_event_callback);
  register_CXM150x_event_buffer_overflow(&g_buffer_overflow_info,
                                         event_buffer_overflow_callback);

  /* Power on */

  retcode = set_CXM150x_power(CXM150x_POWER_ON, &res_set_power, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: set_CXM150x_power (%d)\n", retcode);
    }

  /* Process sub command */

  for (i = 0; i < sizeof(subcmd) / sizeof(subcmd[0]); i++)
    {
      if (!strncmp(subcmd[i].name, argv[1], strlen(subcmd[i].name)))
        {
          ret = subcmd[i].func(argc, argv);
          break;
        }
    }

  /* Power off */

  retcode = set_CXM150x_power(CXM150x_POWER_OFF, &res_set_power, NULL);
  if (retcode != RETURN_OK)
    {
      printf("ERROR: set_CXM150x_power (%d)\n", retcode);
    }

  return ret;
}
