/**
 * examples/i2c_direct/include/debug_printf.h
 *
 *  Copyright (C) 2022, Ixy Design Studio, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef __EXAMPLES_I2C_DIRECT_INCLUDE_DEBUG_PRINTF_H
#define __EXAMPLES_I2C_DIRECT_INCLUDE_DEBUG_PRINTF_H

#ifdef __cplusplus
extern "C"
  {
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/**
 * @brief Debug level logging macro.
 *
 * Macro to expose function, line number as well as desired log message.
 */
#ifdef CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_DEBUG
#define DPRINT_DEBUG(...)    \
{\
printf("DEBUG:   %s L#%d ", __func__, __LINE__);  \
printf(__VA_ARGS__); \
printf("\n"); \
}
#else
#define DPRINT_DEBUG(...)
#endif

/**
 * @brief Debug level trace logging macro.
 *
 * Macro to print message function entry and exit
 */
#ifdef CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_TRACE
#define FUNC_ENTRY    \
{\
printf("FUNC_ENTRY:   %s L#%d \n", __func__, __LINE__);  \
}
#define FUNC_EXIT    \
{\
printf("FUNC_EXIT:   %s L#%d \n", __func__, __LINE__);  \
}
#define FUNC_EXIT_RC(x)    \
{\
printf("FUNC_EXIT:   %s L#%d Return Code : %d \n", __func__, __LINE__, (int)(x));  \
return x; \
}
#else
#define FUNC_ENTRY

#define FUNC_EXIT
#define FUNC_EXIT_RC(x) { return x; }
#endif

/**
 * @brief Info level logging macro.
 *
 * Macro to expose desired log message.
 * Info messages do not include automatic function names and line numbers.
 */
#ifdef CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_INFO
#define DPRINT_INFO(...)    \
{\
printf(__VA_ARGS__); \
printf("\n"); \
}
#else
#define DPRINT_INFO(...)
#endif

/**
 * @brief Warn level logging macro.
 *
 * Macro to expose function, line number as well as desired log message.
 */
#ifdef CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_WARN
#define DPRINT_WARN(...)   \
{ \
printf("WARN:  %s L#%d ", __func__, __LINE__);  \
printf(__VA_ARGS__); \
printf("\n"); \
}
#else
#define DPRINT_WARN(...)
#endif

/**
 * @brief Error level logging macro.
 *
 * Macro to expose function, line number as well as desired log message.
 */
#ifdef CONFIG_EXAMPLES_I2C_DIRECT_DPRINT_ERROR
#define DPRINT_ERROR(...)  \
{ \
printf("ERROR: %s L#%d ", __func__, __LINE__); \
printf(__VA_ARGS__); \
printf("\n"); \
}
#else
#define DPRINT_ERROR(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_I2C_DIRECT_INCLUDE_DEBUG_PRINTF_H */
