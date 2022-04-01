/****************************************************************************
 * system/ambient/ambient.h
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

#ifndef __APPS_SYSTEM_AMBIENT_H
#define __APPS_SYSTEM_AMBIENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ambient field number */

#define AMBIENT_D1      1
#define AMBIENT_D2      2
#define AMBIENT_D3      3
#define AMBIENT_D4      4
#define AMBIENT_D5      5
#define AMBIENT_D6      6
#define AMBIENT_D7      7
#define AMBIENT_D8      8
#define AMBIENT_LAT     9
#define AMBIENT_LNG     10
#define AMBIENT_MAXNUM  10

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct ambient_ctx_s ambient_ctx_t;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ambient_create
 *
 * Description:
 *  This function creates an Ambient context.
 *
 * Input Parameters
 *   channel   - A number of user's channel ID
 *   write_key - A string of user's write key
 *
 * Returned Value:
 *   Returns a pointer to an Ambient context on success.
 *   Returns NULL on failure.
 *
 ****************************************************************************/

ambient_ctx_t *ambient_create(int channel, FAR const char *write_key);

/****************************************************************************
 * Name: ambient_set
 *
 * Description:
 *  This function set a data into the specified field.
 *
 * Input Parameters
 *   ctx       - A pointer to an Ambient context
 *   field     - A field number to input a data
 *   data      - A string of data
 *
 * Returned Value:
 *   0: if the set operation completed successfully
 *   -ENXIO: Not exist a pointer to an Ambient context
 *   -EINVAL: Invalid argument
 *
 ****************************************************************************/

int ambient_set(FAR ambient_ctx_t *ctx, int field, FAR const char *data);

/****************************************************************************
 * Name: ambient_set_int
 *
 * Description:
 *  This function set a data into the specified field.
 *
 * Input Parameters
 *   ctx       - A pointer to an Ambient context
 *   field     - A field number to input a data
 *   data      - A integer of data
 *
 * Returned Value:
 *   0: if the set operation completed successfully
 *   -ENXIO: Not exist a pointer to an Ambient context
 *   -EINVAL: Invalid argument
 *
 ****************************************************************************/

int ambient_set_int(FAR ambient_ctx_t *ctx, int field, int data);

/****************************************************************************
 * Name: ambient_set_double
 *
 * Description:
 *  This function set a data into the specified field.
 *
 * Input Parameters
 *   ctx       - A pointer to an Ambient context
 *   field     - A field number to input a data
 *   data      - A double float of data
 *
 * Returned Value:
 *   0: if the set operation completed successfully
 *   -ENXIO: Not exist a pointer to an Ambient context
 *   -EINVAL: Invalid argument
 *
 ****************************************************************************/

int ambient_set_double(FAR ambient_ctx_t *ctx, int field, double data);

/****************************************************************************
 * Name: ambient_clear
 *
 * Description:
 *  This function clears a data into the specified field.
 *
 * Input Parameters
 *   ctx       - A pointer to an Ambient context
 *   field     - A field number to clear a data
 *
 * Returned Value:
 *   0: if the clear operation completed successfully
 *   -ENXIO: Not exist a pointer to an Ambient context
 *   -EINVAL: Invalid argument
 *
 ****************************************************************************/

int ambient_clear(FAR ambient_ctx_t *ctx, int field);

/****************************************************************************
 * Name: ambient_send
 *
 * Description:
 *  This function sends all of the data to the Ambient server.
 *
 * Input Parameters
 *   ctx       - A pointer to an Ambient context
 *
 * Returned Value:
 *   0: if the send operation completed successfully
 *   -ENXIO: Not exist a pointer to an Ambient context
 *
 ****************************************************************************/

int ambient_send(FAR ambient_ctx_t *ctx);

/****************************************************************************
 * Name: ambient_delete
 *
 * Description:
 *  This function deletes an Ambient context.
 *
 * Input Parameters
 *   ctx       - A pointer to an Ambient context
 *
 * Returned Value:
 *   0: if the delete operation completed successfully
 *   -ENXIO: Not exist a pointer to an Ambient context
 *
 ****************************************************************************/

int ambient_delete(FAR ambient_ctx_t *ctx);

#endif /* __APPS_SYSTEM_AMBIENT_H */
