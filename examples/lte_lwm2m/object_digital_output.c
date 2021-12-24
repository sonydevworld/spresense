
/*
 * Implements an object for digital output
 *
 *                  Multiple
 * Object |  ID   | Instances | Mandatory |
 *  Test  | 3201  |    Yes    |    No     |
 *
 *  Resources:
 *
 *          Name         | ID | Oper. | Inst. | Mand.|  Type   | Range | Units |
 *  ---------------------+----+-------+-------+------+---------+-------+-------+
 *  Digital Output State |5550|   RW  | Single|  Yes | Boolean |  1bit |       |
 *              Polarity |5551|   RW  | Single|  No  | Boolean |  1bit |       |
 *  Application Type     |5750|   RW  | Single|  No  | String  |       |       |
 *  Timestamp            |5518|   R   | Single|  No  | Time    |       |       |
 *  Fractional Timestamp |6050|   R   | Single|  No  | Float   |  0..1 | s     |
 */

#include "liblwm2m.h"
#include "lwm2mclient.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

// Resource Id's:
#define RES_M_STATE           5550
#define RES_M_POLARITY        5551
#define RES_M_APPTYPE         5750
#define RES_O_TIMESTAMP       5518
#define RES_O_TIMESTAMP_FRAC  6050

typedef struct _prv_instance_
{
    struct _prv_instance_ * next;   // matches lwm2m_list_t::next
    uint16_t shortID;               // used as a pin number
    bool     state;
    bool     polarity;
    char     *str;
    int64_t  timestamp;
    float    timestamp_frac;
} prv_instance_t;

static uint8_t prv_delete(lwm2m_context_t *contextP,
                          uint16_t id,
                          lwm2m_object_t * objectP);
static uint8_t prv_create(lwm2m_context_t *contextP,
                          uint16_t instanceId,
                          int numData,
                          lwm2m_data_t * dataArray,
                          lwm2m_object_t * objectP);

static uint8_t prv_read(lwm2m_context_t *contextP,
                        uint16_t instanceId,
                        int * numDataP,
                        lwm2m_data_t ** dataArrayP,
                        lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i;

    /* unused parameter */
    (void)contextP;

    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(5);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 5;
        (*dataArrayP)[0].id = RES_M_STATE;
        (*dataArrayP)[1].id = RES_M_POLARITY;
        (*dataArrayP)[2].id = RES_M_APPTYPE;
        (*dataArrayP)[3].id = RES_O_TIMESTAMP;
        (*dataArrayP)[4].id = RES_O_TIMESTAMP_FRAC;
    }

    for (i = 0 ; i < *numDataP ; i++)
    {
        if ((*dataArrayP)[i].type == LWM2M_TYPE_MULTIPLE_RESOURCE)
        {
            return COAP_404_NOT_FOUND;
        }

        switch ((*dataArrayP)[i].id)
        {
            case RES_M_STATE:
                lwm2m_data_encode_bool(targetP->state, *dataArrayP + i);
                break;
            case RES_M_POLARITY:
                lwm2m_data_encode_bool(targetP->polarity, *dataArrayP + i);
                break;
            case RES_M_APPTYPE:
                lwm2m_data_encode_string(targetP->str, *dataArrayP + i);
                break;
            case RES_O_TIMESTAMP:
                lwm2m_data_encode_int(targetP->timestamp, *dataArrayP + i);
                break;
            case RES_O_TIMESTAMP_FRAC:
                lwm2m_data_encode_float(targetP->timestamp_frac, *dataArrayP + i);
                break;
            default:
                return COAP_404_NOT_FOUND;
        }
    }

    return COAP_205_CONTENT;
}

static uint8_t prv_write(lwm2m_context_t *contextP,
                         uint16_t instanceId,
                         int numData,
                         lwm2m_data_t * dataArray,
                         lwm2m_object_t * objectP,
                         lwm2m_write_type_t writeType)
{
    prv_instance_t * targetP;
    int i;
    int ret;
    struct timespec time;

    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (writeType == LWM2M_WRITE_REPLACE_INSTANCE)
    {
        uint8_t result = prv_delete(contextP, instanceId, objectP);
        if (result == COAP_202_DELETED)
        {
            result = prv_create(contextP, instanceId, numData, dataArray, objectP);
            if (result == COAP_201_CREATED)
            {
                result = COAP_204_CHANGED;
            }
        }
        return result;
    }

    for (i = 0 ; i < numData ; i++)
    {
        /* No multiple instance resources */
        if (dataArray[i].type == LWM2M_TYPE_MULTIPLE_RESOURCE) return  COAP_404_NOT_FOUND;

        switch (dataArray[i].id)
        {
            case RES_M_STATE:
                lwm2m_data_decode_bool(dataArray + i, &targetP->state);
                break;
            case RES_M_POLARITY:
                lwm2m_data_decode_bool(dataArray + i, &targetP->polarity);
                break;
            case RES_M_APPTYPE:
                if (dataArray[i].type == LWM2M_TYPE_STRING || dataArray[i].type == LWM2M_TYPE_OPAQUE)
                {
                    char *tmp;
                    tmp = targetP->str;
                    targetP->str = lwm2m_malloc(dataArray[i].value.asBuffer.length + 1);
                    strncpy(targetP->str, (char*)dataArray[i].value.asBuffer.buffer, dataArray[i].value.asBuffer.length);
                    targetP->str[dataArray[i].value.asBuffer.length] = '\0';
                    lwm2m_free(tmp);
                    break;
                }
                else
                {
                    return COAP_400_BAD_REQUEST;
                }
            default:
                return COAP_404_NOT_FOUND;
        }
    }

    /* Output digital pin */

    gpio_write(instanceId, targetP->state, targetP->polarity);

    /* Update timestamp */
    ret = clock_gettime(CLOCK_REALTIME, &time);
    if (ret == OK)
    {
        targetP->timestamp = time.tv_sec + time.tv_nsec / 1000000000;
        targetP->timestamp -= get_utc_offset_sec();
        time.tv_nsec %= 1000000000;
        targetP->timestamp_frac = (float)time.tv_nsec / 1000000000.0;
    }

    return COAP_204_CHANGED;
}

static uint8_t prv_delete(lwm2m_context_t *contextP,
                          uint16_t id,
                          lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;

    /* unused parameter */
    (void)contextP;

    objectP->instanceList = lwm2m_list_remove(objectP->instanceList, id, (lwm2m_list_t **)&targetP);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    lwm2m_free(targetP->str);
    targetP->str = NULL;
    lwm2m_free(targetP);

    return COAP_202_DELETED;
}

static uint8_t prv_create(lwm2m_context_t *contextP,
                          uint16_t instanceId,
                          int numData,
                          lwm2m_data_t * dataArray,
                          lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    uint8_t result;
    int ret;

    targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
    if (NULL == targetP) return COAP_500_INTERNAL_SERVER_ERROR;

    memset(targetP, 0, sizeof(prv_instance_t));

    targetP->shortID = instanceId;
    objectP->instanceList = LWM2M_LIST_ADD(objectP->instanceList, targetP);

    ret = gpio_output_config(instanceId);
    if (ret != 0)
    {
        prv_delete(contextP, instanceId, objectP);
        return COAP_402_BAD_OPTION;
    }

    targetP->state = false;
    targetP->polarity = false;
    targetP->str = lwm2m_malloc(1);
    targetP->str[0] = '\0';
    targetP->timestamp = 0;
    targetP->timestamp_frac = 0.0;

    result = prv_write(contextP, instanceId, numData, dataArray, objectP, LWM2M_WRITE_REPLACE_RESOURCES);

    if (result != COAP_204_CHANGED)
    {
        (void)prv_delete(contextP, instanceId, objectP);
    }
    else
    {
        result = COAP_201_CREATED;
    }

    return result;
}

void display_digital_output_object(lwm2m_object_t * object)
{
    fprintf(stdout, "  /%u: Digital Output object, instances:\r\n", object->objID);
    prv_instance_t * instance = (prv_instance_t *)object->instanceList;
    while (instance != NULL)
    {
        fprintf(stdout, "    /%u/%u: shortId: %u state:%d\r\n",
                object->objID, instance->shortID,
                instance->shortID, instance->state);
        instance = (prv_instance_t *)instance->next;
    }
}

lwm2m_object_t * get_digital_output_object(void)
{
    lwm2m_object_t * object;

    object = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != object)
    {
        memset(object, 0, sizeof(lwm2m_object_t));

        object->objID = LWM2M_DIGITAL_OUTPUT_OBJECT_ID;
#ifndef LWM2M_VERSION_1_0
        // Not required, but useful for testing.
        object->versionMajor = 1;
        object->versionMinor = 1;
#endif
        object->readFunc = prv_read;
        object->writeFunc = prv_write;
        object->createFunc = prv_create;
        object->deleteFunc = prv_delete;
    }

    return object;
}

void free_digital_output_object(lwm2m_object_t * object)
{
    prv_instance_t * targetP = (prv_instance_t *)object->instanceList;
    while (targetP != NULL)
    {
        prv_instance_t * next = targetP->next;

        lwm2m_free(targetP->str);

        targetP = next;
    }
    LWM2M_LIST_FREE(object->instanceList);
    lwm2m_free(object);
}
