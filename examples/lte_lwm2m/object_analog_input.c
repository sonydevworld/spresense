
/*
 * Implements an object for analog input
 *
 *                  Multiple
 * Object |  ID   | Instances | Mandatory |
 *  Test  | 3202  |    Yes    |    No     |
 *
 *  Resources:
 *
 *          Name         | ID | Oper. | Inst. | Mand.|  Type   | Range | Units |
 *  ---------------------+----+-------+-------+------+---------+-------+-------+
 *  Anglog Input Value   |5600|   R   | Single|  Yes | Float   |       |       |
 *  Min Measured Value   |5601|   R   | Single|  No  | Float   |       |       |
 *  Max Measured Value   |5602|   R   | Single|  No  | Float   |       |       |
 *  Min Range Value      |5603|   R   | Single|  No  | Float   |       |       |
 *  Max Range Value      |5604|   R   | Single|  No  | Float   |       |       |
 *  Application Type     |5750|   RW  | Single|  No  | String  |       |       |
 *  Sensor Type          |5751|   R   | Single|  No  | String  |       |       |
 *  Reset Measured Values|5605|   E   | Single|  No  |         |       |       |
 *  Timestamp            |5518|   R   | Single|  No  | Time    |       |       |
 *  Fractional Timestamp |6050|   R   | Single|  No  | Float   | 0..1  | s     |
 *  Quality Indicator    |6042|   R   | Single|  No  | Integer | 0..23 |       |
 *  Quality Level        |6049|   R   | Single|  No  | Integer | 0..100|       |
 */

#include "liblwm2m.h"
#include "lwm2mclient.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <arch/board/board.h>
#include <arch/chip/pin.h>

#define PRV_SENSOR_TYPE       ""

// Resource Id's:
#define RES_O_VALUE           5600
#define RES_O_MIN_VALUE       5601
#define RES_O_MAX_VALUE       5602
#define RES_O_MIN_RANGE       5603
#define RES_O_MAX_RANGE       5604
#define RES_M_APPTYPE         5750
#define RES_O_SENSOR_TYPE     5751
#define RES_O_RESET_VALUES    5605
#define RES_O_TIMESTAMP       5518
#define RES_O_TIMESTAMP_FRAC  6050

typedef struct _prv_instance_
{
    struct _prv_instance_ * next;   // matches lwm2m_list_t::next
    uint16_t shortID;               // used as a pin number
    float    value;
    float    value_min;
    float    value_max;
    float    range_min;
    float    range_max;
    char     *str;
    char     *sensor_type;
    int64_t  timestamp;
    float    timestamp_frac;
} prv_instance_t;

static lwm2m_object_t * g_objectP;

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
        *dataArrayP = lwm2m_data_new(9);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 9;
        (*dataArrayP)[0].id = RES_O_VALUE;
        (*dataArrayP)[1].id = RES_O_MIN_VALUE;
        (*dataArrayP)[2].id = RES_O_MAX_VALUE;
        (*dataArrayP)[3].id = RES_O_MIN_RANGE;
        (*dataArrayP)[4].id = RES_O_MAX_RANGE;
        (*dataArrayP)[5].id = RES_M_APPTYPE;
        (*dataArrayP)[6].id = RES_O_SENSOR_TYPE;
        (*dataArrayP)[7].id = RES_O_TIMESTAMP;
        (*dataArrayP)[8].id = RES_O_TIMESTAMP_FRAC;
    }

    for (i = 0 ; i < *numDataP ; i++)
    {
        if ((*dataArrayP)[i].type == LWM2M_TYPE_MULTIPLE_RESOURCE)
        {
            return COAP_404_NOT_FOUND;
        }

        switch ((*dataArrayP)[i].id)
        {
            case RES_O_VALUE:
                lwm2m_data_encode_float(targetP->value, *dataArrayP + i);
                break;
            case RES_O_MIN_VALUE:
                lwm2m_data_encode_float(targetP->value_min, *dataArrayP + i);
                break;
            case RES_O_MAX_VALUE:
                lwm2m_data_encode_float(targetP->value_max, *dataArrayP + i);
                break;
            case RES_O_MIN_RANGE:
                lwm2m_data_encode_float(targetP->range_min, *dataArrayP + i);
                break;
            case RES_O_MAX_RANGE:
                lwm2m_data_encode_float(targetP->range_max, *dataArrayP + i);
                break;
            case RES_M_APPTYPE:
                lwm2m_data_encode_string(targetP->str, *dataArrayP + i);
                break;
            case RES_O_SENSOR_TYPE:
                lwm2m_data_encode_string(targetP->sensor_type, *dataArrayP + i);
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

    /* Stop ADC system */
    adc_stop(id);
    lwm2m_free(targetP->str);
    lwm2m_free(targetP->sensor_type);
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

    /* Start ADC system */
    ret = adc_start(instanceId);
    if (ret != 0)
    {
        prv_delete(contextP, instanceId, objectP);
        return COAP_402_BAD_OPTION;
    }
    targetP->str = lwm2m_malloc(1);
    targetP->str[0] = '\0';
    targetP->sensor_type = lwm2m_malloc(strlen(PRV_SENSOR_TYPE) + 1);
    targetP->sensor_type[strlen(PRV_SENSOR_TYPE)] = '\0';

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

static uint8_t prv_exec(lwm2m_context_t *contextP,
                        uint16_t instanceId,
                        uint16_t resourceId,
                        uint8_t * buffer,
                        int length,
                        lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;

    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    /* unused parameter */
    (void)contextP;

    switch (resourceId)
    {
        case RES_O_RESET_VALUES:
            targetP->value_min = targetP->range_max;
            targetP->value_max = targetP->range_min;
            return COAP_204_CHANGED;
        default:
            return COAP_405_METHOD_NOT_ALLOWED;
    }
}

void analog_input_setValue(uint16_t id, float value,
                           float range_min, float range_max)
{
    int ret;
    struct timespec time;
    prv_instance_t * targetP;

    if (NULL == g_objectP) return;

    targetP = (prv_instance_t *)lwm2m_list_find(g_objectP->instanceList, id);
    if (NULL == targetP) return;

    targetP->value = value;

    if (targetP->value < targetP->value_min)
    {
        targetP->value_min = targetP->value;
    }
    if (targetP->value > targetP->value_max)
    {
        targetP->value_max = targetP->value;
    }
    targetP->range_min = range_min;
    targetP->range_max = range_max;

    ret = clock_gettime(CLOCK_REALTIME, &time);
    if (ret == OK)
    {
        targetP->timestamp = time.tv_sec + time.tv_nsec / 1000000000;
        targetP->timestamp -= get_utc_offset_sec();
        time.tv_nsec %= 1000000000;
        targetP->timestamp_frac = (float)time.tv_nsec / 1000000000.0;
    }

    return;
}

void display_analog_input_object(lwm2m_object_t * object)
{
    fprintf(stdout, "  /%u: Analog Input object, instances:\r\n", object->objID);
    prv_instance_t * instance = (prv_instance_t *)object->instanceList;
    while (instance != NULL)
    {
        fprintf(stdout, "    /%u/%u: shortId: %u value:%.3f min:%.3f max:%.3f\r\n",
                object->objID, instance->shortID,
                instance->shortID, instance->value,
                instance->value_min, instance->value_max);
        instance = (prv_instance_t *)instance->next;
    }
}

lwm2m_object_t * get_analog_input_object(void)
{
    lwm2m_object_t * object;

    object = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != object)
    {
        memset(object, 0, sizeof(lwm2m_object_t));

        object->objID = LWM2M_ANALOG_INPUT_OBJECT_ID;
#ifndef LWM2M_VERSION_1_0
        // Not required, but useful for testing.
        object->versionMajor = 1;
        object->versionMinor = 1;
#endif
        object->readFunc = prv_read;
        object->writeFunc = prv_write;
        object->executeFunc = prv_exec;
        object->createFunc = prv_create;
        object->deleteFunc = prv_delete;
    }

    g_objectP = object;
    return object;
}

void free_analog_input_object(lwm2m_object_t * object)
{
    prv_instance_t * targetP = (prv_instance_t *)object->instanceList;
    while (targetP != NULL)
    {
        prv_instance_t * next = targetP->next;

        /* Stop ADC system */
        adc_stop(targetP->shortID);
        lwm2m_free(targetP->str);
        lwm2m_free(targetP->sensor_type);

        targetP = next;
    }
    LWM2M_LIST_FREE(object->instanceList);
    lwm2m_free(object);
    g_objectP = NULL;
}

