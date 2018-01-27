/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include <os/os.h>
#include <bsp/bsp.h>
#include <adc/adc.h>

#include <string.h>

#include <nrf_saadc.h>
#include <nrfx_saadc.h>

#define ADC_NUMBER_SAMPLES (2)
#define ADC_NUMBER_CHANNELS (1)

/* ADC Task settings */
#define ADC_TASK_PRIO           (OS_STACK_PRI_HIGHEST)
#define ADC_STACK_SIZE          (OS_STACK_ALIGN(336))

struct os_eventq adc_evq;
struct os_task adc_task;
os_stack_t adc_stack[ADC_STACK_SIZE];

struct adc_dev *adc;
uint8_t *sample_buffer1;
uint8_t *sample_buffer2;

int
adc_read(void *buffer, int buffer_len)
{
    int i;
    int adc_result;
    int my_result_mv = 0;
    int rc;
    for (i = 0; i < ADC_NUMBER_SAMPLES; i++) {
        rc = adc_buf_read(adc, buffer, buffer_len, i, &adc_result);
        if (rc != 0) {
            goto err;
        }
        my_result_mv = adc_result_mv(adc, 0, adc_result);
    }
    adc_buf_release(adc, buffer, buffer_len);
    return my_result_mv;
err:
    return (rc);
}

void
adc_init()
{
    nrf_saadc_channel_config_t cc = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(3);
    adc = (struct adc_dev *) os_dev_open("adc0", 0, NULL);
    assert(adc != NULL);
    adc_chan_config(adc, 3, NULL);
    sample_buffer1 = malloc(adc_buf_size(adc, ADC_NUMBER_CHANNELS, ADC_NUMBER_SAMPLES));
    sample_buffer2 = malloc(adc_buf_size(adc, ADC_NUMBER_CHANNELS, ADC_NUMBER_SAMPLES));
    memset(sample_buffer1, 0, adc_buf_size(adc, ADC_NUMBER_CHANNELS, ADC_NUMBER_SAMPLES));
    memset(sample_buffer2, 0, adc_buf_size(adc, ADC_NUMBER_CHANNELS, ADC_NUMBER_SAMPLES));
    adc_buf_set(adc, sample_buffer1, sample_buffer2,
                adc_buf_size(adc, ADC_NUMBER_CHANNELS, ADC_NUMBER_SAMPLES));
}

int
adc_read_event(struct adc_dev *dev, void *arg, uint8_t etype,
        void *buffer, int buffer_len)
{
    int value;
    uint16_t chr_val_handle;
    int rc;

    value = adc_read(buffer, buffer_len);
    if (value >= 0) {
        console_printf("Got %d\n", value);
    } else {
        console_printf("Error while reading: %d\n", value);
        goto err;
    }
    gatt_adc_val = value;
    rc = ble_gatts_find_chr(&gatt_svr_svc_adc_uuid.u, BLE_UUID16_DECLARE(ADC_SNS_VAL), NULL, &chr_val_handle);
    assert(rc == 0);
    ble_gatts_chr_updated(chr_val_handle);
    return (0);
err:
    return (rc);
}

/**
 * Event loop for the sensor task.
 */
static void
adc_task_handler(void *unused)
{
    struct adc_dev *adc;
    int rc;
    /* ADC init */
    adc_init();
    rc = adc_event_handler_set(adc, adc_read_event, (void *) NULL);
    assert(rc == 0);

    while (1) {
        adc_sample(adc);
        /* Wait 2 second */
        os_time_delay(OS_TICKS_PER_SEC * 2);
    }
}

int
main(int argc, char **argv)
{
    sysinit();

    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);
    return(0);
}
