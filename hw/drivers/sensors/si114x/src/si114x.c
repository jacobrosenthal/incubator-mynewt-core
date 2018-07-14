/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * resarding copyright ownership.  The ASF licenses this file
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

#include <string.h>
#include <errno.h>
#include <assert.h>

#include "os/mynewt.h"
#include "hal/hal_i2c.h"
#include "hal/hal_gpio.h"
#include "si114x/si114x.h"
#include "si114x_priv.h"
#include "sensor/sensor.h"
#include "sensor/proximity.h"

#if MYNEWT_VAL(SI114X_LOG)
#include "modlog/modlog.h"
#endif

#if MYNEWT_VAL(SI114X_STATS)
#include "stats/stats.h"
#endif

/*
 * Max time to wait for interrupt.
 */
#define SI114X_MAX_INT_WAIT (4 * OS_TICKS_PER_SEC)

//todo I need events (or user defined??)
const struct si114x_notif_cfg dflt_notif_cfg[] = {
    {
      .event     = SENSOR_EVENT_TYPE_SINGLE_TAP,
      .int_cfg   = SI114X_IRQ_STATUS_ALS_INT_MASK
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_X_CHANGE,
      .int_cfg   = SI114X_IRQ_STATUS_PS1_INT_MASK
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_Y_CHANGE,
      .int_cfg   = SI114X_IRQ_STATUS_PS2_INT_MASK
    },
    {
      .event     = SENSOR_EVENT_TYPE_ORIENT_Z_CHANGE,
      .int_cfg   = SI114X_IRQ_STATUS_PS3_INT_MASK
    },
   };

#if MYNEWT_VAL(SI114X_STATS)
/* Define the stats section and records */
STATS_SECT_START(si114x_stat_section)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(read_errors)
#if MYNEWT_VAL(SI114X_STATS) && MYNEWT_VAL(SI114X_NOTIF_STATS)
    STATS_SECT_ENTRY(als_notify)
    STATS_SECT_ENTRY(ps1_notify)
    STATS_SECT_ENTRY(ps2_notify)
    STATS_SECT_ENTRY(ps3_notify)
#endif
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(si114x_stat_section)
    STATS_NAME(si114x_stat_section, write_errors)
    STATS_NAME(si114x_stat_section, read_errors)
#if MYNEWT_VAL(SI114X_STATS) && MYNEWT_VAL(SI114X_NOTIF_STATS)
    STATS_NAME(si114x_stat_section, als_notify)
    STATS_NAME(si114x_stat_section, ps1_notify)
    STATS_NAME(si114x_stat_section, ps2_notify)
    STATS_NAME(si114x_stat_section, ps3_notify)
#endif
STATS_NAME_END(si114x_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(si114x_stat_section) g_si114xstats;
#endif

#if MYNEWT_VAL(SI114X_LOG)
#define SI114X_LOG(lvl_, ...) \
    MODLOG_ ## lvl_(MYNEWT_VAL(SI114X_LOG_MODULE), __VA_ARGS__)
#else
#define SI114X_LOG(lvl_, ...)
#endif

/* Exports for the sensor API */
static int si114x_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int si114x_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);
static int si114x_sensor_set_notification(struct sensor *,
                                            sensor_event_type_t);
static int si114x_sensor_unset_notification(struct sensor *,
                                              sensor_event_type_t);
static int si114x_sensor_handle_interrupt(struct sensor *);
static int si114x_sensor_set_config(struct sensor *, void *);

static const struct sensor_driver g_si114x_sensor_driver = {
    .sd_read               = si114x_sensor_read,
    .sd_set_config         = si114x_sensor_set_config,
    .sd_get_config         = si114x_sensor_get_config,
    .sd_set_notification   = si114x_sensor_set_notification,
    .sd_unset_notification = si114x_sensor_unset_notification,
    .sd_handle_interrupt   = si114x_sensor_handle_interrupt

};

/**
 * Write multiple length data to SI114X sensor over I2C  (MAX: 19 bytes)
 *
 * @param The sensor interface
 * @param register address
 * @param variable length payload
 * @param length of the payload to write
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_writelen(struct sensor_itf *itf, uint8_t addr, uint8_t *buffer,
                      uint8_t len)
{
    int rc;
    uint8_t payload[20] = { addr, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0};

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = len + 1,
        .buffer = payload
    };

    if (len > (sizeof(payload) - 1)) {
        rc = OS_EINVAL;
        goto err;
    }

    memcpy(&payload[1], buffer, len);

    rc = sensor_itf_lock(itf, MYNEWT_VAL(SI114X_ITF_LOCK_TMO));
    if (rc) {
        goto err;
    }

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        SI114X_LOG(ERROR, "I2C access failed at address 0x%02X\n", data_struct.address);
        STATS_INC(g_si114xstats, write_errors);
        goto err;
    }

err:
    sensor_itf_unlock(itf);
    return rc;
}

/**
 * Write byte to sensor over different interfaces
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value)
{
    return si114x_writelen(itf, reg, &value, 1);;
}


/**
 * Reads a single byte from the specified register
 *
 * @param The Sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value)
{
    int rc;
    uint8_t payload;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &payload
    };

    rc = sensor_itf_lock(itf, MYNEWT_VAL(SI114X_ITF_LOCK_TMO));
    if (rc) {
        goto err;
    }

    /* Register write */
    payload = reg;
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        SI114X_LOG(ERROR, "I2C register write failed at address 0x%02X:0x%02X\n",
                   data_struct.address, reg);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
        goto err;
    }

    /* Read one byte back */
    payload = 0;
    rc = hal_i2c_master_read(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);
    *value = payload;
    if (rc) {
        SI114X_LOG(ERROR, "Failed to read from 0x%02X:0x%02X\n", data_struct.address, reg);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
    }

err:
    sensor_itf_unlock(itf);
    return rc;
}

/**
 * Write parameter located in Parameter RAM, see section 4.6
 *
 * @param itf The sensor interface
 * @param cmd Combined offset and encoding of parameter to write
 * @param value Value to set parameter to
 * @return 0 on success, non-zero on failure
 */
int
si114x_write_param(struct sensor_itf *itf, uint8_t cmd, uint8_t value)
{
    int rc;
    uint8_t previous, response;
    uint8_t retries = 5;
    uint8_t payload[2] = { value, cmd };

    // 1. Write 0x00 to Command register to clear the Response register.
    rc = si114x_write8(itf, SI114X_COMMAND_ADDR, 0x00);
    if (rc) {
        goto err;
    }

    // 2. Read Response register and verify contents are 0x00.
    rc = si114x_read8(itf, SI114X_RESPONSE_ADDR, &previous);
    if (rc) {
        goto err;
    }

    if (previous) {
        rc = SYS_EUNKNOWN;
        goto err;
    }

    // 3. Write Command value from Table 5 into Command register.
    rc = si114x_writelen(itf, SI114X_PARAM_WR_ADDR, payload, sizeof(payload));
    if (rc) {
        goto err;
    }

    // Step4 is not applicable to the Reset Command because the device will reset itself and does not increment the Response register after reset
    // presumably NOP too
    if(cmd == SI114X_PARAM_RESET || cmd == SI114X_PARAM_NOP){
        return 0;
    }

    // 4. Read the Response register and verify contents are now non-zero
    // If the Response register remains 0x00 for over 25 ms after the Command write, the entire Command process should be repeated from Step 1.
    do {
        rc = si114x_read8(itf, SI114X_RESPONSE_ADDR, &response);
        if (rc) {
            goto err;
        }

        if(!retries--){
            rc = SYS_ETIMEOUT;
            goto err;
        }

        /* wait 5ms */
        os_time_delay(5 * OS_TICKS_PER_SEC / 1000);
    } while(!response);
    
    return 0;
err:
    return rc;
}

/**
 * Send command to device, see section 4.2
 *
 * @param itf The sensor interface
 * @param cmd Command to set parameter to
 * @return 0 on success, non-zero on failure
 */
int
si114x_write_command(struct sensor_itf *itf, uint8_t cmd)
{
    return si114x_write_param(itf, cmd, 0x00);
}

int
si114x_set_param(struct sensor_itf *itf, uint8_t offset, uint8_t value)
{
    return si114x_write_param(itf, SI114X_PARAM_SET | offset, value);
}

int
si114x_and_param(struct sensor_itf *itf, uint8_t offset, uint8_t value)
{
    return si114x_write_param(itf, SI114X_PARAM_AND | offset, value);
}

int
si114x_or_param(struct sensor_itf *itf, uint8_t offset, uint8_t value)
{
    return si114x_write_param(itf, SI114X_PARAM_OR | offset, value);
}

/**
 * Get parameter located in Parameter RAM, see section 4.6
 *
 * @param itf The sensor interface
 * @param offset The offset of parameter to read
 * @param value Pointer to the variable to fill parameter in
 * @return 0 on success, non-zero on failure
 */
int
si114x_query_param(struct sensor_itf *itf, uint8_t offset, uint8_t *value)
{
    int rc;

    rc = si114x_write_command(itf, SI114X_PARAM_QUERY | offset);
    if (rc) {
        goto err;
    }

    rc = si114x_read8(itf, SI114X_PARAM_RD_ADDR, value);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

/**
 * Clear interrupt pin configuration for interrupt 1
 *
 * @param itf The sensor interface
 * @param cfg int1 config
 * @return 0 on success, non-zero on failure
 */
int
si114x_clear_int_pin_cfg(struct sensor_itf *itf, uint8_t cfg)
{
    int rc;
    uint8_t reg;

    reg = 0;

    rc = si114x_read8(itf, SI114X_IRQ_ENABLE_ADDR, &reg);
    if (rc) {
        goto err;
    }

    reg &= ~cfg;

    rc = si114x_write8(itf, SI114X_IRQ_ENABLE_ADDR, reg);

err:
    return rc;
}

/**
 * Set interrupt pin configuration for interrupt 1
 *
 * @param the sensor interface
 * @param config
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_int_pin_cfg(struct sensor_itf *itf, uint8_t cfg)
{
    int rc;
    uint8_t reg;

    reg = 0;

    rc = si114x_read8(itf, SI114X_IRQ_ENABLE_ADDR, &reg);
    if (rc) {
        goto err;
    }

    reg |= cfg;

    rc = si114x_write8(itf, SI114X_IRQ_ENABLE_ADDR, reg);

err:
    return rc;
}

/**
 * Clear interrupt pin configuration for interrupt 1
 *
 * @param itf The sensor interface
 * @param cfg int1 config
 * @return 0 on success, non-zero on failure
 */
int
si114x_clear_irq1_mode(struct sensor_itf *itf, uint8_t cfg)
{
    int rc;
    uint8_t reg;

    reg = 0;

    rc = si114x_read8(itf, SI114X_IRQ_MODE1_ADDR, &reg);
    if (rc) {
        goto err;
    }

    reg &= ~cfg;

    rc = si114x_write8(itf, SI114X_IRQ_MODE1_ADDR, reg);

err:
    return rc;
}

/**
 * Set interrupt pin configuration for interrupt 1
 *
 * @param the sensor interface
 * @param config
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_irq1_mode(struct sensor_itf *itf, uint8_t cfg)
{
    int rc;
    uint8_t reg;

    reg = 0;

    rc = si114x_read8(itf, SI114X_IRQ_MODE1_ADDR, &reg);
    if (rc) {
        goto err;
    }

    reg |= cfg;

    rc = si114x_write8(itf, SI114X_IRQ_MODE1_ADDR, reg);

err:
    return rc;
}

/**
 * Clear interrupt pin configuration for interrupt 2
 *
 * @param itf The sensor interface
 * @param cfg int1 config
 * @return 0 on success, non-zero on failure
 */
int
si114x_clear_irq2_mode(struct sensor_itf *itf, uint8_t cfg)
{
    int rc;
    uint8_t reg;

    reg = 0;

    rc = si114x_read8(itf, SI114X_IRQ_MODE2_ADDR, &reg);
    if (rc) {
        goto err;
    }

    reg &= ~cfg;

    rc = si114x_write8(itf, SI114X_IRQ_MODE2_ADDR, reg);

err:
    return rc;
}

/**
 * Set interrupt pin configuration for interrupt 2
 *
 * @param the sensor interface
 * @param config
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_irq2_mode(struct sensor_itf *itf, uint8_t cfg)
{
    int rc;
    uint8_t reg;

    reg = 0;

    rc = si114x_read8(itf, SI114X_IRQ_MODE2_ADDR, &reg);
    if (rc) {
        goto err;
    }

    reg |= cfg;

    rc = si114x_write8(itf, SI114X_IRQ_MODE2_ADDR, reg);

err:
    return rc;
}

/**
 * Clear interrupt 1
 *
 * @param the sensor interface
 */
int
si114x_clear_int(struct sensor_itf *itf, uint8_t src)
{
    return si114x_write8(itf, SI114X_IRQ_STATUS_ADDR, src);
}

/**
 * Get Interrupt Status
 *
 * @param the sensor interface
 * @param pointer to return interrupt status in
 * @return 0 on success, non-zero on failure
 */
int si114x_get_int_status(struct sensor_itf *itf, uint8_t *status)
{
    return si114x_read8(itf, SI114X_IRQ_STATUS_ADDR, status);
}

/**
 * Set whether interrupts are enabled in device
 *
 * @param the sensor interface
 * @param value to set (0 = disabled, 1 = enabled)
 * @return 0 on success, non-zero on failure
 */
int si114x_set_int_enable(struct sensor_itf *itf, uint8_t enabled)
{
    uint8_t reg;
    int rc;

    rc = si114x_read8(itf, SI114X_INT_CFG_ADDR, &reg);
    if (rc) {
        return rc;
    }

    if (enabled) {
        reg |= SI114X_INT_CFG_INT_OE_MASK;
    } else {
        reg &= ~SI114X_INT_CFG_INT_OE_MASK;
    }

    return si114x_write8(itf, SI114X_INT_CFG_ADDR, reg);
}

/**
 * Set read clear interrupts
 *
 * @param itf The sensor interface
 * @param enabled The internal sequencer clears the INT pin automatically.
 * @return 0 on success, non-zero on failure
 */
int si114x_set_rd_clr(struct sensor_itf *itf, uint8_t enabled)
{
    uint8_t reg;
    int rc;

    rc = si114x_read8(itf, SI114X_INT_CFG_ADDR, &reg);
    if (rc) {
        return rc;
    }

    if (enabled) {
        reg |= SI114X_INT_CFG_INT_MODE_MASK;
    } else {
        reg &= ~SI114X_INT_CFG_INT_MODE_MASK;
    }

    return si114x_write8(itf, SI114X_INT_CFG_ADDR, reg);
}

static void
init_interrupt(struct si114x_int *interrupt, struct sensor_int *ints)
{
    os_error_t error;

    error = os_sem_init(&interrupt->wait, 0);
    assert(error == OS_OK);

    interrupt->active = false;
    interrupt->asleep = false;
    interrupt->ints = ints;
}

static void
undo_interrupt(struct si114x_int * interrupt)
{
    OS_ENTER_CRITICAL(interrupt->lock);
    interrupt->active = false;
    interrupt->asleep = false;
    OS_EXIT_CRITICAL(interrupt->lock);
}

static int
wait_interrupt(struct si114x_int *interrupt)
{
    bool wait;
    os_error_t error;

    OS_ENTER_CRITICAL(interrupt->lock);

    /* Check if we did not missed interrupt */
    if (hal_gpio_read(interrupt->ints[0].host_pin) ==
                                            interrupt->ints[0].active) {
        OS_EXIT_CRITICAL(interrupt->lock);
        return OS_OK;
    }

    if (interrupt->active) {
        interrupt->active = false;
        wait = false;
    } else {
        interrupt->asleep = true;
        wait = true;
    }
    OS_EXIT_CRITICAL(interrupt->lock);

    if (wait) {
        error = os_sem_pend(&interrupt->wait, SI114X_MAX_INT_WAIT);
        if (error == OS_TIMEOUT) {
            return error;
        }
        assert(error == OS_OK);
    }
    return OS_OK;
}

static void
wake_interrupt(struct si114x_int *interrupt)
{
    bool wake;

    OS_ENTER_CRITICAL(interrupt->lock);
    if (interrupt->asleep) {
        interrupt->asleep = false;
        wake = true;
    } else {
        interrupt->active = true;
        wake = false;
    }
    OS_EXIT_CRITICAL(interrupt->lock);

    if (wake) {
        os_error_t error;

        error = os_sem_release(&interrupt->wait);
        assert(error == OS_OK);
    }
}

static void
si114x_int_irq_handler(void *arg)
{
    struct sensor *sensor = arg;
    struct si114x *si114x;

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);

    if(si114x->pdd.interrupt) {
        wake_interrupt(si114x->pdd.interrupt);
    }

    sensor_mgr_put_interrupt_evt(sensor);
}

static int
init_intpin(struct si114x *si114x, hal_gpio_irq_handler_t handler,
            void *arg)
{
    hal_gpio_irq_trig_t trig;
    int pin = -1;
    int rc;
    int i;

    for (i = 0; i < MYNEWT_VAL(SENSOR_MAX_INTERRUPTS_PINS); i++){
        pin = si114x->sensor.s_itf.si_ints[i].host_pin;
        if (pin >= 0) {
            break;
        }
    }

    if (pin < 0) {
        SI114X_LOG(ERROR, "Interrupt pin not configured\n");
        return SYS_EINVAL;
    }

    if (si114x->sensor.s_itf.si_ints[i].active) {
        trig = HAL_GPIO_TRIG_RISING;
    } else {
        trig = HAL_GPIO_TRIG_FALLING;
    }

    rc = hal_gpio_irq_init(pin,
                           handler,
                           arg,
                           trig,
                           HAL_GPIO_PULL_NONE);
    if (rc != 0) {
        SI114X_LOG(ERROR, "Failed to initialise interrupt pin %d\n", pin);
        return rc;
    }

    return 0;
}

/**
 * Disable interrupts
 *
 * @param itf The sensor interface
 * @param int_to_disable One of the global interrupt masks above
 * @param int_mode Mode to disable from si114x_proximity_int_mode or si114x_ambient_int_mode depending
 * @return 0 on success, non-zero on failure
 */
int
disable_interrupt(struct sensor *sensor, uint8_t int_to_disable, uint8_t int_mode)
{
    struct si114x *si114x;
    struct si114x_pdd *pdd;
    struct sensor_itf *itf;
    int rc;

    if (int_to_disable == 0) {
        return SYS_EINVAL;
    }

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);
    pdd = &si114x->pdd;

    rc = si114x_clear_int_pin_cfg(itf, int_to_disable);
    if (rc) {
        return rc;
    }
    pdd->int_enable &= ~int_to_disable;

    /* disable int pin */
    if (!pdd->int_enable) {
        hal_gpio_irq_disable(itf->si_ints[0].host_pin);
        /* disable interrupt in device */
        rc = si114x_set_int_enable(itf, 0);
        if (rc) {
            pdd->int_enable |= int_to_disable;
            return rc;
        }
    }

    /* update interrupt setup in device */
    if (int_to_disable <= 0xff) {
        rc = si114x_clear_irq1_mode(itf, int_to_disable);
        pdd->int1_mode &= ~int_mode;
    } else {
        rc = si114x_clear_irq2_mode(itf, int_to_disable);
        pdd->int2_mode &= ~int_mode;
    }

    return rc;
}

/**
 * Enable interrupts
 *
 * @param itf The sensor interface
 * @param int_to_enable One of the global interrupt masks above
 * @param int_mode Mode to enable from si114x_proximity_int_mode or si114x_ambient_int_mode depending
 * @return 0 on success, non-zero on failure
 */
int
enable_interrupt(struct sensor *sensor, uint8_t int_to_enable, uint8_t int_mode)
{
    struct si114x *si114x;
    struct si114x_pdd *pdd;
    struct sensor_itf *itf;
    int rc;

    if (!int_to_enable) {
        rc = SYS_EINVAL;
        goto err;
    }

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);
    pdd = &si114x->pdd;

    rc = si114x_clear_int(itf, int_to_enable);
    if (rc) {
        goto err;
    }

    /* if no interrupts are currently in use enable int pin */
    if (!pdd->int_enable) {
        hal_gpio_irq_enable(itf->si_ints[0].host_pin);

        rc = si114x_set_int_enable(itf, 1);
        if (rc) {
            goto err;
        }
    }

    rc = si114x_set_int_pin_cfg(itf, int_to_enable);
    if (rc) {
        goto err;
    }
    pdd->int_enable |= int_to_enable;

    /* enable interrupt in device */
    if (int_to_enable <= 0xff) {
        rc = si114x_set_irq1_mode(itf, int_mode);
        pdd->int1_mode |= int_mode;
    } else {
        rc = si114x_set_irq2_mode(itf, int_mode);
        pdd->int2_mode |= int_mode;
    }

    if (rc) {
        disable_interrupt(sensor, int_to_enable, int_mode);
        goto err;
    }

    return 0;
err:
    return rc;
}

/**
 * Gets a new data sample from the sensor.
 *
 * @param The sensor interface
 * @param x axis data
 * @param y axis data
 * @param z axis data
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_data(struct sensor_itf *itf, int16_t *x)
{
    int rc;
    uint8_t payload[2] = {0};

    *x = 0;

    //todo, what about others, how to tell?
    rc = si114x_readlen(itf, SI114X_PS1_DATA0_ADDR, payload, 2);
    if (rc) {
        goto err;
    }

    *x = payload[0] | (payload[1] << 8);

    return 0;
err:
    return rc;
}

static int si114x_do_read(struct sensor *sensor, sensor_data_func_t data_func,
                            void * data_arg)
{
    struct sensor_proximity_data proximity_data;
    struct sensor_itf *itf;
    int16_t ps1;
    int rc;

    itf = SENSOR_GET_ITF(sensor);

    ps1 = 0;

    rc = si114x_get_data(itf, &ps1);
    if (rc) {
        goto err;
    }

    //todo fill others
    proximity_data.ps1 = ps1;
    // proximity_data.ps2 = ps2;
    // proximity_data.ps3 = ps3;
    // proximity_data.als = als;

    proximity_data.ps1_is_valid = 1;
    proximity_data.ps2_is_valid = 0;
    proximity_data.ps3_is_valid = 0;
    proximity_data.als_is_valid = 0;

    /* Call data function */
    rc = data_func(sensor, data_arg, &proximity_data, SENSOR_TYPE_PROXIMITY);
    if (rc != 0) {
        goto err;
    }

    return 0;
err:
    return rc;  
}

/**
 * Do accelerometer polling reads
 *
 * @param sensor The sensor ptr
 * @param sensor_type The sensor type
 * @param read_func The function pointer to invoke for each accelerometer reading.
 * @param read_arg The opaque pointer that will be passed in to the function.
 * @param time_ms If non-zero, how long the stream should run in milliseconds.
 *
 * @return 0 on success, non-zero on failure.
 */
int
si114x_poll_read(struct sensor *sensor,
                     sensor_type_t sensor_type,
                     sensor_data_func_t read_func,
                     void *read_arg,
                     uint32_t time_ms)
{
    struct si114x_pdd *pdd;
    struct si114x *si114x;
    struct si114x_cfg *cfg;
    os_time_t time_ticks;
    os_time_t stop_ticks = 0;
    int rc, rc2;
    struct sensor_itf *itf;

    /* If the read isn't looking for our data, don't do anything. */
    if (!(sensor_type & SENSOR_TYPE_PROXIMITY)) {
        return SYS_EINVAL;
    }

    itf = SENSOR_GET_ITF(sensor);

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    pdd = &si114x->pdd;
    cfg = &si114x->cfg;

    if (cfg->read_mode.mode != SI114X_READ_M_STREAM) {
        return SYS_EINVAL;
    }

    undo_interrupt(&si114x->intr);

    if (pdd->interrupt) {
        return SYS_EBUSY;
    }

    /* enable interrupt */
    pdd->interrupt = &si114x->intr;

    rc = enable_interrupt(sensor, cfg->read_mode.int_cfg,
                          0);
    if (rc) {
        return rc;
    }

    if (time_ms != 0) {
        rc = os_time_ms_to_ticks(time_ms, &time_ticks);
        if (rc) {
            goto err;
        }
        stop_ticks = os_time_get() + time_ticks;
    }

    for (;;) {
        //todo what about others
        si114x_proximity_force(itf);

        /* force at least one read for cases when fifo is disabled */
        rc = wait_interrupt(&si114x->intr);
        if (rc) {
            goto err;
        }

        rc = si114x_do_read(sensor, read_func, read_arg);
        if (rc) {
            goto err;
        }

        if (time_ms != 0 && OS_TIME_TICK_GT(os_time_get(), stop_ticks)) {
            break;
        }

    }

err:
    /* disable interrupt */
    pdd->interrupt = NULL;
    rc2 = disable_interrupt(sensor, cfg->read_mode.int_cfg,
                           0);

    if (rc) {
        return rc;
    } else {
        return rc2;
    }
}

int
si114x_stream_read(struct sensor *sensor,
                     sensor_type_t sensor_type,
                     sensor_data_func_t read_func,
                     void *read_arg,
                     uint32_t time_ms)
{
    struct si114x_pdd *pdd;
    struct si114x *si114x;
    struct si114x_cfg *cfg;
    os_time_t time_ticks;
    os_time_t stop_ticks = 0;
    int rc, rc2;
    struct sensor_itf *itf;

    /* If the read isn't looking for our data, don't do anything. */
    if (!(sensor_type & SENSOR_TYPE_PROXIMITY)) {
        return SYS_EINVAL;
    }

    itf = SENSOR_GET_ITF(sensor);

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    pdd = &si114x->pdd;
    cfg = &si114x->cfg;

    if (cfg->read_mode.mode != SI114X_READ_M_STREAM) {
        return SYS_EINVAL;
    }

        //todo what about others
        si114x_proximity_force(itf);

    undo_interrupt(&si114x->intr);

    if (pdd->interrupt) {
        return SYS_EBUSY;
    }

    /* enable interrupt */
    pdd->interrupt = &si114x->intr;

    rc = enable_interrupt(sensor, cfg->read_mode.int_cfg,
                          0);
    if (rc) {
        return rc;
    }

    if (time_ms != 0) {
        rc = os_time_ms_to_ticks(time_ms, &time_ticks);
        if (rc) {
            goto err;
        }
        stop_ticks = os_time_get() + time_ticks;
    }

    for (;;) {

        /* force at least one read for cases when fifo is disabled */
        rc = wait_interrupt(&si114x->intr);
        if (rc) {
            goto err;
        }

        rc = si114x_do_read(sensor, read_func, read_arg);
        if (rc) {
            goto err;
        }

        if (time_ms != 0 && OS_TIME_TICK_GT(os_time_get(), stop_ticks)) {
            break;
        }

    }

err:
    /* disable interrupt */
    pdd->interrupt = NULL;
    rc2 = disable_interrupt(sensor, cfg->read_mode.int_cfg,
                           0);

    if (rc) {
        return rc;
    } else {
        return rc2;
    }
}

static int
si114x_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    int rc;
    const struct si114x_cfg *cfg;
    struct si114x *si114x;

    /* If the read isn't looking for our data, don't do anything. */
    if (!(type & SENSOR_TYPE_PROXIMITY)) {
        rc = SYS_EINVAL;
        goto err;
    }

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    cfg = &si114x->cfg;

    if (cfg->read_mode.mode == SI114X_READ_M_POLL) {
        rc = si114x_poll_read(sensor, type, data_func, data_arg, timeout);
    } else {
        rc = si114x_stream_read(sensor, type, data_func, data_arg, timeout);
    }
err:
    if (rc) {
        return SYS_EINVAL; /* XXX */
    } else {
        return SYS_EOK;
    }
}

static struct si114x_notif_cfg *
si114x_find_notif_cfg_by_event(sensor_event_type_t event,
                                 struct si114x_cfg *cfg)
{
    int i;
    struct si114x_notif_cfg *notif_cfg = NULL;

    if (!cfg) {
        goto err;
    }

    for (i = 0; i < cfg->max_num_notif; i++) {
        if (event == cfg->notif_cfg[i].event) {
            notif_cfg = &cfg->notif_cfg[i];
            break;
        }
    }

    if (i == cfg->max_num_notif) {
       /* here if type is set to a non valid event or more than one event
        * we do not currently support registering for more than one event
        * per notification
        */
        goto err;
    }

    return notif_cfg;
err:
    return NULL;
}

static int
si114x_sensor_set_notification(struct sensor *sensor, sensor_event_type_t event)
{
    struct si114x *si114x;
    struct si114x_pdd *pdd;
    struct si114x_notif_cfg *notif_cfg;
    int rc;

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    pdd = &si114x->pdd;

    notif_cfg = si114x_find_notif_cfg_by_event(event, &si114x->cfg);
    if (!notif_cfg) {
        rc = SYS_EINVAL;
        goto err;
    }

    //todo need to be able to set the interrupt mode somehow
    rc = enable_interrupt(sensor, notif_cfg->int_cfg, 0);
    if (rc) {
        goto err;
    }

    pdd->notify_ctx.snec_evtype |= event;

    return 0;
err:
    return rc;
}

static int
si114x_sensor_unset_notification(struct sensor *sensor, sensor_event_type_t event)
{
    struct si114x_notif_cfg *notif_cfg;
    struct si114x *si114x;
    int rc;

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);

    si114x->pdd.notify_ctx.snec_evtype &= ~event;

    notif_cfg = si114x_find_notif_cfg_by_event(event, &si114x->cfg);
    if (!notif_cfg) {
        rc = SYS_EINVAL;
        goto err;
    }

    //todo need to be able to set the interrupt mode somehow
    rc = disable_interrupt(sensor, notif_cfg->int_cfg, 0);

err:
    return rc;
}

static int
si114x_sensor_set_config(struct sensor *sensor, void *cfg)
{
    struct si114x *si114x;

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);

    return si114x_config(si114x, (struct si114x_cfg*)cfg);
}

static void
si114x_inc_notif_stats(sensor_event_type_t event)
{

#if MYNEWT_VAL(SI114X_NOTIF_STATS)
    switch (event) {
        case SENSOR_EVENT_TYPE_SINGLE_TAP:
            STATS_INC(g_si114xstats, als_notify);
            break;
        case SENSOR_EVENT_TYPE_ORIENT_X_CHANGE:
            STATS_INC(g_si114xstats, ps1_notify);
            break;
        case SENSOR_EVENT_TYPE_ORIENT_Y_CHANGE:
            STATS_INC(g_si114xstats, ps2_notify);
            break;
        case SENSOR_EVENT_TYPE_ORIENT_Z_CHANGE:
            STATS_INC(g_si114xstats, ps3_notify);
            break;
        default:
            break;
    }
#endif

    return;
}

static int
si114x_sensor_handle_interrupt(struct sensor *sensor)
{
    struct si114x *si114x;
    struct sensor_itf *itf;
    uint8_t int_status;
    int rc;

    si114x = (struct si114x *)SENSOR_GET_DEVICE(sensor);
    itf = SENSOR_GET_ITF(sensor);

    rc = si114x_get_int_status(itf, &int_status);
    if (rc) {
        SI114X_LOG(ERROR, "Could not read int status err=0x%02x\n", rc);
        goto err;
    }

    rc = si114x_clear_int(itf, int_status);
    if (rc) {
        SI114X_LOG(ERROR, "Could not read int src err=0x%02x\n", rc);
        goto err;
    }

    if(int_status & SI114X_IRQ_STATUS_ALS_INT_MASK){
        sensor_mgr_put_notify_evt(&si114x->pdd.notify_ctx,
                                  SENSOR_EVENT_TYPE_SINGLE_TAP);
        si114x_inc_notif_stats(SENSOR_EVENT_TYPE_SINGLE_TAP);
    }

    if(int_status & SI114X_IRQ_STATUS_PS1_INT_MASK){
        sensor_mgr_put_notify_evt(&si114x->pdd.notify_ctx,
                                  SENSOR_EVENT_TYPE_ORIENT_Y_CHANGE);
        si114x_inc_notif_stats(SENSOR_EVENT_TYPE_ORIENT_Y_CHANGE);
    }

    if(int_status & SI114X_IRQ_STATUS_PS2_INT_MASK){
        sensor_mgr_put_notify_evt(&si114x->pdd.notify_ctx,
                                  SENSOR_EVENT_TYPE_SINGLE_TAP);
        si114x_inc_notif_stats(SENSOR_EVENT_TYPE_SINGLE_TAP);
   }

   if(int_status & SI114X_IRQ_STATUS_PS3_INT_MASK){
        sensor_mgr_put_notify_evt(&si114x->pdd.notify_ctx,
                                  SENSOR_EVENT_TYPE_ORIENT_Z_CHANGE);
        si114x_inc_notif_stats(SENSOR_EVENT_TYPE_ORIENT_Z_CHANGE);
   }

    return 0;
err:
    return rc;
}

static int
si114x_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    int rc;

    if ((type != SENSOR_TYPE_PROXIMITY)) {
        rc = SYS_EINVAL;
        goto err;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_INT32;

    return 0;
err:
    return rc;
}

/**
 * Initialize the si114x. This is normally used as an os_dev_create initialization function
 *
 * @param dev  Pointer to the si114x_dev device descriptor
 * @param arg  Pointer to the sensor interface
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_init(struct os_dev *dev, void *arg)
{
    struct si114x *si114x;
    struct sensor *sensor;
    int rc;
    uint8_t id;

    if (!arg || !dev) {
        rc = SYS_ENODEV;
        goto err;
    }

    si114x = (struct si114x *) dev;

    sensor = &si114x->sensor;

#if MYNEWT_VAL(SI114X_STATS)
    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_si114xstats),
        STATS_SIZE_INIT_PARMS(g_si114xstats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(si114x_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_si114xstats));
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    rc = sensor_init(sensor, dev);
    if (rc != 0) {
        goto err;
    }

    /* wait 30ms startup time*/
    os_time_delay(30 * OS_TICKS_PER_SEC / 1000);

    /* Check if we can read the chip address */
    rc = si114x_get_chip_id(arg, &id);
    if (rc) {
        SI114X_LOG(ERROR, "unable to get chip id [1]: %d\n", rc);
        goto err;
    }

    if (id != SI114X_PART_ID_SI1141 && id != SI114X_PART_ID_SI1142 && id != SI114X_PART_ID_SI1143) {
        rc = SYS_EINVAL;
        SI114X_LOG(ERROR, "id not as expected: got: %d, expected %d or %d or %d\n", id,
                    SI114X_PART_ID_SI1141, SI114X_PART_ID_SI1142, SI114X_PART_ID_SI1143);
        goto err;
    }

    rc = si114x_get_chip_sequence(arg, &id);
    if (rc) {
        SI114X_LOG(ERROR, "unable to get sequence id: %d\n", rc);
        goto err;
    }

    if (id != 0x09) {
        rc = SYS_EINVAL;
        SI114X_LOG(ERROR, "seq id not as expected: got: %d, expected 0x09\n", id);
        goto err;
    }

    /* Set the interface */
    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        goto err;
    }

    rc = sensor_mgr_register(sensor);
    if (rc) {
        goto err;
    }

    init_interrupt(&si114x->intr, si114x->sensor.s_itf.si_ints);
    
    si114x->pdd.notify_ctx.snec_sensor = sensor;
    si114x->pdd.interrupt = NULL;

    rc = init_intpin(si114x, si114x_int_irq_handler, sensor);
    if (rc) {
        return rc;
    }

    return (0);
err:
    SI114X_LOG(ERROR, "Error initializing SI114X: %d\n", rc);
    return (rc);
}

/**
 * Get chip id from the sensor
 *
 * @param itf The sensor interface
 * @param id Pointer to the variable to fill up chip id in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_id(struct sensor_itf *itf, uint8_t *id)
{
    int rc;

    /* Check if we can read the chip address */
    rc = si114x_read8(itf, SI114X_PART_ID_ADDR, id);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

/**
 * Get chip sequence from the sensor
 *
 * @param itf The sensor interface
 * @param seq Pointer to the variable to fill up chip sequence in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_sequence(struct sensor_itf *itf, uint8_t *seq)
{
    int rc;

    /* Check if we can read the chip address */
    rc = si114x_read8(itf, SI114X_SEQ_ID_ADDR, seq);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

/**
 * Get chip revision from the sensor
 *
 * @param itf The sensor interface
 * @param rev Pointer to the variable to fill up chip revision in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_revision(struct sensor_itf *itf, uint8_t *rev)
{
    int rc;

    /* Check if we can read the chip address */
    rc = si114x_read8(itf, SI114X_REV_ID_ADDR, rev);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

int
si114x_set_proximity_current(struct sensor_itf *itf, uint8_t number, uint8_t current)
{
    int rc;
    uint8_t reg, mask, position, prev;

    switch (number) {
        case 1:
            reg = SI114X_PS_LED21_ADDR;
            mask = SI114X_PS_LED21_LED1_MASK;
            position = SI114X_PS_LED21_LED1_POS;
            break;

        case 2:
            reg = SI114X_PS_LED21_ADDR;
            mask = SI114X_PS_LED21_LED2_MASK;
            position = SI114X_PS_LED21_LED2_POS;
            break;

        case 3:
            reg = SI114X_PS_LED3_ADDR;
            mask = SI114X_PS_LED3_LED3_MASK;
            position = SI114X_PS_LED3_LED3_POS;
            break;

        default:
            return SYS_EINVAL;
    }

    rc = si114x_read8(itf, reg, &prev);
    if (rc) {
        goto err;
    }

    prev &= ~mask;
    prev |= current << position;

    rc = si114x_write8(itf, reg, prev);

    return 0;
err:
    return rc;
}

int
si114x_set_proximity_threshold(struct sensor_itf *itf, uint8_t number, uint16_t threshold)
{
    int rc;
    uint8_t reg0, reg1;

    switch (number) {
        case 1:
            reg0 = SI114X_PS1_TH0_ADDR;
            reg1 = SI114X_PS1_TH1_ADDR;
            break;

        case 2:
            reg0 = SI114X_PS2_TH0_ADDR;
            reg1 = SI114X_PS2_TH1_ADDR;
            break;

        case 3:
            reg0 = SI114X_PS3_TH0_ADDR;
            reg1 = SI114X_PS3_TH1_ADDR;
            break;

        default:
            return SYS_EINVAL;
    }

    rc = si114x_write8(itf, reg0, threshold & 0xff);
    if (rc) {
        goto err;
    }

    rc = si114x_write8(itf, reg1, (threshold >> 8) & 0xff);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

int
si114x_set_ambient_threshold_low(struct sensor_itf *itf, uint16_t threshold)
{
    int rc;

    rc = si114x_write8(itf, SI114X_ALS_LOW_TH0_ADDR, threshold & 0xff);
    if (rc) {
        goto err;
    }

    rc = si114x_write8(itf, SI114X_ALS_LOW_TH1_ADDR, (threshold >> 8) & 0xff);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

int
si114x_set_ambient_threshold_high(struct sensor_itf *itf, uint16_t threshold)
{
    int rc;

    rc = si114x_write8(itf, SI114X_ALS_HI_TH0_ADDR, threshold & 0xff);
    if (rc) {
        goto err;
    }

    rc = si114x_write8(itf, SI114X_ALS_HI_TH1_ADDR, (threshold >> 8) & 0xff);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

int
si114x_set_proximity_rate(struct sensor_itf *itf, uint8_t proximity_rate)
{
    return si114x_write8(itf, SI114X_PS_RATE_ADDR, proximity_rate);
}

int
si114x_set_proximity_adc_gain(struct sensor_itf *itf, uint8_t proximity_adc_gain)
{
    return si114x_set_param(itf, SI114X_PARAM_PS_ADC_GAIN_OFFSET, proximity_adc_gain);
}

int
si114x_set_proximity_adc_recovery(struct sensor_itf *itf, uint8_t proximity_adc_recovery)
{
    return si114x_set_param(itf, SI114X_PARAM_PS_ADC_COUNTER_OFFSET, proximity_adc_recovery << 4);
}

//todo protect ps_range, setter for it?
int
si114x_set_proximity_adc_mode(struct sensor_itf *itf, enum si114x_proximity_adc_mode proximity_adc_mode)
{
    int rc;

    rc = si114x_set_param(itf, SI114X_PARAM_PS_ADC_MISC_OFFSET, proximity_adc_mode << 2);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

int
si114x_set_proximity_drive_mask(struct sensor_itf *itf, uint8_t number, uint8_t drive_mask)
{
    int rc;
    uint8_t offset, mask;

    switch (number) {
        case 1:
            offset = SI114X_PARAM_PSLED12_SELECT_OFFSET;
            mask = 0x07;
            break;

        case 2:
            offset = SI114X_PARAM_PSLED12_SELECT_OFFSET;
            mask = 0x07 << 4;
            break;

        case 3:
            offset = SI114X_PARAM_PSLED3_SELECT_OFFSET;
            mask = 0x07;
            break;

        default:
            return SYS_EINVAL;
    }

    rc = si114x_and_param(itf, offset, ~mask);
    if (rc) {
        goto err;
    }

    rc = si114x_or_param(itf, offset, drive_mask);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

int
si114x_set_proximity_sense(struct sensor_itf *itf, uint8_t number, struct proximity_sense *ps)
{
    int rc;
    uint8_t int_mask, mux_offset;

    switch (number) {
        case 1:
            int_mask = SI114X_IRQ_ENABLE_PS1_IE_MASK;
            mux_offset = SI114X_PARAM_PS1_ADCMUX_OFFSET;
            break;

        case 2:
            int_mask = SI114X_IRQ_ENABLE_PS2_IE_MASK;
            mux_offset = SI114X_PARAM_PS2_ADCMUX_OFFSET;
            break;

        case 3:
            int_mask = SI114X_IRQ_ENABLE_PS3_IE_MASK;
            mux_offset = SI114X_PARAM_PS3_ADCMUX_OFFSET;
            break;

        default:
            return SYS_EINVAL;
    }

    if(ps->int_en){
        rc = si114x_set_int_pin_cfg(itf, int_mask);
        if (rc) {
            goto err;
        }

        if (int_mask <= 0xff) {
            rc = si114x_set_irq1_mode(itf, ps->int_mode);
            if (rc) {
                goto err;
            }
        } else {
            rc = si114x_set_irq2_mode(itf, ps->int_mode);
            if (rc) {
                goto err;
            }
        }

    }else{
        rc = si114x_clear_int_pin_cfg(itf, int_mask);
        if (rc) {
            goto err;
        }

        if (int_mask <= 0xff) {
            rc = si114x_clear_irq1_mode(itf, ps->int_mode);
            if (rc) {
                goto err;
            }
        } else {
            rc = si114x_clear_irq2_mode(itf, ps->int_mode);
            if (rc) {
                goto err;
            }
        }
    }


    rc = si114x_set_proximity_current(itf, number, ps->current);
    if (rc) {
        goto err;
    }

    si114x_set_proximity_threshold(itf, number, ps->threshold);
    if (rc) {
        goto err;
    }

    rc = si114x_set_param(itf, mux_offset, ps->mux);
    if (rc) {
        goto err;
    }

    si114x_set_proximity_drive_mask(itf, number, ps->led_drive_mask);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

/**
 * Set a proximity struct
 *
 * @param itf The sensor interface
 * @param proximity The proximity struct
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_proximity(struct sensor_itf *itf, struct proximity *proximity)
{
    int rc;

    rc = si114x_set_proximity_sense(itf, 1, &proximity->one);
    if (rc) {
        goto err;
    }

    rc = si114x_set_proximity_sense(itf, 2, &proximity->two);
    if (rc) {
        goto err;
    }

    rc = si114x_set_proximity_sense(itf, 3, &proximity->three);
    if (rc) {
        goto err;
    }

    rc = si114x_set_proximity_adc_gain(itf, proximity->adc_gain);
    if (rc) {
        goto err;
    }

    rc = si114x_set_proximity_adc_mode(itf, proximity->adc_mode);
    if (rc) {
        goto err;
    }

    rc = si114x_set_proximity_adc_recovery(itf, proximity->adc_recovery);
    if (rc) {
        goto err;
    }

    rc = si114x_set_proximity_rate(itf, proximity->rate);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;
}

/**
 * Set an ambient struct
 *
 * @param itf The sensor interface
 * @param ambient The ambient struct
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_ambient(struct sensor_itf *itf, struct ambient *ambient)
{
    int rc;

    rc = si114x_write8(itf, SI114X_ALS_RATE_ADDR, ambient->rate);
    if (rc) {
        goto err;
    }

    rc =si114x_set_ambient_threshold_low(itf, ambient->threshold_high);
    if (rc) {
        goto err;
    }

    rc = si114x_set_ambient_threshold_low(itf, ambient->threshold_low);
    if (rc) {
        goto err;
    }

    if (ambient->int_en) {
        rc = si114x_set_int_pin_cfg(itf, SI114X_IRQ_ENABLE_ALS_IE_MASK);
        if (rc) {
            goto err;
        }

        rc = si114x_set_irq1_mode(itf, ambient->int_mode);
        if (rc) {
            goto err;
        }
    }else{
        rc = si114x_clear_int_pin_cfg(itf, SI114X_IRQ_ENABLE_ALS_IE_MASK);
        if (rc) {
            goto err;
        }

        rc = si114x_clear_irq1_mode(itf, ambient->int_mode);
        if (rc) {
            goto err;
        }
    }

    return 0;
err:
    return rc;
}

/**
 * Set the operation mode
 *
 * @param itf The sensor interface
 * @param op_mode Enum op mode to select
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_op_mode(struct sensor_itf *itf, enum si114x_op_mode op_mode)
{
    int rc;

    //The Si1141/42/43 operates in Autonomous Operation mode when MEAS_RATE is non-zero. The MEAS_RATE represents the time interval at which the Si1141/42/43 wakes up periodically. Once the internal sequencer has awoken, the sequencer manages an internal PS Counter and ALS Counter based on the PS_RATE and ALS_RATE registers.
    //fuck does SI114X_PARAM_PS_AUTO do then???
    //todo need to tie MEAS_RATE and this together somehow?
    switch (op_mode) {
        case SI114X_OP_FORCED:
            break;

        case SI114X_OP_PS_AUTO:
            rc = si114x_write_command(itf, SI114X_PARAM_PS_AUTO);
            if (rc) {
                goto err;
            }
            break;

        case SI114X_OP_ALS_AUTO:
            rc = si114x_write_command(itf, SI114X_PARAM_ALS_AUTO);
            if (rc) {
                goto err;
            }
            break;

        case SI114X_OP_PS_ALS_AUTO:
            rc = si114x_write_command(itf, SI114X_PARAM_PSALS_AUTO);
            if (rc) {
                goto err;
            }
            break;

        default:
            return SYS_EINVAL;
    }

    return 0;
err:
    return rc;
}

/**
 * Set the measurement rate
 *
 * Measurement Rate is an 8-bit compressed value representing a 16-bit integer.
 * The uncompressed 16-bit value, when multiplied by 31.25 us, represents the time
 * duration between wake-up periods where measurements are made.
 *
 * @param itf The sensor interface
 * @param measure_rate The multiple of 31.25 μs measurement rate (IE 0x99 x 31.25 μs = 4.78125ms)
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_measure_rate(struct sensor_itf *itf, uint8_t measure_rate)
{
    return si114x_write8(itf, SI114X_MEAS_RATE_ADDR, measure_rate);
}

/**
 * Set the measurement mask
 *
 * @param itf The sensor interface
 * @param measure_mask The measure_mask, an OR of the defines above
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_measure_mask(struct sensor_itf *itf, uint8_t measure_mask)
{
    return si114x_set_param(itf, SI114X_PARAM_CHLIST_OFFSET, measure_mask);
}

/**
 * Force a proximity reading when in forced mode
 *
 * @param itf The sensor interface
 * @return 0 on success, non-zero on failure
 */
int
si114x_proximity_force(struct sensor_itf *itf)
{
    return si114x_write_command(itf, SI114X_PARAM_PS_FORCE);
}

/**
 * Force a proximity reading when in forced mode.
 * The reading is returned in the handler
 *
 * @param itf The sensor interface
 * @return 0 on success, non-zero on failure
 */
int
si114x_ambient_force(struct sensor_itf *itf)
{
    return si114x_write_command(itf, SI114X_PARAM_ALS_FORCE);
}

/**
 * Force both an ambient and proximity reading when in forced mode
 * The reading is returned in the handler
 *
 * @param itf The sensor interface
 * @return 0 on success, non-zero on failure
 */
int
si114x_force(struct sensor_itf *itf)
{
    return si114x_write_command(itf, SI114X_PARAM_PSALS_FORCE);
}

/**
 * Set up  the si114x with the configuration parameters given
 *
 * @param dev  Pointer to the si114x_dev device descriptor
 * @param cfg  Pointer to the si114x_cfg settings for mode, type, calibration
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_config(struct si114x *si114x, struct si114x_cfg *cfg)
{
    int i, rc;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(si114x->sensor));

    rc = si114x_write_command(itf, SI114X_PARAM_RESET);
    if (rc) {
        goto err;
    }

    // No Commands should be issued to the device for at least 1 ms after a Reset is issued.
    os_time_delay(OS_TICKS_PER_SEC / 1000 + 1);

    //doesnt seem to give responses for a few commands, run some nops through
    for (i = 5; i>0; i--){
        rc = si114x_write_command(itf, SI114X_PARAM_NOP);
        if (rc) {
            goto err;
        }
    }

    rc = si114x_write8(itf, SI114X_HW_KEY_ADDR, 0x17);
    if (rc) {
        goto err;
    }

    rc = si114x_set_measure_mask(itf, cfg->measure_mask);
    if (rc) {
        goto err;
    }
    rc = si114x_set_proximity(itf, &cfg->proximity);
    if (rc) {
        goto err;
    }

    rc = si114x_set_ambient(itf, &cfg->ambient);
    if (rc) {
        goto err;
    }

    rc = si114x_set_measure_rate(itf, cfg->measure_rate);
    if (rc) {
        goto err;
    }

    rc = si114x_set_op_mode(itf, cfg->op_mode);
    if (rc) {
        goto err;
    }

    rc = si114x_set_rd_clr(itf, cfg->int_rd_clr);
    if (rc) {
        goto err;
    }

    rc = sensor_set_type_mask(&(si114x->sensor), cfg->mask);
    if (rc) {
        goto err;
    }

    //todo .. this isnt kept up if anyone uses accessor methods..
    //dont know why wasting all this memory to save this atm..
    si114x_sensor_set_config(&(si114x->sensor), &(si114x->cfg));

    si114x->cfg.read_mode.int_cfg = cfg->read_mode.int_cfg;
    si114x->cfg.read_mode.mode = cfg->read_mode.mode;

    if (!cfg->notif_cfg) {
        si114x->cfg.notif_cfg = (struct si114x_notif_cfg *)dflt_notif_cfg;
        si114x->cfg.max_num_notif = sizeof(dflt_notif_cfg)/sizeof(*dflt_notif_cfg);
    } else {
        si114x->cfg.notif_cfg = cfg->notif_cfg;
        si114x->cfg.max_num_notif = cfg->max_num_notif;
    }

    si114x->cfg.mask = cfg->mask;

    return 0;
err:
    return rc;
}
