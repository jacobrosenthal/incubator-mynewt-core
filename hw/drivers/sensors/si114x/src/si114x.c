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

#if MYNEWT_VAL(SI114X_LOG)
#include "log/log.h"
#endif

#if MYNEWT_VAL(SI114X_STATS)
#include "stats/stats.h"
#endif

#if MYNEWT_VAL(SI114X_STATS)
/* Define the stats section and records */
STATS_SECT_START(si114x_stat_section)
    STATS_SECT_ENTRY(errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(si114x_stat_section)
    STATS_NAME(si114x_stat_section, errors)
STATS_NAME_END(si114x_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(si114x_stat_section) g_si114xstats;
#endif

#if MYNEWT_VAL(SI114X_LOG)
#define LOG_MODULE_SI114X (306)
#define SI114X_INFO(...)  LOG_INFO(&_log, LOG_MODULE_SI114X, __VA_ARGS__)
#define SI114X_ERR(...)   LOG_ERROR(&_log, LOG_MODULE_SI114X, __VA_ARGS__)
static struct log _log;
#else
#define SI114X_INFO(...)
#define SI114X_ERR(...)
#endif


/**
 * Writes a single byte to the specified register
 *
 * @param The Sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value)
{
    int rc;
    uint8_t payload[2] = { reg, value};

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 2,
        .buffer = payload
    };

    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC, 1);
    if (rc) {
        SI114X_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                       data_struct.address, reg, value);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
    }

    return rc;
}

/**
 * Writes multiple bytes starting at the specified register (MAX: 8 bytes)
 *
 * @param The Sensor interface
 * @param The register address to write to
 * @param The data buffer to write from
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_writelen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer,
                      uint8_t len)
{
    int rc;
    uint8_t payload[9] = { reg, 0, 0, 0, 0, 0, 0, 0, 0};

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

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        SI114X_ERR("I2C access failed at address 0x%02X\n", data_struct.address);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
        goto err;
    }

    return 0;
err:
    return rc;
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

    /* Register write */
    payload = reg;
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        SI114X_ERR("I2C register write failed at address 0x%02X:0x%02X\n",
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
        SI114X_ERR("Failed to read from 0x%02X:0x%02X\n", data_struct.address, reg);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
    }

err:
    return rc;
}

/**
 * Read data from the sensor of variable length (MAX: 8 bytes)
 *
 * @param The Sensor interface
 * @param Register to read from
 * @param Buffer to read into
 * @param Length of the buffer
 *
 * @return 0 on success and non-zero on failure
 */
int
si114x_readlen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer,
               uint8_t len)
{
    int rc;
    uint8_t payload[23] = { reg, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0};

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = payload
    };

    /* Clear the supplied buffer */
    memset(buffer, 0, len);

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        SI114X_ERR("I2C access failed at address 0x%02X\n", data_struct.address);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
        goto err;
    }

    /* Read len bytes back */
    memset(payload, 0, sizeof(payload));
    data_struct.len = len;
    rc = hal_i2c_master_read(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        SI114X_ERR("Failed to read from 0x%02X:0x%02X\n", data_struct.address, reg);
#if MYNEWT_VAL(SI114X_STATS)
        STATS_INC(g_si114xstats, errors);
#endif
        goto err;
    }

    /* Copy the I2C results into the supplied buffer */
    memcpy(buffer, payload, len);

    return 0;
err:
    return rc;
}

// 1. Write 0x00 to Command register to clear the Response register.
// 2. Read Response register and verify contents are 0x00.
// 3. Write Command value from Table 5 into Command register.
// 4. Read the Response register and verify contents are now non-zero. If contents are still 0x00, repeat this step.

/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this haptic feedback controller
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_init(struct os_dev *dev, void *arg)
{
    struct si114x *si114x;
    struct sensor *sensor;
    uint8_t id;
    int rc;

    if (!arg || !dev) {
        rc = SYS_ENODEV;
        goto err;
    }

    si114x = (struct si114x *) dev;

#if MYNEWT_VAL(SI114X_LOG)
    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);
#endif

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

    /* Set the interface */
    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        goto err;
    }

    /* Check if we can read the chip address */
    rc = si114x_get_chip_id(arg, &id);
    if (rc) {
        SI114X_ERR("unable to get chip id [1]: %d\n", rc);
        goto err;
    }

    if (id != SI114X_PART_ID_SI1141 && id != SI114X_PART_ID_SI1142 && id != SI114X_PART_ID_SI1143) {
        os_time_delay((OS_TICKS_PER_SEC * 100)/1000 + 1);

        rc = si114x_get_chip_id(arg, &id);
        if (rc) {
            SI114X_ERR("unable to get chip id [2]: %d\n", rc);
            goto err;
        }

        if (id != SI114X_PART_ID_SI1141 && id != SI114X_PART_ID_SI1142 && id != SI114X_PART_ID_SI1143) {
            rc = SYS_EINVAL;
            SI114X_ERR("id not as expected: got: %d, expected %d or %d or %d\n", id,
                        SI114X_PART_ID_SI1141, SI114X_PART_ID_SI1142, SI114X_PART_ID_SI1143);
            goto err;
        }
    }

    return (0);
err:
    SI114X_ERR("Error initializing SI114X: %d\n", rc);
    return (rc);
}

/**
 * Get chip ID from the sensor
 *
 * @param The sensor interface
 * @param Pointer to the variable to fill up chip ID in
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

// note device MUST be reconfigured for an operational state after a an
// error or after succsessful diag and calibration and reset upon success
// the device is always left in SI114X_POWER_STANDBY state no device state
// is guaranteed for error returns
int
si114x_config(struct si114x *si114x, struct si114x_cfg *cfg)
{
    // int rc;
    // struct sensor_itf *itf;

    // itf = SENSOR_GET_ITF(&(si114x->sensor));


    return 0;
}

