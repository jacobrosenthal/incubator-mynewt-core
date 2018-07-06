/*
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

#ifndef __SI114X_H__
#define __SI114X_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif



//  Off Mode
//  Initialization Mode
//  Standby Mode
//  Forced Conversion Mode  Autonomous Mode

// Allowing VDD to be less than VDD_OFF is intended to serve as a hardware method of resetting the Si1141/42/43 without a dedicated reset pin.
enum si114x_op_mode {
    SI114X_OP_OFF = 0x00,  //ie the vdd is disabled
    SI114X_OP_INIT,//wait Start-up time and it will transition to standby
    SI114X_OP_STANDBY,//The host must write 0x17 to the HW_KEY register for proper operation.
    SI114X_OP_FORCED,//ALS_FORCE or the PS_FORCE command is sent. It is possible to initiate both an ALS and multiple PS measurements with one command register write access by using the PSALS_FORCE command.
    SI114X_OP_AUTONOMOUS//PS_AUTO, ALS_AUTO and PSALS_AUTO commands are used to place the Si1141/42/43 in the Autonomous Operation Mode
};


struct si114x_cfg {
    enum si114x_op_mode op_mode;
};

struct si114x {
    struct os_dev dev;
    struct sensor sensor;
    struct si114x_cfg cfg;
};


/**
 * Initialize the si114x. This is normally used as an os_dev_create initialization function
 *
 * @param dev  Pointer to the si114x_dev device descriptor
 * @param arg  Pointer to the sensor interface
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_init(struct os_dev *dev, void *arg);


/**
 * Set up  the si114x with the configuration parameters given
 *
 * @param dev  Pointer to the si114x_dev device descriptor
 * @param cfg  Pointer to the si114x_cfg settings for mode, type, calibration
 *
 * @return 0 on success, non-zero on failure
 */
int
si114x_config(struct si114x *si114x, struct si114x_cfg *cfg);


#if MYNEWT_VAL(SI114X_CLI)
int
si114x_shell_init(void);
#endif



/**
 * Get chip ID from the sensor
 *
 * @param The sensor interface
 * @param Pointer to the variable to fill up chip ID in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_id(struct sensor_itf *itf, uint8_t *id);


#ifdef __cplusplus
}
#endif

#endif /* __SI114X_H__ */
