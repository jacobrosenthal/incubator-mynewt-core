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

#ifndef __SI114X_PRIV_H__
#define __SI114X_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>


#define SI114X_PART_ID_ADDR 0x00

#define SI114X_PART_ID_SI1141                           0x41
#define SI114X_PART_ID_SI1142                           0x42
#define SI114X_PART_ID_SI1143                          	0x43

#define SI114X_REV_ID_ADDR 0x01
#define SI114X_SEQ_ID_ADDR 0x02

#define SI114X_INT_CFG_ADDR 0x03
#define SI114X_INT_CFG_INT_OE_POS                            	0
#define SI114X_INT_CFG_INT_OE_MASK                           	(0x1 << SI114X_INT_CFG_INT_OE_POS)
#define SI114X_INT_CFG_INT_MODE_POS                            	1
#define SI114X_INT_CFG_INT_MODE_MASK                           	(0x1 << SI114X_INT_CFG_INT_MODE_POS)


#define SI114X_IRQ_ENABLE_ADDR 0x04
#define SI114X_IRQ_ENABLE_ALS_IE_POS							0
#define SI114X_IRQ_ENABLE_ALS_IE_MASK                           (0x3 << SI114X_IRQ_ENABLE_ALS_IE_POS)
#define SI114X_IRQ_ENABLE_PS1_IE_POS							2
#define SI114X_IRQ_ENABLE_PS1_IE_MASK                           (0x1 << SI114X_IRQ_ENABLE_PS1_IE_POS)
#define SI114X_IRQ_ENABLE_PS2_IE_POS							3
#define SI114X_IRQ_ENABLE_PS2_IE_MASK                           (0x1 << SI114X_IRQ_ENABLE_PS2_IE_POS)
#define SI114X_IRQ_ENABLE_PS3_IE_POS							4
#define SI114X_IRQ_ENABLE_PS3_IE_MASK                           (0x1 << SI114X_IRQ_ENABLE_PS3_IE_POS)
#define SI114X_IRQ_ENABLE_CMD_IE_POS							5
#define SI114X_IRQ_ENABLE_CMD_IE_MASK                           (0x1 << SI114X_IRQ_ENABLE_CMD_IE_POS)


#define SI114X_IRQ_MODE1_ADDR 0x05
#define SI114X_IRQ_MODE1_ALS_IM_POS								0
#define SI114X_IRQ_MODE1_ALS_IM_MASK                           	(0x7 << SI114X_IRQ_MODE1_ALS_IM_POS)
#define SI114X_IRQ_MODE1_PS1_IM_POS								4
#define SI114X_IRQ_MODE1_PS1_IM_MASK                           	(0x3 << SI114X_IRQ_MODE1_PS1_IM_POS)
#define SI114X_IRQ_MODE1_PS2_IM_POS								6
#define SI114X_IRQ_MODE1_PS2_IM_MASK                           	(0x3 << SI114X_IRQ_MODE1_PS2_IM_POS)

#define SI114X_IRQ_MODE2_ADDR 0x06
#define SI114X_IRQ_MODE2_PS3_IM_POS								0
#define SI114X_IRQ_MODE2_PS3_IM_MASK                           	(0x3 << SI114X_IRQ_MODE2_PS3_IM_POS)
#define SI114X_IRQ_MODE2_CMD_IM_POS								0
#define SI114X_IRQ_MODE2_CMD_IM_MASK                           	(0x3 << SI114X_IRQ_MODE2_CMD_IM_POS)

#define SI114X_HW_KEY_ADDR 0x07
#define SI114X_MEAS_RATE_ADDR 0x08
#define SI114X_ALS_RATE_ADDR 0x09
#define SI114X_PS_RATE_ADDR 0x0A
#define SI114X_ALS_LOW_TH0_ADDR 0x0B
#define SI114X_ALS_LOW_TH1_ADDR 0x0C
#define SI114X_ALS_HI_TH0_ADDR 0x0D
#define SI114X_ALS_HI_TH1_ADDR 0x0E

#define SI114X_PS_LED21_ADDR 0x0F
#define SI114X_PS_LED21_LED2_POS								4
#define SI114X_PS_LED21_LED2_MASK                           	(0x0F << SI114X_PS_LED21_LED2_POS)
#define SI114X_PS_LED21_LED1_POS								0
#define SI114X_PS_LED21_LED1_MASK                           	(0x0F << SI114X_PS_LED21_LED1_POS)

#define SI114X_PS_LED3_ADDR 0x10
#define SI114X_PS_LED3_LED3_POS 								0
#define SI114X_PS_LED3_LED3_MASK								(0x0F << SI114X_PS_LED3_LED3_POS)

#define SI114X_PS1_TH0_ADDR 0x11
#define SI114X_PS1_TH1_ADDR 0x12
#define SI114X_PS2_TH0_ADDR 0x13
#define SI114X_PS2_TH1_ADDR 0x14
#define SI114X_PS3_TH0_ADDR 0x15
#define SI114X_PS3_TH1_ADDR 0x16
#define SI114X_PARAM_WR_ADDR 0x17
#define SI114X_COMMAND_ADDR 0x18

#define SI114X_RESPONSE_ADDR 0x20

#define SI114X_IRQ_STATUS_ADDR 0x21
#define SI114X_IRQ_STATUS_ALS_INT_POS							0
#define SI114X_IRQ_STATUS_ALS_INT_MASK                         	(0x3 << SI114X_IRQ_STATUS_ALS_INT_POS)
#define SI114X_IRQ_STATUS_PS1_INT_POS							2
#define SI114X_IRQ_STATUS_PS1_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_PS1_INT_POS)
#define SI114X_IRQ_STATUS_PS2_INT_POS							3
#define SI114X_IRQ_STATUS_PS2_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_PS2_INT_POS)
#define SI114X_IRQ_STATUS_PS3_INT_POS							4
#define SI114X_IRQ_STATUS_PS3_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_PS3_INT_POS)
#define SI114X_IRQ_STATUS_CMD_INT_POS							5
#define SI114X_IRQ_STATUS_CMD_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_CMD_INT_POS)


#define SI114X_ALS_VIS_DATA0_ADDR 0x22
#define SI114X_ALS_VIS_DATA1_ADDR 0x23
#define SI114X_ALS_IR_DATA0_ADDR 0x24
#define SI114X_ALS_IR_DATA1_ADDR 0x25
#define SI114X_PS1_DATA0_ADDR 0x26
#define SI114X_PS1_DATA1_ADDR 0x27
#define SI114X_PS2_DATA0_ADDR 0x28
#define SI114X_PS2_DATA1_ADDR 0x29
#define SI114X_PS3_DATA0_ADDR 0x2A
#define SI114X_PS3_DATA1_ADDR 0x2B
#define SI114X_AUX_DATA0_ADDR 0x2C
#define SI114X_AUX_DATA1_ADDR 0x2D
#define SI114X_PARAM_RD_ADDR 0x2E

#define SI114X_CHIP_STAT_ADDR 0x30
#define SI114X_CHIP_STAT_SLEEP_POS								0
#define SI114X_CHIP_STAT_SLEEP_MASK                         	(0x1 << SI114X_CHIP_STAT_SLEEP_POS)
#define SI114X_CHIP_STAT_SUSPEND_POS							1
#define SI114X_CHIP_STAT_SUSPEND_MASK                         	(0x1 << SI114X_CHIP_STAT_SUSPEND_POS)
#define SI114X_CHIP_STAT_RUNNING_POS							2
#define SI114X_CHIP_STAT_RUNNING_MASK                         	(0x1 << SI114X_CHIP_STAT_RUNNING_POS)

#define SI114X_ANA_IN_KEY1_ADDR 0x3B
#define SI114X_ANA_IN_KEY2_ADDR 0x3C
#define SI114X_ANA_IN_KEY3_ADDR 0x3D
#define SI114X_ANA_IN_KEY4_ADDR 0x3E


/**
 * Reads a single byte from the specified register
 *
 * @param The sensor interface
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value);


/**
 * Read data from the sensor of variable length (MAX: 12 bytes)
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
               uint8_t len);

/**
 * Writes a single byte to the specified register
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value);

/**
 * Writes a multiple bytes to the specified register (MAX: 8 bytes)
 *
 * @param The sensor interface
 * @param The register address to write to
 * @param The data buffer to write from
 *
 * @return 0 on success, non-zero error on failure.
 */
int
si114x_writelen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer,
                uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SI114X_PRIV_H__ */

