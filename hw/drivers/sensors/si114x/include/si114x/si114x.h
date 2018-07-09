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

/* global interrupt masks */
#define SI114X_IRQ_STATUS_ALS_INT_MASK                         	(0x3 << SI114X_IRQ_STATUS_ALS_INT_POS)
#define SI114X_IRQ_STATUS_PS1_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_PS1_INT_POS)
#define SI114X_IRQ_STATUS_PS2_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_PS2_INT_POS)
#define SI114X_IRQ_STATUS_PS3_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_PS3_INT_POS)
#define SI114X_IRQ_STATUS_CMD_INT_MASK                         	(0x1 << SI114X_IRQ_STATUS_CMD_INT_POS)

/* si114x_send_command masks */
#define SI114X_PARAM_QUERY 0x80
#define SI114X_PARAM_SET 0xA0
#define SI114X_PARAM_AND 0xC0
#define SI114X_PARAM_OR 0xE0

#define SI114X_PARAM_PARAM_AND 0xC0
#define SI114X_PARAM_PARAM_OR 0xE0
#define SI114X_PARAM_NOP 0x00
#define SI114X_PARAM_RESET 0x01
#define SI114X_PARAM_BUSADDR 0x02


/* si114x_write_param and si114x_read_param offsets */
#define SI114X_PARAM_PS_FORCE 0x05
#define SI114X_PARAM_ALS_FORCE 0x06
#define SI114X_PARAM_PSALS_FORCE 0x07

#define SI114X_PARAM_PS_PAUSE 0x09
#define SI114X_PARAM_ALS_PAUSE 0x0A
#define SI114X_PARAM_PSALS_PAUSE 0x0B

#define SI114X_PARAM_PS_AUTO 0x0D
#define SI114X_PARAM_ALS_AUTO 0x0E
#define SI114X_PARAM_PSALS_AUTO 0x0F

#define SI114X_PARAM_I2C_ADDR_OFFSET 0x00
#define SI114X_PARAM_CHLIST_OFFSET 0x01
#define SI114X_PARAM_PSLED12_SELECT_OFFSET 0x02
#define SI114X_PARAM_PSLED3_SELECT_OFFSET 0x03

#define SI114X_PARAM_PS_ENCODING_OFFSET 0x05
#define SI114X_PARAM_ALS_ENCODING_OFFSET 0x06
#define SI114X_PARAM_PS1_ADCMUX_OFFSET 0x07
#define SI114X_PARAM_PS2_ADCMUX_OFFSET 0x08
#define SI114X_PARAM_PS3_ADCMUX_OFFSET 0x09
#define SI114X_PARAM_PS_ADC_COUNTER_OFFSET 0x0A
#define SI114X_PARAM_PS_ADC_GAIN_OFFSET 0x0B
#define SI114X_PARAM_PS_ADC_MISC_OFFSET 0x0C

#define SI114X_PARAM_ALS_IR_ADCMUX_OFFSET 0x0E
#define SI114X_PARAM_AUX_ADCMUX_OFFSET 0x0F
#define SI114X_PARAM_ALS_VIS_ADC_COUNTER_OFFSET 0x10
#define SI114X_PARAM_ALS_VIS_ADC_GAIN_OFFSET 0x11
#define SI114X_PARAM_ALS_VIS_ADC_MISC_OFFSET 0x12

#define SI114X_PARAM_ALS_HYST_OFFSET 0x16
#define SI114X_PARAM_PS_HYST_OFFSET 0x17
#define SI114X_PARAM_PS_HISTORY_OFFSET 0x18
#define SI114X_PARAM_ALS_HISTORY_OFFSET 0x19
#define SI114X_PARAM_ADC_OFFSET 0x1A

#define SI114X_PARAM_LED_REC_OFFSET 0x1C
#define SI114X_PARAM_ALS_IR_ADC_COUNTER_OFFSET 0x1D
#define SI114X_PARAM_ALS_IR_ADC_GAIN_OFFSET 0x1E
#define SI114X_PARAM_ALS_IR_ADC_MISC_OFFSET 0x1F

/* interrupt mode for ambient sense */
enum si114x_ambient_int_mode {
    SI114X_ALS_IM_ALS_VIS_EVERY,
    SI114X_ALS_IM_ALS_VIS_EXITING,
    SI114X_ALS_IM_ALS_VIS_ENTERING,
    SI114X_ALS_IM_ALS_IR_EXITING,
    SI114X_ALS_IM_ALS_IR_ENTERING
};

/* ambient sense setup struct */
struct ambient {
	enum si114x_ambient_int_mode int_mode;
	uint8_t rate;
	uint16_t threshold_high;
	uint16_t threshold_low;
	uint8_t int_en : 1;
};

/* interrupt mode for proximity sense */
enum si114x_proximity_int_mode {
    SI114X_PS_IM_MEASURE_COMPLETE = 0,
    SI114X_PS_IM_MEASURE_CROSS_THS = 1,
    SI114X_PS_IM_MEASURE_GREATER_THS = 2,
};

/* proximity sense adc input selection */
enum si114x_proximity_mux {
    SI114X_MUX_SMALL = 0x0,
    SI114X_MUX_VISIBLE = 0x2,
    SI114X_MUX_LARGE = 0x3,
    SI114X_MUX_NO_PHOTODIODE = 0x6,
    SI114X_MUX_GND = 0x25,
    SI114X_MUX_TEMPERATURE = 0x65,
    SI114X_MUX_VDD = 0x75
};

/* led_drive_mask */
#define SI114X_PROXIMITY_DRIVE_PS1 0x01
#define SI114X_PROXIMITY_DRIVE_PS2 0x02
#define SI114X_PROXIMITY_DRIVE_PS3 0x04

/* proximity sense adc input selection */
struct proximity_sense {
	enum si114x_proximity_int_mode int_mode;
	enum si114x_proximity_mux mux;
	uint8_t current : 4;
	uint16_t threshold;
	uint8_t led_drive_mask : 3;
	uint8_t int_en : 1;
};

/* proximity sense adc mode  */
enum si114x_proximity_adc_mode {
    SI114X_ADC_MODE_RAW = 0,
    SI114X_ADC_MODE_NORMAL
};

/* proximity sense setup struct  */
struct proximity {
	enum si114x_proximity_adc_mode adc_mode;
    struct proximity_sense one;
    struct proximity_sense two;
    struct proximity_sense three;

	uint8_t rate;
	uint8_t adc_gain : 3;
	uint8_t adc_recovery: 3;
};

/* measure_mask */
#define SI114X_MEASURE_AUX 0x40
#define SI114X_MEASURE_IR 0x20
#define SI114X_MEASURE_VIS 0x10
#define SI114X_MEASURE_PS3 0x04
#define SI114X_MEASURE_PS2 0x02
#define SI114X_MEASURE_PS1 0x01

/* si114x operation mode  */
enum si114x_op_mode {
    SI114X_OP_FORCED = 0,
    SI114X_OP_PS_AUTO,
    SI114X_OP_ALS_AUTO,
    SI114X_OP_PS_ALS_AUTO
};

struct si114x_notif_cfg {
    sensor_event_type_t event;
    uint8_t int_cfg;
};

/* top level si114x configuration struct  */
struct si114x_cfg {
    enum si114x_op_mode op_mode;

    uint8_t measure_rate;
    uint8_t measure_mask;

	/* ambient setup */
    struct ambient ambient;

    /* proximity setup */
    struct proximity proximity;

    /* global interrupts */
	uint8_t int_rd_clr : 1; //disable latched interrupts

    /* Notif config */
    struct si114x_notif_cfg *notif_cfg;
    uint8_t max_num_notif;

    /* Sensor type mask to track enabled sensors */
    sensor_type_t mask;
};

/* Used to track interrupt state to wake any present waiters */
struct si114x_int {
    /* Synchronize access to this structure */
    os_sr_t lock;
    /* Sleep waiting for an interrupt to occur */
    struct os_sem wait;
    /* Is the interrupt currently active */
    bool active;
    /* Is there a waiter currently sleeping */
    bool asleep;
    /* Configured interrupts */
    struct sensor_int *ints;
};

/* Private per driver data */
struct si114x_pdd {
    /* Notification event context */
    struct sensor_notify_ev_ctx notify_ctx;
    //  Inetrrupt state 
    struct si114x_int *interrupt;
    /* Interrupt enabled mask */
    uint8_t int_enable;
    /* Interrupt config masks */
    uint8_t int1_mode;
    uint8_t int2_mode;
};


struct si114x {
    struct os_dev dev;
    struct sensor sensor;
    struct si114x_cfg cfg;
    struct si114x_int intr;
    struct si114x_pdd pdd;
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


/* Advanced use only below */

/**
 * Get chip id from the sensor
 *
 * @param itf The sensor interface
 * @param id Pointer to the variable to fill up chip id in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_id(struct sensor_itf *itf, uint8_t *id);

/**
 * Get chip sequence from the sensor
 *
 * @param itf The sensor interface
 * @param seq Pointer to the variable to fill up chip sequence in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_sequence(struct sensor_itf *itf, uint8_t *seq);

/**
 * Get chip revision from the sensor
 *
 * @param itf The sensor interface
 * @param rev Pointer to the variable to fill up chip revision in
 * @return 0 on success, non-zero on failure
 */
int
si114x_get_chip_revision(struct sensor_itf *itf, uint8_t *rev);

/**
 * Set a proximity struct
 *
 * @param itf The sensor interface
 * @param proximity The proximity struct
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_proximity(struct sensor_itf *itf, struct proximity *proximity);

/**
 * Set an ambient struct
 *
 * @param itf The sensor interface
 * @param ambient The ambient struct
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_ambient(struct sensor_itf *itf, struct ambient *ambient);

/**
 * Set the operation mode
 *
 * @param itf The sensor interface
 * @param op_mode Enum op mode to select
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_op_mode(struct sensor_itf *itf, enum si114x_op_mode op_mode);

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
si114x_set_measure_rate(struct sensor_itf *itf, uint8_t measure_rate);

/**
 * Set the measurement mask
 *
 * @param itf The sensor interface
 * @param measure_mask The measure_mask, an OR of the defines above
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_measure_mask(struct sensor_itf *itf, uint8_t measure_mask);

/**
 * Set read clear interrupts
 *
 * @param itf The sensor interface
 * @param enabled The internal sequencer clears the INT pin automatically.
 * @return 0 on success, non-zero on failure
 */
int
si114x_set_rd_clr(struct sensor_itf *itf, uint8_t enabled);



/* Really Advanced use only below */

/**
 * Get parameter located in Parameter RAM, see section 4.6
 *
 * @param itf The sensor interface
 * @param offset The offset of parameter to read
 * @param value Pointer to the variable to fill parameter in
 * @return 0 on success, non-zero on failure
 */
int
si114x_query_param(struct sensor_itf *itf, uint8_t offset, uint8_t *value);

/**
 * Write parameter located in Parameter RAM, see section 4.6
 *
 * @param itf The sensor interface
 * @param cmd Combined offset and encoding of parameter to write
 * @param value Value to set parameter to
 * @return 0 on success, non-zero on failure
 */
int
si114x_write_param(struct sensor_itf *itf, uint8_t cmd, uint8_t value);

/**
 * Send command to device, see section 4.2
 *
 * @param itf The sensor interface
 * @param cmd Command to set parameter to
 * @return 0 on success, non-zero on failure
 */
int
si114x_write_command(struct sensor_itf *itf, uint8_t cmd);



#ifdef __cplusplus
}
#endif

#endif /* __SI114X_H__ */
