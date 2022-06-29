/**
 * @file acc_stream_config.h
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2022
 */

#ifndef ACC_STREAM_CONFIG_H_
#define ACC_STREAM_CONFIG_H_

#include "em_ldma.h"
#include "retargetserialconfig.h"
#include "cmsis_os2.h"
#include "mma8653fc_reg.h"

/**
 * Define how many measuring points constitutes a data frame. 
 * This is not equal to number of bytes in a frame!
 * Data is transmitted over serial in frames. Beware of long 
 * wait times if your frame is large and sampling speed is low.
 */
#define NUM_ELEMENTS_IN_DATA_FRAME       800
//#define NUM_ELEMENTS_IN_DATA_FRAME       40 // 6 sec @ 6,25Hz
//#define NUM_ELEMENTS_IN_DATA_FRAME        2400UL  // 3 sec @ 800Hz, 6 min 24 sec @ 6,25 Hz !!!

// Consider frame size! 
#define SENSOR_SAMPLING_SPEED MMA8653FC_CTRL_REG1_DR_800HZ

// Data range +-2g, +-4g or +-8g
#define SENSOR_DATA_RANGE MMA8653FC_XYZ_DATA_CFG_8G_RANGE

// Use high resolution power mode. (does a lot of oversampling)
#define SENSOR_SAMPLING_POWER_MODE MMA8653FC_CTRL_REG2_POWMOD_HIGHRES






#endif // ACC_STREAM_CONFIG_H_
