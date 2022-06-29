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

/**
 * Define how many measuring points constitutes a data frame. 
 * This is not equal to number of bytes in a frame!
 * Data is transmitted over serial in frames. Beware of long 
 * wait times if your frame is large and sampling speed is low.
 */
#define NUM_ELEMENTS_IN_DATA_FRAME       40      // 3 sec @ 6,25Hz
//#define ACC_XYZ_DATA_LEN        2400UL  // 3 sec @ 800Hz, 15 min 38 sec @ 1,56 Hz !!!

#endif // ACC_STREAM_CONFIG_H_
