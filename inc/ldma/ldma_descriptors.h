/**
 * @file ldma_descriptors.h
 * 
 * @brief LDMA descriptors are generated for data transfer to serial.
 * 
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2022
 */

#ifndef LDMA_DESCRIPTORS_H_
#define LDMA_DESCRIPTORS_H_

#include "retargetserialconfig.h"
#include "acc_stream_config.h"

#define DATA_LEN_FOR_ONE_AXIS_BYTES         NUM_ELEMENTS_IN_DATA_FRAME * 2 // Data element is 16 bit
#define LDMA_TRANSFER_DATA_LEN_BYTES        DATA_LEN_FOR_ONE_AXIS_BYTES // Max number of bytes in one LDMA transfer
#define NUM_LDMA_DESCRIPTORS                ((LDMA_TRANSFER_DATA_LEN_BYTES / 2048UL) + 1)

// tsb0 and smnt-mb platforms use different USART for log communication
#define USART_FOR_LDMA RETARGET_UART

LDMA_Descriptor_t* data_descriptor_config(uint32_t * memAddr, uint32_t data_len_bytes);
LDMA_Descriptor_t* token_descriptor_config(uint32_t * memAddr, uint32_t data_len_bytes);

#endif // LDMA_DESCRIPTORS_H_
