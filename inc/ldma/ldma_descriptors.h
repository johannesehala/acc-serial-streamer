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

#define MAX_DATA_LEN_BYTES          114 // Max length for radio msg
#define NUM_LDMA_DESCRIPTORS        ((MAX_DATA_LEN_BYTES / 2048UL) + 1)

// tsb0 and smnt-mb platforms use different USART for log communication
#define USART_FOR_LDMA RETARGET_UART

LDMA_Descriptor_t* msg_descriptor_config(uint32_t * memAddr, uint32_t payload_len_bytes);
LDMA_Descriptor_t* token_descriptor_config(uint32_t * memAddr, uint32_t data_len_bytes);

#endif // LDMA_DESCRIPTORS_H_
