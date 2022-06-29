/**
 * @file ldma_config.h
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2022
 */

#ifndef LDMA_CONFIG_H_
#define LDMA_CONFIG_H_

#include "em_ldma.h"
#include "retargetserialconfig.h"
#include "cmsis_os2.h"

#define ACC_LDMA_CHANNEL_UART	        2 // Channel number 0...7
#define ACC_LDMA_CHANNEL_UART_MASK      (1 << ACC_LDMA_CHANNEL_UART)

// tsb0 and smnt-mb platforms use different USART for log communication
// Using LOGGER_LDMA_USART define from retargetserialconfig.h to know which is used 
#if defined(LOGGER_LDMA_USART0)
    #define CNF_LDMA_PERIPHERAL_SIGNAL ldmaPeripheralSignal_USART0_TXBL
#elif defined(LOGGER_LDMA_USART2)
    #define CNF_LDMA_PERIPHERAL_SIGNAL ldmaPeripheralSignal_USART2_TXBL
#else
    #error "Unknown USART used for logging. Check retargetserialconfig.h."
#endif

void ldma_init (osThreadId_t thread_id, uint32_t thread_flag);
void ldma_uart_start (LDMA_Descriptor_t* uartDescriptor);
void ldma_uart_stop ();
bool ldma_busy ();

#endif // LDMA_CONFIG_H_
