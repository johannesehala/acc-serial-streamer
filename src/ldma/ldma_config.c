/**
 * @file ldma_config.c
 *
 * @brief   Here LDMA is initialized and LDMA channels are configured and started.
 *          LDMA IRQ handler is here.
 * 
 * 
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2022
 */

#include "em_cmu.h"
#include "ldma_config.h"

osThreadId_t ldma_ready_callback_thread;
uint32_t ldma_ready_flag;

/**
 * @brief LDMA IRQ handler.
 */
void LDMA_IRQHandler(void)
{
    /* Get all pending and enabled interrupts. */
    uint32_t pending = LDMA_IntGetEnabled();

    /* Loop here on an LDMA error to enable debugging. */
    while (pending & LDMA_IF_ERROR) 
    {
        PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
    }

    if(pending & ACC_LDMA_CHANNEL_UART_MASK)
    {
        /* Clear interrupt flag. */
        LDMA->IFC = ACC_LDMA_CHANNEL_UART_MASK;
        osThreadFlagsSet(ldma_ready_callback_thread, ldma_ready_flag);
    }
}

/**
 * @brief Initialize the LDMA controller.
 */
void ldma_init (osThreadId_t thread_id, uint32_t thread_flag)
{
    LDMA_Init_t init = LDMA_INIT_DEFAULT; // Only priority based arbitration, no round-robin.
    ldma_ready_callback_thread = thread_id;
    ldma_ready_flag = thread_flag;
    
    CMU_ClockEnable(cmuClock_LDMA, true);
    
    LDMA_Init(&init);
    NVIC_ClearPendingIRQ(LDMA_IRQn);
    NVIC_EnableIRQ(LDMA_IRQn);
    NVIC_SetPriority(LDMA_IRQn, 3);
}

/**
 * @brief Start LDMA for memory to UART transfer.
 */
void ldma_uart_start(LDMA_Descriptor_t* uartDescriptor)
{
    LDMA_TransferCfg_t memToUartCfg = LDMA_TRANSFER_CFG_PERIPHERAL(CNF_LDMA_PERIPHERAL_SIGNAL);

    LDMA_IntEnable(ACC_LDMA_CHANNEL_UART_MASK);
    
    LDMA_StartTransfer(ACC_LDMA_CHANNEL_UART, &memToUartCfg, uartDescriptor);
}

void ldma_uart_stop (void)
{
    LDMA_StopTransfer(ACC_LDMA_CHANNEL_UART);
}

bool ldma_busy()
{
    return !LDMA_TransferDone(ACC_LDMA_CHANNEL_UART);
}
