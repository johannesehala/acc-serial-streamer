/**
 * Receive lots of messages over radio and 
 * signal alarm if messages are dropped or missed.
 *
 * Possible speed gains:
 *  - let ldma do ntoh conversion (This is done already)
 *  - use higher serial speed
 *  - don't defer msg handling to thread, use ldma from receive msg 
 *    interrupt (cuz queue does two copy operations)
 *
 * Copyright Thinnect Inc. 2019
 * Copyright Proactivity-Lab, Taltech 2022
 * @license MIT
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "retargetserial.h"

#include "cmsis_os2.h"

#include "platform.h"

#include "SignatureArea.h"
#include "DeviceSignature.h"

#include "ldma_config.h"
#include "ldma_descriptors.h"

#include "endianness.h"

// Include the information header binary
#include "incbin.h"
INCBIN(Header, "header.bin");

#define MAX_PAYLOAD_SIZE 114

static osThreadId_t dr_thread_id;
static osMessageQueueId_t dr_queue_id;

/**
 * @note    Expecting msg payload first 4 bytes to be msg sequence number.
 *          Expecting msg payload size (includeing seq.nr.) to be 100 bytes.
 *
 */
void data_receive_loop ()
{
    #define LDMA_READY_FLAG         0x04
    #define LDMA_READY_WAIT_TIME    500 // Kernel ticks
    
    static uint8_t msg[MAX_PAYLOAD_SIZE];
    static const uint16_t token[] = {0xDEAD, 0xBEEF};
    uint32_t msg_nr, last_msg_nr, tf_wait;
    
    dr_queue_id = osMessageQueueNew(5, MAX_PAYLOAD_SIZE, NULL);
    
    osDelay(500);
    
    ldma_init(dr_thread_id, LDMA_READY_FLAG);
    ldma_uart_start(token_descriptor_config((uint32_t *)token, 4));
    
    for(;;)
    {
        // TOKEN TEST
        /*osDelay(10);
        *((uint32_t*)msg) = hton32(0xDEADBEEF);
        tf_wait = osThreadFlagsWait(LDMA_READY_FLAG, osFlagsWaitAny, LDMA_READY_WAIT_TIME);
        if(tf_wait == LDMA_READY_FLAG)
        {
            ldma_uart_start(msg_descriptor_config(((uint32_t*)msg), 100));
            PLATFORM_LedsSet(PLATFORM_LedsGet() ^ 0x01);
        }
        else;*/
        
        if(osMessageQueueGet(dr_queue_id, &msg, NULL, 3000) == osOK)
        {
            // Check msg sequence number
            msg_nr = ntoh32(*((uint32_t*)msg));
            
            if(msg_nr != (last_msg_nr + 1));//info3("Message lost %lu", msg_nr-last_msg_nr);
            else ;//info3("msg ok");
            
            last_msg_nr = msg_nr;
            
            // Write bytes to serial using ldma
            // ldma also converts network byte order to host byte order
            *((uint32_t*)msg) = hton32(0xDEADBEEF);
            tf_wait = osThreadFlagsWait(LDMA_READY_FLAG, osFlagsWaitAll, LDMA_READY_WAIT_TIME);
            if(tf_wait == LDMA_READY_FLAG)
            {
                ldma_uart_start(msg_descriptor_config(((uint32_t*)msg), 100));
                //info3("send bytes %lu - %u", ((uint16_t*)msg)+4, ntoh16(*(((uint16_t*)msg)+4)));
                PLATFORM_LedsSet(PLATFORM_LedsGet() ^ 0x01);
            }
            else
            {
                //info3("ldma busy error");
            }
        }
        else ;//info3("No msg yet");
    }
}

// HB loop - increment and send counter
void hb_loop ()
{    
    for (;;)
    {
        osDelay(10*osKernelGetTickFreq()); // 10 sec
        //info1("Heartbeat");
    }
}

int logger_fwrite_boot (const char *ptr, int len)
{
    fwrite(ptr, len, 1, stdout);
    fflush(stdout);
    return len;
}

int main ()
{
    PLATFORM_Init();

    // LEDs
    PLATFORM_LedsInit();

    // Configure debug output
    RETARGET_SerialInit();

    // Initialize OS kernel
    osKernelInitialize();

    // Create a thread
    const osThreadAttr_t hp_thread_attr = { .name = "hp" };
    osThreadNew(hb_loop, NULL, &hp_thread_attr);

    const osThreadAttr_t recv_thread_attr = { .name = "recv" };
    dr_thread_id = osThreadNew(data_receive_loop, NULL, &recv_thread_attr);

    if (osKernelReady == osKernelGetState())
    {
        // Start the kernel
        osKernelStart();
    }
    else
    {
        //err1("!osKernelReady");
    }

    for(;;);
}
