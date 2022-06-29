/**
 * Generate lots of data, send lots of messages over radio and 
 * signal alarm if messages are dropped.
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

#include "loggers_ext.h"
#include "logger_fwrite.h"

#include "endianness.h"

#include "loglevels.h"
#define __MODUUL__ "main"
#define __LOG_LEVEL__ (LOG_LEVEL_main & BASE_LOG_LEVEL)
#include "log.h"

// Include the information header binary
#include "incbin.h"
INCBIN(Header, "header.bin");

#define MSG_RECEIVE_FLAG    0x01

static osThreadId_t dr_thread_id;
static osMessageQueueId_t dr_queue_id;
static osMutexId_t statistics_mutex_id;

static uint32_t radiobytes = 0;
static bool dataloss = false;

typedef struct
{
    uint8_t bytes;
    uint8_t data_items;
    uint32_t msgnr;
    uint16_t x_first;
    uint16_t y_first;
    uint16_t z_first;
    uint16_t x_last;
    uint16_t y_last;
    uint16_t z_last;
} msg_content_t;

void data_receive_loop ()
{
    static msg_content_t msg_cont;
    bool msg_lost, data_lost;
    uint16_t last_x = -1;
    
    osDelay(500);

    for(;;)
    {
        if(osMessageQueueGet(dr_queue_id, &msg_cont, NULL, 100) == osOK)
        {
            // Increment received bytes
            if(osMutexAcquire(statistics_mutex_id, 1000) == osOK)
            {
                radiobytes += msg_cont.bytes;
                osMutexRelease(statistics_mutex_id);
            }
            else info("Mutex unavailable");
            
            // Check data for data loss (lost messages)
            msg_lost = data_lost = false;
            if(msg_cont.x_first == last_x + 1)msg_lost = true;
            if(msg_cont.x_first + msg_cont.data_items == msg_cont.x_last)data_lost = true;
            if(msg_cont.z_first == msg_cont.z_last && msg_cont.z_first == 127)data_lost = true;
            
            // NB! There will be data loss when x value wraps around (from 0xffff to 0)
            if(msg_lost | data_lost)
            {
                if(osMutexAcquire(statistics_mutex_id, 1000) == osOK)
                {
                    dataloss = true;
                    osMutexRelease(statistics_mutex_id);
                }
                else info("Mutex unavailable");
            }
            last_x = msg_cont.x_last;
        }
    }
}

void statistics_loop ()
{
    #define REPORT_INTERVAL     1 // Seconds
    static uint32_t bytes;
    static bool loss;
    
    if(osMutexAcquire(statistics_mutex_id, 1000) == osOK)
    {
        radiobytes = 0;
        dataloss = false;
        osMutexRelease(statistics_mutex_id);
    }
    else info("Mutex unavailable");
    
    for(;;)
    {
        osDelay(REPORT_INTERVAL * osKernelGetTickFreq());
        if(osMutexAcquire(statistics_mutex_id, 1000) == osOK)
        {
            bytes = radiobytes;
            loss = dataloss;
            radiobytes = 0;
            dataloss = false;
            osMutexRelease(statistics_mutex_id);
        }
        else info("Mutex unavailable");
        
        // NB! There will be data loss when x value wraps around (from 0xffff to 0)
        if(!loss)info3("During %u seconds - %lu bytes received, no loss", REPORT_INTERVAL, bytes);
        else info3("Data lost! during %u seconds - %lu bytes received", REPORT_INTERVAL, bytes);
    }
}

// HB loop - increment and send counter
void hb_loop ()
{
    dr_queue_id = osMessageQueueNew(5, sizeof(msg_content_t), NULL);
    statistics_mutex_id = osMutexNew(NULL);
    
    for (;;)
    {
        osDelay(10*osKernelGetTickFreq()); // 10 sec
        info1("Heartbeat");
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
    log_init(BASE_LOG_LEVEL, &logger_fwrite_boot, NULL);

    info1("Radio-test "VERSION_STR" (%d.%d.%d)", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

    // Initialize OS kernel
    osKernelInitialize();

    // Create a thread
    const osThreadAttr_t hp_thread_attr = { .name = "hp" };
    osThreadNew(hb_loop, NULL, &hp_thread_attr);

    const osThreadAttr_t stat_thread_attr = { .name = "stat" };
    osThreadNew(statistics_loop, NULL, &stat_thread_attr);

    const osThreadAttr_t recv_thread_attr = { .name = "recv" };
    dr_thread_id = osThreadNew(data_receive_loop, NULL, &recv_thread_attr);

    if (osKernelReady == osKernelGetState())
    {
        // Switch to a thread-safe logger
        logger_fwrite_init();
        log_init(BASE_LOG_LEVEL, &logger_fwrite, NULL);

        // Start the kernel
        osKernelStart();
    }
    else
    {
        err1("!osKernelReady");
    }

    for(;;);
}
