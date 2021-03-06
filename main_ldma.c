/**
 * @brief   Communicates with accelerometer over I2C protocol. Writes results to
 *          log output or sends constant data stream to serial.
 *
 *          To stream accelerometer x, y, z axis data to serial, turn off logging, 
 *          and turn on LDMA. x, y, z axis data is int16_t type and data is sent
 *          in sequence x1, x2, x3, ... y1, y2, y3 ... z1, z2, z3 ...
 *
 *          Data is transfed per frame. NUM_ELEMENTS_IN_DATA_FRAME determines frame
 *          size. A token is sent before each data frame.
 *
 * @note    Data transfer over serial is initiated when one of the two data buffers
 *          is filled. Depending on the chosen data rate and buffer size, this may
 *          take a lot of time. It is reccomended you adjust your data buffer size 
 *          (NUM_ELEMENTS_IN_DATA_FRAME) to suit your data rate.
 * 
 * @note    If you are using this application for streaming accelerometer readout
 *          to PC, then use convert_to_count() function and stream 'count' values to 
 *          serial. Using convert_to_g() changes value type from int16_t to float
 *          and float values are difficult to send over serial correctly. 
 
 * @Datasheet https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
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

#include "i2c_config.h"
#include "gpio_config.h"
#include "ldma_config.h"
#include "ldma_descriptors.h"
#include "mma8653fc_reg.h"
#include "mma8653fc_driver.h"
#include "acc_stream_config.h"

#include "endianness.h"

// Include the information header binary
#include "incbin.h"
INCBIN(Header, "header.bin");

#define START_TRANSFER_FLAG 0x00000008U

typedef enum 
{
    BUF_1 = 5,
    BUF_2 = 6
} xyz_buf_select_t;

static osThreadId_t transfer_thread_id;

static int16_t buf1_x[NUM_ELEMENTS_IN_DATA_FRAME];
static int16_t buf1_y[NUM_ELEMENTS_IN_DATA_FRAME];
static int16_t buf1_z[NUM_ELEMENTS_IN_DATA_FRAME];
static int16_t buf2_x[NUM_ELEMENTS_IN_DATA_FRAME];
static int16_t buf2_y[NUM_ELEMENTS_IN_DATA_FRAME];
static int16_t buf2_z[NUM_ELEMENTS_IN_DATA_FRAME];
static xyz_buf_select_t buf_transmitting = BUF_2;
static bool xyz_transmit_done = true, transmit_late = false;


/**
 * @brief   Configures I2C, GPIO and sensor, wakes up on MMA8653FC data ready interrupt, fetches
 *          a batch of sensor data and analyzes data.
 */
static void data_acquisition_loop (void *args)
{
    #define DATA_READY_FLAG     0x00000001U
    #define DATA_WAIT_TIME      3 // Time to wait for data before timeout, seconds.
    
    xyz_rawdata_t rawData;
    static xyz_buf_select_t buf_filling = BUF_1;
    static uint16_t buf_index = 0;
    static bool trigger_transmit = false;
    static uint32_t flags;

    // Initialize and enable I2C.
    i2c_init();
    i2c_enable();

    // To configure sensor put sensor in standby mode.
    set_sensor_standby();

    // Configure sensor for xyz data acquisition.
    configure_xyz_data(SENSOR_SAMPLING_SPEED, SENSOR_DATA_RANGE, SENSOR_SAMPLING_POWER_MODE);

    // Configure sensor to generate interrupt when new data becomes ready.
    configure_interrupt(MMA8653FC_CTRL_REG3_POLARITY_LOW, MMA8653FC_CTRL_REG3_PINMODE_PP, 
            (MMA8653FC_CTRL_REG4_DRDY_INT_EN << MMA8653FC_CTRL_REG4_DRDY_INT_SHIFT), 
            (MMA8653FC_CTRL_REG5_DRDY_INTSEL_INT1<< MMA8653FC_CTRL_REG5_DRDY_INTSEL_SHIFT));

    // Configure GPIO for external interrupts and enable external interrupts.
    gpio_external_interrupt_init();
    gpio_external_interrupt_enable(osThreadGetId(), DATA_READY_FLAG);

    // Activate sensor.
    set_sensor_active();
    
    for (;;)
    {
        // Wait for data ready interrupt signal from MMA8653FC sensor.
        osThreadFlagsClear(DATA_READY_FLAG);
        flags = osThreadFlagsWait(DATA_READY_FLAG, osFlagsWaitAny, DATA_WAIT_TIME*osKernelGetTickFreq()); // Flags are automatically cleared
        
        // The DATA_READY_FLAG flag is set and no 'osThreadFlagsWait' errors
        if(~(0x8000000U & flags) && flags & DATA_READY_FLAG)
        {
            rawData = get_xyz_data();
            
            if (0xf0 & rawData.status)
            {
                // MMA8653FC internal overflow occured! Stop everything and wait for user reset.
                gpio_external_interrupt_disable();
                set_sensor_standby();
                sensor_reset();
                i2c_disable();
                
                //err("MMA8653FC internal overflow");
                PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
            }
            else if (!(MMA8653FC_STATUS_ZYXDR_MASK & rawData.status)) // if (ZYXDR bit == 0)
            {
                // Why has this thread been awakened if no new data???
                //err("Unknown MMA8653FC interrupt");
                PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
            }
            else
            {
                if (buf_filling == BUF_1)
                {
                    if (buf_index < NUM_ELEMENTS_IN_DATA_FRAME)
                    {
                        buf1_x[buf_index] = convert_to_count(rawData.out_x);
                        buf1_y[buf_index] = convert_to_count(rawData.out_y);
                        buf1_z[buf_index] = convert_to_count(rawData.out_z);
                        
                        buf_index++;
                        
                        if (buf_index == NUM_ELEMENTS_IN_DATA_FRAME) 
                        {
                            if (xyz_transmit_done)
                            {
                                buf_index = 0;
                                buf_filling = BUF_2;
                                buf_transmitting = BUF_1;
                                trigger_transmit = true;
                            }
                            else
                            {
                                // Data overflow imminent! Stop everything and wait for user reset.
                                transmit_late = true;
                                gpio_external_interrupt_disable();
                                set_sensor_standby();
                                sensor_reset();
                                i2c_disable();
                                //err("uC data buffer 1 overflow");
                                PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
                            }
                        }
                        else ; // Index still in bounds
                    }
                    else ; // Buf 1 overflow, should never get here
                }
                else if (buf_filling == BUF_2)
                {
                    if (buf_index < NUM_ELEMENTS_IN_DATA_FRAME)
                    {
                        buf2_x[buf_index] = convert_to_count(rawData.out_x);
                        buf2_y[buf_index] = convert_to_count(rawData.out_y);
                        buf2_z[buf_index] = convert_to_count(rawData.out_z);
                        
                        buf_index++;
                        
                        if (buf_index == NUM_ELEMENTS_IN_DATA_FRAME)
                        {
                            if (xyz_transmit_done)
                            {
                                buf_index = 0;
                                buf_filling = BUF_1;
                                buf_transmitting = BUF_2;
                                trigger_transmit = true;
                            }
                            else
                            {
                                // Data overflow imminent! Stop everything and wait for user reset.
                                transmit_late = true;
                                gpio_external_interrupt_disable();
                                set_sensor_standby();
                                sensor_reset();
                                i2c_disable();
                                //err("uC data buffer 2 overflow");
                                PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
                            }
                        }
                        else ; // Index still in bounds
                    }
                    else ; // Buf 2 overflow, should never get here
                }
                else ; // Erroneous buffer pointer, should never get here
                
                // Trigger data transfer to PC
                if (trigger_transmit)
                {
                    trigger_transmit = false;
                    xyz_transmit_done = false;
                    osThreadFlagsSet(transfer_thread_id, START_TRANSFER_FLAG);
                }
                else ; // Buffer not filled yet
            }
        
        }
        else if(flags == osFlagsErrorTimeout)
        {
            //err("Waited %u sec, no data", DATA_WAIT_TIME);
            PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
        }
        else 
        {
            //err("Thread flag error");
            PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
        }
    }
}

/**
 * @brief   This is a callback function for LDMA_IRQHandler().
 *          It checks whether the previous transfer was late
 *          and triggers another serial transfer.
 *
 */
static bool transfer_was_late ()
{
    if (transmit_late);
    else
    {
        osThreadFlagsSet(transfer_thread_id, START_TRANSFER_FLAG);
    }
    return transmit_late;
}

/**
 * @brief   Transfer a frame of data over serial. Uses LDMA. First a 
 *          token is sent to serial, then data from all three axis 
 *          is sent in x,y,z order.
 *          The osThreadFlagsWait function is triggered by both
 *          mma8653fc_data_acquisition_loop() and LDMA_IRQHandler().
 *
 */
static void data_transfer_loop (void *args)
{
    #define TRANSFER_WAIT_TIME  30 // Time to wait for data frame to become ready for transfer, seconds
    
    typedef enum 
    {
        DONE = 0,
        Z_AXIS = 1,
        Y_AXIS = 2,
        X_AXIS = 3,
        TOKEN = 4
    }transfer_order_t;
    
    static uint32_t flags;
    static transfer_order_t transfer_type_select = TOKEN;
    static const uint16_t token[] = {0xDEAD, 0xBEEF};
    static LDMA_Descriptor_t* p_descriptor;
    
    ldma_init();
    
    for (;;)
    {
        osThreadFlagsClear(START_TRANSFER_FLAG);
        flags = osThreadFlagsWait(START_TRANSFER_FLAG, osFlagsWaitAny, TRANSFER_WAIT_TIME*osKernelGetTickFreq()); // Flags are automatically cleared
        
        // The START_TRANSFER_FLAG flag is set and no 'osThreadFlagsWait' errors
        if(~(0x8000000U & flags) && flags & START_TRANSFER_FLAG)
        {
            if (BUF_1 == buf_transmitting)
            {
                switch (transfer_type_select)
                {
                    case X_AXIS :
                        p_descriptor = data_descriptor_config((uint32_t*)&buf1_x, NUM_ELEMENTS_IN_DATA_FRAME * 2);
                        transfer_type_select--;
                        break;
                        
                    case Y_AXIS :
                        p_descriptor = data_descriptor_config((uint32_t*)&buf1_y, NUM_ELEMENTS_IN_DATA_FRAME * 2);
                        transfer_type_select--;
                        break;
                        
                    case Z_AXIS :
                        p_descriptor = data_descriptor_config((uint32_t*)&buf1_z, NUM_ELEMENTS_IN_DATA_FRAME * 2);
                        transfer_type_select--;
                        break;
                        
                    case DONE :
                        xyz_transmit_done = true;
                        break;
                    
                    case TOKEN :
                        p_descriptor = token_descriptor_config((uint32_t*)token, sizeof(token));
                        transfer_type_select--;
                        break;
                    
                    default :
                        break;
                }
            }
            else if (BUF_2 == buf_transmitting)
            {
                switch (transfer_type_select)
                {
                    case X_AXIS :
                        p_descriptor = data_descriptor_config((uint32_t*)&buf2_x, NUM_ELEMENTS_IN_DATA_FRAME * 2);
                        transfer_type_select--;
                        break;
                        
                    case Y_AXIS :
                        p_descriptor = data_descriptor_config((uint32_t*)&buf2_y, NUM_ELEMENTS_IN_DATA_FRAME * 2);
                        transfer_type_select--;
                        break;
                        
                    case Z_AXIS :
                        p_descriptor = data_descriptor_config((uint32_t*)&buf2_z, NUM_ELEMENTS_IN_DATA_FRAME * 2);
                        transfer_type_select--;
                        break;
                        
                    case DONE :
                        xyz_transmit_done = true;
                        break;
                    
                    case TOKEN :
                        p_descriptor = token_descriptor_config((uint32_t*)token, sizeof(token));
                        transfer_type_select--;
                        break;
                    
                    default :
                        break;
                }
            }
            else ; // Unknown transmit buffer identifier.
            
            if(!xyz_transmit_done)ldma_uart_start(p_descriptor, transfer_was_late);
            else 
            {
                PLATFORM_LedsSet(PLATFORM_LedsGet() ^ 0x01);
                transfer_type_select = TOKEN;
            }
        }
        else if(flags == osFlagsErrorTimeout)
        {
            //err("Waited %u sec, no data", START_TRANSFER_FLAG);
            PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
        }
        else 
        {
            //err("Thread flag error");
            PLATFORM_LedsSet(PLATFORM_LedsGet() | 0x02);
        }
    }
}

// HB loop - increment and send counter
void hb_loop ()
{    
    for (;;)
    {
        osDelay(10*osKernelGetTickFreq()); // 10 sec
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
    osThreadNew(data_acquisition_loop, NULL, &recv_thread_attr);
    
    // Create thread to transfer collected data over serial to PC.
    const osThreadAttr_t data_transfer_thread_attr = { .name = "data_transfer_thread" };
    transfer_thread_id = osThreadNew(data_transfer_loop, NULL, &data_transfer_thread_attr);
    
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
