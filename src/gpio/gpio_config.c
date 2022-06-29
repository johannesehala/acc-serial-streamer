/**
 * @file gpio_config.c
 *
 * @brief Initialize GPIO and configure for I2C data transfer (SDA and SCL). 
 *        Configure GPIO pin for external interrupts.
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2022
 */

#include "em_cmu.h"
#include "em_gpio.h"
#include "cmsis_os2.h"

#include "gpio_config.h"

#include "loglevels.h"
#define __MODUUL__ "gpio"
#define __LOG_LEVEL__ (LOG_LEVEL_gpio & BASE_LOG_LEVEL)
#include "log.h"

volatile static osThreadId_t resumeThreadID;
volatile static uint32_t resumeThreadFlagID;

/**
 * @brief Initialize GPIO interface and configure for I2C communication. 
 *
 * Accelerometer sensor is connected to port A pin 2 (SCL) and pin 3 (SDA) on TTTW lab-kit.
 */
void gpio_i2c_pin_init (void)
{
    // Enable GPIO peripheral
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Take control of SDC and SCL output pins.
	GPIO_PinModeSet(MMA8653FC_SCL_PORT, MMA8653FC_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(MMA8653FC_SDA_PORT, MMA8653FC_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
}

/**
 * @brief Initialize GPIO interface and configure for external interrupts. 
 *
 */
void gpio_external_interrupt_init (void)
{
    // Enable GPIO peripheral
    CMU_ClockEnable(cmuClock_GPIO, true);

    // Configure pin
    GPIO_PinModeSet(MMA8653FC_INT_PORT, MMA8653FC_INT_PIN, gpioModeInputPullFilter, 1);
    
    // Configure external interrupts
    GPIO_IntDisable(GPIO_IF_EXTI_NUM);
    GPIO_ExtIntConfig(MMA8653FC_INT_PORT, MMA8653FC_INT_PIN, GPIO_EXTI_NUM, false, true, false); // Port, pin, EXTIx, rising edge, falling edge, enabled
    GPIO_InputSenseSet(GPIO_INSENSE_INT, GPIO_INSENSE_INT);
}

void gpio_external_interrupt_disable ()
{
    GPIO_IntDisable(GPIO_IF_EXTI_NUM);
    NVIC_DisableIRQ(GPIO_ODD_IRQn);
}

/**
 * @brief Enable exteranl interrupts on GPIO pins.
 *
 * @param   tID, ID of the thread that handles the interrupt (deferred interrupt handling)
 * @param   tFlag, flag id for thread tID
 * 
 * @note    The EXTIx number (index) determines the external interrupt to use. GPIO ports and pins
 *          are wired to the interrupt controller (NVIC) based on the EXTI number (index). Only
 *          certain ports and pins are available for each external interrupt. Check EFR32MG12 manual
 *          p1114 and GPIO application note p11-12 for more information.
 *          EXTIx number also determines which IRQHandler function to use - GPIO_ODD_IRQHandler or
 *          GPIO_EVEN_IRQHandler.
 *
 *          EXTIx number and interrupt flag number are related. If EXTIx number is zero, then 
 *          interrupt flag is 0x01. If EXTIx number is 3, then interrupt flag is 0x08.
 */
void gpio_external_interrupt_enable (osThreadId_t tID, uint32_t tFlag)
{
    resumeThreadID = tID;
    resumeThreadFlagID = tFlag;
    
    GPIO_IntClear(GPIO_IF_EXTI_NUM);
    
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_SetPriority(GPIO_ODD_IRQn, 3);

    GPIO_IntEnable(GPIO_IF_EXTI_NUM);
}

void GPIO_ODD_IRQHandler (void)
{
    // Get all pending and enabled interrupts.
    uint32_t pending = GPIO_IntGetEnabled();

    if (pending & GPIO_IF_EXTI_NUM)
    {
        // Clear interrupt flag.
        GPIO_IntClear(GPIO_IF_EXTI_NUM);

        osThreadFlagsSet(resumeThreadID, resumeThreadFlagID);
    }
}
