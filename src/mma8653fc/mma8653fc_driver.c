/**
 * @file mma8653fc_driver.c
 *
 * @note    I2C set-up must be done separately and before the usage of this driver.
 *          GPIO interrupt set-up must be done separately if MMA8653FC interrupts are used.
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#include "cmsis_os2.h" // For osDelay() in sensor_reset() function.
#include "mma8653fc_reg.h"
#include "mma8653fc_driver.h"
#include "em_i2c.h"
#include "i2c_config.h"

#include "loglevels.h"
#define __MODUUL__ "sdrv"
#define __LOG_LEVEL__ (LOG_LEVEL_mmadrv & BASE_LOG_LEVEL)
#include "log.h"

static uint8_t read_registry(uint8_t regAddr);
static void read_multiple_registries(uint8_t startRegAddr, uint8_t *rxBuf, uint16_t rxBufLen);
static void write_registry(uint8_t regAddr, uint8_t regVal);

uint8_t read_whoami(uint8_t reg)
{
    return read_registry(reg);
}

/**
 * @brief   Reset MMA8653FC sensor (software reset).
 */
void sensor_reset (void)
{
    uint8_t regVal;
    
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG2);
    regVal = (regVal & ~MMA8653FC_CTRL_REG2_SOFTRST_MASK) | (MMA8653FC_CTRL_REG2_SOFTRST_EN << MMA8653FC_CTRL_REG2_SOFTRST_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG2, regVal);
    osDelay(5*osKernelGetTickFreq()/1000); // Wait a little for reset to finish.
}

/**
 * @brief   Sets sensor to active mode. 
 */
void set_sensor_active ()
{
    uint8_t regVal;
    
    // Change mode to ACTIVE
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG1);
    regVal = (regVal & ~MMA8653FC_CTRL_REG1_SAMODE_MASK) | (MMA8653FC_CTRL_REG1_SAMODE_ACTIVE << MMA8653FC_CTRL_REG1_SAMODE_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG1, regVal);
}

/**
 * @brief   Sets sensor to standby mode. Sensor must be in standby mode when writing to
 *          different config registries.
 */
void set_sensor_standby ()
{
    uint8_t regVal;
    
    // Change mode to STANDBY
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG1);
    regVal = (regVal & ~MMA8653FC_CTRL_REG1_SAMODE_MASK) | (MMA8653FC_CTRL_REG1_SAMODE_STANDBY << MMA8653FC_CTRL_REG1_SAMODE_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG1, regVal);
}

/**
 * @brief   Configures MMA8653FC sensor to start collecting xyz acceleration data.
 *
 * @param   dataRate Set data rate (1.56, 6.26, 12, 50, 100, 200, 400, 800 Hz)
 * @param   range Set dynamic range (+- 2g, +- 4g, +- 8g)
 * @param   powerMod Set power mode (normal, low-noise-low-power, highres, low-power)
 * 
 * @return  -1 if sensor is not in standby mode
 *           0 if configuration succeeded (no check)
 */
int8_t configure_xyz_data (uint8_t dataRate, uint8_t range, uint8_t powerMod)
{
    uint8_t regVal;
    
    // Check if sensor is in standby mode, control registers can only be modified in standby mode.
    regVal = read_registry(MMA8653FC_REGADDR_SYSMOD);
    if (MMA8653FC_SYSMOD_MOD_STANDBY != (regVal & MMA8653FC_SYSMOD_MOD_MASK))return -1;
    
    // Set data rate.
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG1);
    regVal = (regVal & ~MMA8653FC_CTRL_REG1_DATA_RATE_MASK) | (dataRate << MMA8653FC_CTRL_REG1_DATA_RATE_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG1, regVal);
    
    // Set dynamic range.
    regVal = read_registry(MMA8653FC_REGADDR_XYZ_DATA_CFG);
    regVal = (regVal & ~MMA8653FC_XYZ_DATA_CFG_RANGE_MASK) | (range << MMA8653FC_XYZ_DATA_CFG_RANGE_SHIFT);
    write_registry(MMA8653FC_REGADDR_XYZ_DATA_CFG, regVal);
    
    // Set power mode (oversampling). 
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG2);
    regVal = (regVal & ~MMA8653FC_CTRL_REG2_ACTIVEPOW_MASK) | (powerMod << MMA8653FC_CTRL_REG2_ACTIVEPOW_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG2, regVal);
    
    return 0;
}

/**
 * @brief   Configures MMA8653FC sensor to start collecting xyz acceleration data.
 *
 * @param   polarity Set interrupt pin polarity.
 * @param   pinmode Set interrupt pin pinmode.
 * @param   interrupt Set interrupts to use.
 * @param   int_select Route interrupts to selected pin.
 *
 * @return  -1 if sensor is not in standby mode
 *           0 if configuration succeeded (no check)
 */
int8_t configure_interrupt (uint8_t polarity, uint8_t pinmode, uint8_t interrupt, uint8_t int_select)
{
    uint8_t regVal;
    
    // Check if sensor is in standby mode, control registers can only be modified in standby mode.
    regVal = read_registry(MMA8653FC_REGADDR_SYSMOD);
    if (MMA8653FC_SYSMOD_MOD_STANDBY != (regVal & MMA8653FC_SYSMOD_MOD_MASK))return -1;
    
    // Configure interrupt pin pinmode and interrupt transition direction
    regVal = (polarity << MMA8653FC_CTRL_REG3_POLARITY_SHIFT) | (pinmode << MMA8653FC_CTRL_REG3_PINMODE_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG3, regVal);

    // Enable data ready interrupt
    regVal = interrupt;
    write_registry(MMA8653FC_REGADDR_CTRL_REG4, regVal);

    // Route data ready interrupt to sensor INT1 output pin (connected to port PA1 on the TTTW uC)
    regVal = int_select;
    write_registry(MMA8653FC_REGADDR_CTRL_REG5, regVal);

    return 0;
}

/**
 * @brief   Reads MMA8653FC STATUS and data registries.
 *
 * @return  Returns value of STATUS registry and x, y, z, 10 bit raw values (left-justified 2's complement)
 */
xyz_rawdata_t get_xyz_data()
{
    #define NUM_STATUS_AND_DATA_REG 7
    
    uint8_t dataBuf[NUM_STATUS_AND_DATA_REG];
    xyz_rawdata_t data;
    read_multiple_registries(MMA8653FC_REGADDR_STATUS, dataBuf, NUM_STATUS_AND_DATA_REG);
        
    data.status = dataBuf[0];
    
    // Shift result MSB and LSB bytes into 16-bit unsigned data type
    data.out_x = dataBuf[1] << 8 | (0x0000 | dataBuf[2]);
    data.out_y = dataBuf[3] << 8 | (0x0000 | dataBuf[4]);
    data.out_z = dataBuf[5] << 8 | (0x0000 | dataBuf[6]);
    
    return data;
}

/**
 * @brief   Read value of one registry of MMA8653FC.
 *
 * @param   regAddr Address of registry to read.
 *
 * @return  value of registry with address regAddr.
 */
static uint8_t read_registry(uint8_t regAddr)
{
    #define READREG_TXBUF_LEN     1
    #define READREG_RXBUF_LEN     1
    
    static I2C_TransferSeq_TypeDef readReg, *retSeq;
    static uint8_t txBuf[READREG_TXBUF_LEN], rxBuf[READREG_RXBUF_LEN];

    readReg.addr = MMA8653FC_SLAVE_ADDRESS_READ;
        
    txBuf[0] = regAddr;
    readReg.buf[0].data = txBuf;
    readReg.buf[0].len = READREG_TXBUF_LEN;
    
    rxBuf[0] = 0;
    readReg.buf[1].data = rxBuf;
    readReg.buf[1].len = READREG_RXBUF_LEN;

    readReg.flags = I2C_FLAG_WRITE_READ;
    
    retSeq = i2c_transaction(&readReg);
    
    debug1("RReg 0x%02x, val 0x%02x", retSeq->buf[0].data[0], retSeq->buf[1].data[0]);
    
    return retSeq->buf[1].data[0];
}

/**
 * @brief   Write a value to one registry of MMA8653FC.
 *
 * @param   regAddr Address of registry to read.
 * @param   regVal Value to write to MMA8653FC registry.
 *
 * @note    rxBuf is not used I think, maybe replace with NULL pointer.
 */
static void write_registry(uint8_t regAddr, uint8_t regVal)
{
    #define WRITEREG_TXBUF_LEN     2
    #define WRITEREG_RXBUF_LEN     1
    
    static I2C_TransferSeq_TypeDef writeReg, *retSeq;
    static uint8_t txBuf[WRITEREG_TXBUF_LEN], rxBuf[WRITEREG_RXBUF_LEN];

    writeReg.addr = MMA8653FC_SLAVE_ADDRESS_WRITE;
        
    txBuf[0] = regAddr;
    txBuf[1] = regVal;
    writeReg.buf[0].data = txBuf;
    writeReg.buf[0].len = WRITEREG_TXBUF_LEN;
    
    rxBuf[0] = 0;
    writeReg.buf[1].data = rxBuf;
    writeReg.buf[1].len = WRITEREG_RXBUF_LEN;

    writeReg.flags = I2C_FLAG_WRITE_WRITE;
    
    retSeq = i2c_transaction(&writeReg);
    
    debug1("WReg 0x%02x, val 0x%02x", retSeq->buf[0].data[0], retSeq->buf[0].data[1]);
    
    return ;
}

/**
 * @brief   Read multiple registries of MMA8653FC in one go.
 * @note    MMA8653FC increments registry pointer to read internally according to its own logic. 
 *          Registries next to each other are not necessarily read in succession. Check MMA8653FC
 *          datasheet to see how registry pointer is incremented.
 *
 * @param   startRegAddr Address of registry to start reading from.
 * @param   *rxBuf Pointer to memory area where read values are stored.
 * @param   rxBufLen Length/size of rxBuf memory area.
 */
static void read_multiple_registries(uint8_t startRegAddr, uint8_t *rxBuf, uint16_t rxBufLen)
{
    #define RMULTI_TX_BUF_LEN     1
    
    static I2C_TransferSeq_TypeDef rMultiReg, *retSeq;
    static uint8_t txBuf[RMULTI_TX_BUF_LEN], i;
    
    txBuf[0] = startRegAddr;
    for (i = 0; i < rxBufLen; i++)rxBuf[i] = 0;
    
    rMultiReg.addr = MMA8653FC_SLAVE_ADDRESS_READ;
    rMultiReg.buf[0].data = txBuf;
    rMultiReg.buf[0].len = RMULTI_TX_BUF_LEN;
    rMultiReg.buf[1].data = rxBuf;
    rMultiReg.buf[1].len = rxBufLen;
    rMultiReg.flags = I2C_FLAG_WRITE_READ;
    
    retSeq = i2c_transaction(&rMultiReg);
    
    debug1("MRReg 0x%02x, val 0x%02x", retSeq->buf[0].data[0], retSeq->buf[1].data[0]);
    
    return ;
}

/**
 * @brief   Converts MMA8653FC sensor output value (left-justified 10-bit 2's complement
 *          number) to decimal number representing MMA8653FC internal ADC read-out 
 *          (including bias).
 *          
 * @param raw_val   is expected to be left-justified 10-bit 2's complement number
 *
 * @return          decimal number ranging between -512 ... 511
 */
int16_t convert_to_count(uint16_t raw_val)
{
    #define MSB_SIGN_MASK                   0x8000UL
    #define RIGHT_SHIFT_COUNT               6

    if (raw_val & MSB_SIGN_MASK)
    {
        // Negative number, flip bits, shift right by 6 bits, add one, include minus sign.
        return (((uint16_t)(~raw_val) >> RIGHT_SHIFT_COUNT) + 1) * -1;
    }
    else
    {
        // Positive number, shift right by 6 bits
        return ((uint16_t)raw_val >> RIGHT_SHIFT_COUNT);
    }
}

/**
 * @brief   Converts MMA8653FC sensor output value (left-justified 10-bit 2's complement
 *          number) to floating point number representing acceleration rate in g.
 *
 * @param raw_val       is expected to be left-justified 10-bit 2's complement number
 * @param sensor_scale  sensor scale 2g, 4g or 8g
 *
 * @return          floating point number, value depending on chosen sensor range 
 *                  +/- 2g  ->  range -2 ... 1.996
 *                  +/- 4g  ->  range -4 ... 3.992
 *                  +/- 8g  ->  range -8 ... 7.984
 */
float convert_to_g(uint16_t raw_val, uint8_t sensor_scale)
{
    // Datasheet page 11
    #define DYNAMIC_RANGE_2G_SENSITIVITY    256
    #define DYNAMIC_RANGE_4G_SENSITIVITY    128
    #define DYNAMIC_RANGE_8G_SENSITIVITY    64
    
    #define MSB_SIGN_MASK                   0x8000UL
    #define RIGHT_SHIFT_COUNT               6
    
    int16_t count, sensitivity_range;
    /*
    if (raw_val & MSB_SIGN_MASK)
    {
        // Negative number, flip bits, shift right by 6 bits, add one, include minus sign.
        count = (-1 * ((uint16_t)(~raw_val) >> RIGHT_SHIFT_COUNT) + 1);
    }
    else
    {
        // Positive number, shift right by 6 bits
        count = ((uint16_t)raw_val >> RIGHT_SHIFT_COUNT);
    }
    */
    
    count = convert_to_count(raw_val);
    
    // Determine sensor sensitivity
    if (MMA8653FC_XYZ_DATA_CFG_2G_RANGE == sensor_scale)sensitivity_range = DYNAMIC_RANGE_2G_SENSITIVITY;
    if (MMA8653FC_XYZ_DATA_CFG_4G_RANGE == sensor_scale)sensitivity_range = DYNAMIC_RANGE_4G_SENSITIVITY;
    if (MMA8653FC_XYZ_DATA_CFG_8G_RANGE == sensor_scale)sensitivity_range = DYNAMIC_RANGE_8G_SENSITIVITY;
    
    return (float) count / sensitivity_range;
}
