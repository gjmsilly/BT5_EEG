/**
 * @file    bq25895.c
 * @author  gjmsilly
 * @brief   bq25895 Firmware for CC2640R2F
 * @version 1.0
 * @date    2020-07-27
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
 
#include <bq25895.h>
#include <stdint.h>
#include <stdio.h>

/* Board Header files */
#include "Board.h"

/* Driver Header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/devices/cc26x0r2/driverlib/i2c.h>

static I2C_Params      i2cParams;

/*
 *  ======== BQ25895 init ========
 *
 */
void BQ25895_init( )
{
    I2C_init();

    // Configure I2C parameters to default value
    I2C_Params_init(&i2cParams); //  I2C_MODE_BLOCKING
    i2cParams.bitRate = I2C_400kHz;

    I2C_open(Board_I2C0, &i2cParams);
}

/*
 *  ========   BQ25895_Getdata   ========
 *  @ brief  I2C single register read
 *  @ param  slaveAddr - slave address
 *           regAddr - register address
 *  @ return data - reg data
 *
 */
uint8_t BQ25895_Getdata( uint8_t slaveAddr, uint8_t regAddr )
{

    uint8_t data;

    /* Step1 set slave address to send */
    I2CMasterSlaveAddrSet ( I2C0_BASE,                \
                            slaveAddr,                \
                            false);  // false - I2C write
    /* Step2 set the register address to send */
    I2CMasterDataPut(I2C0_BASE, regAddr);
    /* Step3 note! NO STOP command */
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);

    while ( I2CMasterBusy(I2C0_BASE) == true ); // wait util master free
    if ( I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE ) // no error
       {
        // start to read data from reg
        /* Step4 set slave address to send */
        I2CMasterSlaveAddrSet ( I2C0_BASE,                \
                                slaveAddr,                \
                                true);  // true - I2C read
        /* Step5 only read one byte , no ACK from master */
        I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
        while ( I2CMasterBusy(I2C0_BASE) == true ); // wait util master free
        if ( I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE ) // no error
            {
            /* Step6 get the reg data */
            data=I2CMasterDataGet(I2C0_BASE);
            }
        else
            /* finish the transfer due to error */
            I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
       }
    else
        /* finish the transfer due to error */
        I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);

    return data;

}

/*
 *  ========   BQ25895_SetParam    ========
 *  @ brief  I2C single register write
 *  @ param  slaveAddr - slave address
 *           regAddr - register address
 *           data - data to write
 *  @ return NULL
 *
 */
 
void BQ25895_SetParam ( uint8_t slaveAddr , uint8_t regAddr, uint8_t data)
{
    /* Step1 set slave address to send */
    I2CMasterSlaveAddrSet ( I2C0_BASE,                \
                            slaveAddr,                \
                            false);  // false - I2C write
    /* Step2 set the register address to send */
    I2CMasterDataPut(I2C0_BASE, regAddr);
    /* Step3 Master TRANSMIT With Repeated Start Condition */
    I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_START);
    /* Step4 check the state */
    while ( I2CMasterBusy(I2C0_BASE) == true ); // wait util master free
    if ( I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE ) // no error
    {
        /* Step5 set the data to send to the reg */
        I2CMasterDataPut(I2C0_BASE, data);
        /* Step6 finish the transfer */
        I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
    }
    else
        /* finish the transfer due to error */
        I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);

}

/* reference :file:///C:/ti/simplelink_cc2640r2_sdk_4_20_00_04/docs/tidrivers/doxygen/html/_i2_c_8h.html#ac5d827b67fe77d7d179026941cc069d7 */
