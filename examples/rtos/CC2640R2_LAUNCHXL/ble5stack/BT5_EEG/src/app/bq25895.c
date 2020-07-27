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

static I2C_Handle      i2c;
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
	
	i2c = I2C_open(Board_I2C0, &i2cParams);	
}

/*
 *  ========   BQ25895_Getdata   ========
 *	@ brief  I2C single register read
 *  @ param  slaveAddr - slave address
 *           regAddr - register address
 *  @ return data - reg data
 *
 */
uint8_t BQ25895_Getdata( uint8_t slaveAddr, uint8_t regAddr )
{
	I2C_Transaction i2cTransaction;
	uint8_t       txBuffer[3];  // Transmit buffer
	uint8_t       rxBuffer[1];  // Receive buffer
	
	txBuffer[0]= slaveAddr;
	txBuffer[1]= regAddr;
	txBuffer[2]= slaveAddr;
	
	// Initialize master I2C transaction structure
	i2cTransaction.writeCount   = 3;
	i2cTransaction.writeBuf     = txBuffer;
	i2cTransaction.readCount    = 1;
	i2cTransaction.readBuf      = rxBuffer;
	i2cTransaction.slaveAddress = slaveAddr;
	I2C_transfer(i2c, &i2cTransaction);
	
	return rxBuffer[0];
}

/*
 *  ========   BQ25895_SetParam    ========
 *	@ brief  I2C single register write
 *  @ param  slaveAddr - slave address
 *           regAddr - register address
 *           data - data to write
 *  @ return NULL
 *
 */
 
void BQ25895_SetParam ( uint8_t slaveAddr , uint8_t regAddr, uint8_t data)
{
	I2C_Transaction i2cTransaction;
	uint8_t       txBuffer[3];  // Transmit buffer
	uint8_t       rxBuffer[1];  // Receive buffer
	
	txBuffer[0]= slaveAddr;
	txBuffer[1]= regAddr;
	txBuffer[2]= data;
	
	// Initialize master I2C transaction structure
	i2cTransaction.writeCount   = 3;
	i2cTransaction.writeBuf     = txBuffer;
	i2cTransaction.readCount    = 0;
	i2cTransaction.readBuf      = NULL;
	i2cTransaction.slaveAddress = slaveAddr;
	I2C_transfer(i2c, &i2cTransaction);
	
	return rxBuffer[0];
}

/* reference :file:///C:/ti/simplelink_cc2640r2_sdk_4_20_00_04/docs/tidrivers/doxygen/html/_i2_c_8h.html#ac5d827b67fe77d7d179026941cc069d7 */