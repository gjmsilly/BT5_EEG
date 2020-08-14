/**
 * @file    ads1299.c
 * @author  gjmsilly
 * @brief   ads1299 Firmware for CC2640R2F
 * @version 1.0
 * @date    2020-08-01
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/SPI.h>
#include <driverlib/ssi.h>
#include <driverlib/ioc.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/udma.h>

#include <ads1299.h>

/*********************************************************************
 * GLOBAL VARIABLES
 */

/* Buffer for EEG data */
uint8_t                            Buffer1[BufferSize];
uint8_t                            Buffer2[BufferSize];

/* The control table used by the uDMA controller, primary and alternate for 31 channels */
tDMAControlTable                   DMAControlTable[64] __attribute__ ((aligned (1024)));

/*********************************************************************
 * LOCAL VARIABLES
 */
/* variables for SPI  */
uint8_t         DummyByte = 0x00;

/*********************************************************************
 * @fn      ADS1299_init
 *
 * @brief   initial the SPI module and DReady interrupts
 */
void ADS1299_init()
{

    // ******************************************************************
    // Hardware initialization
    // ******************************************************************
    unsigned int    key;

    /* Disable interrupts when transfer */
    key = HwiP_disable();

    /* power up and enable clock for SPI. */
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI1);             // SSI1
    PRCMPeripheralSleepEnable(PRCM_PERIPH_SSI1);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_SSI1);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    HwiP_restore(key);

    /* Configure IOs for SSI1 */
    IOCPinTypeSsiMaster( SSI1_BASE,                                 \
                         CC2640R2F_EEG_SPI1_MISO,                   \
                         CC2640R2F_EEG_SPI1_MOSI,                   \
                         CC2640R2F_EEG_SPI1_CSN,                    \
                         CC2640R2F_EEG_SPI1_CLK);

    /* Configure SSI1 */
    //disable operation of the SSI to make sure the CR1:SSE is cleared
    SSIDisable(SSI1_BASE);

    // Motorola SPI Frame Format With SPO = 0 and SPH = 1 that is SSI_FRF_MOTO_MODE_1
    // core clock is 48MHz
    // bit rate set equation is core clock/((SCR+1)*CPSR.CPSDVSR)) SCR[0-255] CPSR EVEN NUNBER
    // data size set to 8 (4-16 is okay)

    SSIConfigSetExpClk(SSI1_BASE,            \
                       SysCtrlClockGet(),    \
                       SSI_FRF_MOTO_MODE_1,  \
                       SSI_MODE_MASTER,      \
                       12000000,             \
                       8);

    SSIIntDisable(SSI1_BASE,SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR);

    /* Configure DMA driver */
    SPIDMA_init();

    // enable the SPI module
    SSIEnable(SSI1_BASE);

    /* Initial ads1299 */
    ADS1299_PowerOn(0);
    ADS1299_Reset(0);
    ADS1299_SendCommand(ADS1299_CMD_SDATAC); // Stop Read Data Continuously mode
    ADS1299_Parameter_Config(ADS1299_ParaGroup_TSIG,1); // test signal mode , sample rate 250Hz ,gain x2
}

/*********************************************************************
 * @fn      WaitUs
 *
 * @brief   cpu delay in us
 *
 */

void WaitUs(int iWaitUs)
{
    // delay 1us means 1*24/7 `= 6
    CPUdelay(6*iWaitUs);
}

/*********************************************************************
 * @fn      WaitMs
 *
 * @brief   cpu delay in ms
 *
 */
void WaitMs(int iWaitMs)
{
    // delay 1ms means 1000*24/4 `= 4000
    CPUdelay(8000*iWaitMs);
}

/****************************************************************/
/* ADS1299_Reset                                                */
/** Operation:
 *      - Reset ADS1299 chip
 *
 * Parameters:
 *      -  dev:ADS1299 chip number
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Reset(uint8_t dev)
{
    Mod_RESET_L
    WaitUs(40); // at least 2 tCLK
    Mod_RESET_H
	Mod_CS_Disable
	WaitUs(40); // at least 18 tCLK

}

/****************************************************************/
/* ADS1299_PowerOn                                             */
/** Operation:
 *      - PowerOn ADS1299 chip
 *
 * Parameters:
 *      -  dev:ADS1299 chip number
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_PowerOn(uint8_t dev)
{
	Mod_PDWN_H
    Mod_RESET_H
	WaitMs(400);    // wait for at least tPOR = 128ms
}



/****************************************************************/
/* ADS1299_WriteREG()                                          */
/** Operation:
 *      - Configuring the ADS1299 register
 *
 * Parameters:
 *      - dev:ADS1299 chip number
 *      - address:Destination register address
 *      - value:The value of destination register
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_WriteREG (uint8_t dev, uint8_t address, uint8_t value)
{
    address += 0x40;
    uint8_t         transmitBuffer[3] = {address,0x00,value}; // address / reg number-1 =0x00 / value
    uint8_t         receiveBuffer;
    unsigned int    key;

    /* Disable interrupts when transfer */
    key = HwiP_disable();

    Mod_CS_Enable

    SSIDataPut(SSI1_BASE,transmitBuffer[0]);              // command 1
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(10);
    SSIDataPut(SSI1_BASE,transmitBuffer[1]);              // command 2
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(10);
    SSIDataPut(SSI1_BASE,transmitBuffer[2]);             // write value into register
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(10);
    Mod_CS_Disable

    /* Re-enable interrupts */
    HwiP_restore(key);

  
}

/****************************************************************/
/* ADS1299_ReadREG()                                            */
/** Operation:
 *      - Configuring the ADS1299 register
 *
 * Parameters:
 *      - dev:ADS1299 chip number
 *      - address:Destination register address
 *      - value:The value of destination register
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
uint8_t ADS1299_ReadREG (uint8_t dev, uint8_t address)
{
	address += 0x20;
    uint8_t         transmitBuffer[3] = {address,0x00,DummyByte}; // address / reg number-1 =0x00 / DummyByte=0xaa
    uint8_t         receiveBuffer;
    unsigned int    key;

    /* Disable interrupts when transfer*/
    key = HwiP_disable();

    Mod_CS_Enable

    SSIDataPut(SSI1_BASE,transmitBuffer[0]);                  // command 1
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(10);
    SSIDataPut(SSI1_BASE,transmitBuffer[1]);                  // command 2
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(10);
    SSIDataPut(SSI1_BASE,transmitBuffer[2]);
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);

    Mod_CS_Disable

	/* Re-enable interrupts when transfer*/
    HwiP_restore(key);

  return receiveBuffer;
}



/****************************************************************/
/* ADS1299_SendCommand()                                        */
/** Operation:
 *      - Send command to the ADS1299 chip
 *
 * Parameters:
 *      - dev:ADS1299 chip number
 *      - command:command to the ADS1299 chip
 *
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_SendCommand(uint8_t command)
{
    uint8_t         transmitBuffer = command;
    uint8_t         receiveBuffer;
    unsigned int    key;

    /* Disable interrupts */
    key = HwiP_disable();

	Mod_CS_Enable

	WaitUs(10);
    SSIDataPut(SSI1_BASE,transmitBuffer);    // send command
    while(SSIBusy(SSI1_BASE));
    receiveBuffer=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(10);

	Mod_CS_Disable

	/* Re-enable interrupts */
    HwiP_restore(key);

}

/****************************************************************/
/* ADS1299_Channel_Config()                                     */
/** Operation:
 *      - Configuring ADS1299 parameters
 *
 * Parameters:
 *      - dev: ADS1299 chip number
 *      - channel : ADS1299 channel number
 *      - Para : CHnSET value
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Channel_Config(uint8_t dev, uint8_t channel, TADS1299CHnSET Para)
{
	ADS1299_WriteREG (0, (ADS1299_REG_CH1SET + channel), Para.value );
}

/****************************************************************/
/* ADS1299_Parameter_Config()                                  */
/** Operation:
 *      - Configuring ADS1299 parameters
 *
 * Parameters:
 *      - ADS1299_ParaGroup : predefined Para Mode
 *      - sample:The sampling rate of ADS1299
 *               1 - 250Hz
 *               2 - 500Hz
 *               3 - 1000Hz
 *
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Parameter_Config(uint8_t ADS1299_ParaGroup, uint8_t sample)
{
	uint8_t i = 0;
	TADS1299CHnSET     ChVal;
	TADS1299CONFIG1    CFG1;
	TADS1299CONFIG2    CFG2;
	TADS1299CONFIG3    CFG3;
	TADS1299CONFIG4    CFG4;
	TADS1299LOFF       LOFF;
	TADS1299BIASSENSP  BIASSP;
	TADS1299BIASSENSN  BIASSN;
	TADS1299MISC1      MISC1;

	
	switch (ADS1299_ParaGroup)
  {
  	case ADS1299_ParaGroup_ACQ:
  	{
			CFG2.value = 0xC0;
			
			CFG3.value = 0x60;
			CFG3.control_bit.pdbrefbuf = 1;
			CFG3.control_bit.pdbbias = 1;
			CFG3.control_bit.biasrefint = 1;
			
			CFG4.value = 0x00;
			
			LOFF.value = 0x00;
			
			
			ChVal.control_bit.gain = 6;
			ChVal.control_bit.pd = 0;
			ChVal.control_bit.mux = 0;
			
			
			BIASSP.value = 0xFF;    // route the INnP to the BIAS buffer
			MISC1.control_bit.srb1 = 1;
			
			break;
		}
  	case ADS1299_ParaGroup_IMP:
  		break;
	case ADS1299_ParaGroup_STBY:
  		break;
	case ADS1299_ParaGroup_TSIG:
  	{
			
			CFG2.value = 0xC0;              // default value
			CFG2.control_bit.inttest = 1;   // test source
			CFG2.control_bit.testamp = 0;
			CFG2.control_bit.testfreq = 1;
			
			CFG3.value = 0x60;              // default value
			CFG3.control_bit.pdbrefbuf = 1; // enable internal reference buffer
			CFG3.control_bit.pdbbias = 1;   // BIAS buffer enabled
			CFG3.control_bit.biasrefint = 1; // BIAS signal 2.5V
			
			CFG4.value = 0x00;
			
			LOFF.value = 0x00;
			

			ChVal.control_bit.gain = 1;     // x2
			ChVal.control_bit.pd = 0;
			ChVal.control_bit.mux = 5;      // tset signal
			
			BIASSP.value = 0x00;            // route the INnP to the BIAS buffer
			BIASSN.value = 0xFF;

			MISC1.value=0x00;
			MISC1.control_bit.srb1 = 1;
			
			break;
		}
  	default:
  		break;
  }
	
	
	// configure the sample rate
	switch(sample)
	{
		case 1:		//250Hz
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0xF6);		// Sample rate 250Hz
			break;
		}
		case 2:		//500Hz
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0xF5);		// Sample rate 500Hz
			break;
		}
		case 3:		//1000Hz
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0xF4);		// Sample rate 1000Hz
			break;
		}
	}

	WaitUs(20);
	
	ADS1299_WriteREG(0,ADS1299_REG_CONFIG2,CFG2.value); //0xd1
	WaitUs(20);
    ADS1299_WriteREG(0,ADS1299_REG_CONFIG3,CFG3.value); //0xe6
    WaitUs(20);
    ADS1299_WriteREG(0,ADS1299_REG_CONFIG4,CFG4.value);
    WaitUs(20);

	// configure the BIAS for INnP and INnN
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,BIASSP.value);
	WaitUs(20);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,BIASSN.value);
	WaitUs(20);
	ADS1299_WriteREG(0,ADS1299_REG_MISC1,MISC1.value);		// SRB1ͳһ�ο� 0x20
	WaitUs(20);
	
	// configure all the channels
	for(i=0;i<8;i++)
	{
		ADS1299_Channel_Config(0,i,ChVal);
		WaitUs(20);
	}
		
}

/*********************************************************************
 * @fn      SPIDMA_Init
 *
 * @brief   initial the DMA module for ads1299
 */
void SPIDMA_init()
{

    unsigned int    key;

    /* Disable interrupts when transfer*/
    key = HwiP_disable();

    /* power up and enable clock for DMA. */
    PRCMPeripheralRunEnable(PRCM_PERIPH_UDMA);             // UDMA
    PRCMPeripheralSleepEnable(PRCM_PERIPH_UDMA);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_UDMA);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    /* Set the base for the channel control table. */
    uDMAControlBaseSet  (UDMA0_BASE, &DMAControlTable);
    /* Enable DMA. */
    uDMAEnable(UDMA0_BASE);

    /* DMA settings for SPI RX */
    // Put the attributes in a known state for the uDMA SSI1RX channel.  Disable the attr by default
    uDMAChannelAttributeDisable(UDMA0_BASE,UDMA_CHAN_SSI1_RX,                       \
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |          \
                                UDMA_ATTR_HIGH_PRIORITY |                           \
                                UDMA_ATTR_REQMASK);

    uDMAChannelAttributeEnable(UDMA0_BASE,UDMA_CHAN_SSI1_RX,UDMA_ATTR_USEBURST);

    // Configure the control parameters for the primary & alternate control structure for SSI1RX

    /* source address - increment is 8-bit
     * destination address - increment is 8-bit
     * arbitration size - 4 // fixed at 4
     * transfer data size - 8 bit
     */
    uDMAChannelControlSet   ( UDMA0_BASE,                                            \
                              UDMA_CHAN_SSI1_RX | UDMA_PRI_SELECT,                   \
                              UDMA_SIZE_8  | UDMA_SRC_INC_NONE   | UDMA_DST_INC_8|   \
                              UDMA_ARB_4 | UDMA_NEXT_USEBURST);

    /* DMA settings for SPI TX */
    // Put the attributes in a known state for the uDMA SSI1RX TX channel.  Disable the attr by default
    uDMAChannelAttributeDisable(UDMA0_BASE,UDMA_CHAN_SSI1_TX,                       \
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |          \
                                UDMA_ATTR_HIGH_PRIORITY |                           \
                                UDMA_ATTR_REQMASK);

    uDMAChannelAttributeEnable(UDMA0_BASE,UDMA_CHAN_SSI1_TX,UDMA_ATTR_USEBURST);

    // Configure the control parameters for the primary & alternate control structure for SSI1TX

    /* source address - no increment
     * destination address - increment is 8-bit
     * arbitration size - 4  // fixed at 4
     * transfer data size - 8 bit
     */
    uDMAChannelControlSet   ( UDMA0_BASE,                                              \
                              UDMA_CHAN_SSI1_TX | UDMA_PRI_SELECT,                     \
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE|     \
                              UDMA_ARB_4  | UDMA_NEXT_USEBURST);

    /* Re-enable interrupts */
    HwiP_restore(key);

    /* enable the spi module for dma */
    SSIDisable(SSI1_BASE); //disable operation of the SSI to make sure the CR1:SSE is cleared
    SSIDMAEnable(SSI1_BASE,SSI_DMA_TX | SSI_DMA_RX);

}

/*********************************************************************
 * @fn      SPIDMA_transfer
 *
 * @brief   EEG data transfer by dma
 */
void SPIDMA_transfer(uint8_t* BufferNum)
{
//    uint32_t status;
//    status = uDMAIntStatus(UDMA0_BASE);
//    uDMAIntClear(UDMA0_BASE,status);

    if(!(uDMAChannelIsEnabled (UDMA0_BASE,UDMA_CHAN_SSI1_TX )))
    {
    // Set up the transfer parameters for the uDMA SSI1TX channel
    uDMAChannelTransferSet  ( UDMA0_BASE,                                       \
                              UDMA_CHAN_SSI1_TX | UDMA_PRI_SELECT,              \
                              UDMA_MODE_BASIC,                                  \
                              &DummyByte,                                       \
                              (void *)(SSI1_BASE + SSI_O_DR),                   \
                              28); // 28 Byte for one EEG conversion


    uDMAChannelEnable( UDMA0_BASE,UDMA_CHAN_SSI1_TX );
    }

   if(!(uDMAChannelIsEnabled (UDMA0_BASE,UDMA_CHAN_SSI1_RX )))
    {
    // Set up the transfer parameters for the uDMA SSI1RX channel
    uDMAChannelTransferSet  ( UDMA0_BASE,                                       \
                              UDMA_CHAN_SSI1_RX | UDMA_PRI_SELECT,              \
                              UDMA_MODE_BASIC,                                  \
                              (void *)(SSI1_BASE + SSI_O_DR),                   \
                              BufferNum,                                        \
                              28);

    uDMAChannelEnable( UDMA0_BASE,UDMA_CHAN_SSI1_RX );
    }

}
