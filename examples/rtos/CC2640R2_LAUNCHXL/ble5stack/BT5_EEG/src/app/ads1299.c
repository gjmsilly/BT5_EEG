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

#include  <ads1299.h>
#include  <ti/drivers/SPI.h>
#include  <xdc/runtime/System.h>

uint8_t ResultByte = 0;
uint8_t ResultBuffer[27];
//uint8_t DummyByte;

/*
 *  ======== ADS1299 init ========
 *
 */
void ADS1299_init()
{

    /* Initialize the SPI1 module
     * Note: ads1299 use Motorola SPI Frame Format With SPO = 0 and SPH = 1 that is SPI_POL0_PHA1
     *       default value: SPI_master/blocking mode */
    SPI_init();

    SPI_Params      spiParams;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate  = 12000000; // 12MHz
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spi = SPI_open(Board_SPI1, &spiParams);
    if (spi == NULL)
    {
        while (1);  // SPI_open() failed
    }
}

/****************************************************************/
/* WaitUs()                 Modified                            */
/** Operation:
 *      -
 *
 * Parameters:
 *      -
 *
 * Return value:
 *     -
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     -
 *
 * Note:
 *     -
 */
/****************************************************************/
/*void WaitUs(int iWaitUs)
{
      int iPreTickVal = SysTickValueGet(); //当前值
      int iCounterTargetValue; //目标值
      int iCounterTargetInterval; //计数次数 即 延时时间

      iCounterTargetInterval = iWaitUs * ((SysCtrlClockGet()/1000000)-42);//-30;
      if(iPreTickVal < iCounterTargetInterval)
      {
            iCounterTargetValue =  iPreTickVal + SysTickPeriodGet() - iCounterTargetInterval;
        }
        else
        {
            iCounterTargetValue =  iPreTickVal - iCounterTargetInterval;
        }

        while((SysTickValueGet() > iCounterTargetValue));//||(SysTickValueGet()<iCounterTargetInterval));

}

 */

void WaitUs(int iWaitUs)
{
    CPUdelay(20*iWaitUs);
}

/****************************************************************/
/* ADS1299_Reset                                                  */
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
	WaitUs(40);
	Mod_RESET_H
	Mod_CS_Disable
	WaitUs(40);

}

/****************************************************************/
/* ADS1299_PowerOn                                                  */
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
	WaitUs(400);
}



/****************************************************************/
/* ADS1299_WriteREG()                                                  */
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
    uint8_t DRchar;
	address += 0x40;
	//__disable_irq();
	
	Mod_CS_Enable
	SSIDataPut(SSI1_BASE,address);            // command 1
    while(SSIBusy(SSI1_BASE));
    DRchar=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(40);
	SSIDataPut(SSI1_BASE,0);                  // command 2
    while(SSIBusy(SSI1_BASE));
    DRchar=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(40);
	SSIDataPut(SSI1_BASE,value);             // write value into register
    while(SSIBusy(SSI1_BASE));
    DRchar=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(40);
	Mod_CS_Disable

  //__enable_irq();
  
  
}

/****************************************************************/
/* ADS1299_ReadREG()                                                  */
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

	uint8_t DRchar;	
		
	//__disable_irq();
	
	Mod_CS_Enable
	SSIDataPut(SSI1_BASE,address);            // command 1
	while(SSIBusy(SSI1_BASE));
	DRchar=HWREG(SSI1_BASE + SSI_O_DR);
	WaitUs(40);
    SSIDataPut(SSI1_BASE,0);                  // command 2
    while(SSIBusy(SSI1_BASE));
    DRchar=HWREG(SSI1_BASE + SSI_O_DR);
    WaitUs(40);
	SSIDataPut(SSI1_BASE,0xaa);
    while(SSIBusy(SSI1_BASE));
    DRchar=HWREG(SSI1_BASE + SSI_O_DR);

	WaitUs(40);
	Mod_CS_Disable
  //__enable_irq();

  return DRchar;
}



/****************************************************************/
/* ADS1299_SendCommand()                                                  */
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
    uint8_t DRchar;
	//__disable_irq();
	Mod_CS_Enable
	WaitUs(40);
	SSIDataPut(SSI1_BASE,command);    // send command
    while(SSIBusy(SSI1_BASE));
    DRchar=HWREG(SSI1_BASE + SSI_O_DR);
	WaitUs(40);
	Mod_CS_Disable
  //__enable_irq();
	WaitUs(40);
}

/****************************************************************/
/* ADS1299_ReadByte()                                                  */
/** Operation:
 *      - Read ADS1299 output byte
 *
 * Parameters:
 *      - None
 *
 * Return result of byte:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
inline uint8_t ADS1299_ReadByte(void)
{
	ResultByte = 0;
	Mod_CS_Enable               // keep CS low
    SSIDataPut(SSI1_BASE,0x00); // keep MOSI low
    while(SSIBusy(SSI1_BASE));
    ResultByte=HWREG(SSI1_BASE + SSI_O_DR);
	WaitUs(8);
    //Mod_CS_Disable
	return ResultByte;
}

/****************************************************************/
/* ADS1299_ReadResult()                                         */
/** Operation:
 *      - Read ADS1299 output data
 *
 * Parameters:
 *      - None
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
/*void ADS1299_ReadResult(uint8_t *result)
//{
	uint8_t i;
    Mod_CS_Enable

	//DMA test
//	ADS1299_ReadResult_DMA((uint32_t)result, 27);
	
//	while(!LL_DMA_IsActiveFlag_TC3(DMA1));
	
	
//	LL_SPI_DisableDMAReq_TX(SPI2);
//	LL_SPI_DisableDMAReq_RX(SPI2);
	
//	LL_DMA_ClearFlag_TC3(DMA1);
//	LL_DMA_ClearFlag_TC4(DMA1);
	
	//printf("R%d %d %d ",ResultBuffer[3],ResultBuffer[4],ResultBuffer[5]);
	
	//WaitUs(4);
	//Mod_CS_Disable;
//}

*/

/****************************************************************/
/* ADS1299_Channel_Config()                                                  */
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
	ADS1299_WriteREG(0,ADS1299_REG_MISC1,MISC1.value);		// SRB1统一参考 0x20
	WaitUs(20);
	
	// configure all the channels
	for(i=0;i<8;i++)
	{
		ADS1299_Channel_Config(0,i,ChVal);
		WaitUs(20);
	}
		
}




