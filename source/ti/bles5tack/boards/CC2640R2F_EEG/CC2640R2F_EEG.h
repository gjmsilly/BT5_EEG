/**
 * @file    CC2640R2F_EEG.h
 * @author  gjmsilly
 * @brief   CC2640R2F_EEG v2 configuration
 * @version 0.1
 * @date    2020-07-22
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
 
#ifndef __CC2640R2F_EEG_BOARD_H__
#define __CC2640R2F_EEG_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#ifndef CC2640R2F_EEG
  #define CC2640R2F_EEG
#endif /* CC2640R2F_EEG */
/*
 *  ============================================================================
 *  RF Front End and Bias configuration symbols for TI reference designs and
 *  kits. This symbol sets the RF Front End configuration in ble_user_config.h
 *  and selects the appropriate PA table in ble_user_config.c.
 *  Other configurations can be used by editing these files.
 *
 *  Define only one symbol:
 *  CC2650EM_7ID    - Differential RF and internal biasing
                      (default for CC2640R2 LaunchPad)
 *  CC2650EM_5XD    锟� Differential RF and external biasing
 *  CC2650EM_4XS    锟� Single-ended RF on RF-P and external biasing
 *  CC2640R2DK_CXS  - WCSP: Single-ended RF on RF-N and external biasing
 *                    (Note that the WCSP is only tested and characterized for
 *                     single ended configuration, and it has a WCSP-specific
 *                     PA table)
 *
 *  Note: CC2650EM_xxx reference designs apply to all CC26xx devices.
 *  ==========================================================================
 */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                  <pin mapping>
 */
 
/* LEDs */
#define CC2640R2F_EEG_PIN_LED_ON            0
#define CC2640R2F_EEG_PIN_LED_OFF           1
#define CC2640R2F_EEG_PIN_RLED              IOID_1
#define CC2640R2F_EEG_PIN_GLED              IOID_0

/* I2C */
#define CC2640R2F_EEG_I2C0_SCL0             IOID_18
#define CC2640R2F_EEG_I2C0_SDA0             IOID_19

/* SPI */
#define CC2640R2F_EEG_SPI0_MISO             PIN_UNASSIGNED
#define CC2640R2F_EEG_SPI0_MOSI             PIN_UNASSIGNED
#define CC2640R2F_EEG_SPI0_CLK              PIN_UNASSIGNED
#define CC2640R2F_EEG_SPI0_CSN              PIN_UNASSIGNED
#define CC2640R2F_EEG_SPI1_MISO             IOID_24
#define CC2640R2F_EEG_SPI1_MOSI             IOID_30
#define CC2640R2F_EEG_SPI1_CLK              IOID_25
#define CC2640R2F_EEG_SPI1_CSN              PIN_UNASSIGNED

/* SPI ads1299 */
#define CC2640R2F_EEG_SPI_ADS1299_CS		IOID_26
#define CC2640R2F_EEG_ADS1299_CS_ON         0
#define CC2640R2F_EEG_ADS1299_CS_OFF        1

#define CC2640R2F_EEG_SPI_ADS1299_nDRDY     IOID_23
#define CC2640R2F_EEG_ADS1299_READY         0
#define CC2640R2F_EEG_ADS1299_NOT_READY     1

#define Mod_START                           IOID_27
#define Mod_RESET                           IOID_28
#define Mod_nPDWN                           IOID_29

/* Oscillator 2.048MHz for ads1299 */
#define CC2640R2F_EEG_ADC_CLK				IOID_22
#define CC2640R2F_EEG_ADC_CLK_ON         	1
#define CC2640R2F_EEG_ADC_CLK_OFF       	0


/* UART */
#define CC2640R2F_EEG_UART_RX               IOID_2          /* RXD */
#define CC2640R2F_EEG_UART_TX               IOID_3          /* TXD */
#define CC2640R2F_EEG_UART_CTS              PIN_UNASSIGNED
#define CC2640R2F_EEG_UART_RTS              PIN_UNASSIGNED

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC2640R2F_EEG_initGeneral(void);

/*!
 *  @def    CC2640R2F_EEG_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC2640R2F_EEG_I2CName {
    CC2640R2F_EEG_I2C0 = 0,
	
    CC2640R2F_EEG_I2CCOUNT
} CC2640R2F_EEG_I2CName;

/*!
 *  @def    CC2640R2F_EEG_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum CC2640R2F_EEG_SPIName {
    CC2640R2F_EEG_SPI0 = 0,
    CC2640R2F_EEG_SPI1,

    CC2640R2F_EEG_SPICOUNT
} CC2640R2F_EEG_SPIName;

/*!
 *  @def    CC2640R2F_EEG_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum CC2640R2F_EEG_UARTName {
    CC2640R2F_EEG_UART0 = 0,

    CC2640R2F_EEG_UARTCOUNT
} CC2640R2F_EEG_UARTName;

/*!
 *  @def    CC2640R2F_EEG_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2640R2F_EEG_UDMAName {
    CC2640R2F_EEG_UDMA0 = 0,

    CC2640R2F_EEG_UDMACOUNT
} CC2640R2F_EEG_UDMAName;

/*!
 *  @def    CC2640R2F_EEG_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC2640R2F_EEG_CryptoName {
    CC2640R2F_EEG_CRYPTO0 = 0,

    CC2640R2F_EEG_CRYPTOCOUNT
} CC2640R2F_EEG_CryptoName;

/*!
 *  @def    CC2640R2F_EEG_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC2640R2F_EEG_AESCCMName {
    CC2640R2F_EEG_AESCCM0 = 0,

    CC2640R2F_EEG_AESCCMCOUNT
} CC2640R2F_EEG_AESCCMName;

/*!
 *  @def    CC2640R2F_EEG_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC2640R2F_EEG_AESGCMName {
    CC2640R2F_EEG_AESGCM0 = 0,

    CC2640R2F_EEG_AESGCMCOUNT
} CC2640R2F_EEG_AESGCMName;

/*!
 *  @def    CC2640R2F_EEG_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC2640R2F_EEG_AESCBCName {
    CC2640R2F_EEG_AESCBC0 = 0,

    CC2640R2F_EEG_AESCBCCOUNT
} CC2640R2F_EEG_AESCBCName;

/*!
 *  @def    CC2640R2F_EEG_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC2640R2F_EEG_AESCTRName {
    CC2640R2F_EEG_AESCTR0 = 0,

    CC2640R2F_EEG_AESCTRCOUNT
} CC2640R2F_EEG_AESCTRName;

/*!
 *  @def    CC2640R2F_EEG_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC2640R2F_EEG_AESECBName {
    CC2640R2F_EEG_AESECB0 = 0,

    CC2640R2F_EEG_AESECBCOUNT
} CC2640R2F_EEG_AESECBName;

/*!
 *  @def    CC2640R2F_EEG_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC2640R2F_EEG_AESCTRDRBGName {
    CC2640R2F_EEG_AESCTRDRBG0 = 0,

    CC2640R2F_EEG_AESCTRDRBGCOUNT
} CC2640R2F_EEG_AESCTRDRBGName;

/*!
 *  @def    CC2640R2F_EEG_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum CC2640R2F_EEG_NVSName {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    CC2640R2F_EEG_NVSCC26XX0 = 0,
#endif

    CC2640R2F_EEG_NVSCOUNT
} CC2640R2F_EEG_NVSName;

/*!
 *  @def    CC2640R2F_EEG_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2640R2F_EEG_TRNGName {
    CC2640R2F_EEG_TRNG0 = 0,
    CC2640R2F_EEG_TRNGCOUNT
} CC2640R2F_EEG_TRNGName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2640R2F_EEG_BOARD_H__ */
