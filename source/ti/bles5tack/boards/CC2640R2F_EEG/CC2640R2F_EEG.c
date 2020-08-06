/**
 * @file    CC2640R2F_EEG.c
 * @author  gjmsilly
 * @brief   CC2640R2F_EEG v2 Configuration
 * @version 0.1
 * @date    2020-07-22
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
 
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/cc26x0r2/driverlib/ioc.h>
#include <ti/devices/cc26x0r2/driverlib/udma.h>
#include <ti/devices/cc26x0r2/inc/hw_ints.h>
#include <ti/devices/cc26x0r2/inc/hw_memmap.h>

#include "CC2640R2F_EEG.h"

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = &PowerCC26XX_standbyPolicy,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = true,
#ifdef USE_RCOSC
    .calibrateRCOSC_LF  = true,
#else
    .calibrateRCOSC_LF  = false,
#endif
    .calibrateRCOSC_HF  = true,
};

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

const PIN_Config BoardGpioInitTable[] = {

    CC2640R2F_EEG_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,         /* LED initially off */
    CC2640R2F_EEG_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,          /* LED initially on */

    CC2640R2F_EEG_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                                 /* UART RX via debugger back channel */
    CC2640R2F_EEG_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,                            /* UART TX via debugger back channel */

    CC2640R2F_EEG_SPI_ADS1299_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,   /* ADS1299 chip select - default off */
    Mod_RESET | PIN_GPIO_OUTPUT_EN ,                                                                     /* ADS1299 reset - default off */
    Mod_nPDWN | PIN_GPIO_OUTPUT_EN ,                                                                     /* ADS1299 power down - default off */
    Mod_START | PIN_GPIO_OUTPUT_EN ,                                                                     /* ADS1299 START PIN */
    CC2640R2F_EEG_ADC_CLK | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL ,                          /* Oscillator 2.048MHz - default on */
    //CC2640R2F_EEG_SPI_ADS1299_nDRDY | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,                     /* ADS1299 ready to transfer - set as edge dect */

    PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  =============================== I2C ===============================
*/
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>


I2CCC26XX_Object i2cCC26xxObjects[CC2640R2F_EEG_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC2640R2F_EEG_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = CC2640R2F_EEG_I2C0_SDA0,
        .sclPin      = CC2640R2F_EEG_I2C0_SCL0,
    }
};

const I2C_Config I2C_config[CC2640R2F_EEG_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object      = &i2cCC26xxObjects[CC2640R2F_EEG_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CC2640R2F_EEG_I2C0]
    },
};

const uint_least8_t I2C_count = CC2640R2F_EEG_I2CCOUNT;

/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

SPICC26XXDMA_Object spiCC26XXDMAObjects[CC2640R2F_EEG_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC2640R2F_EEG_SPICOUNT] = {
    {
        .baseAddr           = SSI0_BASE,
        .intNum             = INT_SSI0_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin            = CC2640R2F_EEG_SPI0_MOSI,
        .misoPin            = CC2640R2F_EEG_SPI0_MISO,
        .clkPin             = CC2640R2F_EEG_SPI0_CLK,
        .csnPin             = CC2640R2F_EEG_SPI0_CSN,
        .minDmaTransferSize = 10
    },
    {
        .baseAddr           = SSI1_BASE,
        .intNum             = INT_SSI1_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI1,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI1_TX,
        .mosiPin            = CC2640R2F_EEG_SPI1_MOSI,
        .misoPin            = CC2640R2F_EEG_SPI1_MISO,
        .clkPin             = CC2640R2F_EEG_SPI1_CLK,
        .csnPin             = CC2640R2F_EEG_SPI1_CSN,
        .minDmaTransferSize = 10
    }
};

const SPI_Config SPI_config[CC2640R2F_EEG_SPICOUNT] = {
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC2640R2F_EEG_SPI0],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC2640R2F_EEG_SPI0]
    },
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC2640R2F_EEG_SPI1],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC2640R2F_EEG_SPI1]
    },
};

const uint_least8_t SPI_count = CC2640R2F_EEG_SPICOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects[CC2640R2F_EEG_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CC2640R2F_EEG_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CC2640R2F_EEG_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = CC2640R2F_EEG_UART_TX,
        .rxPin          = CC2640R2F_EEG_UART_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[CC2640R2F_EEG_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[CC2640R2F_EEG_UART0]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    }
};

const UART_Config UART_config[CC2640R2F_EEG_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CC2640R2F_EEG_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[CC2640R2F_EEG_UART0]
    },
};

const uint_least8_t UART_count = CC2640R2F_EEG_UARTCOUNT;

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC2640R2F_EEG_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC2640R2F_EEG_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC2640R2F_EEG_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC2640R2F_EEG_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC2640R2F_EEG_UDMA0]
    },
};

/*
 *  =============================== Crypto ===============================
 */
#include <ti/drivers/crypto/CryptoCC26XX.h>

CryptoCC26XX_Object cryptoCC26XXObjects[CC2640R2F_EEG_CRYPTOCOUNT];

const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC2640R2F_EEG_CRYPTOCOUNT] = {
    {
        .baseAddr       = CRYPTO_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_CRYPTO,
        .intNum         = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority    = ~0,
    }
};

const CryptoCC26XX_Config CryptoCC26XX_config[CC2640R2F_EEG_CRYPTOCOUNT] = {
    {
         .object  = &cryptoCC26XXObjects[CC2640R2F_EEG_CRYPTO0],
         .hwAttrs = &cryptoCC26XXHWAttrs[CC2640R2F_EEG_CRYPTO0]
    },
};

/*
 *  =============================== AESCCM ===============================
 */
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

AESCCMCC26XX_Object aesccmCC26XXObjects[CC2640R2F_EEG_AESCCMCOUNT];

const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CC2640R2F_EEG_AESCCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCCM_Config AESCCM_config[CC2640R2F_EEG_AESCCMCOUNT] = {
    {
         .object  = &aesccmCC26XXObjects[CC2640R2F_EEG_AESCCM0],
         .hwAttrs = &aesccmCC26XXHWAttrs[CC2640R2F_EEG_AESCCM0]
    },
};

const uint_least8_t AESCCM_count = CC2640R2F_EEG_AESCCMCOUNT;


/*
 *  =============================== AESGCM ===============================
 */
#include <ti/drivers/AESGCM.h>
#include <ti/drivers/aesgcm/AESGCMCC26XX.h>

AESGCMCC26XX_Object aesgcmCC26XXObjects[CC2640R2F_EEG_AESGCMCOUNT];

const AESGCMCC26XX_HWAttrs aesgcmCC26XXHWAttrs[CC2640R2F_EEG_AESGCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESGCM_Config AESGCM_config[CC2640R2F_EEG_AESGCMCOUNT] = {
    {
         .object  = &aesgcmCC26XXObjects[CC2640R2F_EEG_AESGCM0],
         .hwAttrs = &aesgcmCC26XXHWAttrs[CC2640R2F_EEG_AESGCM0]
    },
};

const uint_least8_t AESGCM_count = CC2640R2F_EEG_AESGCMCOUNT;

/*
 *  =============================== AESCBC ===============================
 */
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>

AESCBCCC26XX_Object aescbcCC26XXObjects[CC2640R2F_EEG_AESCBCCOUNT];

const AESCBCCC26XX_HWAttrs aescbcCC26XXHWAttrs[CC2640R2F_EEG_AESCBCCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCBC_Config AESCBC_config[CC2640R2F_EEG_AESCBCCOUNT] = {
    {
         .object  = &aescbcCC26XXObjects[CC2640R2F_EEG_AESCBC0],
         .hwAttrs = &aescbcCC26XXHWAttrs[CC2640R2F_EEG_AESCBC0]
    },
};

const uint_least8_t AESCBC_count = CC2640R2F_EEG_AESCBCCOUNT;

/*
 *  =============================== AESCTR ===============================
 */
#include <ti/drivers/AESCTR.h>
#include <ti/drivers/aesctr/AESCTRCC26XX.h>

AESCTRCC26XX_Object aesctrCC26XXObjects[CC2640R2F_EEG_AESCTRCOUNT];

const AESCTRCC26XX_HWAttrs aesctrCC26XXHWAttrs[CC2640R2F_EEG_AESCTRCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCTR_Config AESCTR_config[CC2640R2F_EEG_AESCTRCOUNT] = {
    {
         .object  = &aesctrCC26XXObjects[CC2640R2F_EEG_AESCTR0],
         .hwAttrs = &aesctrCC26XXHWAttrs[CC2640R2F_EEG_AESCTR0]
    },
};

const uint_least8_t AESCTR_count = CC2640R2F_EEG_AESCTRCOUNT;

/*
 *  =============================== AESECB ===============================
 */
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

AESECBCC26XX_Object aesecbCC26XXObjects[CC2640R2F_EEG_AESECBCOUNT];

const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CC2640R2F_EEG_AESECBCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESECB_Config AESECB_config[CC2640R2F_EEG_AESECBCOUNT] = {
    {
         .object  = &aesecbCC26XXObjects[CC2640R2F_EEG_AESECB0],
         .hwAttrs = &aesecbCC26XXHWAttrs[CC2640R2F_EEG_AESECB0]
    },
};

const uint_least8_t AESECB_count = CC2640R2F_EEG_AESECBCOUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */
#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

AESCTRDRBGXX_Object aesctrdrbgXXObjects[CC2640R2F_EEG_AESCTRDRBGCOUNT];

const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CC2640R2F_EEG_AESCTRDRBGCOUNT] = {
    {
        .aesctrIndex       = CC2640R2F_EEG_AESCTR0,
    }
};

const AESCTRDRBG_Config AESCTRDRBG_config[CC2640R2F_EEG_AESCTRDRBGCOUNT] = {
    {
         .object  = &aesctrdrbgXXObjects[CC2640R2F_EEG_AESCTRDRBG0],
         .hwAttrs = &aesctrdrbgXXHWAttrs[CC2640R2F_EEG_AESCTRDRBG0]
    },
};

const uint_least8_t AESCTRDRBG_count = CC2640R2F_EEG_AESCTRDRBGCOUNT;

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,       /* Lowest HWI priority */
    .swiPriority        = 0,        /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,     /* Keep XOSC dependency while in standby */
    .globalCallback     = NULL,     /* No board specific callback */
    .globalEventMask    = 0         /* No events subscribed to */
};

/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#define NVS_REGIONS_BASE 0x1A000
#define SECTORSIZE       0x1000
#define REGIONSIZE       (SECTORSIZE * 4)

#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH

/*
 * Reserve flash sectors for NVS driver use by placing an uninitialized byte
 * array at the desired flash address.
 */
#if defined(__TI_COMPILER_VERSION__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
#pragma LOCATION(flashBuf, NVS_REGIONS_BASE);
#pragma NOINIT(flashBuf);
static char flashBuf[REGIONSIZE];

#elif defined(__IAR_SYSTEMS_ICC__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
static __no_init char flashBuf[REGIONSIZE] @ NVS_REGIONS_BASE;

#elif defined(__GNUC__)

/*
 * Place the flash buffers in the .nvs section created in the gcc linker file.
 * The .nvs section enforces alignment on a sector boundary but may
 * be placed anywhere in flash memory.  If desired the .nvs section can be set
 * to a fixed address by changing the following in the gcc linker file:
 *
 * .nvs (FIXED_FLASH_ADDR) (NOLOAD) : AT (FIXED_FLASH_ADDR) {
 *      *(.nvs)
 * } > REGION_TEXT
 */
__attribute__ ((section (".nvs")))
static char flashBuf[REGIONSIZE];

#endif

/* Allocate objects for NVS Internal Regions */
NVSCC26XX_Object nvsCC26xxObjects[1];

/* Hardware attributes for NVS Internal Regions */
const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[1] = {
    {
        .regionBase = (void *)flashBuf,
        .regionSize = REGIONSIZE,
    },
};

#endif /* Board_EXCLUDE_NVS_INTERNAL_FLASH */

/* NVS Region index 0 refer to NVS */
const NVS_Config NVS_config[CC2640R2F_EEG_NVSCOUNT] = {
#ifndef Board_EXCLUDE_NVS_INTERNAL_FLASH
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26xxObjects[0],
        .hwAttrs = &nvsCC26xxHWAttrs[0],
    },
#endif
};

const uint_least8_t NVS_count = CC2640R2F_EEG_NVSCOUNT;

/*
 *  ========================= TRNG begin ====================================
 */
#include <TRNGCC26XX.h>

/* TRNG objects */
TRNGCC26XX_Object trngCC26XXObjects[CC2640R2F_EEG_TRNGCOUNT];

/* TRNG configuration structure, describing which pins are to be used */
const TRNGCC26XX_HWAttrs TRNGCC26XXHWAttrs[CC2640R2F_EEG_TRNGCOUNT] = {
    {
        .powerMngrId    = PowerCC26XX_PERIPH_TRNG,
    }
};

/* TRNG configuration structure */
const TRNGCC26XX_Config TRNGCC26XX_config[] = {
    {
         .object  = &trngCC26XXObjects[0],
         .hwAttrs = &TRNGCC26XXHWAttrs[0]
    },
    {NULL, NULL}
};

/*
 *  ========================= TRNG end ====================================
 */


/*
 *  ======== CC2640R2F_EEG_initGeneral ========
 */
void CC2640R2F_EEG_initGeneral(void)
{
    Power_init();

    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }
}

/*
 *  ======== Board_init ========
 */
void Board_init(void)
{
    CC2640R2F_EEG_initGeneral();
}

