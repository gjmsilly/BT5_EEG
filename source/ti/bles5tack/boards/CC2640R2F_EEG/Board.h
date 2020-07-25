/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef __BOARD_H
#define __BOARD_H

#define Board_CC2640R2F_EEG

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Board.h>

#define Board_initGeneral()     Board_init()  /* deprecated */
#include "CC2640R2F_EEG.h"

/* These #defines allow us to reuse TI-RTOS across other device families */

#define Board_GPIO_LED0         CC2640R2F_EEG_GPIO_LED_GREEN
#define Board_GPIO_LED1         CC2640R2F_EEG_GPIO_LED_RED
#define Board_GPIO_RLED         CC2640R2F_EEG_GPIO_LED_RED
#define Board_GPIO_GLED         CC2640R2F_EEG_GPIO_LED_GREEN
#define Board_GPIO_LED_ON       CC2640R2F_EEG_GPIO_LED_ON
#define Board_GPIO_LED_OFF      CC2640R2F_EEG_GPIO_LED_OFF

#define Board_I2C0              CC2640R2F_EEG_I2C0

#define Board_PIN_LED0          CC2640R2F_EEG_PIN_GLED
#define Board_PIN_LED1          CC2640R2F_EEG_PIN_RLED
#define Board_PIN_RLED          CC2640R2F_EEG_PIN_RLED
#define Board_PIN_GLED          CC2640R2F_EEG_PIN_GLED

#define Board_SPI0              CC2640R2F_EEG_SPI0

#define Board_UART0             CC2640R2F_EEG_UART0

#define Board_CRYPTO0           CC2640R2F_EEG_CRYPTO0
#define Board_AESCCM0           CC2640R2F_EEG_AESCCM0
#define Board_AESGCM0           CC2640R2F_EEG_AESGCM0
#define Board_AESCBC0           CC2640R2F_EEG_AESCBC0
#define Board_AESCTR0           CC2640R2F_EEG_AESCTR0
#define Board_AESECB0           CC2640R2F_EEG_AESECB0
#define Board_AESCTRDRBG0       CC2640R2F_EEG_AESCTRDRBG0

#define Board_NVSINTERNAL       CC2640R2F_EEG_NVSCC26XX0

/*
 * These macros are provided for backwards compatibility.
 * Please use the <Driver>_init functions directly rather
 * than Board_init<Driver>.
 */
#define Board_initGPIO()        GPIO_init()
#define Board_initSPI()         SPI_init()
#define Board_initI2C()         I2C_init()
#define Board_initUART()        UART_init()

/*
 * These macros are provided for backwards compatibility.
 * Please use the 'Board_PIN_xxx' macros to differentiate
 * them from the 'Board_GPIO_xxx' macros.
 */
#define Board_LED_ON            Board_GPIO_LED_ON
#define Board_LED_OFF           Board_GPIO_LED_OFF
#define Board_LED0              Board_PIN_LED0
#define Board_LED1              Board_PIN_LED1
#define Board_RLED              Board_PIN_RLED
#define Board_GLED              Board_PIN_GLED


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
