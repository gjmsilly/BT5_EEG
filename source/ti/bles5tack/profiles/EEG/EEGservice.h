/******************************************************************************

 @file  EEGService.h

 @brief This file contains the EEG profile definitions

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2010-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#ifndef EEGservice_H
#define EEGservice_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// EEG Service defines
#define EEG_SERV_UUID                		 0xFFF0  // UUID

// Battery level Characteristic defines
#define BATTERY_LEVEL_ID         		     0
#define BATTERY_LEVEL_UUID		             0xFFF1
#define EEG_BATTERY_LEVEL_LEN         	     5		// MAX Length of Characteristic in bytes
#define EEG_BATTERY_LEVEL_LEN_MIN            1       // MIN Length of Characteristic in bytes

// ADS1299 COMMAND Characteristic defines
#define ADS1299_COMMAND_ID                   1
#define ADS1299_COMMAND_UUID                 0xFFF2
#define EEG_ADS1299_COMMAND_LEN              1      // MAX Length of Characteristic in bytes


// Proximity Profile Services bit fields
#define EEG_SERVICE               			0x00000001




/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*EEG_charChange_t)( uint8 paramID );

typedef struct
{
  EEG_charChange_t        pfnChangeCb;  // Called when characteristic value changes
} EEG_CBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * EEGservice_AddService- Initializes the EEG service by registering
 *         				  GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t EEGservice_AddService( uint32 services );

/*
 * EEGservice_RegisterAppCBs - Registers the application callback function.
 *                             Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t EEGservice_RegisterAppCBs( EEG_CBs_t *appCallbacks );

/*
 * EEGservice_SetParameter - Set a EEG parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *        	  the parameter ID and WILL be cast to the appropriate
 *        	  data type (example: data type of uint16 will be cast to
 *        	  uint16 pointer).
 */
extern bStatus_t EEGservice_SetParameter( uint8 param, uint8 len, void *value );

/*
 * EEGservice_GetParameter - Get a EEG parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *        	  the parameter ID and WILL be cast to the appropriate
 *         	 data type (example: data type of uint16 will be cast to
 *         	 uint16 pointer).
 */
extern bStatus_t EEGservice_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /*  EEGservice_H */
