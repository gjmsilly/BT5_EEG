/******************************************************************************

 @file  EEGservice.c

 @brief This file contains the EEG service profile
        for use with the BLE sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "EEGservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// EEG_SERV_UUID: 0xFFF0
CONST uint8 EEGServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(EEG_SERV_UUID), HI_UINT16(EEG_SERV_UUID)
};

// BATTERY_LEVEL UUID: 0xFFF1
CONST uint8 BatterylevelUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BATTERY_LEVEL_UUID), HI_UINT16(BATTERY_LEVEL_UUID)
};



/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static EEG_CBs_t *PAppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// EEG Service declaration
static CONST gattAttrType_t EEGServiceDecl = { ATT_BT_UUID_SIZE, EEGServUUID };

// Characteristic "Battery level" Properties (for declaration)
static uint8 BatterylevelProps = GATT_PROP_NOTIFY | GATT_PROP_READ | GATT_PROP_WRITE ;

// Characteristic "Battery level" Value variable
static uint8 BatterylevelVal[EEG_BATTERY_LEVEL_LEN] = {};

// Length of data in characteristic "Battery level" Value variable, initialized to minimal size.
static uint16_t BatterylevelValLen = EEG_BATTERY_LEVEL_LEN_MIN;

// Characteristic "Battery level" User Description
static uint8 BatterylevelUserDesp[12] = "Batterylevel";

// Characteristic "Battery level" Client Characteristic Configuration Descriptor
static gattCharCfg_t *BatterylevelConfig;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t EEGservice_AttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // EEG Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&EEGServiceDecl           		  /* pValue */
  },

    // Characteristic "Battery level" Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &BatterylevelProps
    },

      // Characteristic "Battery level" Value
      {
        { ATT_BT_UUID_SIZE, BatterylevelUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        BatterylevelVal
      },

      // Characteristic "Battery level" User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        BatterylevelUserDesp
      },
	  
      // Characteristic "Battery level" CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&BatterylevelConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t EEGservice_ReadAttrCB	(uint16_t connHandle,
										 gattAttribute_t *pAttr,
										 uint8_t *pValue, uint16_t *pLen,
										 uint16_t offset, uint16_t maxLen,
										 uint8_t method);
static bStatus_t EEGservice_WriteAttrCB (uint16_t connHandle,
										 gattAttribute_t *pAttr,
										 uint8_t *pValue, uint16_t len,
										 uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple EEG Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t EEGservice_CBs =
{
  EEGservice_ReadAttrCB, 			 // Read callback function pointer
  EEGservice_WriteAttrCB,			 // Write callback function pointer
  NULL                      		 // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      EEGservice_AddService
 *
 * @brief   Initializes the EEG service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t EEGservice_AddService( uint32 services )
{
  uint8 status;

   // Allocate Client Characteristic Configuration table
  BatterylevelConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( BatterylevelConfig == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( CONNHANDLE_INVALID, BatterylevelConfig ); 

  if ( services & EEG_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( EEGservice_AttrTbl,
                                          GATT_NUM_ATTRS( EEGservice_AttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &EEGservice_CBs );
  }
  else
  {
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn      EEGservice_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t EEGservice_RegisterAppCBs( EEG_CBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    PAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      EEGservice_SetParameter
 *
 * @brief   Set a EEG service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t EEGservice_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BATTERY_LEVEL_ID:
      if ( len == BatterylevelValLen )
      {
		    memcpy( BatterylevelVal, value, EEG_BATTERY_LEVEL_LEN );
			
			// Try to send notification
			GATTServApp_ProcessCharCfg( BatterylevelConfig, BatterylevelVal, FALSE,
			                            EEGservice_AttrTbl, GATT_NUM_ATTRS( EEGservice_AttrTbl ),
			                            INVALID_TASK_ID, EEGservice_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      EEGservice_GetParameter
 *
 * @brief   Get a EEG service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t EEGservice_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BATTERY_LEVEL_ID:
	  memcpy( value, BatterylevelVal, BatterylevelValLen );
      break;
	  
    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          EEGservice_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t EEGservice_ReadAttrCB (uint16_t connHandle,
									    gattAttribute_t *pAttr,
										uint8_t *pValue, uint16_t *pLen,
										uint16_t offset, uint16_t maxLen,
										uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      case BATTERY_LEVEL_UUID:
        *pLen = BatterylevelValLen;
        memcpy( pValue, pAttr->pValue, BatterylevelValLen );
        break;

      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      EEGservice_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t EEGservice_WriteAttrCB(uint16_t connHandle,
										gattAttribute_t *pAttr,
										uint8_t *pValue, uint16_t len,
										uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint16_t writeLenMin;
  uint16_t writeLenMax;
  uint16_t *pValueLenVar;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case BATTERY_LEVEL_UUID:

          writeLenMin  = EEG_BATTERY_LEVEL_LEN_MIN;     // MIN Length of Characteristic
          writeLenMax  = EEG_BATTERY_LEVEL_LEN;         // MAX Length of Characteristic in bytes
          pValueLenVar = &BatterylevelValLen;           // Current Length of Characteristic in bytes

          // Check whether the length is within bounds.
          if ( offset >= writeLenMax )
          {
            status = ATT_ERR_INVALID_OFFSET;
          }
          else if ( offset + len > writeLenMax )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
          else if ( offset + len < writeLenMin && ( method == ATT_EXECUTE_WRITE_REQ || method == ATT_WRITE_REQ ) )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }

        // Write the value
          else
        {
	       // Copy pValue into the variable we point to from the attribute table.
		   memcpy(pAttr->pValue + offset, pValue, len);
		   
		   // Only notify application and update length if enough data is written.
		   // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
		   //       the application will get a callback for every write with an offset + len larger than EEG_BATTERY_LEVEL_LEN.
		   // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
           //       because the write fragments are concatenated before being sent here.
			
			if ( offset + len >= writeLenMin )
		   {
				notifyApp = BATTERY_LEVEL_ID;
				*pValueLenVar = offset + len; // Update data length.
			}

        }

        break;

     case GATT_CLIENT_CHAR_CFG_UUID: // Request is regarding a Client Characterisic Configuration
		  status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                   offset, GATT_CLIENT_CFG_NOTIFY );
        break;

      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && PAppCBs && PAppCBs->pfnChangeCb )
  {
      PAppCBs->pfnChangeCb( notifyApp );
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/
