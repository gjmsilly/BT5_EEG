/******************************************************************************

 @file  BT5_EEG.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2020, Texas Instruments Incorporated
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
#include <stdio.h>
#include <stdint.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <icall.h>
#include <util.h>
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

/* Bluetooth Profiles */
#include <devinfoservice.h>
#include <EEGservice.h>

/* Application specific includes */
#include <Board.h>
#include <BT5_EEG.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#ifdef PTM_MODE
#include "npi_task.h"              // To allow RX event registration
#include "npi_ble.h"               // To enable transmission of messages to UART
#include "icall_hci_tl.h"          // To allow ICall HCI Transport Layer
#endif // PTM_MODE

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or 
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with 
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     12

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     36

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// How often to perform periodic event (in ms)
//#define EEG_PERIODIC_EVT_PERIOD               5000

// Task configuration
#define EEG_TASK_PRIORITY                     1

#ifndef EEG_TASK_STACK_SIZE
#define EEG_TASK_STACK_SIZE                   1024
#endif

// Application events
#define EEG_DATA_READY_EVT                    0     // nRDY in ads1299 event
#define EEG_CHAR_CHANGE_EVT                   1
#define EEG_ADV_EVT                           2
#if defined(GAP_BOND_MGR)
#define EEG_PAIR_STATE_EVT                    3
#define EEG_PASSCODE_EVT                      4
#endif
//#define EEG_PERIODIC_EVT                    5
#define EEG_CONN_EVT                          6

// Internal Events for RTOS application
#define EEG_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define EEG_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define EEG_ALL_EVENTS                        (EEG_ICALL_EVT             | \
                                              EEG_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define EEG_ADDR_STR_SIZE                    15

// Spin if the expression is not true
#define BT5_EEG_ASSERT(expr) if (!(expr)) BT5_EEG_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} Evt_t;

// Struct for message about ads1299 state
typedef struct
{
    PIN_Id pinId;
    bool state;
} Ads1299State_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} PairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} PasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} GapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                
  uint8_t data[];
} ClockEventData_t;

// Connected device information
typedef struct
{
  uint16_t              connHandle;                        // Connection Handle
} ConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Task configuration
Task_Struct EEGTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(EEGTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t EEGTaskStack[EEG_TASK_STACK_SIZE];

// Buffer point and state for EEG date transfer
uint8_t         EEGBuffer_Num=0;       // state to mark the EEGbuffer point
bool            EEGBuffer_Page=0;      //  mark the buffer1 or buffer2
uint8_t         *pBuffer1;
uint8_t         *pBuffer2;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
//ClockEventData_t argPeriodic =
//{ .event = EEG_PERIODIC_EVT };

// Per-handle connection info
static ConnRec_t connList[MAX_NUM_BLE_CONNS];

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "BT5_EEG";

// Advertisement data
static uint8_t advertData[] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(EEG_SERV_UUID),
  HI_UINT16(EEG_SERV_UUID)
};

// Scan Response Data
static uint8_t scanRspData[] =
{
  // complete name
  8,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B',
  'T',
  '5',
  '_',
  'E',
  'E',
  'G',

  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};
#endif // PRIVACY_1_2_CFG

/*
 * ADS1299 pin configuration table:
 *   - nDRDY interrupts are configured to trigger on falling edge.
 */
PIN_Config Ads1299PinTable[] = {
    CC2640R2F_EEG_SPI_ADS1299_nDRDY | PIN_INPUT_EN |
    PIN_IRQ_NEGEDGE,   /* ADS1299 ready to transfer - set as edge dect */
    PIN_TERMINATE
};

/* Pin driver handles */
PIN_Handle                  nDRDYPinHandle;
/* Global memory storage for a PIN_Config table */
PIN_State                   nDRDYPinState;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
/* Task functions */
static void BT5_EEG_init( void );
static void BT5_EEG_taskFxn(UArg a0, UArg a1);

/* Event message processing functions */
static uint8_t BT5_EEG_processStackMsg(ICall_Hdr *pMsg);
static uint8_t BT5_EEG_processGATTMsg(gattMsgEvent_t *pMsg);
static void BT5_EEG_processGapMessage(gapEventHdr_t *pMsg);
static void BT5_EEG_processAppMsg(Evt_t *pMsg);
static void BT5_EEG_processAdvEvent(GapAdvEventData_t *pEventData);
static void BT5_EEG_processCharValueChangeEvt(uint8_t paramId);

/* Stack or profile callback function */
static void BT5_EEG_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void BT5_EEG_charValueChangeCB(uint8_t paramId);

/* Connection handling functions */
static uint8_t BT5_EEG_getConnIndex(uint16_t connHandle);
static uint8_t BT5_EEG_removeConn(uint16_t connHandle);
static uint8_t BT5_EEG_clearConnListEntry(uint16_t connHandle);
static uint8_t BT5_EEG_addConn(uint16_t connHandle);

/* PHY handling functions */
#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
static void BT5_EEG_updateRPA(void);
#endif // PRIVACY_1_2_CFG

/* PIN interrupt handling functions */
static void nDRDYCallbackFxn(PIN_Handle handle, PIN_Id pinId);
static void BT5_EEG_handleADS1299nDRDY(Ads1299State_t *pMsg);

/* Utility functions */
static void BT5_EEG_clockHandler(UArg arg);
static void BT5_EEG_performPeriodicTask(void);
static status_t BT5_EEG_enqueueMsg(uint8_t event, void *pData);

/* unused */
#if defined(GAP_BOND_MGR)
static void BT5_EEG_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void BT5_EEG_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void BT5_EEG_processPairState(PairStateData_t *pPairState);
static void BT5_EEG_processPasscode(PasscodeData_t *pPasscodeData);
#endif
#ifdef PTM_MODE
void BT5_EEG_handleNPIRxInterceptEvent(uint8_t *pMsg);      // Declaration
static void BT5_EEG_sendToNPI(uint8_t *buf, uint16_t len);  // Declaration
#endif // PTM_MODE

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler (uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS/SERVICE CALLBACKS 
 */

#if defined(GAP_BOND_MGR)
// GAP Bond Manager Callbacks
static gapBondCBs_t BT5_EEG_BondMgrCBs =
{
  BT5_EEG_passcodeCb,       // Passcode callback
  BT5_EEG_pairStateCb       // Pairing/Bonding state Callback
};
#endif

// EEG service Callbacks
static EEG_CBs_t BT5_EEG_EEGServCBs =
{
  .pfnChangeCb = BT5_EEG_charValueChangeCB  // EEG service Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BT5_EEG_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void BT5_EEG_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

#ifdef PTM_MODE
/*********************************************************************
* @fn      BT5_EEG_handleNPIRxInterceptEvent
*
* @brief   Intercept an NPI RX serial message and queue for this application.
*
* @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
*
* @return  none.
*/
void BT5_EEG_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
 // Send Command via HCI TL
 HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

 // The data is stored as a message, free this first.
 ICall_freeMsg(((NPIMSG_msg_t *)pMsg)->pBuf);

 // Free container.
 ICall_free(pMsg);
}

/*********************************************************************
* @fn      BT5_EEG_sendToNPI
*
* @brief   Create an NPI packet and send to NPI to transmit.
*
* @param   buf - pointer HCI event or data.
*
* @param   len - length of buf in bytes.
*
* @return  none
*/
static void BT5_EEG_sendToNPI(uint8_t *buf, uint16_t len)
{
 npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

 if (pNpiPkt)
 {
   pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
   pNpiPkt->hdr.status = 0xFF;
   pNpiPkt->pktLen = len;
   pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

   memcpy(pNpiPkt->pData, buf, len);

   // Send to NPI
   // Note: there is no need to free this packet.  NPI will do that itself.
   NPITask_sendToHost((uint8_t *)pNpiPkt);
 }
}
#endif // PTM_MODE

/*********************************************************************
 * @fn      BT5_EEG_createTask
 *
 * @brief   Task creation function for the BT5_EEG.
 */
void BT5_EEG_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = EEGTaskStack;
  taskParams.stackSize = EEG_TASK_STACK_SIZE;
  taskParams.priority = EEG_TASK_PRIORITY;

  Task_construct(&EEGTask, BT5_EEG_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      BT5_EEG_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void BT5_EEG_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // Create one-shot clock for internal periodic events.
//  Util_constructClock(&clkPeriodic, BT5_EEG_clockHandler, \
//                      EEG_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

  /* Initialize the PIN module */

  // Open nDRDY pins
  nDRDYPinHandle = PIN_open(&nDRDYPinState, Ads1299PinTable);
  if(!nDRDYPinHandle)
  {
      Task_exit();
  }

  // Setup callback for nDRDY pins
  if(PIN_registerIntCb(nDRDYPinHandle, &nDRDYCallbackFxn) != 0)
  {
      Task_exit();
  }


  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

#if defined(GAP_BOND_MGR)
  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    // Don't send a pairing request after connecting; the peer device must
    // initiate pairing
    uint8_t pairMode = GAPBOND_PAIRING_MODE_NO_PAIRING ;
    // Use authenticated pairing: require passcode.
    uint8_t mitm = TRUE;
    // This device only has display capabilities. Therefore, it will display the
    // passcode during pairing. However, since the default passcode is being
    // used, there is no need to display anything.
    uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    // Request bonding (storing long-term keys for re-encryption upon subsequent
    // connections without repairing)
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }
#endif

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  EEGservice_AddService(GATT_ALL_SERVICES);    // EEG Service

  // Setup the EEG Service Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {

    uint8_t batterylevelVal[EEG_BATTERY_LEVEL_LEN_MIN] = {0x00};
    uint8_t AdcommandVal = 0x00;

    EEGservice_SetParameter(BATTERY_LEVEL_ID, sizeof(EEG_BATTERY_LEVEL_LEN_MIN),
                            batterylevelVal);
    EEGservice_SetParameter(ADS1299_COMMAND_ID, sizeof(EEG_ADS1299_COMMAND_LEN),
                            &AdcommandVal);
  }

  // Register callback with EEG SERVICE
  EEGservice_RegisterAppCBs(&BT5_EEG_EEGServCBs);

#if defined(GAP_BOND_MGR)
  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&BT5_EEG_BondMgrCBs);
#endif

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Initialize Connection List
  BT5_EEG_clearConnListEntry(CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // Initialize I2C0 for BQ25895
  BQ25895_init();

  // Initialize SPI and interrupts for ads1299
  ADS1299_init();

}

/*********************************************************************
 * @fn      BT5_EEG_taskFxn
 *
 * @brief   Application task entry point for the BT5_EEG.
 *
 * @param   a0, a1 - not used.
 */
static void BT5_EEG_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  BT5_EEG_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, EEG_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = BT5_EEG_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & EEG_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          Evt_t *pMsg = (Evt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            BT5_EEG_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      BT5_EEG_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t BT5_EEG_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      BT5_EEG_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = BT5_EEG_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
        //  AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        default:
          break;
      }

      break;
  }

  default:
      // do nothing
      break;
  }

#ifdef PTM_MODE
  // Check for NPI Messages
  hciPacket_t *pBuf = (hciPacket_t *)pMsg;

  // Serialized HCI Event
  if (pBuf->hdr.event == HCI_CTRL_TO_HOST_EVENT)
  {
    uint16_t len = 0;

    // Determine the packet length
    switch(pBuf->pData[0])
    {
      case HCI_EVENT_PACKET:
        len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
        break;

      case HCI_ACL_DATA_PACKET:
        len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
        break;

      default:
        break;
    }

    // Send to Remote Host.
    BT5_EEG_sendToNPI(pBuf->pData, len);

    // Free buffers if needed.
    switch (pBuf->pData[0])
    {
      case HCI_ACL_DATA_PACKET:
      case HCI_SCO_DATA_PACKET:
        BM_free(pBuf->pData);
      default:
        break;
    }
  }
#endif // PTM_MODE

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      BT5_EEG_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t BT5_EEG_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
      System_printf( "FC Violated: %d\r\n", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
      System_printf("MTU Size: %d\r\n", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      BT5_EEG_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void BT5_EEG_processAppMsg(Evt_t *pMsg)
{
  bool dealloc = TRUE;

  switch (pMsg->event)
  {

    case EEG_DATA_READY_EVT:
        BT5_EEG_handleADS1299nDRDY((Ads1299State_t*)(pMsg->pData));
      break;

    case EEG_CHAR_CHANGE_EVT:
      BT5_EEG_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case EEG_ADV_EVT:
      BT5_EEG_processAdvEvent((GapAdvEventData_t*)(pMsg->pData));
      break;
#if defined(GAP_BOND_MGR)
    case EEG_PAIR_STATE_EVT:
      BT5_EEG_processPairState((PairStateData_t*)(pMsg->pData));
      break;

    case EEG_PASSCODE_EVT:
      BT5_EEG_processPasscode((PasscodeData_t*)(pMsg->pData));
      break;
#endif
//    case EEG_PERIODIC_EVT:
//      BT5_EEG_performPeriodicTask();
//      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      BT5_EEG_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void BT5_EEG_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        System_printf("Initialized\r\n");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&BT5_EEG_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        BT5_EEG_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        BT5_EEG_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        BT5_EEG_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        BT5_EEG_ASSERT(status == SUCCESS);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
        if (addrMode > ADDRMODE_RANDOM)
        {
//          BT5_EEG_updateRPA();
//
//          // Create one-shot clock for RPA check event.
//          Util_constructClock(&clkRpaRead, BT5_EEG_clockHandler,
//                              EEG_READ_RPA_EVT_PERIOD, 0, true,
//                              (UArg) &argRpaRead);
        }
#endif // PRIVACY_1_2_CFG
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      System_printf  ( "Num Conns: %d\r\n",    \
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list
        BT5_EEG_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        System_printf (  "Connected to %s\r\n",    \
                       Util_convertBdAddr2Str(pPkt->devAddr));


        // Start Periodic Clock.
        //Util_startClock(&clkPeriodic);
      }

      if (numActive < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLegacy);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      System_printf ( "Device Disconnected!\r\n");
      System_printf ( "Num Conns: %d \r\n", \
                    (uint16_t)numActive);

      // Remove the connection from the list
      BT5_EEG_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
       // Util_stopClock(&clkPeriodic);

      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      BT5_EEG_charValueChangeCB
 *
 * @brief   Callback from EEG SERVICE indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void BT5_EEG_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (BT5_EEG_enqueueMsg(EEG_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}

/*********************************************************************
 * @fn      BT5_EEG_processCharValueChangeEvt
 *
 * @brief   Process a pending EEG SERVICE characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void BT5_EEG_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue;
  switch(paramId)
  {
    case BATTERY_LEVEL_ID:
      EEGservice_GetParameter(BATTERY_LEVEL_ID, &newValue);
      System_printf("Char: %2x\r\n",
                    newValue);
      break;
    case ADS1299_COMMAND_ID:
        EEGservice_GetParameter(ADS1299_COMMAND_ID, &newValue);
        ADS1299_SendCommand(newValue);
        System_printf("command: %2x\r\n",
                      newValue);
        break;
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      BT5_EEG_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (EEG_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void BT5_EEG_performPeriodicTask(void)
{

}

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
/*********************************************************************
 * @fn      BT5_EEG_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void BT5_EEG_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
//    Display_printf(dispHandle, EEG_ROW_RPA, 0, "RP Addr: %s",
//                   Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}
#endif // PRIVACY_1_2_CFG

/*********************************************************************
 * @fn      BT5_EEG_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void BT5_EEG_clockHandler(UArg arg)
{
//  ClockEventData_t *pData = (ClockEventData_t *)arg;

// if (pData->event == EEG_PERIODIC_EVT)
// {
//   // Start the next period
//   Util_startClock(&clkPeriodic);
//
//   // Post event to wake up the application
//   BT5_EEG_enqueueMsg(EEG_PERIODIC_EVT, NULL);
// }
 //else
//     if (pData->event == EEG_READ_RPA_EVT)
// {
//   // Start the next period
//   Util_startClock(&clkRpaRead);
//
//   // Post event to read the current RPA
//   BT5_EEG_enqueueMsg(EEG_READ_RPA_EVT, NULL);
// }
}

/*********************************************************************
 * @fn      BT5_EEG_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void BT5_EEG_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  GapAdvEventData_t *pData = ICall_malloc(sizeof(GapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(BT5_EEG_enqueueMsg(EEG_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      BT5_EEG_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void BT5_EEG_processAdvEvent(GapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
//      Display_printf(dispHandle, EEG_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
//                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
//      Display_printf(dispHandle, EEG_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
//                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

/*********************************************************************
 * @fn     nDRDYCallbackFxn
 *
 * @brief  Callback from PIN driver on interrupt
 *
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */

static void nDRDYCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{

    PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);  // disable the pin interrupt

    Ads1299State_t          ads1299Msg ={ .pinId = pinId };
    uint8_t                 sendMsg = FALSE; // notify the app
    uint32_t                status; // status of dma

    pBuffer1=Buffer1;
    pBuffer2=Buffer2;


    status = uDMAIntStatus(UDMA0_BASE);
    uDMAIntClear(UDMA0_BASE,status);

    Mod_CS_Enable

     if( !EEGBuffer_Page ) // Buffer1
     {
        if(EEGBuffer_Num<3)
        {
            SPIDMA_transfer((pBuffer1 + 28*EEGBuffer_Num));
            EEGBuffer_Num++;
        }
        else
        {
            SPIDMA_transfer((pBuffer1 + 84));
            EEGBuffer_Num=0;        // reset
            EEGBuffer_Page=1;       // move to buffer2
            sendMsg = TRUE;         // notify the ble application
            ads1299Msg.state = 0;   // ble ready to read buffer1
        }
     }
     else // Buffer2
     {
         if(EEGBuffer_Num<3)
         {
             SPIDMA_transfer((pBuffer2 + 28*EEGBuffer_Num));
             EEGBuffer_Num++;
         }
         else
         {
             SPIDMA_transfer((pBuffer2 + 84));
             EEGBuffer_Num=0;        // reset
             EEGBuffer_Page=0;       // move to buffer1
             sendMsg = TRUE;         // notify the application
             ads1299Msg.state = 1;   // ble ready to read buffer2
         }
     }

     if(sendMsg == TRUE)
     {
         Ads1299State_t *pads1299State = ICall_malloc(sizeof(Ads1299State_t));
         if(pads1299State != NULL)
         {
             *pads1299State = ads1299Msg;
             if(BT5_EEG_enqueueMsg(EEG_DATA_READY_EVT, pads1299State) != SUCCESS)
             {
               ICall_free(pads1299State);
             }
         }
     }

     PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_NEGEDGE);  // disable the pin interrupt

}

/*********************************************************************
 * @fn     BT5_EEG_handleADS1299nDRDY
 *
 * @brief
 *
 * @param  handle    The PIN_Handle instance this is about
 */
static void BT5_EEG_handleADS1299nDRDY(Ads1299State_t *pMsg)
{
    pBuffer1=Buffer1;
    pBuffer2=Buffer2;

    if (pMsg->state == 0)
    {
        EEGservice_SetParameter(BATTERY_LEVEL_ID,112,pBuffer1);
    }
    else
        EEGservice_SetParameter(BATTERY_LEVEL_ID,112,pBuffer2);

}


#if defined(GAP_BOND_MGR)
/*********************************************************************
 * @fn      BT5_EEG_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void BT5_EEG_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  PairStateData_t *pData = ICall_malloc(sizeof(PairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(BT5_EEG_enqueueMsg(EEG_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      BT5_EEG_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void BT5_EEG_passcodeCb(uint8_t *pDeviceAddr,
                               uint16_t connHandle,
                               uint8_t uiInputs,
                               uint8_t uiOutputs,
                               uint32_t numComparison)
{
  PasscodeData_t *pData = ICall_malloc(sizeof(PasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(BT5_EEG_enqueueMsg(EEG_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}
#endif

/*********************************************************************
 * @fn      BT5_EEG_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
/*static void BT5_EEG_processPairState(PairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
//      Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
//        Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
//        Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
//        Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
//        Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
//        Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
//        Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}
*/

/*********************************************************************
 * @fn      BT5_EEG_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
/*static void BT5_EEG_processPasscode(PasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
//    Display_printf(dispHandle, EEG_ROW_CONNECTION, 0, "Passcode: %d",
                //   B_APP_DEFAULT_PASSCODE);
  }

#if defined(GAP_BOND_MGR)
  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
#endif
}
*/


/*********************************************************************
 * @fn      BT5_EEG_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t BT5_EEG_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  Evt_t *pMsg = ICall_malloc(sizeof(Evt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}



/*********************************************************************
 * @fn      BT5_EEG_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t BT5_EEG_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      BT5_EEG_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t BT5_EEG_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      BT5_EEG_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. CONNHANDLE_ALL will always succeed.
 */
static uint8_t BT5_EEG_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = BT5_EEG_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
    {
      return(bleInvalidRange);
    }
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == CONNHANDLE_ALL))
    {
      connList[i].connHandle = CONNHANDLE_INVALID;
    }
  }

  return(SUCCESS);
}


/*********************************************************************
 * @fn      BT5_EEG_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t BT5_EEG_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = BT5_EEG_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    // Clear Connection List Entry
    BT5_EEG_clearConnListEntry(connHandle);
  }

  return connIndex;
}

#ifdef PTM_MODE
/*********************************************************************
* @fn      BT5_EEG_doEnablePTMMode
*
* @brief   Stop advertising, configure & start PTM mode
*
* @param   index - item index from the menu
*
* @return  always true
*/
bool BT5_EEG_doEnablePTMMode(uint8_t index)
{
  // Clear Display
  Display_clearLines(dispHandle, 0, 15);

  // Indicate in screen that PTM Mode is initializing
  Display_printf(dispHandle, 1, 0, "PTM Mode initializing!\n\n\rPlease note UART feed will now stop...");  
  
  // Before starting the NPI task close Display driver to make sure there is no shared resource used by both
  Display_close(dispHandle);
  
  // Start NPI task
  NPITask_createTask(ICALL_SERVICE_CLASS_BLE);

  // Disable Advertising and destroy sets
  GapAdv_destroy(advHandleLegacy,GAP_ADV_FREE_OPTION_ALL_DATA);

  // Intercept NPI RX events.
  NPITask_registerIncomingRXEventAppCB(BT5_EEG_handleNPIRxInterceptEvent, INTERCEPT);

  // Register for Command Status information
  HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) simple_peripheral_sendToNPI, NULL, selfEntity);

  // Register for Events
  HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity));

  // Inform Stack to Initialize PTM
  HCI_EXT_EnablePTMCmd();

  // Open back the display to avoid crashes to future calls to Display_printf (even though they won't go through until reboot)
  dispHandle = Display_open(Display_Type_ANY, NULL);
  
  return TRUE;
}
#endif
/*********************************************************************
*********************************************************************/
