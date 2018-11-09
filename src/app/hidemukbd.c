/******************************************************************************

 @file       hidemukbd.c

 @brief This file contains the HID emulated keyboard sample application for use
        with the CC2650 Bluetooth Low Energy
        Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2011-2017, Texas Instruments Incorporated
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
 Release Name: simplelink_cc2640r2_sdk_ble_example_pack_01_50_00_62
 Release Date: 2017-11-01 10:38:41
 *****************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>
#include <icall.h>
#include <string.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hidkbdservice.h"
#include "hiddev.h"
#include "ll_common.h"

#include "simple_gatt_profile.h"
//#include "hci.h"

#include "peripheral.h"
#include "board_key.h"
#include "board.h"

#include "hidemukbd.h"

//#include "inc/sdi_task.h"
//#include "inc/sdi_tl_uart.h"
#include "task_uart.h"
#include "sc_uart.h"

#include "Display_EPD_W21.h"
#include <ti/mw/extflash/ExtFlash.h>
#include "audio_duplex.h"
/*********************************************************************
 * MACROS
 */

#define KEY_NONE                    0x00

// Selected HID LED bitmaps
#define LED_NUM_LOCK                0x01
#define LED_CAPS_LOCK               0x02

// Selected HID mouse button values
#define MOUSE_BUTTON_1              0x01
#define MOUSE_BUTTON_NONE           0x00

// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

#define APP_SUGGESTED_PDU_SIZE 251
#define APP_SUGGESTED_TX_TIME 2120


/*********************************************************************
 * CONSTANTS
 */

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              180000

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled.
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled.
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         50

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled.
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// Whether to enable automatic parameter update request when a connection is
// formed.
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6

// Key bindings, can be modified to any HID value.
//#ifndef CC2650_LAUNCHXL
//#define KEY_UP_HID_BINDING                    HID_KEYBOARD_UP_ARROW
//#define KEY_DOWN_HID_BINDING                  HID_KEYBOARD_DOWN_ARROW
//#define KEY_SELECT_HID_BINDING                MOUSE_BUTTON_1
//#define USE_HID_MOUSE
//#endif // !CC2650_LAUNCHXL
//#define KEY_LEFT_HID_BINDING                  HID_KEYBOARD_LEFT_ARROW
//#define KEY_RIGHT_HID_BINDING                 HID_KEYBOARD_RIGHT_ARROW

// Task configuration
#define HIDEMUKBD_TASK_PRIORITY               1

#ifndef HIDEMUKBD_TASK_STACK_SIZE
#define HIDEMUKBD_TASK_STACK_SIZE             1024//644
#endif

#define HIDEMUKBD_KEY_CHANGE_EVT              0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002

// Task Events
#define HIDEMUKBD_ICALL_EVT                   ICALL_MSG_EVENT_ID // Event_Id_31
#define HIDEMUKBD_QUEUE_EVT                   UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_UART_QUEUE_EVT                    Event_Id_00
#define AUDIO_EVT                             Event_Id_03

#define HIDEMUKBD_ALL_EVENTS                  (HIDEMUKBD_ICALL_EVT | \
                                               HIDEMUKBD_QUEUE_EVT | \
                                               SBP_UART_QUEUE_EVT  | \
                                               AUDIO_EVT)

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // Event header
} hidEmuKbdEvt_t;

// RTOS queue for profile/app messages.
typedef struct _queueRec_
{
  Queue_Elem _elem;          // queue element
  uint8_t *pData;            // pointer to app data
} queueRec_t;

// App event passed from profiles.
typedef struct
{
  uint8_t event;  // Type of event
  uint8_t *pData;  // New data
  uint8_t length; // New status
} sbpUARTEvt_t;

//static uint16_t phyOptions = HCI_PHY_OPT_NONE;
/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct ScanTimeoutClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;
// Queue object used for UART messages
static Queue_Struct appUARTMsg;
static Queue_Handle appUARTMsgQueue;

// Queue object used for audio messages
static uint8_t audio_flag;

// Task configuration
Task_Struct hidEmuKbdTask;
Char hidEmuKbdTaskStack[HIDEMUKBD_TASK_STACK_SIZE];

// GAP Profile - Name attribute for SCAN RSP data
static uint8_t scanData[] =
{
  0x0D,                             // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   // AD Type = Complete local name
  'H',
  'I',
  'D',
  ' ',
  'K',
  'e',
  'y',
  'b',
  'o',
  'a',
  'r',
  'd'
};

// Advertising data
static uint8_t advData[] =
{
  // flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // appearance
  0x03,   // length of this data
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16(GAP_APPEARE_HID_KEYBOARD),
  HI_UINT16(GAP_APPEARE_HID_KEYBOARD),

  // service UUIDs
  0x07,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HID_SERV_UUID),
  HI_UINT16(HID_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID),
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID) 
};

// Device name attribute value
static CONST uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "HID Keyboard";

// HID Dev configuration
static hidDevCfg_t hidEmuKbdCfg =
{
  DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
  HID_KBD_FLAGS               // HID feature flags
};

#ifdef USE_HID_MOUSE
// TRUE if boot mouse enabled
static uint8_t hidBootMouseEnabled = FALSE;
#endif // USE_HID_MOUSE
const uint8_t ASCtoHID[128]={
0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,44,30,52,32,33,34,36,52,
38,39,37,46,54,45,55,56,39,30,31,32,33,34,35,36,37,38,51,51,54,46,55,56,31,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,
19,20,21,22,23,24,25,26,27,28,29,47,49,48,35,45,53,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,
27,28,29,47,49,48,53,42
};
/*********************************************************************
 * LOCAL FUNCTIONS
 */

// Application task and event processing.
static void HidEmuKbd_init(void);
static void HidEmuKbd_taskFxn(UArg a0, UArg a1);
static void HidEmuKbd_processAppMsg(hidEmuKbdEvt_t *pMsg);
static void HidEmuKbd_processStackMsg(ICall_Hdr *pMsg);
static void HidEmuKbd_processGattMsg(gattMsgEvent_t *pMsg);
static uint8_t HidEmuKbd_enqueueMsg(uint16_t event, uint8_t state);

// Key press.
static void HidEmuKbd_keyPressHandler(uint8_t keys);
static void HidEmuKbd_handleKeys(uint8_t shift, uint8_t keys);

// HID reports.
//static void HidEmuKbd_sendReport(uint8_t keycode);
static void HidEmuKbd_sendReport(uint8_t Modifier,uint8_t keycode);
#ifdef USE_HID_MOUSE
static void HidEmuKbd_sendMouseReport(uint8_t buttons);
#endif // USE_HID_MOUSE
static uint8_t HidEmuKbd_receiveReport(uint8_t len, uint8_t *pData);
static uint8_t HidEmuKbd_reportCB(uint8_t id, uint8_t type, uint16_t uuid,
                                  uint8_t oper, uint16_t *pLen, uint8_t *pData);
static void HidEmuKbd_hidEventCB(uint8_t evt);


static void Scan_clockHandler(UArg arg);

//simple callback
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);

static void HID_processCharValueChangeEvt(uint8_t paramID);
//uart 接收回调
//void SPPBLEServer_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);
void BLE_enqueueUARTMsg(uint8_t *data, uint8_t len);

//bool SimpleBLEPeripheral_doSetPhy(uint8 index);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static hidDevCB_t hidEmuKbdHidCBs =
{
  HidEmuKbd_reportCB,
  HidEmuKbd_hidEventCB,
  NULL
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HidEmuKbd_createTask
 *
 * @brief   Task creation function for the HID emulated keyboard.
 *
 * @param   none
 *
 * @return  none
 */
void HidEmuKbd_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = hidEmuKbdTaskStack;
  taskParams.stackSize = HIDEMUKBD_TASK_STACK_SIZE;
  taskParams.priority = HIDEMUKBD_TASK_PRIORITY;

  Task_construct(&hidEmuKbdTask, HidEmuKbd_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HidEmuKbd_init
 *
 * @brief   Initialization function for the HidEmuKbd App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ...).
 *
 * @param   none
 *
 * @return  none
 */
void HidEmuKbd_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x5A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg); 
  appUARTMsgQueue = Util_constructQueue(&appUARTMsg); 

  Util_constructClock(&ScanTimeoutClock, Scan_clockHandler,
                      6000, 0, false, 0);//Scan_clock_Evt);

  // Setup the GAP
  VOID GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL,DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    uint8_t initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t gapRole_AdvertOffTime = 0;

    uint8_t enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &gapRole_AdvertOffTime);

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advData), advData);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanData), scanData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enable_update_request);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desired_conn_timeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }
  
 // SimpleBLEPeripheral_doSetPhy(1);

  // Setup Battery Characteristic Values
  {
    uint8_t critical = DEFAULT_BATT_CRITICAL_LEVEL;

    Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8_t), &critical);
  }

/******************************************************************************/
//This API is documented in hci.h
  // HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE ,APP_SUGGESTED_TX_TIME);
  
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
//    // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
/******************************************************************************/

  // Set up HID keyboard service
  HidKbd_AddService();

  // Register for HID Dev callback
  HidDev_Register(&hidEmuKbdCfg, &hidEmuKbdHidCBs);

  // Start the GAP Role and Register the Bond Manager.
  HidDev_StartDevice();

  // Initialize keys on SmartRF06EB.
  Board_initKeys(HidEmuKbd_keyPressHandler);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)
  
  // Use default data Tx / Rx data length and times
  HCI_EXT_SetMaxDataLenCmd(LL_MIN_LINK_DATA_LEN, LL_MIN_LINK_DATA_TIME, LL_MAX_LINK_DATA_TIME, LL_MAX_LINK_DATA_TIME);
  
  //Register to receive UART messages
 // SDITask_registerIncomingRXEventAppCB(SPPBLEServer_enqueueUARTMsg); //ZH
  
 //SDITask_PrintfToUART("%s\r\n", "Hello from SPP BLE Server!");
    // sc uart 串口部分
  ScUARTInit();
  ScUARTWrite("sc_uart\r\n",9);
  
  //Playsound(sound_02);
  ExtFlash_test();
  
  GY_UartTask_RegisterPacketReceivedCallback(BLE_enqueueUARTMsg);
  TaskUARTdoWrite("sample_uart\r\n",13,"%s%d\r\n",NULL,NULL);
  
  EPD_init_Full(); 
  EPD_Dis_Full(NULL,0);   
  //EPD_Dis_Full((unsigned char *)logo,1);  
  EPD_close();
/*  uint8_t white=0xff;
  EPD_init_Part();
  EPD_Dis_Part(0,199,0,199,(unsigned char *)&white,0);
  Task_sleep(10000/Clock_tickPeriod);
  for(uint8_t i=1;i<26;i++)
    EPD_Dis_Part(0,199,(i-1)*8,(i*8-1),(unsigned char *)&logo[25-i],1);
  test();
  EPD_close();*/
}

/*********************************************************************
 * @fn      HidEmuKbd_taskFxn
 *
 * @brief   HidEmuKbd Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   a0, a1 - not used.
 *
 * @return  none
 */
void HidEmuKbd_taskFxn(UArg a0, UArg a1)
{
  // Initialize the application.
  HidEmuKbd_init();

  // Application main loop.
  for (;;)
  {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, HIDEMUKBD_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          HidEmuKbd_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
      
       // If RTOS queue is not empty, process app message.
      if (events & SBP_UART_QUEUE_EVT)
      {
        // If RTOS queue is not empty, process app message.
        if (!Queue_empty(appUARTMsgQueue))
        {
          //Get the message at the front of the queue but still keep it in the queue
          queueRec_t *pRec = Queue_head(appUARTMsgQueue);
          sbpUARTEvt_t *pMsg = (sbpUARTEvt_t *)pRec->pData;

       //   if (pMsg && ((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV)))
          {
                //Send the notification
                for(uint8_t i=0;i<pMsg->length;i++)
                { 
                  if(pMsg->pData[i]>61&&pMsg->pData[i]<91||(pMsg->pData[i]>32&&pMsg->pData[i]<44&&pMsg->pData[i]!=39)
                     ||pMsg->pData[i]==95||pMsg->pData[i]==94||pMsg->pData[i]==60||pMsg->pData[i]==58
                       ||(pMsg->pData[i]<127&&pMsg->pData[i]>122))
                    HidEmuKbd_sendReport(0x02,ASCtoHID[pMsg->pData[i]]);
                  else
                    HidEmuKbd_sendReport(0x00,ASCtoHID[pMsg->pData[i]]);

                  HidEmuKbd_sendReport(0,KEY_NONE);
                  Task_sleep(10000/Clock_tickPeriod);
                }

                {
                  //Remove from queue
                  Util_dequeueMsg(appUARTMsgQueue);

                  //Toggle LED to indicate data received from UART terminal and sent over the air
                  //SPPBLEServer_toggleLed(Board_GLED, Board_LED_TOGGLE);

                  //Deallocate data payload being transmitted.
                  ICall_freeMsg(pMsg->pData);
                  // Free the space from the message.
                  ICall_free(pMsg);
                }

                if(!Queue_empty(appUARTMsgQueue))
                {
                  // Wake up the application to flush out any remaining UART data in the queue.
                  Event_post(syncEvent, SBP_UART_QUEUE_EVT);
                }
          }
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & HIDEMUKBD_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          hidEmuKbdEvt_t *pMsg = (hidEmuKbdEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            HidEmuKbd_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
      if (events & AUDIO_EVT)
      {
   //     soundtask(audio_flag);
      }
    }
  }
}

/*********************************************************************
 * @fn      HidEmuKbd_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidEmuKbd_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      HidEmuKbd_processGattMsg((gattMsgEvent_t *) pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          default:
            break;
        }
      }
      break;

    default:
      // Do nothing
      break;
  }
}

/*********************************************************************
 * @fn      HidEmuKbd_processGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void HidEmuKbd_processGattMsg(gattMsgEvent_t *pMsg)
{
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      HidEmuKbd_processAppMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void HidEmuKbd_processAppMsg(hidEmuKbdEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case HIDEMUKBD_KEY_CHANGE_EVT:
      HidEmuKbd_handleKeys(0, pMsg->hdr.state);
      break;
    case SBP_CHAR_CHANGE_EVT:
      HID_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      //Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HidKEmukbd_keyPressHandler
 *
 * @brief   Key event handler function.
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void HidEmuKbd_keyPressHandler(uint8_t keys)
{
  // Enqueue the event.
  HidEmuKbd_enqueueMsg(HIDEMUKBD_KEY_CHANGE_EVT, keys);
}

/*********************************************************************
 * @fn      HidEmuKbd_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_UP
 *                 KEY_RIGHT
 *
 * @return  none
 */
static bool scanflag = 0;
static void HidEmuKbd_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {

  }

  if (keys & KEY_RIGHT)
  {

  }
  
  if (keys & KEY_SCAN)
  {
    Board_GPIOSet(Board_SCAN,0);
    if(scanflag==0)
    {
      Util_startClock(&ScanTimeoutClock);
      scanflag = 1;
    }   
  }
  if (keys & POWER_OFF)
  {
     audio_flag = 5;
     Event_post(syncEvent, AUDIO_EVT);
  }
}

/*********************************************************************
 * @fn      HidEmuKbd_sendReport
 *
 * @brief   Build and send a HID keyboard report.
 *
 * @param   keycode - HID keycode.
 *
 * @return  none
 */
static void HidEmuKbd_sendReport(uint8_t Modifier,uint8_t keycode)
{
  uint8_t buf[HID_KEYBOARD_IN_RPT_LEN];

  buf[0] = Modifier;         // Modifier keys
  buf[1] = 0;         // Reserved
  buf[2] = keycode;   // Keycode 1
  buf[3] = 0;         // Keycode 2
  buf[4] = 0;         // Keycode 3
  buf[5] = 0;         // Keycode 4
  buf[6] = 0;         // Keycode 5
  buf[7] = 0;         // Keycode 6

  HidDev_Report(HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT,
                HID_KEYBOARD_IN_RPT_LEN, buf);
}

#ifdef USE_HID_MOUSE
/*********************************************************************
 * @fn      HidEmuKbd_sendMouseReport
 *
 * @brief   Build and send a HID mouse report.
 *
 * @param   buttons - Mouse button code
 *
 * @return  none
 */
static void HidEmuKbd_sendMouseReport(uint8_t buttons)
{
  uint8_t buf[HID_MOUSE_IN_RPT_LEN];

  buf[0] = buttons;   // Buttons
  buf[1] = 0;         // X
  buf[2] = 0;         // Y
  buf[3] = 0;         // Wheel
  buf[4] = 0;         // AC Pan

  HidDev_Report(HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT,
                HID_MOUSE_IN_RPT_LEN, buf);
}
#endif // USE_HID_MOUSE

/*********************************************************************
 * @fn      HidEmuKbd_receiveReport
 *
 * @brief   Process an incoming HID keyboard report.
 *
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  status
 */
static uint8_t HidEmuKbd_receiveReport(uint8_t len, uint8_t *pData)
{
  // Verify data length
  if (len == HID_LED_OUT_RPT_LEN)
  {
    // Set keyfob LEDs
    //HalLedSet(HAL_LED_1, ((*pData & LED_CAPS_LOCK) == LED_CAPS_LOCK));
    //HalLedSet(HAL_LED_2, ((*pData & LED_NUM_LOCK) == LED_NUM_LOCK));

    return SUCCESS;
  }
  else
  {
    return ATT_ERR_INVALID_VALUE_SIZE;
  }
}

/*********************************************************************
 * @fn      HidEmuKbd_reportCB
 *
 * @brief   HID Dev report callback.
 *
 * @param   id - HID report ID.
 * @param   type - HID report type.
 * @param   uuid - attribute uuid.
 * @param   oper - operation:  read, write, etc.
 * @param   len - Length of report.
 * @param   pData - Report data.
 *
 * @return  GATT status code.
 */
static uint8_t HidEmuKbd_reportCB(uint8_t id, uint8_t type, uint16_t uuid,
                                  uint8_t oper, uint16_t *pLen, uint8_t *pData)
{
  uint8_t status = SUCCESS;

  // Write
  if (oper == HID_DEV_OPER_WRITE)
  {
    if (uuid == REPORT_UUID)
    {
      // Process write to LED output report; ignore others
      if (type == HID_REPORT_TYPE_OUTPUT)
      {
        status = HidEmuKbd_receiveReport(*pLen, pData);
      }
    }

    if (status == SUCCESS)
    {
      status = HidKbd_SetParameter(id, type, uuid, *pLen, pData);
    }
  }
  // Read
  else if (oper == HID_DEV_OPER_READ)
  {
    uint8_t len;

    status = HidKbd_GetParameter(id, type, uuid, &len, pData);
    if (status == SUCCESS)
    {
      *pLen = len;
    }
  }
  // Notifications enabled
  else if (oper == HID_DEV_OPER_ENABLE)
  {
    if (id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT)
    {
#ifdef USE_HID_MOUSE
      hidBootMouseEnabled = TRUE;
#endif // USE_HID_MOUSE
    }
  }
  // Notifications disabled
  else if (oper == HID_DEV_OPER_DISABLE)
  {
    if (id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT)
    {
#ifdef USE_HID_MOUSE
      hidBootMouseEnabled = FALSE;
#endif // USE_HID_MOUSE
    }
  }

  return status;
}

/*********************************************************************
 * @fn      HidEmuKbd_hidEventCB
 *
 * @brief   HID Dev event callback.
 *
 * @param   evt - event ID.
 *
 * @return  HID response code.
 */
static void HidEmuKbd_hidEventCB(uint8_t evt)
{
  static uint8_t temp;
  uint8_t HID_STATE;
  uint8_t bound_STATE;
  // Process enter/exit suspend or enter/exit boot mode
//  if(evt==HID_DEV_GAPROLE_STATE_CHANGE_EVT)
    HidDev_GetParameter(HIDDEV_GAPROLE_STATE, &HID_STATE);    
//  if(evt==HID_DEV_GAPBOND_STATE_CHANGE_EVT)
    HidDev_GetParameter(HIDDEV_GAPBOND_STATE, &bound_STATE);
  if(temp==4&&HID_STATE == GAPROLE_ADVERTISING) //蓝牙断开
//  if(HID_STATE == GAPROLE_ADVERTISING_NONCONN)  
  {
     audio_flag = 2;
     Event_post(syncEvent, AUDIO_EVT);
  }
  if(HID_STATE == GAPROLE_CONNECTED && ( bound_STATE == 3||bound_STATE == 2))//连接
  {
     audio_flag = 1;
     Event_post(syncEvent, AUDIO_EVT);
  }
    
  TaskUARTdoWrite(NULL,NULL,"ble_CB:%d,%d,%d\r\n",evt,HID_STATE,bound_STATE);
  temp = HID_STATE;
}

/*********************************************************************
 * @fn      HidEmuKbd_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event  - message event.
 * @param   state  - message state.
 *
 * @return  TRUE or FALSE
 */
static uint8_t HidEmuKbd_enqueueMsg(uint16_t event, uint8_t state)
{
  hidEmuKbdEvt_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(hidEmuKbdEvt_t)))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

static void Scan_clockHandler(UArg arg)
{
  //switch(arg)
  {
 // case Scan_clock_Evt:
    scanflag = 0;
    Board_GPIOSet(Board_SCAN,1);//扫描超时
    //Event_post(syncEvent, Event_Id_03);
 //   break;
  }
}

static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  HidEmuKbd_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}

static void HID_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t newValue[200];
  
  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, newValue);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, newValue);
      break;

    default:
      // should not reach here!
      break;
  }
}
/*********************************************************************
 * @fn      SimpleBLEPeripheral_doSetPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0, 1, 2(, 3, 4)
 *
 * @return  always true
 */
/*bool SimpleBLEPeripheral_doSetPhy(uint8 index)
{
  uint8_t gapRoleState;
  uint16_t connectionHandle;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_CODED,
  };

  static uint8_t options[] = {
    HCI_PHY_OPT_NONE, HCI_PHY_OPT_NONE,
    HCI_PHY_OPT_S2, HCI_PHY_OPT_S8,
  };

  // Assign phyOptions
  phyOptions = options[index];

  GAPRole_GetParameter(GAPROLE_STATE, &gapRoleState);
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  //HCI_LE_SetPhyCmd(connectionHandle, 0, phy[index], phy[index], phyOptions);
  HCI_LE_SetPhyCmd(connectionHandle, 0, phy[index], phy[index]);

  return true;
}*/

void BLE_enqueueUARTMsg(uint8_t *data, uint8_t len)
{
  sbpUARTEvt_t *pMsg;
  queueRec_t *pRec;

  //Enqueue message only in a connected state
//  if((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV))
  {
    // Create dynamic pointer to message.
    if (pMsg = ICall_malloc(sizeof(sbpUARTEvt_t)))
    {
      pMsg->pData = (uint8 *)ICall_allocMsg(len);
      if(pMsg->pData)
      {
        //payload
        memcpy(pMsg->pData , data, len);
      }
      pMsg->length = len;

    Util_stopClock(&ScanTimeoutClock);
    scanflag=0;

    audio_flag = 4;
    Board_GPIOSet(Board_SCAN,1);
      
      
      // Enqueue the message.
      if ((pRec = ICall_malloc(sizeof(queueRec_t))))
      {
        pRec->pData = (uint8*)pMsg;
        // This is an atomic operation
        Queue_enqueue(appUARTMsgQueue, &pRec->_elem);

        Event_post(syncEvent, SBP_UART_QUEUE_EVT | AUDIO_EVT);
      }else
      {
//        DEBUG("appUARTMsgQueue ERROR");
        ICall_free(pMsg);
      }
    }
  }
}