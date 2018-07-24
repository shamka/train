/******************************************************************************

 @file  simpleBLEPeripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"

#include "trainGATTprofile.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#ifdef SHAMKA_UART_DEBUG
#include "hal_uart.h"
#endif

#include "osal_snv.h"
#include "hal_timer.h"

#include "stdio.h"
#include <string.h>
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   80

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          (uint16)(1000/0.625)

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     (uint16)(40/1.25)

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     (uint16)(200/1.25)

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
void initTrain(void);

#ifdef SHAMKA_UART_DEBUG
uint8 buffrx[10];
uint8 bufftx[64];
static CONST halUARTBufControl_t txBuff={0,0,64,bufftx};
static CONST halUARTBufControl_t rxBuff={0,0,10,buffrx};
halUARTCfg_t uart_conf;
static CONST char hello[]="\r\nShamka's Train Version 1.00\r\n";
#endif

//static CONST uint8 locNameDef[]={0x0F,GAP_ADTYPE_LOCAL_NAME_COMPLETE,'S','h','a','m','k','a','\'','s',' ','T','r','a','i','n',};
static CONST uint8 locNameDef[]={0x09,GAP_ADTYPE_LOCAL_NAME_COMPLETE, 0xD9, 0x82, 0xD8, 0xB7, 0xD8, 0xA7, 0xD8, 0xB1};
uint8 temp[32];

static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;


// GAP - SCAN RSP data (max size = 31 bytes)
uint8 scanRspData[31];

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static CONST uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // connection interval range
  //0x05,   // length of this data
  //GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  //LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  //HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  //LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  //HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
  
  6,
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  LO_UINT16( TI_COMPANY_ID ),
  HI_UINT16( TI_COMPANY_ID ),
  0x32,0x16,0x38,

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0,       // 0dBm

  3,
  GAP_ADTYPE_APPEARANCE,
  LO_UINT16( GAP_APPEARE_GENERIC_HID ),
  HI_UINT16( GAP_APPEARE_GENERIC_HID ),

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void trainProfileChangeCB( uint8 paramID );

#ifdef SHAMKA_UART_DEBUG
static void HalUARTCback (uint8 port, uint8 event){
port++;
};
#endif

static float getVolt(uint16 adc){
  return ((adc*1.20)/511)*3;//sprintf((char *)temp,"Voltage: %#5.4f",volt);
}
static uint8 getPerc(float volt){
  if(volt>=3.0)return 100;
  if(volt<=2.0)return 0;
  volt-=2.0;
  return (uint8)(volt*100);
}

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static trainProfileCBs_t simpleBLEPeripheral_TrainProfileCBs =
{
  trainProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  osal_snv_read(SH_SNV_RESP,31,scanRspData);
  
#ifdef SHAMKA_UART_DEBUG
  osal_memset(&uart_conf,0,sizeof(uart_conf));
  uart_conf.baudRate=HAL_UART_BR_115200;
  uart_conf.rx=rxBuff;
  uart_conf.tx=txBuff;
  uart_conf.callBackFunc=HalUARTCback;
  HalUARTOpen(HAL_UART_PORT_0,&uart_conf);
#endif
  
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = -1;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    if(scanRspData[0]!=0xFF){
      if(scanRspData[0]>0){GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, 1+scanRspData[0], (void*)scanRspData );}
      else{GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof(locNameDef), (void*)locNameDef );}
    }
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), (void*)advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    osal_snv_read(SH_SNV_PASS,4,&passkey);
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }
  TrainProfile_AddService( GATT_ALL_SERVICES );
  // Setup the TrainProfile Characteristic Values
  initTrain();
  // Register callback with SimpleGATTprofile
  VOID TrainProfile_RegisterAppCBs( &simpleBLEPeripheral_TrainProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    
#ifdef SHAMKA_UART_DEBUG
  HalUARTWrite(HAL_UART_PORT_0,(uint8*)hello,sizeof(hello));
#endif

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {     
    case GATT_MSG_EVENT:
      // Process GATT message
      simpleBLEPeripheral_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */

#ifdef PLUS_BROADCASTER
static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
uint8 curState;
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  curState=newState;
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
printText("state GAPROLE_STARTED\r\n");
      }
      break;

    case GAPROLE_ADVERTISING:
      {
         printText("state GAPROLE_ADVERTISING\r\n");
      }
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8 advertEnabled = FALSE;
        printText("state GAPROLE_ADVERTISING_NONCONN\r\n");
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                           &advertEnabled);
        
        // Reset flag for next connection.
        first_conn_flag = 0;
      }
      break;
#endif //PLUS_BROADCASTER         
      
    case GAPROLE_CONNECTED:
      {        
          printText("state GAPROLE_CONNECTED\r\n");
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
            uint8 advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;
            
            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
printText("state GAPROLE_CONNECTED_ADV\r\n");
      }
      break;      
    case GAPROLE_WAITING:
      {
          printText("state GAPROLE_WAITING\r\n");
#ifdef PLUS_BROADCASTER                
        uint8 advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                             &advertEnabled);
#endif //PLUS_BROADCASTER
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
         printText("state GAPROLE_WAITING_AFTER_TIMEOUT\r\n"); 
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
printText("state GAPROLE_ERROR\r\n"); 
      }
      break;

    default:
      {
printText("state default\r\n"); 
      }
      break;

  }

  gapProfileState = newState;

  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
}




/***************************************************************************

                              TRAIN LOGIC BLOCK

****************************************************************************/
#include "logicBlock.h"