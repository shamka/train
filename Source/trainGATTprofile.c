/******************************************************************************

 @file  simpleGATTprofile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

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
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "hal_uart.h"

#include "trainGATTprofile.h"

#include "stdio.h"

/*********************************************************************
 * MACROS
 */
#define GATT_MAX_NUM_CONN 8 
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// TRAIN service
CONST uint8 trainProfileServUUID[] ={TRAINPROFILE_SERV_UUID};
// MOTOR PWM
CONST uint8 trainProfile_MOTOR_PWM_UUID[] ={TRAINPROFILE_MOTOR_PWM_UUID};
CONST uint8 trainProfileMOTOR_PWM_access = GATT_PROP_READ|GATT_PROP_WRITE;
static uint8 trainProfileMOTOR_PWM_value     = 0;
// MOTOR Current
CONST uint8 trainProfile_MOTOR_CURRENT_UUID[] ={TRAINPROFILE_MOTOR_CURRENT_UUID};
CONST uint8 trainProfileMOTOR_CURRENT_access = GATT_PROP_READ|GATT_PROP_NOTIFY;
static uint8 trainProfileMOTOR_CURRENT_value     = 0;
static gattCharCfg_t *trainProfile_MOTOR_CURRENT_conns=0;
// LED PWM
CONST uint8 trainProfile_LED_PWM_UUID[] ={TRAINPROFILE_LED_PWM_UUID};
CONST uint8 trainProfileLED_PWM_access = GATT_PROP_READ|GATT_PROP_WRITE;
static uint8 trainProfileLED_PWM_value     = 0;
// OConfig
CONST uint8 trainProfile_CONFIG_UUID[] ={TRAINPROFILE_CONFIG_UUID};
CONST uint8 trainProfileCONFIG_access = GATT_PROP_READ|GATT_PROP_WRITE;
static uint8 trainProfileCONFIG_value[TRAIN_OPERATE_CONFIG_LEN] = {0,0};//ignoreWall,ignoreGnd
// SConfig
CONST uint8 trainProfile_DEF_CONFIG_UUID[] ={TRAINPROFILE_DEF_CONFIG_UUID};
CONST uint8 trainProfileDEF_CONFIG_access = GATT_PROP_READ|GATT_PROP_WRITE;
static uint8 trainProfileDEF_CONFIG_value[TRAIN_STATIC_CONFIG_LEN] = {0,0,0,0,0};//ignoreWall,ignoreGnd,maxLed,maxMotor,motorWhenOnTrain
// ADC 1 - wall proximity
CONST uint8 trainProfile_PROXADC1_UUID[] ={TRAINPROFILE_PROXADC1_UUID};
CONST uint8 trainProfilePROXADC1_access = GATT_PROP_READ|GATT_PROP_NOTIFY;
static uint8 trainProfilePROXADC1_value     = 0;
static gattCharCfg_t *trainProfile_PROXADC1_conns=0;
// ADC 2 - Ground proximity
CONST uint8 trainProfile_PROXADC2_UUID[] ={TRAINPROFILE_PROXADC2_UUID};
CONST uint8 trainProfilePROXADC2_access = GATT_PROP_READ|GATT_PROP_NOTIFY;
static uint8 trainProfilePROXADC2_value     = 0;
static gattCharCfg_t *trainProfile_PROXADC2_conns=0;

// BATTARY service
CONST uint8 trainBattaryServiceUUID[ATT_BT_UUID_SIZE]={LO_UINT16( TRAINPROFILE_BATT_SERV_UUID ), HI_UINT16( TRAINPROFILE_BATT_SERV_UUID )};
// BATTARY VALUE
CONST uint8 trainProfile_BATT_UUID[] ={LO_UINT16(TRAINPROFILE_BATT_UUID),HI_UINT16(TRAINPROFILE_BATT_UUID)};
CONST uint8 trainProfileBATT_access = GATT_PROP_READ|GATT_PROP_NOTIFY;
static uint8 trainProfileBATT_value     = 100;
static gattCharCfg_t *trainProfile_BATT_conns=0;
// BATTARY voltage
CONST uint8 trainProfile_BATT_VOLT_UUID[] ={TRAINPROFILE_BATT_VOLT_UUID};
CONST uint8 trainProfileBATT_VOLT_access = GATT_PROP_READ|GATT_PROP_NOTIFY;
static float trainProfileBATT_VOLT_value = 0;
static gattCharCfg_t *trainProfile_BATT_VOLT_conns=0;
// BATTARY ADC
static uint16 trainProfileBATT_ADC_value = 0;
#ifdef BATT_ADC
CONST uint8 trainProfile_BATT_ADC_UUID[] ={TRAINPROFILE_BATT_ADC_UUID};
CONST uint8 trainProfileBATT_ADC_access = GATT_PROP_READ|GATT_PROP_NOTIFY;
static gattCharCfg_t *trainProfile_BATT_ADC_conns=0;
#endif

// device name
CONST uint8 trainProfile_DEV_NAME_UUID[] ={LO_UINT16(TRAINPROFILE_DEV_NAME_UUID),HI_UINT16(TRAINPROFILE_DEV_NAME_UUID)};
CONST uint8 trainProfileDEV_NAME_access = GATT_PROP_READ|GATT_PROP_WRITE_NO_RSP;
extern uint8 scanRspData[31];
// appearance
CONST uint8 trainProfile_DEV_APPE_UUID[] ={LO_UINT16(TRAINPROFILE_DEV_APPE_UUID),HI_UINT16(TRAINPROFILE_DEV_APPE_UUID)};
CONST uint8 trainProfileDEV_APPE_access = GATT_PROP_READ;
CONST uint16 trainProfileDEV_APPE_value = GAP_APPEARE_GENERIC_HID;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static trainProfileCBs_t *trainProfile_AppCBs = NULL;
static CONST gattAttrType_t trainDeviceProfileService = { ATT_BT_UUID_SIZE, gapServiceUUID };
static CONST gattAttrType_t trainBattaryProfileService = { ATT_BT_UUID_SIZE, trainBattaryServiceUUID };
static CONST gattAttrType_t trainProfileService = { ATT_UUID_SIZE, trainProfileServUUID };

/*********************************************************************
 * Profile Attributes - variables
 */


/*********************************************************************
 * Profile Attributes - Table
 */
// GENERIC ACCESS SERVICE
static gattAttribute_t trainDeviceProfileAttrTbl[] = 
{
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8*)&trainDeviceProfileService            /* pValue */
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,                     
    0,                                    
    (uint8 *)&trainProfileDEV_NAME_access  
  },
  { 
    { ATT_BT_UUID_SIZE, trainProfile_DEV_NAME_UUID }, 
    GATT_PERMIT_READ|GATT_PERMIT_WRITE,  
    0,                                     
    &scanRspData[2]   
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,     
    0,
    (uint8 *)&trainProfileDEV_APPE_access
  },
  { 
    { ATT_BT_UUID_SIZE, trainProfile_DEV_APPE_UUID },
    GATT_PERMIT_READ|GATT_PERMIT_WRITE,
    0,
    (uint8*)&trainProfileDEV_APPE_value
  },

};
// BATTARY SERVICE
static gattAttribute_t trainBattaryProfileAttrTbl[] = 
{
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8*)&trainBattaryProfileService            /* pValue */
  },

#ifdef BATT_ADC
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileBATT_ADC_access
  },
  { 
    { ATT_UUID_SIZE, trainProfile_BATT_ADC_UUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileBATT_ADC_value
  },
  {
     {ATT_BT_UUID_SIZE , clientCharCfgUUID},
     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
     0,
     (uint8 *)&trainProfile_BATT_ADC_conns
  },
#endif
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileBATT_VOLT_access
  },
  { 
    { ATT_UUID_SIZE, trainProfile_BATT_VOLT_UUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileBATT_VOLT_value
  },
  {
     {ATT_BT_UUID_SIZE , clientCharCfgUUID},
     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
     0,
     (uint8 *)&trainProfile_BATT_VOLT_conns
  },

  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileBATT_access
  },
  { 
    { ATT_BT_UUID_SIZE, trainProfile_BATT_UUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileBATT_value
  },
  {
     {ATT_BT_UUID_SIZE , clientCharCfgUUID},
     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
     0,
     (uint8 *)&trainProfile_BATT_conns
  },
 
};
// TRAIN SERVICE
static gattAttribute_t trainProfileAttrTbl[] = 
{
  
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&trainProfileService            /* pValue */
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&trainProfileMOTOR_PWM_access
  },
  { 
    { ATT_UUID_SIZE, trainProfile_MOTOR_PWM_UUID },
    GATT_PERMIT_READ|GATT_PERMIT_WRITE,
    0,                                 
    (uint8 *)&trainProfileMOTOR_PWM_value     
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,                      
    0,                                     
    (uint8*)&trainProfileMOTOR_CURRENT_access  
  },
  { 
    { ATT_UUID_SIZE, trainProfile_MOTOR_CURRENT_UUID }, 
    GATT_PERMIT_READ, 
    0,                               
    &trainProfileMOTOR_CURRENT_value  
  },
  {
     {ATT_BT_UUID_SIZE , clientCharCfgUUID},
     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
     0,
     (uint8 *)&trainProfile_MOTOR_CURRENT_conns
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,                  
    0,                                    
    (uint8 *)&trainProfileLED_PWM_access    
  },
  { 
    { ATT_UUID_SIZE, trainProfile_LED_PWM_UUID }, 
    GATT_PERMIT_READ|GATT_PERMIT_WRITE,  
    0,                                    
    (uint8 *)&trainProfileLED_PWM_value     
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,                   
    0,                                     
    (uint8 *)&trainProfileCONFIG_access     
  },
  { 
    { ATT_UUID_SIZE, trainProfile_CONFIG_UUID }, 
    GATT_PERMIT_READ|GATT_PERMIT_WRITE,  
    0,                                   
    (uint8 *)&trainProfileCONFIG_value     
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,                   
    0,                                  
    (uint8 *)&trainProfileDEF_CONFIG_access     
  },
  { 
    { ATT_UUID_SIZE, trainProfile_DEF_CONFIG_UUID },
    GATT_PERMIT_READ|GATT_PERMIT_WRITE, 
    0,                                  
    (uint8 *)&trainProfileDEF_CONFIG_value   
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,                      
    0,                                
    (uint8 *)&trainProfilePROXADC1_access      
  },
  { 
    { ATT_UUID_SIZE, trainProfile_PROXADC1_UUID }, 
    GATT_PERMIT_READ,  
    0,                             
    (uint8 *)&trainProfilePROXADC1_value 
  },
  {
     {ATT_BT_UUID_SIZE , clientCharCfgUUID},
     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
     0,
     (uint8 *)&trainProfile_PROXADC1_conns
  },
  
  { 
    { ATT_BT_UUID_SIZE, characterUUID }, 
    GATT_PERMIT_READ,  
    0,                               
    (uint8 *)&trainProfilePROXADC2_access       
  },
  { 
    { ATT_UUID_SIZE, trainProfile_PROXADC2_UUID },
    GATT_PERMIT_READ,  
    0,                                    
    (uint8 *)&trainProfilePROXADC2_value     
  },
  {
     {ATT_BT_UUID_SIZE , clientCharCfgUUID},
     GATT_PERMIT_READ | GATT_PERMIT_WRITE,
     0,
     (uint8 *)&trainProfile_PROXADC2_conns
  },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t trainProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method );
static bStatus_t trainProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Train Profile Service Callbacks
CONST gattServiceCBs_t trainProfileCBs =
{
  trainProfile_ReadAttrCB,  // Read callback function pointer
  trainProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      TrainProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t TrainProfile_AddService( uint32 services )
{
  uint8 status;
//****************************************  
  if ( 
      (trainProfile_MOTOR_CURRENT_conns = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns ))==NULL 
     || (trainProfile_PROXADC1_conns = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns ))==NULL 
     || (trainProfile_PROXADC2_conns = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns ))==NULL 
     || (trainProfile_BATT_conns = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns ))==NULL 
#ifdef BATT_ADC
     || (trainProfile_BATT_ADC_conns = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns ))==NULL 
#endif
     || (trainProfile_BATT_VOLT_conns = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) * linkDBNumConns ))==NULL 
     )
  {     
    if(trainProfile_MOTOR_CURRENT_conns!=0)osal_mem_free(trainProfile_MOTOR_CURRENT_conns);
    if(trainProfile_PROXADC1_conns!=0)osal_mem_free(trainProfile_PROXADC1_conns);
    if(trainProfile_PROXADC2_conns!=0)osal_mem_free(trainProfile_PROXADC2_conns);
    if(trainProfile_BATT_conns!=0)osal_mem_free(trainProfile_BATT_conns);
#ifdef BATT_ADC
    if(trainProfile_BATT_ADC_conns!=0)osal_mem_free(trainProfile_BATT_ADC_conns);
#endif
    if(trainProfile_BATT_VOLT_conns!=0)osal_mem_free(trainProfile_BATT_VOLT_conns);
    return ( bleMemAllocError );
  }
//****************************************  
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, trainProfile_MOTOR_CURRENT_conns ); 
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, trainProfile_PROXADC1_conns );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, trainProfile_PROXADC2_conns );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, trainProfile_BATT_conns );
#ifdef BATT_ADC
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, trainProfile_BATT_ADC_conns );
#endif
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, trainProfile_BATT_VOLT_conns );
    
  if ( services & 1 )
  {
    
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( trainDeviceProfileAttrTbl, 
                                          GATT_NUM_ATTRS( trainDeviceProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &trainProfileCBs );
    status = GATTServApp_RegisterService( trainBattaryProfileAttrTbl, 
                                          GATT_NUM_ATTRS( trainBattaryProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &trainProfileCBs );
    status = GATTServApp_RegisterService( trainProfileAttrTbl, 
                                          GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &trainProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      TrainProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t TrainProfile_RegisterAppCBs( trainProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    trainProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      TrainProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
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
bStatus_t TrainProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case U_DEV_NAME:{
    if(len==0){
      scanRspData[0]=0;
    }
    else if(len<=29){
      osal_memcpy(&scanRspData[2],value,len);
      scanRspData[0]=len+1;
      scanRspData[1]=0x09;
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_MOTOR_PWM:{
    if(len==1){
      trainProfileMOTOR_PWM_value=*(uint8*)value;
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_MOTOR_CURRENT:{
    if(len==1){
      if(trainProfileMOTOR_CURRENT_value==*(uint8*)value){
        break;
      }
      trainProfileMOTOR_CURRENT_value=*(uint8*)value;
      GATTServApp_ProcessCharCfg( trainProfile_MOTOR_CURRENT_conns, &trainProfileMOTOR_CURRENT_value, FALSE,
                                    trainProfileAttrTbl, GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                    INVALID_TASK_ID, trainProfile_ReadAttrCB );
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_LED_PWM:{
    if(len==1){
      trainProfileLED_PWM_value=*(uint8*)value;
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_CONFIG:{
    if(len==TRAIN_OPERATE_CONFIG_LEN){
      osal_memcpy(&trainProfileCONFIG_value,value,TRAIN_OPERATE_CONFIG_LEN);
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_DEF_CONFIG:{
    if(len==TRAIN_STATIC_CONFIG_LEN){
      osal_memcpy(&trainProfileDEF_CONFIG_value,value,TRAIN_STATIC_CONFIG_LEN);
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_PROXADC1:{
    if(len==1){
      if(trainProfilePROXADC1_value==*(uint8*)value){
        break;
      }
      trainProfilePROXADC1_value=*(uint8*)value;
      GATTServApp_ProcessCharCfg( trainProfile_PROXADC1_conns, &trainProfilePROXADC1_value, FALSE,
                                    trainProfileAttrTbl, GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                    INVALID_TASK_ID, trainProfile_ReadAttrCB );
    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_PROXADC2:{
    if(len==1){
      if(trainProfilePROXADC2_value!=*(uint8*)value){
        trainProfilePROXADC2_value=*(uint8*)value;
        GATTServApp_ProcessCharCfg( trainProfile_PROXADC2_conns, &trainProfilePROXADC2_value, FALSE,
                                    trainProfileAttrTbl, GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                    INVALID_TASK_ID, trainProfile_ReadAttrCB );
      }

    }
    else{
      ret = bleInvalidRange;
    }
    break;}
    
  case U_BATT:{
    if(len==1){
      if(trainProfileBATT_value!=*(uint8*)value){
        trainProfileBATT_value=*(uint8*)value;
        GATTServApp_ProcessCharCfg( trainProfile_BATT_conns, &trainProfileBATT_value, FALSE,
                                    trainProfileAttrTbl, GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                    INVALID_TASK_ID, trainProfile_ReadAttrCB );
      }
    }
    else{
      ret = bleInvalidRange;
    }
    break;}

  case U_BATT_ADC:{
    if(len==2){
      if(trainProfileBATT_ADC_value==*(uint16*)value){
        break;
      }
      trainProfileBATT_ADC_value=*(uint16*)value;
#ifdef BATT_ADC    
      GATTServApp_ProcessCharCfg( trainProfile_BATT_ADC_conns, (uint8*)&trainProfileBATT_ADC_value, FALSE,
                                    trainProfileAttrTbl, GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                    INVALID_TASK_ID, trainProfile_ReadAttrCB );
#endif
    }
    else{
      ret = bleInvalidRange;
    }
    break;}

  case U_BATT_VOLT:{
    if(len==sizeof(float)){
      if(trainProfileBATT_VOLT_value==*(float*)value){
        break;
      }
      trainProfileBATT_VOLT_value=*(float*)value;
      GATTServApp_ProcessCharCfg( trainProfile_BATT_VOLT_conns, (uint8*)&trainProfileBATT_VOLT_value, FALSE,
                                    trainProfileAttrTbl, GATT_NUM_ATTRS( trainProfileAttrTbl ),
                                    INVALID_TASK_ID, trainProfile_ReadAttrCB );
    }
    else{
      ret = bleInvalidRange;
    }
    break;}

  default:{
    ret = INVALIDPARAMETER;
    break;}
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      TrainProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t TrainProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case U_MOTOR_PWM:
    *(uint8*)value = trainProfileMOTOR_PWM_value;
    break;
    
  case U_MOTOR_CURRENT:
    *(uint8*)value = trainProfileMOTOR_CURRENT_value;
    break;
    
  case U_LED_PWM:
    *(uint8*)value = trainProfileLED_PWM_value;
    break;
    
  case U_CONFIG:
    osal_memcpy(value,&trainProfileCONFIG_value,TRAIN_OPERATE_CONFIG_LEN);
    break;
    
  case U_DEF_CONFIG:
    osal_memcpy(value,&trainProfileDEF_CONFIG_value,TRAIN_STATIC_CONFIG_LEN);
    break;
    
  case U_PROXADC1:
    *(uint8*)value = trainProfilePROXADC1_value;
    break;
    
  case U_PROXADC2:
    *(uint8*)value = trainProfilePROXADC2_value;
    break;
    
  case U_BATT:
    *(uint8*)value = trainProfileBATT_value;
    break;
    
  case U_BATT_ADC:
    *(uint16*)value = trainProfileBATT_ADC_value;
    break;
    
  case U_BATT_VOLT:
    *(float*)value = trainProfileBATT_VOLT_value;
    break;
    
  default:
    ret = INVALIDPARAMETER;
    break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          trainProfile_ReadAttrCB
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
static bStatus_t trainProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  char *text=NULL;
  printText("trainProfile_ReadAttrCB\r\n");
  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 || maxLen==0)
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
    case TRAINPROFILE_BATT_UUID:
      *pLen = MIN(sizeof(trainProfileBATT_value),maxLen);
      pValue[0]=pAttr->pValue[0];
      break;
      
    case TRAINPROFILE_DEV_NAME_UUID:
      if(scanRspData[0]>0){
        *pLen=MIN(scanRspData[0]-1,maxLen);
        osal_memcpy(pValue,pAttr->pValue,*pLen);
      }
      else *pLen=0;
      break;
      
    case TRAINPROFILE_DEV_APPE_UUID:
        *pLen=MIN(sizeof(trainProfileDEV_APPE_value),maxLen);
        osal_memcpy(pValue,pAttr->pValue,*pLen);
      break;
      
    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }
  }
  else
  {
    if(osal_memcmp(&pAttr->type.uuid[0],&trainProfileServUUID[0],15)){
      switch(UUID_LAST(pAttr->type.uuid)){
      case U_MOTOR_PWM:
      case U_MOTOR_CURRENT:
      case U_LED_PWM:
      case U_PROXADC1:
      case U_PROXADC2:
        *pLen = 1;
        pValue[0]=pAttr->pValue[0];
        break;
        
#ifdef BATT_ADC        
      case U_BATT_ADC:
        *pLen=MIN(sizeof(trainProfileBATT_ADC_value),maxLen);
        osal_memcpy(pValue,pAttr->pValue,*pLen);
        break;      
#endif
        
      case U_BATT_VOLT:
        text = osal_mem_alloc(32);
        if(text!=NULL){
          *pLen=MIN(sprintf((char *)text,"%#5.4fV",*(float*)pAttr->pValue),maxLen);
          osal_memcpy(pValue,text,*pLen);
          osal_mem_free(text);
        }
        else{
          *pLen=0;
        }
        break;      
        
      case U_CONFIG:
        *pLen=MIN(TRAIN_OPERATE_CONFIG_LEN,maxLen);
        osal_memcpy(pValue,pAttr->pValue,*pLen);
        break;      
        
      case U_DEF_CONFIG:
        *pLen=MIN(TRAIN_STATIC_CONFIG_LEN,maxLen);
        osal_memcpy(pValue,pAttr->pValue,*pLen);
        break;
        
      default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
      }
    }
    else{
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn      trainProfile_WriteAttrCB
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
static bStatus_t trainProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  printText("trainProfile_WriteAttrCB\r\n");
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len, offset, GATT_CLIENT_CFG_NOTIFY );
      break;
        
    case TRAINPROFILE_DEV_NAME_UUID:
      if(len==0){
        scanRspData[0]=0;
        notifyApp=U_DEV_NAME;
      }
      else if(len<=29){
        scanRspData[0]=len+1;
        scanRspData[1]=0x09;
        osal_memcpy(&scanRspData[2],pValue,len);
        notifyApp=U_DEV_NAME;
      }
      else{
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      break;

    default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    if(memcmp(&pAttr->type.uuid[0],&trainProfileServUUID[0],15)==0){
      if(offset!=0){
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      switch(UUID_LAST(pAttr->type.uuid)){
        
      case U_MOTOR_PWM:
      case U_LED_PWM:
        if(len==1){
          pAttr->pValue[0]=pValue[0];
          notifyApp = UUID_LAST(pAttr->type.uuid);
        }
        else{
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        break;
        
        
      case U_CONFIG:
        if(len==TRAIN_OPERATE_CONFIG_LEN){
          osal_memcpy(pAttr->pValue,pValue,TRAIN_OPERATE_CONFIG_LEN);
          notifyApp = U_CONFIG;
        }
        else{
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        break;      
        
        
      case U_DEF_CONFIG:
        if(len==TRAIN_STATIC_CONFIG_LEN){
          osal_memcpy(pAttr->pValue,pValue,TRAIN_STATIC_CONFIG_LEN);
          notifyApp = U_DEF_CONFIG;
        }
        else{
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        break;
        

      default:
        status = ATT_ERR_ATTR_NOT_FOUND;
      }
    }
    else{
        status = ATT_ERR_INVALID_HANDLE;
    }
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && trainProfile_AppCBs && trainProfile_AppCBs->pfnTrainProfileChange )
  {
    trainProfile_AppCBs->pfnTrainProfileChange( notifyApp );  
  }
 
  return ( status );
}

/*********************************************************************
*********************************************************************/
