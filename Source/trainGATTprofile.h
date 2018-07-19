/******************************************************************************

 @file  simpleGATTprofile.h

 @brief This file contains the Simple GATT profile definitions and prototypes.

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

#ifndef SIMPLEGATTPROFILE_H
#define SIMPLEGATTPROFILE_H

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

#define SIZEOF(f) ((uint8)sizeof(f))
#define makeUUID(uuid) 0xF1,0x4A,0x1D,0x9A,0x29,0x6F,0xB9,0xA9,0xD3,0x44,0x9D,0xE5,0x84,0x13,0x08,uuid,

// Simple Profile Service UUID
#define TRAINPROFILE_SERV_UUID             makeUUID(0)
    
#define U_MOTOR_PWM 1
#define U_MOTOR_CURRENT 2
#define U_LED_PWM 3
#define U_CONFIG 4
#define U_DEF_CONFIG 5
#define U_PROXADC1 6
#define U_PROXADC2 7
#define U_BATT 8
#define U_BATT_ADC 9
#define U_BATT_VOLT 10
#define U_DEV_NAME 11
#define U_DEV_APPE 12

#define TRAINPROFILE_MOTOR_PWM_UUID        makeUUID(U_MOTOR_PWM)
#define TRAINPROFILE_MOTOR_CURRENT_UUID    makeUUID(U_MOTOR_CURRENT)
#define TRAINPROFILE_LED_PWM_UUID          makeUUID(U_LED_PWM)
#define TRAINPROFILE_CONFIG_UUID           makeUUID(U_CONFIG)
#define TRAINPROFILE_DEF_CONFIG_UUID       makeUUID(U_DEF_CONFIG)
#define TRAINPROFILE_PROXADC1_UUID         makeUUID(U_PROXADC1)
#define TRAINPROFILE_PROXADC2_UUID         makeUUID(U_PROXADC2)
  
#define TRAINPROFILE_BATT_SERV_UUID        0x180F  // Battery Service
#define TRAINPROFILE_BATT_UUID             0x2A19
#define TRAINPROFILE_BATT_ADC_UUID         makeUUID(U_BATT_ADC)
#define TRAINPROFILE_BATT_VOLT_UUID        makeUUID(U_BATT_VOLT)
  
#define TRAINPROFILE_DEV_NAME_UUID         DEVICE_NAME_UUID
#define TRAINPROFILE_DEV_APPE_UUID         APPEARANCE_UUID
  
#define UUID_LAST(n) n[15]

#ifdef SHAMKA_UART_DEBUG
#define printText(t) {char tt[]=t;HalUARTWrite(0,(uint8*)tt,sizeof(tt));}
#else
#define printText(t)
#endif
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
typedef void (*trainProfileChange_t)( uint8 paramID );

typedef struct
{
  trainProfileChange_t        pfnTrainProfileChange;  // Called when characteristic value changes
} trainProfileCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * TrainProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t TrainProfile_AddService( uint32 services );

/*
 * TrainProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t TrainProfile_RegisterAppCBs( trainProfileCBs_t *appCallbacks );

/*
 * TrainProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t TrainProfile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * TrainProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t TrainProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
