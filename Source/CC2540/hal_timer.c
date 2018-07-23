/******************************************************************************

 @file  hal_timer.c

 @brief This file contains the interface to the Timer Service.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2006-2016, Texas Instruments Incorporated
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
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

/*********************************************************************
 NOTE: Z-Stack and TIMAC no longer use CC2530 Timer 1, Timer 3, and 
       Timer 4. The supporting timer driver module is removed and left 
       for the users to implement their own application timer 
       functions.
*********************************************************************/

#include "hal_types.h"
#include "hal_timer.h"

#define PREDEC 5000

void HalTimer1ON(void);
inline void HalTimer1ON(){
  T1CTL|=(2<<0);
};

void HalTimer1OFF(void);
inline void HalTimer1OFF(void){
  T1CTL&=~(0<<0);
};

void HalTimer1Init (halTimerCBack_t cBack){
  T1CC0L=LO_UINT16(PREDEC);
  T1CC0H=HI_UINT16(PREDEC);
  T1CNTL=0;
  T1CTL=(3<<2);
};

void halTimer1SetChannelDuty (uint8 channel, uint16 promill){
  uint32 promill2=promill;
  if(promill2>0)promill2=(promill2*PREDEC/65535)+1;
  if(promill2==0){
    switch(channel){
    case 1:
      T1CCTL1=(1<<3)|(1<<2)|(0<<0);
      break;
    case 2:
      T1CCTL2=(1<<3)|(1<<2)|(0<<0);
      break;
    }
    if(((T1CCTL1&3)==0)&&((T1CCTL2&3)==0))HalTimer1OFF();
  }
  else{
    switch(channel){
    case 1:{
      if(promill2>PREDEC){
        T1CCTL1=(0<<3)|(1<<2)|(0<<0);
      } else {
        promill2--;
        T1CC1L=LO_UINT16(promill2);
        T1CCTL1=(1<<3)|(1<<2)|(0<<0);
        T1CC1H=HI_UINT16(promill2);
        T1CCTL1=(6<<3)|(1<<2)|(1<<0);
      }
      break;}
    case 2:{
      if(promill2>PREDEC){
        T1CCTL2=(0<<3)|(1<<2)|(0<<0);
      } else {
        promill2--;
        T1CC2L=LO_UINT16(promill2);
        T1CC2H=HI_UINT16(promill2);
        if((T1CCTL2&0x3)==0)T1CCTL2=(6<<3)|(1<<2)|(1<<0);
      }
      break;}
    }
  }
  if(((T1CCTL1&3)!=0)||((T1CCTL2&3)!=0))HalTimer1ON();
};




/*#pragma vector=T1_VECTOR
__near_func __interrupt void T1_IRQ(void)
{ 

}*/