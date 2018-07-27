/***************************************************************************

                              TRAIN LOGIC BLOCK

****************************************************************************/


static uint8 battary_update = 255;
static uint8 prox_update = 255;
  
// MOTOR PWM
extern uint16 trainProfileMOTOR_PWM_value;
// MOTOR Current
extern uint16 trainProfileMOTOR_CURRENT_value;
// LED PWM
extern uint16 trainProfileLED_PWM_value;
// OConfig
extern S_OP_CONFIG trainProfileCONFIG_value;
// SConfig
extern S_DEFAULT_CONFIG trainProfileDEF_CONFIG_value;
// ADC 1 - wall proximity
extern int16 trainProfilePROXADC1_value;
// ADC 2 - Ground proximity
extern int16 trainProfilePROXADC2_value;

void setMotor(uint16 cur_pwm, bool bt){
  if(cur_pwm==13){
    Onboard_soft_reset();
  }
  if(bt && cur_pwm!=0){
// motor limities    
    if(cur_pwm<trainProfileCONFIG_value.minMotor){cur_pwm=0;}
    else if(trainProfileCONFIG_value.maxMotor>0 && cur_pwm>trainProfileCONFIG_value.maxMotor){cur_pwm=trainProfileCONFIG_value.maxMotor;};
  }
  halTimer1SetChannelDuty(2,cur_pwm);
  TrainProfile_SetParameter(U_MOTOR_CURRENT,2,&cur_pwm);
};
void setLed(uint16 cur_pwm, bool bt){
  if(bt && cur_pwm!=0){
// led limities    
    if(trainProfileCONFIG_value.maxLed>0 && cur_pwm>trainProfileCONFIG_value.maxLed){cur_pwm=trainProfileCONFIG_value.maxLed;};
  }
  halTimer1SetChannelDuty(1,cur_pwm);
};




static void performPeriodicTask( void ) //80ms - 125ticks for 10sec
{
  // PROXIMITY UPDATE
  if( trainProfileCONFIG_value.adc1_max!=0 || trainProfileCONFIG_value.adc2!=0 ){
    HalAdcSetReference( HAL_ADC_REF_AVDD );
    
    if(trainProfileCONFIG_value.adc1_max!=0){
      if(trainProfileCONFIG_value.enLed1!=0){
        int16 withOutLed=HalAdcRead( PORT_ADC_WALL, HAL_ADC_RESOLUTION_10 );
        PORT_GPIO_WALL=1;
        trainProfilePROXADC1_value=HalAdcRead( PORT_ADC_WALL, HAL_ADC_RESOLUTION_10 );
        PORT_GPIO_WALL=0;
        trainProfilePROXADC1_value-=withOutLed;
      }else{
        trainProfilePROXADC1_value=HalAdcRead( PORT_ADC_WALL, HAL_ADC_RESOLUTION_10 );
      }
      if(trainProfilePROXADC1_value>trainProfileCONFIG_value.adc1_max){
        setMotor(0,false);
      }else{
        setMotor(trainProfileMOTOR_PWM_value,true);
      }
    }

    
    if(curState==GAPROLE_CONNECTED){
      // update BT values;
      if(prox_update>12*5){
        TrainProfile_SetParameter(U_PROXADC1, 2,&trainProfilePROXADC1_value);
        TrainProfile_SetParameter(U_PROXADC2, 2,&trainProfilePROXADC2_value);
        prox_update=0;
      } else prox_update++;
    }
  }
  // BATTERY UPDATE 
  if(battary_update>250){// Update battary level every 20 seconds
    battary_update=0;
    HalAdcSetReference( HAL_ADC_REF_125V ); //1.20V
    
    *(uint16*)temp=HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_10 );
    TrainProfile_SetParameter(U_BATT_ADC, 2,&temp);
    
    *(float*)&temp[4]=getVolt(*(uint16*)temp);
    TrainProfile_SetParameter(U_BATT_VOLT, sizeof(float),&temp[4]);
    
    temp[0]=getPerc(*(float*)&temp[4]);
    TrainProfile_SetParameter(U_BATT, 1,&temp);
  } else battary_update++;
}
static void trainProfileChangeCB( uint8 paramID )
{
  printText("trainProfileChangeCB\r\n");

  switch( paramID )
  {
  case U_DEV_NAME:{
    osal_snv_write(SH_SNV_RESP,scanRspData[0]+1,scanRspData);
    if(scanRspData[0]>0){
      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, 1+scanRspData[0], (void*)scanRspData );
    }
    else{
      GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, 0, (void*)scanRspData );
    }
    break;}
    
  case U_MOTOR_PWM:{
    setMotor(trainProfileMOTOR_PWM_value,true);
    //battary_update=255;
    break;}
    
  case U_LED_PWM:{
    setLed(trainProfileLED_PWM_value,true);
    //battary_update=255;
    break;}
    
  case U_DEF_CONFIG:

    osal_snv_write(SH_SNV_TRAIN_DEF_CONF,TRAIN_STATIC_CONFIG_LEN,&trainProfileDEF_CONFIG_value);
    TrainProfile_SetParameter(U_CONFIG, TRAIN_OPERATE_CONFIG_LEN,&trainProfileDEF_CONFIG_value);
    
  case U_CONFIG:
    if(trainProfileCONFIG_value.adc1_max==0)TrainProfile_SetParameter(U_PROXADC1, 2,&trainProfileCONFIG_value.adc1_max);
    if(trainProfileCONFIG_value.adc2==0)TrainProfile_SetParameter(U_PROXADC2, 2,&trainProfileCONFIG_value.adc2);
    break;
  case 0xFE:{
    temp[0]=1;
    osal_snv_write(SH_SNV_BOOT,1,temp);
    Onboard_soft_reset();
    break;
  }

  default:
    // should not reach here!
    break;
  }
}

void initTrain(void){
  memset(temp,0,sizeof(temp));
  
  TrainProfile_SetParameter(U_LED_PWM, 2,&temp[0]);
  
  TrainProfile_SetParameter(U_PROXADC1, 2,&temp[0]);
  TrainProfile_SetParameter(U_PROXADC2, 2,&temp[0]);

  osal_snv_read(SH_SNV_TRAIN_DEF_CONF,TRAIN_STATIC_CONFIG_LEN,temp);
  TrainProfile_SetParameter(U_DEF_CONFIG, TRAIN_STATIC_CONFIG_LEN,temp);
  TrainProfile_SetParameter(U_CONFIG, TRAIN_OPERATE_CONFIG_LEN,temp);
  TrainProfile_SetParameter(U_MOTOR_PWM, 2,&trainProfileDEF_CONFIG_value.motorWhenOnTrain);
  setMotor(trainProfileDEF_CONFIG_value.motorWhenOnTrain,true);
  setLed(trainProfileDEF_CONFIG_value.ledWhenOnTrain,true);
  
  // INIT ADC
  HalAdcRead( PORT_ADC_WALL, HAL_ADC_RESOLUTION_10 );
  HalAdcRead( PORT_ADC_GROUND, HAL_ADC_RESOLUTION_10 );
  
}