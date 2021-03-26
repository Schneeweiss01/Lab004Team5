/*
 Western Engineering base code
2021 02 04 E J Porter

 encoder implement
  
 */

 #ifndef ENCODER_H
  #define ENCODER_H 1

//---------------------------------------------------------------------------

#include "Motion.h";

 

 volatile boolean ENC_btLeftEncoderADataFlag;
 volatile boolean ENC_btLeftEncoderBDataFlag;
 volatile boolean ENC_btRightEncoderADataFlag;
 volatile boolean ENC_btRightEncoderBDataFlag;

 volatile boolean ENC_btLeftMotorRunningFlag;
 volatile boolean ENC_btRightMotorRunningFlag;

 volatile uint16_t ENC_vui16LeftEncoderAMissed;
 volatile uint16_t ENC_vui16LeftEncoderBMissed;
 volatile uint16_t ENC_vui16RightEncoderAMissed;
 volatile uint16_t ENC_vui16RightEncoderBMissed;

 
 uint16_t ENC_uiAlpha = 8196;

 volatile int32_t ENC_vi32LeftEncoderARawTime;
 volatile int32_t ENC_vi32LeftEncoderBRawTime;
 volatile int32_t ENC_vi32RightEncoderARawTime;
 volatile int32_t ENC_vi32RightEncoderBRawTime;


 
 int32_t ENC_ui32LeftEncoderAAveTime;
 int32_t ENC_ui32LeftEncoderBAveTime;
 int32_t ENC_ui32RightEncoderAAveTime;
 int32_t ENC_ui32RightEncoderBAveTime;

 uint32_t ENC_ui32LeftEncoderAveTime;
 uint32_t ENC_ui32RightEncoderAveTime;

 volatile int32_t ENC_vi32LeftOdometer;
 volatile int32_t ENC_vi32RightOdometer;

 volatile int32_t ENC_vi32LeftOdometerCompare;
 volatile int32_t ENC_vi32RightOdometerCompare;


 volatile int32_t ENC_vsi32LastTimeLA;
 volatile int32_t ENC_vsi32ThisTimeLA;

void ENC_Calibrate()
{
  
  
  //asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime)); // @ 240mHz clock each tick is ~4nS 



  
}

boolean ENC_ISMotorRunning()
{
  if((ENC_btLeftMotorRunningFlag) && (ENC_btLeftMotorRunningFlag))
  {
    return(1);
  }
  else
  {
     return(0);
  }
}

void ENC_SetDistance(int32_t i32LeftDistance, int32_t i32RightDistance)
{
  
   ENC_vi32LeftOdometerCompare = ENC_vi32LeftOdometer + i32LeftDistance;
   ENC_vi32RightOdometerCompare = ENC_vi32RightOdometer + i32RightDistance;
   ENC_btLeftMotorRunningFlag = true;
   ENC_btRightMotorRunningFlag = true;
   ui8LeftWorkingSpeed = cui8StartingSpeed;
   ui8RightWorkingSpeed = cui8StartingSpeed;

}

//Encoder interrupt service routines - entered every change in in encoder pin H-> L and L ->H
//---------------------------------------------------------------------------------------------
void IRAM_ATTR ENC_isrLeftA()
{
   volatile static int32_t ENC_vsi32LastTime;
   volatile static int32_t ENC_vsi32ThisTime;
  
 
   // if the last interrupts data wasn't collected, count the miss
  if(ENC_btLeftEncoderADataFlag)
  {
    ENC_vui16LeftEncoderAMissed += 1;
    
  }
 
  //how much time elapsed since last interrupt
  
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime )); // @ 240mHz clock each tick is ~4nS 
  ENC_vi32LeftEncoderARawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  ENC_btLeftEncoderADataFlag = true;

  //if both pins are high or low then count down otherwise wheel is going backwards count up
  //odometer reading
  if((digitalRead(ciEncoderLeftA) && digitalRead(ciEncoderLeftB)) || ((digitalRead(ciEncoderLeftA) == 0 && digitalRead(ciEncoderLeftB) == 0)))
  {
    ENC_vi32LeftOdometer -= 1;
  }
  else
  {
    ENC_vi32LeftOdometer += 1;
  }

  
  if(ENC_btLeftMotorRunningFlag)
  {
    if(ENC_vi32LeftOdometer == ENC_vi32LeftOdometerCompare)
    {
      ENC_btLeftMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      digitalWrite(ciMotorLeftA,HIGH);
      digitalWrite(ciMotorLeftB,HIGH);
      digitalWrite(ciMotorRightA,HIGH);
      digitalWrite(ciMotorRightB,HIGH);
      ledcWrite(2,255);
      ledcWrite(1,255);  //stop with braking Left motor 
      ledcWrite(3,255);
      ledcWrite(4,255);  //stop with braking Right motor 
    }
    
  }
  
}

void IRAM_ATTR ENC_isrLeftB()
{
  volatile static int32_t ENC_vsi32LastTime;
  volatile static int32_t ENC_vsi32ThisTime;
  
   // if the last interrupts data wasn't collected, count the miss
  if(ENC_btLeftEncoderBDataFlag)
  {
    ENC_vui16LeftEncoderBMissed += 1;
  }
  //how much time elapsed since last interrupt
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime)); // @ 240mHz clock each tick is ~4nS 
  ENC_vi32LeftEncoderBRawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_btLeftEncoderBDataFlag = true;

  //Left Encoder B is opposite of A so if  both pins are high or low then count up otherwise wheel is going forwards count down
  //odometer reading
  if((digitalRead(ciEncoderLeftA) && digitalRead(ciEncoderLeftB)) || ((digitalRead(ciEncoderLeftA) == 0 && digitalRead(ciEncoderLeftB) == 0)))
  {
    ENC_vi32LeftOdometer += 1;
  }
  else
  {
    ENC_vi32LeftOdometer -= 1;
  }
  if(ENC_btLeftMotorRunningFlag)
  {
    if(ENC_vi32LeftOdometer == ENC_vi32LeftOdometerCompare)
    {
        ENC_btLeftMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      digitalWrite(ciMotorLeftA,HIGH);
      digitalWrite(ciMotorLeftB,HIGH);
      digitalWrite(ciMotorRightA,HIGH);
      digitalWrite(ciMotorRightB,HIGH);
      ledcWrite(2,255);
      ledcWrite(1,255);  //stop with braking Left motor 
      ledcWrite(3,255);
      ledcWrite(4,255);  //stop with braking Right motor 
    }
    
  }
}

void IRAM_ATTR ENC_isrRightA()
{
  volatile static int32_t ENC_vsi32LastTime;
  volatile static int32_t ENC_vsi32ThisTime;

   // if the last interrupts data wasn't collected, count the miss
  if(ENC_btRightEncoderADataFlag)
  {
    ENC_vui16RightEncoderAMissed += 1;
  }
  //how much time elapsed since last interrupt
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime)); // @ 240mHz clock each tick is ~4nS 
  ENC_vi32RightEncoderARawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_btRightEncoderADataFlag = true;

  //Right Encoder A -  if both pins are high or low then count down otherwise wheel is going forwards count up
  //odometer reading
  if((digitalRead(ciEncoderRightA) && digitalRead(ciEncoderRightB)) || ((digitalRead(ciEncoderRightA) == 0 && digitalRead(ciEncoderRightB) == 0)))
  {
    ENC_vi32RightOdometer -= 1;
  }
  else
  {
    ENC_vi32RightOdometer += 1;
  }
  if(ENC_btRightMotorRunningFlag)
  {
    if(ENC_vi32RightOdometer == ENC_vi32RightOdometerCompare)
    {
        ENC_btLeftMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      digitalWrite(ciMotorLeftA,HIGH);
      digitalWrite(ciMotorLeftB,HIGH);
      digitalWrite(ciMotorRightA,HIGH);
      digitalWrite(ciMotorRightB,HIGH);
      ledcWrite(2,255);
      ledcWrite(1,255);  //stop with braking Left motor 
      ledcWrite(3,255);
      ledcWrite(4,255);  //stop with braking Right motor 
    }
    
  }
}

void IRAM_ATTR ENC_isrRightB()
{
  volatile static int32_t ENC_vsi32LastTime;
  volatile static int32_t ENC_vsi32ThisTime;

  
  // if the last interrupts data wasn't collected, count the miss
  if(ENC_btRightEncoderBDataFlag)
  {
    ENC_vui16RightEncoderBMissed += 1;
  }
  //how much time elapsed since last interrupt
  ENC_vsi32LastTime = ENC_vsi32ThisTime;
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime)); // @ 240mHz clock each tick is ~4nS 
  ENC_vi32RightEncoderBRawTime = ENC_vsi32ThisTime - ENC_vsi32LastTime;
  ENC_btRightEncoderBDataFlag = true;

  //Right Encoder B is opposite of A so if both pins are high or low then count up otherwise wheel is going forwards count down
  //odometer reading
  if((digitalRead(ciEncoderRightA) && digitalRead(ciEncoderRightB)) || ((digitalRead(ciEncoderRightA) == 0 && digitalRead(ciEncoderRightB) == 0)))
  {
    ENC_vi32RightOdometer += 1;
  }
  else
  {
    ENC_vi32RightOdometer -= 1;
  }
  if(ENC_btRightMotorRunningFlag)
  {
    if(ENC_vi32RightOdometer == ENC_vi32RightOdometerCompare)
    {
       ENC_btLeftMotorRunningFlag = false;
      ENC_btRightMotorRunningFlag = false;
      digitalWrite(ciMotorLeftA,HIGH);
      digitalWrite(ciMotorLeftB,HIGH);
      digitalWrite(ciMotorRightA,HIGH);
      digitalWrite(ciMotorRightB,HIGH);
      ledcWrite(2,255);
      ledcWrite(1,255);  //stop with braking Left motor 
      ledcWrite(3,255);
      ledcWrite(4,255);  //stop with braking Right motor  
    }
    
  }
}
//---------------------------------------------------------------------------------------------

void ENC_Init()
{
   //set pin modes
  pinMode(ciEncoderLeftA, INPUT_PULLUP);
  pinMode(ciEncoderLeftB, INPUT_PULLUP);
  pinMode(ciEncoderRightA, INPUT_PULLUP);
  pinMode(ciEncoderRightB, INPUT_PULLUP);

   // enable GPIO interrupt on change
  attachInterrupt(ciEncoderLeftA, ENC_isrLeftA, CHANGE);
  attachInterrupt(ciEncoderLeftB, ENC_isrLeftB, CHANGE);
  attachInterrupt(ciEncoderRightA, ENC_isrRightA, CHANGE);
  attachInterrupt(ciEncoderRightB, ENC_isrRightB, CHANGE);

  ENC_btLeftMotorRunningFlag = false;
  ENC_btRightMotorRunningFlag = false;


  //check to see if calibration is in eeprom and retreive
  
  
}

void ENC_Disable()
{
   
   // disable GPIO interrupt on change
  detachInterrupt(ciEncoderLeftA);
  detachInterrupt(ciEncoderLeftB);
  detachInterrupt(ciEncoderRightA);
  detachInterrupt(ciEncoderRightB);
  
}

int32_t ENC_Averaging()
{
    
  int64_t vi64CalutatedAverageTime;
  
   //yn=yn−1⋅(1−α)+xn⋅α  exponentially weighted moving average IIR Filter 65535 = 1

   //Left Enoder A
   
  if(ENC_btLeftEncoderADataFlag)
   {
       
    ENC_btLeftEncoderADataFlag = false;
    
    if (ENC_uiAlpha == 65535 )
    {
      ENC_ui32LeftEncoderAAveTime = ENC_vi32LeftEncoderARawTime;
    }
    else
    {
      vi64CalutatedAverageTime = (int64_t)ENC_ui32LeftEncoderAAveTime * (65535 - ENC_uiAlpha) + ((int64_t)ENC_vi32LeftEncoderARawTime * ENC_uiAlpha);
      ENC_ui32LeftEncoderAAveTime = (int32_t)((vi64CalutatedAverageTime + 32768) / 65536);
   
    }
    if(ENC_ISMotorRunning())
      {
        ENC_ui32LeftEncoderAveTime = ((ENC_ui32LeftEncoderAAveTime + ENC_ui32LeftEncoderBAveTime) * 3)/1000;
      }
  }

     //Left Enoder B
  if(ENC_btLeftEncoderBDataFlag)
   {
    ENC_btLeftEncoderBDataFlag = false;
    
    if (ENC_uiAlpha == 65535 )
    {
      ENC_ui32LeftEncoderBAveTime = ENC_vi32LeftEncoderBRawTime;
    }
    else
    {
      vi64CalutatedAverageTime = (int64_t)ENC_ui32LeftEncoderBAveTime * (65535 - ENC_uiAlpha) + ((int64_t)ENC_vi32LeftEncoderBRawTime * ENC_uiAlpha);
      ENC_ui32LeftEncoderBAveTime = (int32_t)((vi64CalutatedAverageTime + 32768) / 65536);
      
    }
    if(ENC_ISMotorRunning())
      {
        ENC_ui32LeftEncoderAveTime = ((ENC_ui32LeftEncoderAAveTime + ENC_ui32LeftEncoderBAveTime) * 3)/1000;
      }
  }

  
    //Right Enoder A
  if(ENC_btRightEncoderADataFlag)
   {
    ENC_btRightEncoderADataFlag = false;
    
    if (ENC_uiAlpha == 65535 )
    {
      ENC_ui32RightEncoderAAveTime = ENC_vi32RightEncoderARawTime;
    }
    else
    {
      vi64CalutatedAverageTime = (int64_t)ENC_ui32RightEncoderAAveTime * (65535 - ENC_uiAlpha) + ((int64_t)ENC_vi32RightEncoderARawTime * ENC_uiAlpha);
      ENC_ui32RightEncoderAAveTime = (int32_t)((vi64CalutatedAverageTime + 32768) / 65536);
      
    }
    if(ENC_ISMotorRunning())
      {
        ENC_ui32RightEncoderAveTime = ((ENC_ui32RightEncoderAAveTime + ENC_ui32RightEncoderBAveTime) * 3)/1000;
      }
  }

     //Right Enoder B
  if(ENC_btRightEncoderBDataFlag)
   {
    ENC_btRightEncoderBDataFlag = false;
    
    if (ENC_uiAlpha == 65535 )
    {
      ENC_ui32RightEncoderBAveTime = ENC_vi32RightEncoderBRawTime;
    }
    else
    {
      vi64CalutatedAverageTime = (int64_t)ENC_ui32RightEncoderBAveTime * (65535 - ENC_uiAlpha) + ((int64_t)ENC_vi32RightEncoderBRawTime * ENC_uiAlpha);
      ENC_ui32RightEncoderBAveTime = (int32_t)((vi64CalutatedAverageTime + 32768) / 65536);
      
    }
    if(ENC_ISMotorRunning())
      {
        ENC_ui32RightEncoderAveTime = ((ENC_ui32RightEncoderAAveTime + ENC_ui32RightEncoderBAveTime) * 3)/1000;
      }
  }

  
}


void ENC_ClearLeftOdometer()
{
  ENC_vi32LeftOdometer = 0;
 
}


void ENC_ClearRightOdometer()
{
  ENC_vi32RightOdometer = 0;
}




#endif
