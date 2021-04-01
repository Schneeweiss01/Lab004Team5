
/* MSE 2202 
 * Western Engineering Design Project Code
 * By: Benjamin Schneeweiss, Annabelle Pundaky, Evan Michaelson, Harrison Angellotti
 * First Version of code that drives Robot straight, then turns left, then straight, then turns left, then stops
 * Still not Included: Have not included Rope Climbing Mechanism, have not adjusted wheel speeds
 * 
 * v2 cleaned through some code that was required in lab but not for final project
*/

/*
  esp32                                           MSE-DuinoV2
  pins         description                        Brd Jumpers /Labels                                                                  User (Fill in chart with user PIN usage) 
  1             3v3                               PWR 3V3                                                                              3V3
  2             gnd                               GND                                                                                  GND
  3             GPIO15/AD2_3/T3/SD_CMD/           D15 (has connections in both 5V and 3V areas)                    
  4             GPIO2/AD2_2/T2/SD_D0              D2(has connections in both 5V and 3V areas)  /INDICATORLED ( On ESP32 board )        Heartbeat LED
  5             GPIO4/AD2_0/T0/SD_D1              D4(has connections in both 5V and 3V areas)                                          Left Motor, Channel A
  6             GPIO16/RX2                        Slide Switch S1b                                                                     IR Receiver
  7             GPIO17/TX2                        Slide Switch S2b                                                                     Left Encoder, Channel A
  8             GPIO5                             D5 (has connections in both 5V and 3V areas)                                         Left Encoder, Channel B
  9             GPIO18                            D18 (has connections in both 5V and 3V areas)                                        Left Motor, Channel B
  10            GPIO19/CTS0                       D19 (has connections in both 5V and 3V areas)                                        Right Motor, Channel A
  11            GPIO21                            D21/I2C_DA  
  12            GPIO3/RX0                         RX0
  13            GPIO1//TX0                        TX0
  14            GPIO22/RTS1                       D22/I2C_CLK
  15            GPIO23                            D23 (has connections in both 5V and 3V areas)  
  16            EN                                JP4 (Labeled - RST) for reseting ESP32
  17            GPI36/VP/AD1_0                    AD0                   
  18            GPI39/VN/AD1_3/                   AD3
  19            GPI34/AD1_6/                      AD6
  20            GPI35/AD1_7                       Potentiometer R2 / AD7
  21            GPIO32/AD1_4/T9                   Potentiometer R1 / AD4                                                               Pot 1 (R1)
  22            GPIO33/AD1_5/T8                   IMon/D33  monitor board current
  23            GPIO25/AD2_8/DAC1                 SK6812 Smart LEDs / D25                                                              Smart LEDs
  24            GPIO26/A2_9/DAC2                  Push Button PB2                                                                      Limit switch
  25            GPIO27/AD2_7/T7                   Push Button PB1                                                                      PB1
  26            GPOP14/AD2_6/T6/SD_CLK            Slide Switch S2a                                                                     Right Encoder, Channel A
  27            GPIO12/AD2_5/T5/SD_D2/            D12(has connections in both 5V and 3V areas)                                         Right Motor, Channel B
  28            GPIO13/AD2_4/T4/SD_D3/            Slide Switch S1a                                                                     Right Encoder, Channel B
  29            GND                               GND                                                                                  GND
  30            VIN                               PWR 5V t 7V                                                                          PWR 5V to 7V
*/


//Pin assignments
const int ciHeartbeatLED = 2;
const int ciPB1 = 27;     
const int ciPB2 = 26;      
const int ciPot1 = A4;    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12; 
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciSmartLED = 25;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;

volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);
//Setup for needed timers and delays
const int CR1_ciMainTimer =  10000;
const int CR1_ciHeartbeatInterval = 500;
const int CR1_ciMotorRunTime = 1000;
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

//Setup for turns
const uint8_t ci8RightTurn = 18;
const uint8_t ci8LeftTurn = 17;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

//Setup variables used for speed control
uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

//Setup for all other variables
uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number 
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

   int state = 0; //new variable, should be 0 when trying to go forwards, 1 when trying to go backwards, 2 when ready to commence the finale
   unsigned long breakTimer=0;//VARIABLE FOR TIMER

void setup() {
   Serial.begin(115200); 
   
   Core_ZEROInit();

   WDT_EnableFastWatchDogCore1();
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[0] = 0;
   WDT_vfFastWDTWarningCore1[1] = 0;
   WDT_vfFastWDTWarningCore1[2] = 0;
   WDT_vfFastWDTWarningCore1[3] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[4] = 0;
   WDT_vfFastWDTWarningCore1[5] = 0;
   WDT_vfFastWDTWarningCore1[6] = 0;
   WDT_vfFastWDTWarningCore1[7] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[8] = 0;
   WDT_vfFastWDTWarningCore1[9] = 0;
   WDT_ResetCore1(); 

   setupMotion();
   pinMode(ciHeartbeatLED, OUTPUT);
   pinMode(ciPB1, INPUT_PULLUP);

   SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
   SmartLEDs.clear();                          // Set all pixel colours to off
   SmartLEDs.show();                           // Send the updated pixel colours to the hardware

}

void loop()
{

   //average the encoder tick times
   ENC_Averaging();//Essentially used as an indication of the speed of the motor

//*************************************************************************************************
  //Code for pressing button to turn on/off the robot
  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
     CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
     state = 0;
     CR1_ulMotorTimerPrevious=millis();
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
    iButtonState = iButtonValue;               // update current button state

     // only toggle the run condition if the new button state is LOW
     if (iButtonState == LOW)
     {
       ENC_ClearLeftOdometer();//restart odometer if robot is reset
       ENC_ClearRightOdometer();
       btRun = !btRun;
        Serial.println(btRun);
       // if stopping, reset motor states and stop motors
       if(!btRun)
       {
          ucMotorStateIndex = 0; 
          ucMotorState = 0;
          move(0);
       }
      
     }
   }
 }
 iLastButtonState = iButtonValue;             // store button state
 
//Starts a timer 
 CR1_ulMainTimerNow = micros();
 if(CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
 {
   WDT_ResetCore1(); 
   WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;
   
   CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;
//*************************************************************************************************    
       
      //read pot 1 for motor speeds moved to here
      CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);  // adjust to range that will produce motion (starts at 130 not 0 to produce a real speed and eliminate deadzone)
      
      CR1_ucMainTimerCaseCore1 = 2;//will skip repeating this 


      
      if (btRun)                                                                              //If the button the button changes states, the robot will now run
      {
        
        if (state == 0)                                                                       //To begin navigation around the obstacle the robot will first move forwardss
        {
          CR1_ulMotorTimerNow=millis();                                                       //Start a timer
           if( CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <=3750)//start correction       //Compare timer to another time that began at the beggining of the robot start. This ensure the robot drives straight for X seconds
            {
            ENC_SetDistance(100, 100);
            ucMotorState = 1;               //This motor state corresponds to a switch statement in motion.h that tells the robot to move forward (by telling which motors to move, and in which direction to move them at what speed)
            breakTimer=millis();
            }
            else                                                                              //Once enough time has passed
            {
             CR1_ulMotorTimerNow=millis();                                                    //reset the timer
             ucMotorState = 0;                                                                //Make the robot use the 'breaks' (stop moving forward)
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)                                      //Don't move for X seconds
              {
                CR1_ulMotorTimerPrevious=millis();                                            //reset old timer so that the next state can compare how long it has been occuring for
                //The following two lines are only neccesary if the encoders are used (instead of hardcoding with time)
                ENC_vi32RightOdometer = 0;                                                    //reset odometer
                ENC_vi32LeftOdometer = 0;                                                     //reset odometer
                state = 1;
              }
            }
          }
          
          else if(state ==1)                                                                  //The robot will now have to turn right
          {
            CR1_ulMotorTimerNow=millis();                                                     //Poll a new timer
            if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <= 800)                         //Continue to turn for X seconds 
          {
            ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);                                     //set the distance for the turning phase
            ucMotorState = 3;                                                                 //Make the correct wheels spin for robot to turn right
            breakTimer=millis();                                                              //start a timer for the breaks
          }
          else                                                                                //Like in case 1 this code simply makes the robot 'breaks' for a certain amount of time
          {
            CR1_ulMotorTimerNow=millis();
             ucMotorState = 0;//if working try 5 to see if it hard breaks
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)
              {
                ENC_vi32RightOdometer = 0;                                                    //reset odometer (if using odometers)
                ENC_vi32LeftOdometer = 0;                                                     //reset odometer
                state = 2;                                                                    //move onto next stage
              }
            }
          }
          if (state == 2)                                                                     //Move forward again
        {
          CR1_ulMotorTimerNow=millis();                                                        //start a timer
           if( CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <=5100)                          //poll a timer to ensure robot drives forward for correct duration
            {
            ENC_SetDistance(100, 100);                                                        //sets the correct distance
            ucMotorState = 1;                                                                 //wheels rotate in correct direction to drive forward
            breakTimer=millis();                                                              //starts a timer for the breaks
            }
            else                                                                              //Use the 'breaks'                                                            
            {
             CR1_ulMotorTimerNow=millis();
             ucMotorState = 0;                                                                //motors should lock so that robot uses 'breaks'
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)                                      //poll time to ensure it waits 0.25 s
              {
                CR1_ulMotorTimerPrevious=millis();
                ENC_vi32RightOdometer = 0;                                                    //reset odometer
                ENC_vi32LeftOdometer = 0;                                                     //reset odometer
                state = 3;
              }
            }
            
            //if not state 0,
          }
          else if(state ==3)                                                                  //Once again turn Right
          {
            CR1_ulMotorTimerNow=millis();                                                     //Begin timer for polling
            if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <= 780)                         //Poll timer to ensure robot moves turns correct ammount
          {
            ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);
            ucMotorState = 2;                                                                 //Use the correct motors to turn right
            breakTimer=millis();                                                              //Start a timer for the breaks
          }
          else
          {
            CR1_ulMotorTimerNow=millis();                                                     //Concurrent timer used for polling
             ucMotorState = 0;                                                                //motors should lock so that robot uses 'breaks'
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)
              {
                ENC_vi32RightOdometer = 0;                                                    //reset odometer
                ENC_vi32LeftOdometer = 0;                                                     //reset odometer
                state = 4;                                                                    //Move on to case 4
              }
            }
          }
          if (state == 4)                                                                     //Move forward one last time
        {
          CR1_ulMotorTimerNow=millis();                                                       //concurrent timer for polling
           if( CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <=3750)                         //poll to ensure robot moves forward correct distance
            {
            ENC_SetDistance(100, 100);
            ucMotorState = 1;                                                                 //makes both wheels spin forwards so robot moves forward
            breakTimer=millis();                                                              //starts a timer for polling used in breaking sequence
            //Insert code for rope mechanism to start spinning
            }
            else                                                                              //in this case the breaks are not used because the robot will be pulling up the rope
            {
                state = 5;
              }
            }
            if (state ==5)                                                                    //this case is the ascension phase
            {
              //add in code for climbing up rope
              //add in code for whisker switch detection
              //start timer for when whisker switch leaves floor
            }
        }

      if(ENC_ISMotorRunning())                                                                //If the motor is running
      {
        MoveTo(ucMotorState, CR1_ui8LeftWheelSpeed,CR1_ui8RightWheelSpeed);                   //send the motor state (which direction its driving) as well as the wheel speeds to the MoveTo function in motion.h that writes the speed
                                                                                               //and directions to the motors
      }
      
    }
  }
 }
