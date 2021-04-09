/* MSE 2202 
 * Western Engineering Design Project Code
 * By: Benjamin Schneeweiss, Annabelle Pundaky, Evan Michaelson, Harrison Angellotti
 * Evan's Branch
*/

//The code below assigns all pins to the name of device that is connected
const int ciPB1 = 27;         
const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12; 
const int ciEncoderLeftA = 17;
const int ciEncoderLeftB = 5;
const int ciEncoderRightA = 14;
const int ciEncoderRightB = 13;
const int ciStepperMotorDir = 22;
const int ciStepperMotorStep = 21;
const int motorSpinner = 23;
const int switchPin = 16;

//Declare variables to be used in program
int switchState;                              //used to store if limit switch is high or low
int state = 0;                                //Controls motion of robot
unsigned long breakTimer=0;                   //VARIABLE FOR TIMER
unsigned long CR1_ulMotorTimerPrevious;       //Used to determine when an event started
unsigned long CR1_ulMotorTimerNow;            //used for polling of how long event has occured for           
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;


volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

//Include all header files needed to run code

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
unsigned long CR1_ulMainTimerPrevious;        
unsigned long CR1_ulMainTimerNow;  

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

unsigned char ucMotorStateIndex = 0;

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;

void setup() {
   Serial.begin(115200); 
   setupMotion();
   pinMode(ciPB1, INPUT_PULLUP);                //Set pushbutton 1 to input and use pullup resistor
   pinMode(motorSpinner, OUTPUT);               //setup the motor spinner (rope climbing mechanism motor) as an output
   pinMode(switchPin, INPUT_PULLUP);            //setup the limit switch as an input and use pullup resistor


   //Setup of Variables not covered in class. These variables are used in the code for 0_Core_Zero.h and WDT.h
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
}

void loop()
{
  //Code for pressing button to turn on/off the robot
  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
     CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
     state = 0;                               //reset the bot state
     digitalWrite(motorSpinner, LOW);         //reset the motor
     CR1_ulMotorTimerPrevious=millis();       //reset timer
  }

  if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
    iButtonState = iButtonValue;               // update current button state

     // only toggle the run condition if the new button state is LOW
     if (iButtonState == LOW)
     {
       btRun = !btRun;
        Serial.println(btRun);
       if(!btRun)
       {                                      //If the robot is not running, it should not be moving
          ucMotorStateIndex = 0; 
          ucMotorState = 0;
          move(0);
       }
      
     }
   }
 }
 iLastButtonState = iButtonValue;             // store button state onto last button state
 
//The code below is used in many of the header files. We did not discuss these lines in class and so we have opted to not comment on them, or remove them in case
   ENC_Averaging();//Essentially used as an indication of the speed of the motor
 CR1_ulMainTimerNow = micros();
 if(CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
 {
   WDT_ResetCore1(); 
   WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;
   
   CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;
   CR1_ucMainTimerCaseCore1 = 2;//will skip repeating this 
//*******************************************************************

      if (btRun)                                                                              //If the button the button changes states, the robot will now run
      {
        
        if (state == 0)                                                                       //To begin navigation around the obstacle the robot will first move forwardss
        {
          CR1_ulMotorTimerNow=millis();                                                       //Start a timer
           if( CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <=2000)//start correction       //Compare timer to another time that began at the beggining of the robot start. This ensure the robot drives straight for X seconds
            {
            ENC_SetDistance(100, 100);
            //set directions and speed of each motor
             ledcWrite(2,0);
             ledcWrite(1, 180);
             ledcWrite(4,0);
             ledcWrite(3, 170); 

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
                state = 1;                                                                    //move to next state
              }
            }
          }
          
          else if(state ==1)                                                                  //The robot will now have to turn right
          {
            CR1_ulMotorTimerNow=millis();                                                     //Poll a new timer
            if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <= 650)                        //Continue to turn for X seconds 
          {
            ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);                                     //set the distance for the turning phase
          //Set the motors in the correct direction with correct speed
          ledcWrite(1,0);
          ledcWrite(2,170);
          ledcWrite(4,0);
          ledcWrite(3,170);
            breakTimer=millis();                                                              //start a timer for the breaks
          }
          else                                                                                //Like in case 1 this code simply makes the robot 'breaks' for a certain amount of time
          {
            CR1_ulMotorTimerNow=millis();
             ucMotorState = 0;//if working try 5 to see if it hard breaks
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)
              {
                state = 2;                                                                    //move onto next stage
              }
            }
          }
          else if (state == 2)                                                                //Move forward again
        {
          CR1_ulMotorTimerNow=millis();                                                       //start a timer
           if( CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <=2500)                         //poll a timer to ensure robot drives forward for correct duration
            {
            ENC_SetDistance(100, 100);                                                        //sets the correct distance
            //Set the motors in the correct direction with correct speed
             ledcWrite(2,0);
             ledcWrite(1, 180);
             ledcWrite(4,0);
             ledcWrite(3, 170);
            breakTimer=millis();                                                              //starts a timer for the breaks
            }
            else                                                                              //Use the 'breaks'                                                            
            {
             CR1_ulMotorTimerNow=millis();
             ucMotorState = 0;                                                                //motors should lock so that robot uses 'breaks'
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)                                      //poll time to ensure it waits 0.25 s
              {
                CR1_ulMotorTimerPrevious=millis();                                            //check time
                state = 3;                                                                    //move to next state
              }
            }
            
            //if not state 0,
          }
          else if(state ==3)                                                                  //Once again turn Right
          {
            CR1_ulMotorTimerNow=millis();                                                     //Begin timer for polling
            if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious <= 650)                        //Poll timer to ensure robot moves turns correct ammount
          {
            ENC_SetDistance(-(ci8LeftTurn), ci8LeftTurn);                                     //Set wheels for turning
            ledcWrite(1,0);
            ledcWrite(2,170);
            ledcWrite(4,0);
            ledcWrite(3,170);
            breakTimer=millis();                                                              //Start a timer for the breaks
          }
          else
          {
            CR1_ulMotorTimerNow=millis();                                                     //Concurrent timer used for polling
             ucMotorState = 0;                                                                //motors should lock so that robot uses 'breaks'
              move(0);
              if(CR1_ulMotorTimerNow - breakTimer >=250)
              {
                state = 4;                                                                    //Move on to case 4
              }
            }
          }
          else if (state == 4)                                                                //Move forward one last time
        {
          switchState = digitalRead(switchPin);                                               //read wether the limit switch is in contact with ground or not
          digitalWrite(motorSpinner, HIGH);                                                   //Spin rope climbing mechanism
          if (switchState == LOW)                                                             //if still in contact with ground
          {
            ENC_SetDistance(100, 100);                                                        //Set appropriate distance
            //Put motors in correct direction with correct speed to continue driving forward
             ledcWrite(2,0);
             ledcWrite(1, 165);
             ledcWrite(4,0);
             ledcWrite(3, 155);
             Serial.println("low");                                                          //for serial plotter to ensure section is working
          }
          else if (switchState ==HIGH)
          {
             state = 5;                                                                      //move on to next state
             ucMotorState = 0;                                                               //motors should lock so that robot uses 'breaks'
             move(0);
             Serial.println("high");                                                         //trace print
             breakTimer=millis();                                                            //New timer to know how long to climb rope for
             }
          }

         else if (state == 5)
         {
          CR1_ulMotorTimerNow=millis();                                                     //Begin timer for polling
          if (CR1_ulMotorTimerNow-breakTimer >=5500)
          {
            digitalWrite(motorSpinner, LOW);                                                //Spin rope climbing mechanism
          }
          
           Serial.println("if here motor should not spin");
         }
        }
    }
  }
