
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <vl53l1x_x_nucleo_53l1a1_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

//For AVR compatibility where D8 and D2 are undefined
#ifndef D8
#define D8 8
#endif

#ifndef D2
#define D2 2
#endif

#define interruptPin A2

// Components.
STMPE1600DigiOut *xshutdown_top;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_top;
Adafruit_MotorShield *AFMS;
Adafruit_StepperMotor *myMotor;

volatile int interruptCount=0;
volatile int count=0;

void measure()
{
   interruptCount=1;
}

void setup()
{
   VL53L1_Error status;
   // Led.
   pinMode(13, OUTPUT);
   pinMode(interruptPin, INPUT_PULLUP);
   attachInterrupt(interruptPin, measure, RISING);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");
//NOTE: workaround in order to unblock the I2C bus on the Arduino Due
#ifdef ARDUINO_SAM_DUE
   pinMode(71, OUTPUT);
   pinMode(70, OUTPUT);

   for (int i = 0; i<20; i++){
     digitalWrite(70, LOW);
     delay(3);
     digitalWrite(71, HIGH);
     delay(3);
     digitalWrite(70, HIGH);
     delay(3);
     digitalWrite(71, LOW);
     delay(3);
   }
   pinMode(70, INPUT);
   pinMode(71, INPUT);
#endif
//End of workaround


// Initialize motor driver and get motor
   AFMS = new Adafruit_MotorShield();
   myMotor = AFMS->getStepper(513,2);
   
   AFMS->begin();
  
   // Create VL53L1X top component.
   xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
   sensor_vl53l1_top = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_top, A2);

   // Switch off VL53L1X top component.
   sensor_vl53l1_top->VL53L1_Off();


   // Initialize VL53L1X top component.
   status = sensor_vl53l1_top->InitSensor(0x10);
   if(status)
   {
      SerialPort.println("Init sensor_vl53l1x_top failed...");
   }

   //Change timing budget to the minimum consented by the long range mode (20ms)
   status = sensor_vl53l1_top->VL53L1X_SetTimingBudgetInMs(20);
   if( status )
   {
      SerialPort.println("SetMeasurementTimingBudgetMicroSeconds top sensor failed");
   }
   status = sensor_vl53l1_top->VL53L1X_SetInterMeasurementInMs(20);
   if( status )
   {
      SerialPort.println("SetInterMeasurementPeriodMilliSeconds top sensor failed");
   }

   //Start measurement
   sensor_vl53l1_top->VL53L1X_StartRanging();
}

void loop()
{
   if (interruptCount)
   {
      uint16_t distance;
      int status;

      interruptCount=0;
      // Led blinking.
      digitalWrite(13, HIGH);

      //read distance
      status = sensor_vl53l1_top->VL53L1X_GetDistance(&distance);
      if( status )
      {
         SerialPort.println("GetDistance top sensor failed");
      }

      //restart sensor
      status = sensor_vl53l1_top->VL53L1X_ClearInterrupt();
      if( status )
      {
         SerialPort.println("Restart top sensor failed");
      }
      //delay(100);
      myMotor->onestep(FORWARD, SINGLE);
      //delay(100);

      // Output data.
      char report[64];
      snprintf(report, sizeof(report), "| Distance top [mm]: %d | Count: ", distance);
      SerialPort.print(report);
      SerialPort.println(count);

      count = (count + 1) % 171;
      
      //delay(100);
      digitalWrite(13, LOW);
   }
}
