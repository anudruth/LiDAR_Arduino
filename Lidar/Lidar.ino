
/* Includes ------------------------------------------------------------------*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <vl53l1x_x_nucleo_53l1a1_class.h>
#include <stmpe1600_class.h>

#define DEV_I2C Wire
#define SerialPort Serial

//For AVR compatibility where D8 and D2 are undefined
#ifndef D8
#define D8 8
#endif

#ifndef D2
#define D2 2
#endif

#define interruptPin A2

volatile int interruptCount;
int count ;
int state; //0 = forward, 1 = backward

// Components.
STMPE1600DigiOut *xshutdown_top;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_top;
Adafruit_MotorShield *AFMS;
Adafruit_StepperMotor *myMotor;



void measure()
{
  interruptCount = 1;
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
  

  // Initialize motor driver and get motor
  AFMS = new Adafruit_MotorShield();
  myMotor = AFMS->getStepper(513, 2);

  AFMS->begin();

  // Create VL53L1X top component.
  xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
  sensor_vl53l1_top = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_top, A2);

  // Switch off VL53L1X top component.
  sensor_vl53l1_top->VL53L1_Off();


  // Initialize VL53L1X top component.
  status = sensor_vl53l1_top->InitSensor(0x10);
  if (status)
  {
    SerialPort.println("Init sensor_vl53l1x_top failed...");
  }

  //Change timing budget to the minimum consented by the long range mode (20ms)
  status = sensor_vl53l1_top->VL53L1X_SetTimingBudgetInMs(20);
  if ( status )
  {
    SerialPort.println("SetMeasurementTimingBudgetMicroSeconds top sensor failed");
  }
  status = sensor_vl53l1_top->VL53L1X_SetInterMeasurementInMs(20);
  if ( status )
  {
    SerialPort.println("SetInterMeasurementPeriodMilliSeconds top sensor failed");
  }

  //Start measurement
  sensor_vl53l1_top->VL53L1X_StartRanging();

  //SerialPort.println("Setup complete");
}


void reset(){
  count = 0;
  state = 0;
}

void output(int distance, int count, int state){
  
}

void loop()
{
  if (interruptCount)
  {

    uint16_t distance;
    int status;

    interruptCount = 0;
    // Led blinking.
    digitalWrite(13, HIGH);

    //read distance
    status = sensor_vl53l1_top->VL53L1X_GetDistance(&distance);
    if ( status )
    {
      SerialPort.println("GetDistance top sensor failed");
    }

    //restart sensor
    status = sensor_vl53l1_top->VL53L1X_ClearInterrupt();
    if ( status )
    {
      SerialPort.println("Restart top sensor failed");
    }

    // Output data.
    char report[64];
    SerialPort.print(state);
    snprintf(report, sizeof(report), "| Distance top [mm]: %d | Count: ", distance);
    SerialPort.print(report);
    
    SerialPort.println(count);


    if ((count < 171) && (state == 0))
    {
      count = count + 1;
      myMotor->onestep(FORWARD, SINGLE);
      if (count == 170) state = 1;
      
      
    }
    else {
      count = count - 1;
      myMotor->onestep(BACKWARD, SINGLE);
      if (count == 0) reset();
      
      
    }

    
    

    //count = (count + 1) % 171;




    digitalWrite(13, LOW);
    
  }
  
}
