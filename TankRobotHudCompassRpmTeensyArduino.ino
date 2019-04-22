/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */
 /*
   This utility is written specifically to work with the Teensy 3.X family of microcontroller development boards
 */

#include <i2c_t3.h>
#include "I2Cdev.h"
#include "EM7180.h"
#include "IMU.h"
#include "Alarms.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"



// Define function pointers for class instances
I2Cdev      *i2c_0;
EM7180      *Sentral_0;
IMU         *imu_0;

// Declare main loop helper functions
void DRDY_Handler();
void FetchEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void FetchSentralData(EM7180* em7180, IMU* IMu, uint8_t sensorNUM);




#include "Teensy_Adafruit_INA219.h"
Teensy_Adafruit_INA219 ina219;


float motorPulsePerRev = 75; // this is specific to your motor hall sensor setup.
//I am using a motor with 1 tick per hall sensor revolution with a 75:1 gear output

int edgeTransitionThreshold = 500; //for converting the analog signal coming from hall sensor to digital through arduino code

 
unsigned int motorA_pulseCounter = 0;
bool motorA_edgeReading;
bool motorA_edgeReadingPrevious = 0;
float motorA_Rpm = 0;
bool motorA_LastSerialOutputZero = 0;


unsigned int motorB_pulseCounter = 0;
bool motorB_edgeReading;
bool motorB_edgeReadingPrevious = 0;
float motorB_Rpm = 0;
bool motorB_LastSerialOutputZero = 0;


char floatToStringBuffer[9];

unsigned long pulseAvgOutputTime = millis();
unsigned long pulseAvgInterval = 200; //ms
float pulseAvgIntervalMinute = 300; // 60,000 / 200 = 300 intervals per minute (used for rpm calculation)


unsigned long batteryReadTime = millis();
unsigned long batteryReadInterval = 100; //ms

unsigned long batteryOutputTime = millis();
unsigned long batteryOutputInterval = 1000; //ms

int batteryAverageCount = 0;

float batteryPowerAverageTotal;
//float batteryPowerAverage;
float batteryLoadVoltageAverageTotal;
//float batteryLoadVoltageAverage;






void setup()
{
  // Assign Indicator LED
  LEDPIN_PINMODE;
  Alarms::IndLEDon();

  // Assign interrupt pin as input
  pinMode(INT_PIN, INPUT);

  // Open serial port
  Serial.begin(115200);
  delay(1000);


   pinMode(A0, INPUT);//Motor B
   pinMode(A1, INPUT);//Motor A
   
  // Instantiate Wire class for I2C
  SENSOR_0_WIRE_INSTANCE.begin(I2C_MASTER, 0x00, I2C_PINS, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(500);
  
  // Instantiate Sentral_0 classes and create function pointers
  i2c_0     = new I2Cdev(&SENSOR_0_WIRE_INSTANCE);                                                                   // Use class instance/function pointer to specify I2C bus (may be more than one)
  Sentral_0 = new EM7180(i2c_0, 0);                                                                                  // Use class instance/function pointer to specify Sentral board (may be more than one)
  imu_0     = new IMU(Sentral_0, 0);                                                                                 // Use class instance/function pointer to specify the Sentral and I2C instances for the IMU calcs

  // Initialize Sentral_0
  #ifdef SERIAL_DEBUG
//    Serial.print("Initializing Sentral_0");
//    Serial.println("");
  #endif

  // Main function to set up Sentral and sensors 
  Sentral_0->initSensors();

  // Give a little time to see startup results
  delay(2000);

  // Boot Complete, blink 5 times
  Alarms::blink_IndLED(2, 40, 5);
  
  // Spreadsheet output column headings when "SERIAL_DEBUG" is not defined in config.h
  #ifndef SERIAL_DEBUG
    //Serial.print("Timestamp, q0, q1, q2, q3, Heading, Pitch, Roll"); Serial.println("");
  #endif

  // Attach adat ready interrupt
  attachInterrupt(INT_PIN, DRDY_Handler, RISING);

  // Set sketch start time
  Start_time = micros();

  ina219.begin(&SENSOR_0_WIRE_INSTANCE);
  //ina219.setCalibration_16V_400mA();
  //ina219.setCalibration_32V_1A();
  ina219.setCalibration_32V_2A();
  
}

 void readMotorRpm()
 {

 int reading;

  //Read Motor A
   reading = analogRead(A1); //read raw value of hall sensor
   if (reading > edgeTransitionThreshold) {
    motorA_edgeReading = 1; //convert it to digital 0,1 form
   }else{
    motorA_edgeReading = 0;
   }
   if(motorA_edgeReadingPrevious==0 && motorA_edgeReading==1) {//check for rising edge
   //if (valA1previous != valA1 ) { //check for transition
    motorA_pulseCounter++;
   }
   motorA_edgeReadingPrevious = motorA_edgeReading;


  //Read Motor B
   reading = analogRead(A0); //read raw value of hall sensor
   if (reading > edgeTransitionThreshold) {
    motorB_edgeReading = 1; //convert it to digital 0,1 form
   }else{
    motorB_edgeReading = 0;
   }
   if(motorB_edgeReadingPrevious==0 && motorB_edgeReading==1) {//check for rising edge
   //if (valA1previous != valA1 ) { //check for transition
    motorB_pulseCounter++;
   }
   motorB_edgeReadingPrevious = motorB_edgeReading;


  if(millis() - pulseAvgOutputTime > pulseAvgInterval){
    pulseAvgOutputTime = millis();
    
    if(!motorA_LastSerialOutputZero || motorA_pulseCounter > 0){
      if(motorA_pulseCounter == 0){
        motorA_LastSerialOutputZero = 1;
      }else{
        motorA_LastSerialOutputZero = 0;
      }
//      Serial.println("--------------");
//      Serial.print("pulseCounterA=");
//      Serial.println(pulseCounterA);
      motorA_Rpm = (float)motorA_pulseCounter / motorPulsePerRev; //convert pulses to rotations
//      Serial.print("pulseCounterA rotations="); 
//      Serial.println(pulseCounterArpm);
      motorA_Rpm = motorA_Rpm * pulseAvgIntervalMinute; //convert rotations minute
      //format float with 3 places and .1 precision
      dtostrf (motorA_Rpm, 3, 1, floatToStringBuffer);

      Serial.print("MA=");
      Serial.print(floatToStringBuffer);
      Serial.print("\n");
      
      //Serial.print("MA=");
      //Serial.print(floatToStringBuffer);
      //Serial.print("\n");

       //delay(20);
    }
    
    if(!motorB_LastSerialOutputZero || motorB_pulseCounter > 0){
      if(motorB_pulseCounter == 0){
        motorB_LastSerialOutputZero = 1;
      }else{
        motorB_LastSerialOutputZero = 0;
      }
//      Serial.println("--------------");
//      Serial.print("pulseCounterA=");
//      Serial.println(pulseCounterA);
      motorB_Rpm = (float)motorB_pulseCounter / motorPulsePerRev; //convert pulses to rotations
//      Serial.print("pulseCounterA rotations="); 
//      Serial.println(pulseCounterArpm);
      motorB_Rpm = motorB_Rpm * pulseAvgIntervalMinute; //convert rotations minute
      //format float with 3 places and .1 precision
      dtostrf (motorB_Rpm, 3, 1, floatToStringBuffer);

      Serial.print("MB=");
      Serial.print(floatToStringBuffer);      
      Serial.print("\n");
      
      //Serial.print("MB=");
      //Serial.print(floatToStringBuffer);      
      //Serial.print("\n");
    }
    motorA_pulseCounter = 0;
    motorB_pulseCounter = 0;
  }
  
}

void readBatterySensor()
{
batteryAverageCount++;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
//float power_mW = 0;

shuntvoltage = ina219.getShuntVoltage_mV();
busvoltage = ina219.getBusVoltage_V();

current_mA = ina219.getCurrent_mA();
//power_mW = ina219.getPower_mW();
batteryPowerAverageTotal += current_mA;

loadvoltage = busvoltage + (shuntvoltage / 1000);
batteryLoadVoltageAverageTotal += loadvoltage;

//Serial.println("readBatterySensor");
//
//Serial.print(" power_mW=");
//Serial.print(power_mW);
//Serial.print(" loadvoltage=");
//Serial.print(loadvoltage);
//
//Serial.print(" batteryPowerAverageTotal=");
//Serial.print(batteryPowerAverageTotal);
//
//Serial.print(" batteryLoadVoltageAverageTotal=");
//Serial.print(batteryLoadVoltageAverageTotal);
//
//Serial.print(" batteryAverageCount=");
//Serial.print(batteryAverageCount);
//Serial.println();
}

void readBatteryLevels()
{
  if(millis() - batteryReadTime > batteryReadInterval){
    batteryReadTime = millis();
    readBatterySensor();
  }
  if(millis() - batteryOutputTime > batteryOutputInterval){
    batteryOutputTime = millis();
    batteryPowerAverageTotal = batteryPowerAverageTotal / batteryAverageCount;
    batteryLoadVoltageAverageTotal = batteryLoadVoltageAverageTotal / batteryAverageCount;
    dtostrf (batteryPowerAverageTotal, 3, 1, floatToStringBuffer);
    
    Serial.print("BP=");
    Serial.print(floatToStringBuffer);
    Serial.print("\n");
    
    Serial.print("BV=");
    Serial.print(batteryLoadVoltageAverageTotal);
    Serial.print("\n");
    
//    Serial.print("BP=");
//    Serial.print(batteryPowerAverageTotal);
//    Serial.print("\n");
//    
//    Serial.print("BV=");
//    Serial.print(batteryLoadVoltageAverageTotal);
//    Serial.print("\n");
    
    batteryAverageCount = 0;
    batteryPowerAverageTotal = 0;
    batteryLoadVoltageAverageTotal = 0;
  }
}

void loop()
{
  // Calculate loop cycle time
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
  
  // Check for user input. Allows the user to execute calibration functions from a serial monitor
  if(Serial.available()) serial_input = Serial.read();
  if (serial_input == 49 && calibratingA[0] < 1) {calibratingA[0] = 512;;}                                           // Type "1" to initiate Sentral_0 Accel Cal
  if (serial_input == 50 && calibratingA[0] < 1) {Sentral_0->Save_Sentral_WS_params();}                              // Type "2" to initiate Sentral_0 "Warm Start" parameter save
  serial_input = 0;

  // See what data are available from the Sentral by polling the Status register
  if(drdy == 1)
  {
    drdy = 0;
    FetchEventStatus(i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly Sentral instance 0)
  }

  // Acquire data the Sentral
  FetchSentralData(Sentral_0, imu_0, 0);                                                                             // Sentral instance 0, IMU calculation instance 0 and Sensor instance 0
  
  // Update serial
  delt_t = millis() - last_refresh;
  if (delt_t > UPDATE_PERIOD)                                                                                        // Update the serial monitor every "UPDATE_PERIOD" ms
  {
    last_refresh = millis();
    #ifdef SERIAL_DEBUG
      
      // Algorithm status
//      Serial.print("Algorithm Status = "); Serial.print(algostatus[0]); Serial.println(""); Serial.println("");

       // Sentral_0 sensor and raw quaternion outout
//      Serial.print("ax_0 = "); Serial.print((int)(1000.0f*accData[0][0])); Serial.print(" ay_0 = "); Serial.print((int)(1000.0f*accData[0][1]));
//      Serial.print(" az_0 = "); Serial.print((int)(1000.0f*accData[0][2])); Serial.println(" mg");
//      Serial.print("gx_0 = "); Serial.print(gyroData[0][0], 2); Serial.print(" gy_0 = "); Serial.print(gyroData[0][1], 2); 
//      Serial.print(" gz_0 = "); Serial.print(gyroData[0][2], 2); Serial.println(" deg/s"); Serial.println("");
//      Serial.print("mx_0 = "); Serial.print((int)magData[0][0]); Serial.print(" my_0 = "); Serial.print((int)magData[0][1]);
//      Serial.print(" mz_0 = "); Serial.print((int)magData[0][2]); Serial.println(" uT"); Serial.println("");
//      Serial.println("Sentral_0 Quaternion (NED):"); Serial.print("Q0_0 = "); Serial.print(qt[0][0]);
//      Serial.print(" Qx_0 = "); Serial.print(qt[0][1]); Serial.print(" Qy_0 = "); Serial.print(qt[0][2]); 
//      Serial.print(" Qz_0 = "); Serial.print(qt[0][3]); Serial.println(""); Serial.println("");
      
      // Euler angles
//      Serial.print("Sentral_0 Yaw, Pitch, Roll: ");
//      Serial.print(heading[0], 2); Serial.print(", "); Serial.print(angle[0][1], 2); Serial.print(", "); Serial.println(angle[0][0], 2);
//      Serial.println("");
      
      Serial.print("DY="); Serial.print(heading[0], 2);Serial.print("\n");
      Serial.print("DP="); Serial.print(angle[0][1], 2);Serial.print("\n");
      Serial.print("DR="); Serial.print(angle[0][0], 2);Serial.print("\n");

      // Temperature and pressure
//      Serial.print("Baro Pressure: "); Serial.print(pressure[0], 2); Serial.print(" mbar"); Serial.println("");
//      Serial.print("Baro Temperature: "); Serial.print(temperature[0], 2); Serial.print(" deg C"); Serial.println("");

      Serial.print("TP="); Serial.print(temperature[0], 2);Serial.print("\n");
      
//      Serial.println("");
//
//      // Loop cycle time
//      Serial.print("Loop Cycletime:"); Serial.print(cycleTime); Serial.println(" us"); Serial.println("");
//
//      // Hotkey messaging
//      if(calibratingA[0] < 1)
//      {
//        Serial.println("Send '1' for Sentral_0 Accel Cal");
//        Serial.println("Make sure the desired accelerometer axis is properly aligned with gravity and remains still");
//        Serial.println("All three accelerometers must to be calibrated in the +/-1g condition for accurate results");
//      }
//      Serial.println("Send '2' to save Sentral_0 Warm Start params");
//      Serial.println("");
    #endif

    // Spreadsheet output when "SERIAL_DEBUG" is not defined in config.h
    #ifndef SERIAL_DEBUG
//      Serial.print(TimeStamp, 2);              Serial.print(","); Serial.print(qt[0][0], 2);         Serial.print(","); Serial.print(qt[0][1], 2);         
//      Serial.print(",");                       Serial.print(qt[0][2], 2);  Serial.print(",");        Serial.print(qt[0][3], 2); Serial.print(",");
//      Serial.print(heading[0], 2);             Serial.print(","); Serial.print(angle[0][1], 2);      Serial.print(","); Serial.print(angle[0][0], 2);
//      Serial.println("");
    #endif

    // Toggle LED unless calibrating accelerometers
    if(algostatus[0] < 8)
    //if(calibratingA[0] < 1)
    {
      Serial.print("CS=0\n");
      Alarms::toggle_IndLED();
    } else
    {
      Alarms::IndLEDoff();
    }
  }

 readMotorRpm();
 readBatteryLevels();
  
}

void DRDY_Handler()
{
  drdy = 1;
}

void FetchEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
    eventStatus[sensorNUM] = i2c_BUS->readByte(EM7180_ADDRESS, EM7180_EventStatus);
    if(eventStatus[sensorNUM] & 0x04) Quat_flag[sensorNUM] = 1;
    if(eventStatus[sensorNUM] & 0x20) Gyro_flag[sensorNUM] = 1;
    if(eventStatus[sensorNUM] & 0x10) Acc_flag[sensorNUM]  = 1;
    if(eventStatus[sensorNUM] & 0x08) Mag_flag[sensorNUM]  = 1;
    if(eventStatus[sensorNUM] & 0x40) Baro_flag[sensorNUM] = 1;
    algostatus[sensorNUM] = i2c_BUS->readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
}

void FetchSentralData(EM7180* em7180, IMU* IMu, uint8_t sensorNUM)
{
  if(Gyro_flag[sensorNUM] == 1)
  {
    em7180->Gyro_getADC();
    for(uint8_t i=0; i<3; i++)
    {
      gyroData[sensorNUM][i] = (float)gyroADC[sensorNUM][i]*DPS_PER_COUNT;
    }
    Gyro_flag[sensorNUM] = 0;
  }
  if(Quat_flag[sensorNUM] == 1)
  {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }
  if(Acc_flag[sensorNUM])
  {
    em7180->ACC_getADC();
    em7180->ACC_Common();
    for(uint8_t i=0; i<3; i++)
    {
      accData[sensorNUM][i] = (float)accADC[sensorNUM][i]*G_PER_COUNT;
      LINaccData[sensorNUM][i] = (float)accLIN[sensorNUM][i]*G_PER_COUNT;
    }
    Acc_flag[sensorNUM] = 0;
  }
  if(Mag_flag[sensorNUM])
  {
    em7180->Mag_getADC();
    for(uint8_t i=0; i<3; i++)
    {
      magData[sensorNUM][i] = (float)magADC[sensorNUM][i]*SENTRAL_UT_PER_COUNT;
    }
    Mag_flag[sensorNUM] = 0;
  }
  if(Baro_flag[sensorNUM])
  {
    rawPressure[sensorNUM]    = em7180->Baro_getPress();
    pressure[sensorNUM]       = (float)rawPressure[sensorNUM]*0.01f +1013.25f;                                       // Pressure in mBar
    rawTemperature[sensorNUM] = em7180->Baro_getTemp();
    temperature[sensorNUM]    = (float)rawTemperature[sensorNUM]*0.01;                                               // Temperature in degrees C
    Baro_flag[sensorNUM]      = 0;
  }
}
