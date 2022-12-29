/**
  ******************************************************************************
  *
  * @author  Leonardo Cavagnis
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdint.h>

#include <SoftwareSerial.h>
#define tx 3
#define rx 2
SoftwareSerial bt(rx, tx); //RX, TX

#include "neuton.h"

/* Private define ------------------------------------------------------------*/
#define NUM_SAMPLES         50
#define Gravity             9.80665f
#define ACC_THRESHOLD       (2.34f*Gravity)    // threshold of significant in G's
#define SIZE  301 // 6 measurements (a,g)/sample + target

/* Private variables ---------------------------------------------------------*/
int   samplesRead   = NUM_SAMPLES;
float inputs[SIZE]  = {0};
float percentage;
int x;
Adafruit_MPU6050 mpu;

void setup() {
  // init serial port
  Serial.begin(115200);
  bt.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // init IMU sensor
  if (!mpu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(10);
    }
  }

  // configure IMU sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

void loop() {
  
  sensors_event_t a, g, temp;
  
  // wait for significant motion
  while (samplesRead == NUM_SAMPLES) {
    // read the acceleration data
    mpu.getEvent(&a, &g, &temp);
    
    // sum up the absolutes
    float acc_Sum = fabs(a.acceleration.x) + fabs(a.acceleration.y) + fabs(a.acceleration.z);
    
    // check if it's above the threshold
    if (acc_Sum >= ACC_THRESHOLD) {
      // reset the sample read count
      samplesRead = 0;
      break;
    }
  }

  // read samples of the detected motion
  while (samplesRead < NUM_SAMPLES) {
    // read the acceleration and gyroscope data
    mpu.getEvent(&a, &g, &temp);

    // fill the array (model input)
    inputs[samplesRead*6 + 0] = a.acceleration.x;
    inputs[samplesRead*6 + 1] = a.acceleration.y;
    inputs[samplesRead*6 + 2] = a.acceleration.z;
    inputs[samplesRead*6 + 3] = g.gyro.x;
    inputs[samplesRead*6 + 4] = g.gyro.y;
    inputs[samplesRead*6 + 5] = g.gyro.z;
    
    samplesRead++;
    
    delay(10);

    // check the end of gesture acquisition
    if (samplesRead == NUM_SAMPLES) {

      // set model inputs
      neuton_model_set_inputs(inputs);
      
      // run model inference
      uint16_t predictedClass;
      float* probabilities;
      
      int outputValue = neuton_model_run_inference(&predictedClass, &probabilities);

      // check if model inference result is valid
      if (outputValue == 0) {
          percentage = probabilities[predictedClass];
          switch(predictedClass) {
            case 0:
                
                if (percentage > 0.6) {
                    bt.print(predictedClass);
                    bt.print(",");
                    bt.print(percentage);
                    bt.print(",");
                    bt.println("FLEX");
                    
                }
                break;
            case 1:
                
                if (percentage > 0.6) {
                    bt.print(predictedClass);
                    bt.print(",");
                    bt.print(percentage);
                    bt.print(",");
                    bt.println("PUNCH");
                  
                }
                break;
            default:
                 bt.println("NONE");
                break;
          }
      } else {
        bt.println("Inference fail to execute");
      }
    }
  }
}
