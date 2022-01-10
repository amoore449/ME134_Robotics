#include "Adafruit_VL53L0X.h"
#include "MPU9250.h"
#include <Servo.h>

// Declare sensor objects
Servo servo1;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU9250 mpu; // You can also use MPU9255 as is

// Declare variables
static const int servoPin = 13;
int li_dist;
int kp = 2;
float curr_angle;
float prev_angle;
float prev_pos;
float set_point;
float eps = 0.01;

void setup() {
    // Motor Setup
    servo1.attach(servoPin);
    servo1.write(0);
    Serial.begin(115200);
    
    // Lidar Setup
    if (!lox.begin()) {
      Serial.println(F("Failed to boot VL53L0X"));
      while(1);
    }

    // IMU Setup
    Wire.begin();
    delay(2000);
    mpu.setup(0x68);  // change to your own address
    
    curr_angle = 0;
    prev_angle = 0;
    prev_pos = 0;
    servo1.write(0);
}

void loop() {
    // Read from IMU
    if (mpu.update()) {
        curr_angle = mpu.getPitch();
        Serial.print(mpu.getYaw()); Serial.print(", ");
        Serial.print(curr_angle); Serial.print(", ");
        Serial.println(mpu.getRoll());
    }

    // Get average error of previous two values
    float avg_angle = (curr_angle + prev_angle) / 2;
    float motor_pos = 90 - kp * (set_point - avg_angle);
    servo1.write(motor_pos);    
    
    // Lidar Reading
    VL53L0X_RangingMeasurementData_t measure;
      
    Serial.print("Reading a LIDAR measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      li_dist = measure.RangeMilliMeter;
      Serial.print("Distance (mm): "); Serial.println(li_dist);
    } else {
      Serial.println(" Lidar out of range ");
    }
    
    prev_angle = curr_angle;
    prev_pos = motor_pos;
}
