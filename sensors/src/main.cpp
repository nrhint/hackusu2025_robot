// Nathan Hinton
// USU 2025 Hackathon
// Sensors file
// This will take the sensors and then create a fusion of them that will be pushed out to a raspberry pi which will log the data for later processing

// MPU code based on the DMP6 example from the MPU6050 library

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define FRONT_ULTRA_TRIGGER PIN3

MPU6050 mpu;

int const INTERRUPT_PIN = 2; // Define the interruption #0 pin

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

float read_ultrasonic(int trigger, int echo);

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  Serial.begin(115200);
  while (!Serial);

  // Initalize the MPU device

  // Initalize the ultrasonic sensors:
  pinMode(FRONT_ULTRA_TRIGGER, OUTPUT);
  pinMode(FRONT_ULTRA_ECHO, INPUT);
}

unsigned long previous_ultra_millis = 0;
#define ultra_sensor_interval 100
float ultra1_reading = 0;
float ultra2_reading = 0;
float ultra3_reading = 0;

unsigned long previous_report_millis = 0;
#define report_interval 1000

void loop() {
  // Sample the ultrasonic sensor
  unsigned long currentMillis = millis();
  if (currentMillis - previous_ultra_millis >= ultra_sensor_interval) {
    // Save the last time the function was called
    previous_ultra_millis = currentMillis;
    ultra1_reading += read_ultrasonic(FRONT_ULTRA_TRIGGER, FRONT_ULTRA_ECHO);
  }
  if (currentMillis - previous_report_millis >= report_interval) {
    previous_report_millis = currentMillis;
    // Print out lines to pi
    Serial.print((ultra1_reading/(report_interval/ultra_sensor_interval)) + ', ');
    Serial.print((ultra2_reading/(report_interval/ultra_sensor_interval)) + ', ');
    Serial.print((ultra3_reading/(report_interval/ultra_sensor_interval)) + ', ');
    Serial.println();
    // Reset report variables
    ultra1_reading = 0;
  }
}


/// @brief Read an ultrasonic sensor
/// @param trigger The trigger pin for the sensor
/// @param echo  The echo pin for the sensor
/// @return The distance that the sensor measured
float read_ultrasonic(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  unsigned long duration = pulseIn(echo, HIGH);
  float distance = (duration*.0343)/2;
  return distance;
}