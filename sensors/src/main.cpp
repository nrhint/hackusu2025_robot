// Nathan Hinton
// USU 2025 Hackathon
// Sensors file
// This will take the sensors and then create a fusion of them that will be pushed out to a raspberry pi which will log the data for later processing

// MPU code based on the DMP6 example from the MPU6050 library

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

/* SETUP FOR UPTRASONIC SENSORS */
#define FRONT_ULTRA_TRIGGER PIN3
#define FRONT_ULTRA_ECHO PIN_A3

unsigned long previous_ultra_millis = 0;
#define ultra_sensor_interval 100
float ultra1_reading = 0;
float ultra2_reading = 0;
float ultra3_reading = 0;

unsigned long previous_report_millis = 0;
#define report_interval 1000

/* Setup for the MPU6050 */
MPU6050 mpu;

int const MPU_INTERRUPT_PIN = 2; // Define the interruption #0 pin
long int xPosition = 0; // These should be measured in cm which should give us *plenty* of precision
long int yPosition = 0;
double report_angle = 0;

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
float ypr[3];

volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

float read_ultrasonic(int trigger, int echo);
void error_blink(void);

void setup() {
  pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);
  // delay(500);
  // digitalWrite(13, LOW);
  // delay(4500); // wait a while for the robot to be stable before initalizing stuff
  // digitalWrite(13, HIGH); // Write debug light high, if there is a problem it will blink.
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  Serial.begin(115200);
  while (!Serial);

  // initialize the MPU device
  DEBUG_PRINT("initalizing I2C devices");
  mpu.initialize();
  Serial.println("test");
  pinMode(MPU_INTERRUPT_PIN, INPUT);

  DEBUG_PRINT("Testing connection to MPU6050");
  if (mpu.testConnection() == false) {
    Serial.println("FAILED TO CONNECT TO MPU6050!!!");
    // Blink the light if there is an error
    error_blink();
  } else {
    DEBUG_PRINT("Connection success!");
  }

  devStatus = mpu.dmpInitialize();

  // add gyro offsets
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  // Make sure the initalization worked:
  if (devStatus == 0) {
    mpu.CalibrateAccel(6); // the calibration time.
    mpu.CalibrateGyro(6);
    DEBUG_PRINT("Enabling DMP");
    mpu.setDMPEnabled(true);

    // Enable the arduino interupt on pin 2:
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    // Set the dmp ready flag to allow the main loop to run:
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else{
    Serial.print("SMP initalization failed with code: ");
    Serial.println(devStatus);
    
    // initialize the ultrasonic sensors:
    pinMode(FRONT_ULTRA_TRIGGER, OUTPUT);
    pinMode(FRONT_ULTRA_ECHO, INPUT);
  }
}

void loop() {
  // Sample the mpu:
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    report_angle = ypr[0]; // the yaw of the sensor
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);    xPosition += cos(report_angle) * aaReal.x;
    yPosition += sin(report_angle) * aaReal.x;
  }
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
    Serial.print(ultra1_reading/(report_interval/ultra_sensor_interval), 2);
    Serial.print(", ");
    Serial.print(ultra2_reading/(report_interval/ultra_sensor_interval), 2);
    Serial.print(", ");
    Serial.print(ultra3_reading/(report_interval/ultra_sensor_interval), 2);
    Serial.print(", ");
    Serial.print(report_angle * 180/M_PI);
    Serial.print(", ");
    Serial.print(xPosition);
    Serial.print(", ");
    Serial.print(yPosition);
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

void error_blink() {
  while (true){
    digitalWrite(13, LOW);
    delay(250);
    digitalWrite(13, HIGH);
    delay(250);
  }
}
   
