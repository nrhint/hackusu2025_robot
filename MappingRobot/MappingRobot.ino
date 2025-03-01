
/*

USU 2025 Hackathon

Used:
* HC-SR04 example sketch to get started with motors and then moved on
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */
 
 //#include "I2Cdev.h"
 //#include "MPU6050_6Axis_MotionApps20.h"
 
 #define DEBUG
 
 #ifdef DEBUG
 #define DEBUG_PRINT(x) Serial.println(x)
 #else
 #define DEBUG_PRINT(x)
 #endif
 
 /* SETUP FOR UPTRASONIC SENSORS */
 #define FRONT_ULTRA_TRIGGER PIN_A0
 #define FRONT_ULTRA_ECHO PIN_A1
 #define RIGHT_ULTRA_TRIGGER PIN_A2
 #define RIGHT_ULTRA_ECHO PIN_A3
 #define LEFT_ULTRA_TRIGGER 12
 #define LEFT_ULTRA_ECHO 3
 
 unsigned long previous_ultra_millis = 0;
 #define ultraSensorInterval 100
 float frontUltraReading = 0;
 float rightUltraReading = 0;
 float  leftUltraReading = 0;
 float frontUltraReadingPrev = 0;
 float rightUltraReadingPrev = 0;
 float  leftUltraReadingPrev = 0;
 
 unsigned long previous_report_millis = 0;
 #define report_interval 1000
 
 /* Setup for the MPU6050 */
 //MPU6050 mpu;
 
 int const MPU_INTERRUPT_PIN = 2; // Define the interruption #0 pin
 long int xPosition = 0; // These should be measured in cm which should give us *plenty* of precision
 long int yPosition = 0;
 double report_angle = 0;
 float currentRobotAngle = 0;

 bool DMPReady = false;  // Set true if DMP init was successful
 uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
 uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
 uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
 uint8_t FIFOBuffer[64]; // FIFO storage buffer
 /*
 Quaternion q;
 VectorInt16 aa;
 VectorInt16 aaReal;
 VectorFloat gravity;
 float ypr[3];*/
  
 volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
 void DMPDataReady() {
   MPUInterrupt = true;
 }
 
 long int robotAngle = 0;
 long int robotLastReportAngle = 0;
 
 /* MOTOR SETUP */
 const int stepsPerRevolution = 515;  // change this to fit the number of steps per revolution
 const int tenthRevolution = 51;
 long int robotMotorSteps = 0;
 long int robotMotorLastReportSteps = 0;
 
 // Setup the stepper motors for usage
 const int MotorPinsRight[] = {8, 9, 10, 11};
 const int MotorPinsLeft[] = {7, 6, 5, 4};
 int Time_Delay = 3;//3 milliseconds
 
 float read_ultrasonic(int trigger, int echo);
 void error_blink(void);
 void MoveSide(int motor[], int steps);
 void MoveForward(int steps);
 void TurnLeft(int steps);
 void TurnRight(int steps);
 
 
 /// @brief Setup the sensors and motors
 void setup() {
   // Setup light for error
   pinMode(13, OUTPUT);
 
   /* SETUP MOTORS */
   // First motor pin initializations
   for (int i = 0; i < 4; i++){
     pinMode(MotorPinsRight[i], OUTPUT);
     digitalWrite(MotorPinsRight[i], LOW);
   }
   // Second motor pin initializations
   for (int i = 0; i < 4; i++){
     pinMode(MotorPinsLeft[i], OUTPUT);
     digitalWrite(MotorPinsLeft[i], LOW);
   }
 
   /* Setup the ultrasonic sensors: */
   pinMode(FRONT_ULTRA_TRIGGER, OUTPUT);
   pinMode(FRONT_ULTRA_ECHO, INPUT);
   pinMode(RIGHT_ULTRA_TRIGGER, OUTPUT);
   pinMode(RIGHT_ULTRA_ECHO, INPUT);
   pinMode(LEFT_ULTRA_TRIGGER, OUTPUT);
   pinMode(LEFT_ULTRA_ECHO, INPUT);
 
   /* SETUP THE MPU 
   digitalWrite(13, HIGH);
   delay(500);
   digitalWrite(13, LOW);
   delay(4500); // wait a while for the robot to be stable before initalizing stuff
   digitalWrite(13, HIGH); // Write debug light high, if there is a problem it will blink.
   Wire.begin();
   Wire.setClock(400000);*/ // 400kHz I2C clock. Comment on this line if having compilation difficulties
   Serial.begin(115200);
   while (!Serial);
 
   // initialize the MPU device
   DEBUG_PRINT("initalizing I2C devices");
   //mpu.initialize();
   Serial.println("test");
   pinMode(MPU_INTERRUPT_PIN, INPUT);
 
   DEBUG_PRINT("Testing connection to MPU6050");/*
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
   } else {
     Serial.print("SMP initalization failed with code: ");
     Serial.println(devStatus);
   }*/
 }
 
 /// @brief Main look for the robot
 void loop() {
   // Sample the mpu:
   /*if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
     mpu.dmpGetQuaternion(&q, FIFOBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     currentRobotAngle = ypr[0]; // the yaw of the sensor (This is in radians)
   }*/
 
   // Sample the ultrasonic sensors
   unsigned long currentMillis = millis();
   if (currentMillis - previous_ultra_millis >= ultraSensorInterval) {
     // Save the last time the function was called
     previous_ultra_millis = currentMillis;
     frontUltraReadingPrev = frontUltraReading;
     frontUltraReading = read_ultrasonic(FRONT_ULTRA_TRIGGER, FRONT_ULTRA_ECHO);
     delay(2);
     rightUltraReadingPrev = rightUltraReading;
     rightUltraReading = read_ultrasonic(RIGHT_ULTRA_TRIGGER, RIGHT_ULTRA_ECHO);
     delay(2);
     leftUltraReadingPrev = leftUltraReading;
     leftUltraReading = read_ultrasonic(LEFT_ULTRA_TRIGGER, LEFT_ULTRA_ECHO);
   }
 
   /* Send the data to the PI */
   if (currentMillis - previous_report_millis >= report_interval) {
     previous_report_millis = currentMillis;
     // Print out lines to pi
     Serial.print(frontUltraReading, 2);
     Serial.print(", ");
     Serial.print(rightUltraReading, 2);
     Serial.print(", ");
     Serial.print(leftUltraReading, 2);
     Serial.print(", ");
     Serial.print(report_angle * 180/M_PI);
     Serial.print(", ");
     Serial.print(robotMotorSteps);
   }
   int onWall = 1;
   
   
   if (frontUltraReading < 10){
    //If hit forward wall, turn full left
    TurnLeft(500);
    onWall = 1;
   }else if (rightUltraReading < 10){
    //If getting too close to wall slight turn
    TurnLeft(tenthRevolution);
    MoveForward(tenthRevolution);
    TurnLeft(26);
    onWall = 1;
   }/*else if(onWall && (rightUltraReading > 40)){
    //If follwing wall and hit outside turn, turn right
    TurnRight(500);
   }*/else if(frontUltraReading > 10) {
     MoveForward(tenthRevolution*2);
   }
 }
 
 /// @brief Move one side of the wheels
 /// @param motor The motor/set of motors to move
 /// @param steps The number of steps to move
 void MoveSide(int motor[], int steps){
   for (int j = 0; j < steps; j++){
     for (int i = 0; i < 4; i++){
       digitalWrite(motor[i], HIGH);
       delay(Time_Delay);
       digitalWrite(motor[i], LOW);
     }
   }
 }
 
 /// @brief Move the robot forward
 /// @param steps Move the robot forward a number of encoder counts.
 void MoveForward(int steps){
   //accumulate in global where the steps is
   for (int j = 0; j < steps; j++){
     for (int i = 0; i < 4; i++){
       digitalWrite(MotorPinsRight[i], HIGH);
       digitalWrite(MotorPinsLeft[i], HIGH);
       delay(Time_Delay);
       digitalWrite(MotorPinsRight[i], LOW);
       digitalWrite(MotorPinsLeft[i], LOW);
     }
   }
 }
 
 /// @brief Turn the robot to the left
 /// @param steps The number of degrees to turn. This will use feedback from the gyro sensor
 void TurnLeft(int steps){
   for (int n = 0; n < steps; n++){
     int j = 3;
     for (int i = 0; i < 4; i++){
       digitalWrite(MotorPinsRight[j], HIGH);
       digitalWrite(MotorPinsLeft[i], HIGH);
       delay(Time_Delay);
       digitalWrite(MotorPinsRight[j], LOW);
       digitalWrite(MotorPinsLeft[i], LOW);
       j -= 1;
     }
   }
 }
 
 /// @brief Turn the robot to the right
 /// @param steps The number of degrees to turn. This will use feedback from the gyro sensor
 void TurnRight(int steps){
   for (int n = 0; n < steps; n++){
     int j = 3;
     for (int i = 0; i < 4; i++){
       digitalWrite(MotorPinsRight[i], HIGH);
       digitalWrite(MotorPinsLeft[j], HIGH);
       delay(Time_Delay);
       digitalWrite(MotorPinsRight[i], LOW);
       digitalWrite(MotorPinsLeft[j], LOW);
       j -= 1;
     }
   }
 }
 
 /// @brief Read an ultrasonic sensor
 /// @param trigger The trigger pin for the sensor
 /// @param echo  The echo pin for the sensor
 /// @return The distance that the sensor measured
 float read_ultrasonic(int trigger, int echo) {
   float avg = 0;
   for (int ii = 0; ii < 3; ii++) {
     Serial.println(ii);
     digitalWrite(trigger, LOW);
     delayMicroseconds(2);
     digitalWrite(trigger, HIGH);
     delayMicroseconds(10);
     digitalWrite(trigger, LOW);
     unsigned long duration = pulseIn(echo, HIGH);
     delay(2);
     avg += (duration*.0343)/2;
   }
   return avg/3;
 }
 
 /// @brief Blinks the onboard led if there is an error
 void error_blink() {
   while (true){
     digitalWrite(13, LOW);
     delay(250);
     digitalWrite(13, HIGH);
     delay(250);
   }
 }
    
 