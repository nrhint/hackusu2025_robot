
/*
 Stepper Motor Control - speed control

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.
 A potentiometer is connected to analog input 0.

 The motor will rotate in a clockwise direction. The higher the potentiometer value,
 the faster the motor speed. Because setSpeed() sets the delay between steps,
 you may notice the motor is less responsive to changes in the sensor value at
 low speeds.

 Created 30 Nov. 2009
 Modified 28 Oct 2010
 by Tom Igoe

Used:
* HC-SR04 example sketch
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */
 //Ultrasonic Sensors
const int trigPin = 12;
const int echoPin = 13;
float duration_1, distance_1;

int SonicSense(){
  //Send out signal to request distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //Read in pulse
  int duration = pulseIn(echoPin, HIGH);
  int distance = (duration*.0343)/2;
  return distance;
}

#include <Stepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor


// initialize the stepper library on pins 8 through 11:
const int MotorPinsRight[] = {8, 9, 10, 11};
const int MotorPinsLeft[] = {7, 6, 5, 4};
int Time_Delay = 3;//3 milliseconds
void MoveSide(int motor[], int steps){
  for (int j = 0; j < steps; j++){
    for (int i = 0; i < 4; i++){
      digitalWrite(motor[i], HIGH);
      delay(Time_Delay);
      digitalWrite(motor[i], LOW);
    }
  }
}
void MoveForward(int steps){
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
//Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

int stepCount = 0;  // number of steps the motor has taken

void setup() {
  //First motor pin initializations
  for (int i = 0; i < 4; i++){
    pinMode(MotorPinsRight[i], OUTPUT);
    digitalWrite(MotorPinsRight[i], LOW);
  }
  //Second motor pin initializations
  for (int i = 0; i < 4; i++){
    pinMode(MotorPinsLeft[i], OUTPUT);
    digitalWrite(MotorPinsLeft[i], LOW);
  }
  //
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
  //Serial.begin(9600);
}

void loop() {
  // read the sensor value:
  //int sensorReading = analogRead(A0);
  // map it to a range from 0 to 100:
  int motorSpeed = 100; //= map(sensorReading, 0, 1023, 0, 100);
  // set the motor speed:
  if (motorSpeed > 0) {
    //Move motor if signal on
    //MoveSide(MotorPinsRight, 400);
    MoveForward(400);
    delay(2000);
    TurnRight(400);
    delay(2000);
    TurnLeft(400);
    //myStepper.setSpeed(motorSpeed);
    // step 1/100 of a revolution:
    //myStepper.step(stepsPerRevolution / 100);
  }
  //int distance = SonicSense();
  //Serial.print("Distance: ");
  //Serial.println(distance);
  delay(10000);
}


