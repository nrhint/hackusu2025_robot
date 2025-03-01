# hackusu2025 robot

# Goal:

Our goal is to build a robot that is able to map out the room. The minimum is to use ultrasonic sensors to measure distances in the room. Our goal is to add on parts such as a GPS module and/or a gyroscope to make our measurements more precise.

The data will be gathered by a combination RPI and Arduino UNO. Our robot will use steppers to drive around and the base will be 3d printed. We will use a battery to power the PI which will have the Arduino and possibly GPS plugged into it. There will also be a 5V rail that will be connected to the boards to power the steppers and sensors seperate from the PI.

# Design process:

When starting we wanted 4 wheel drive. This turned out to be an issue as the Arduino Uno only had 13 GPIO pins. Four wheel drive would have require 16 just to drive the motors. This also constrained how we were doing sensors. We wanted 3 ultrasonic sensors however again we were constrained by pins. This caused us to discuss solutions such as using some of the motor driver pins to trigger the sensors and then to stop to take measurements and hope the wheels did not move when we used the shared pins. We decided to have the 2 motors on each side of "car" controlled in tandem so as to only use 8 pins for all of our motors. 

So more pin constriction. We realized that the GPS is not locking onto a signal when we are inside. We are moving to using a MPU 6050 to provide gyro and accel gyroscope information for positioning. This means that we **REALLY** do not have enough pins. Our options at this point are to move to another board or use two unos. The boards we have access to are a STM32 board or an arduino mega that we are not sure if it will work (it's from china). For now we are continuing and not worrying about it.

We were hoping to have all of our system running on 1 arduino but with how much time it takes some of our functions to run, we were running into timing issues. We decided to split our workload between 2 arduinos by having the ultrasonic sensors and gyroscope on one and the motors running on the other. 


# Libraries and code used:

Library for reading the MPU 6050: https://github.com/ElectronicCats/mpu6050
