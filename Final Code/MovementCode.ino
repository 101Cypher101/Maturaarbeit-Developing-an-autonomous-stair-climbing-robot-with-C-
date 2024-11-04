#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include <RPLidar.h>

// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal 
                       

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// Ultrasonic sensor pins
#define trigPin A0
#define echoPin A1

// Servo parameters
#define SERVO_NEUTRAL 0
#define SERVO_CLOCKWISE 300
#define SERVO_COUNTERCLOCKWISE 500

// Define channels for the servos
#define SERVO_1 0
#define SERVO_2 1

// Detection parameters
int duration;
int distance = 30;
int detectionThreshold = 5;  // Object detection threshold in centimeters

int startTime;
int rotationTime;
int overshoot = 2000;
int maxRotationTime = 7000;
bool isStair;

int timeTillScan = 30; //60 seconds
int timeNeededForScan = 15000;
int timeRN = 0;

// Define stepper motor objects using AccelStepper
#define MotorInterfaceType 4 // 4 = L298N driver with 4 pins
AccelStepper motor1(MotorInterfaceType, 2, 3, 4, 5); //FrontLeft
AccelStepper motor2(MotorInterfaceType, 6, 7, 8, 9); //BackLeft
AccelStepper motor3(MotorInterfaceType, 10, 11, 12, 13); //Right Side


void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial.begin(115200);
  Serial1.begin(115200);
  lidar.begin(Serial1);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  
  // Initialize PCA9685 for servo control
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos typically run at 60 Hz

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set maximum speed for each motor
  motor1.setMaxSpeed(2000); // Set max speed for motor 1
  motor2.setMaxSpeed(2000); // Set max speed for motor 2
  motor3.setMaxSpeed(2000); // Set max speed for motor 3
}


int getDistance() {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigger pin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time for the echo to return
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.0343 / 2;

  return distance;
}


// defining different funktions 
void moveServoStep1() {
  pwm.setPWM(SERVO_1, 0, SERVO_CLOCKWISE);
}

void moveServoStep2() {
  pwm.setPWM(SERVO_1, 0, SERVO_COUNTERCLOCKWISE);
  pwm.setPWM(SERVO_2, 0, SERVO_CLOCKWISE);
}

void moveServoStep3() {
  pwm.setPWM(SERVO_2, 0, SERVO_COUNTERCLOCKWISE);
}

void moveServoStep4() {
  pwm.setPWM(SERVO_1, 0, SERVO_COUNTERCLOCKWISE);
}

void stopServo() {
  pwm.setPWM(SERVO_1, 0, SERVO_NEUTRAL);
  pwm.setPWM(SERVO_2, 0, SERVO_NEUTRAL);
}


void angleMotors(int angle){
  motor3.setCurrentPosition(0); //reset motor1 position in the code

  while(motor3.currentPosition() <= angle) {
    motor1.setSpeed(300); // Set speed for motor 1
    motor2.setSpeed(300); // Set speed for motor 2
    motor3.setSpeed(300); // Set speed for motor 3
    
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();
  }
}

void turnMotors(int angle){
  motor3.setCurrentPosition(0); //reset motor1 position in the code

  while(motor3.currentPosition() <= angle) {
    motor1.setSpeed(-300); // Set speed for motor 1
    motor2.setSpeed(-300); // Set speed for motor 2
    motor3.setSpeed(300); // Set speed for motor 3
    
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();
  }
}
void stopMotors(){
  motor1.setSpeed(0); // Set speed for motor 1
  motor2.setSpeed(0); // Set speed for motor 2
  motor3.setSpeed(0); // Set speed for motor 3

  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
}


void loop() {
  angleMotors(200);

  // Measure the distance from the sensor
  distance = getDistance();
  
  rotationTime = 0;

  if (distance <= detectionThreshold) {
    stopMotors();

    startTime = millis(); //Time it started rotating

    while(distance <= detectionThreshold) {
      distance = getDistance();
      rotationTime = millis() - startTime;
      moveServoStep1();
      if(rotationTime >= maxRotationTime){
        Serial.println("Break");
        break;
      }
    }

    if (rotationTime >= maxRotationTime) {
      isStair = false;
    }
    else {
      isStair = true;
    }

    if (isStair == true) {
      delay(overshoot); //So wheels can get over the edge
      stopServo();

      rotationTime = millis() - startTime; //How long it too to rotate 

      angleMotors(1000);
      stopMotors();

      moveServoStep2();
      delay(rotationTime);
      stopServo();

      angleMotors(1500);
      stopMotors();

      moveServoStep3();
      delay(rotationTime);
      stopServo();
    }
    else {
      moveServoStep4();
      delay(rotationTime - overshoot);
      stopServo();

      turnMotors(5000);
      stopMotors();
    }
  }
}