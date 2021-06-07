#include <NewPing.h>

#define leftIr A0
#define rightIr A1
#define leftIrPower 8
#define rightIrPower 10

#define leftMotorSpeed 3
#define leftMotorP 4
#define leftMotorN 5
 
#define rightMotorN 6
#define rightMotorP 7
#define rightMotorSpeed 9

#define ultrasonicTrigger 11
#define ultrasonicEcho 12

// Max distance read by ultrasonic sensor
const int maxDistance = 400;
NewPing ultrasonic(ultrasonicTrigger, ultrasonicEcho, maxDistance);

const char GREEN_SIGN = 'd'; // Drive
const char RED_SIGN = 's'; // Stop

const int MIN_DISTANCE_FROM_OBJECT = 5;
// Error of the distance read by the ultrasonic sensor
const float DISTANCE_OFFSET = -0.31;

// Motors speed (0 - 255)
const float FORWARD_SPEED = 112;
const float CURVE_SPEED = FORWARD_SPEED;

// Variable responsible for ignoring small variations in reflectance
const int SENSITIVITY_OFFSET = 30;

// Reflectance thresholds of the external parts to the central band
int leftThreshold;
int rightThreshold;

char vehicleAction;

void setup() {
  Serial.begin(115200);
  
  pinMode(leftIr, INPUT);
  pinMode(rightIr, INPUT);
  pinMode(leftIrPower, OUTPUT);
  pinMode(rightIrPower, OUTPUT);
  leftThreshold = analogRead(leftIr);
  rightThreshold = analogRead(rightIr);
  
  pinMode(leftMotorSpeed, OUTPUT);
  pinMode(leftMotorP, OUTPUT);
  pinMode(leftMotorN, OUTPUT);
  
  pinMode(rightMotorSpeed, OUTPUT);
  pinMode(rightMotorN, OUTPUT);
  pinMode(rightMotorP, OUTPUT);
  
  pinMode(ultrasonicTrigger, OUTPUT); 
  pinMode(ultrasonicEcho, INPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char serial = Serial.read();

    if (serial == GREEN_SIGN || serial == RED_SIGN) {  
      vehicleAction = serial;
    }
  }
  
  if (vehicleAction == GREEN_SIGN)
    drive();
  else if (vehicleAction == RED_SIGN)
    stopCar();
}

void drive() {
  if (hasObstacle()) stopCar();
  while (hasObstacle()) delay(100);
  
  // If a new vehicle action was received, must process it first
  if (Serial.available() > 0) return;
  
  int leftIrValue = analogRead(leftIr) - SENSITIVITY_OFFSET;
  int rightIrValue = analogRead(rightIr) - SENSITIVITY_OFFSET;
  
  if (leftIrValue <= leftThreshold && rightIrValue <= rightThreshold)
    moveForward();
  else if (leftIrValue > leftThreshold && rightIrValue <= rightThreshold)
    moveLeft();
  else if (leftIrValue <= leftThreshold && rightIrValue > rightThreshold)
    moveRight();
  else if (leftIrValue > leftThreshold && rightIrValue > rightThreshold)
    stopCar();
  
  // Runs an action during just 125ms to make sure it doesn't cross the line  
  delay(125);
  stopCar();
  delay(75);
}

boolean hasObstacle() {
  int distance = ultrasonic.ping_cm();
  
  // Check if distance different of 0, because NewPing library returns 0 if 
  // the distance is greater than the specified
  return distance > 0 && distance <= MIN_DISTANCE_FROM_OBJECT;
}

void moveForward() {
  // PWM
  analogWrite(leftMotorSpeed, FORWARD_SPEED);
  analogWrite(rightMotorSpeed, FORWARD_SPEED);
  
  // Direction
  digitalWrite(leftMotorP, HIGH);
  digitalWrite(leftMotorN, LOW);
  digitalWrite(rightMotorP, HIGH);
  digitalWrite(rightMotorN, LOW);
}

void moveLeft() {
  // PWM
  analogWrite(leftMotorSpeed, 0);
  analogWrite(rightMotorSpeed, CURVE_SPEED);
  
  // Direction
  digitalWrite(leftMotorP, LOW);
  digitalWrite(leftMotorN, LOW);
  digitalWrite(rightMotorP, HIGH);
  digitalWrite(rightMotorN, LOW);
}

void moveRight() {
  // PWM
  analogWrite(leftMotorSpeed, CURVE_SPEED);
  analogWrite(rightMotorSpeed, 0);
  
  // Direction
  digitalWrite(leftMotorP, HIGH);
  digitalWrite(leftMotorN, LOW);
  digitalWrite(rightMotorP, LOW);
  digitalWrite(rightMotorN, LOW);
}

void stopCar() {
  // PWM
  analogWrite(leftMotorSpeed, 0);
  analogWrite(rightMotorSpeed, 0);
  
  // Direction
  digitalWrite(leftMotorP, LOW);
  digitalWrite(leftMotorN, LOW);
  digitalWrite(rightMotorP, LOW);
  digitalWrite(rightMotorN, LOW);
}