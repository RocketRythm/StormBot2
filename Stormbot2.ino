/*
Storm Drain Robot
Copyright 2021  David Perkinson and Kyle Perkinson
 */
//#include <Arduino.h>

int leftMotorProtocolId = 9991;
int leftMotorEnablePin = 8;
int leftMotorFirstPin = 11;
int leftMotorSecondPin = 3;
int leftMotorCurrentSpeed = 0;
int leftMotorTargetSpeed = 0;

int rightMotorProtocolId = 9992;
int rightMotorEnablePin = 12;
int rightMotorFirstPin = 9;
int rightMotorSecondPin = 10;
int rightMotorCurrentSpeed = 0;
int rightMotorTargetSpeed = 0;

// create reusable response objects for responses we expect to handle 
uint8_t option = 0;
uint8_t data = 0;

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(leftMotorEnablePin, OUTPUT);     
  pinMode(leftMotorFirstPin, OUTPUT);
  pinMode(leftMotorSecondPin, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);     
  pinMode(rightMotorFirstPin, OUTPUT);
  pinMode(rightMotorSecondPin, OUTPUT);
  //setPwmFrequency(leftMotorFirstPin, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  //setPwmFrequency(rightMotorFirstPin, 8);  // change Timer2 divisor to 8 gives 3.9kHz PWM freq
  enableLeftMotor();
  enableRightMotor();
  leftMotorSetTargetSpeed(0);
  rightMotorSetTargetSpeed(0);
  Serial.println("init complete\n");
  //Serial.begin(9600);
  delay(1000);
}

void loop() 
{
  // read target speeds from USB serial
  readMotorTargetSpeeds();
  checkMotorSpeedLimits();
  leftMotorCurrentSpeed = adjustLeftMotorSpeed(leftMotorCurrentSpeed,leftMotorTargetSpeed);
  rightMotorCurrentSpeed = adjustRightMotorSpeed(rightMotorCurrentSpeed,rightMotorTargetSpeed);
  printMotorDebug();
  delay(500);
}

void readMotorTargetSpeeds()
{
  //uint8_t buffer[32] = "L050R053";
  String buffer = "";
  while (Serial.available() > 0) {
    buffer += (char)Serial.read();
  }
  if (buffer.length() > 0) {
    Serial.println(buffer);
  }
    int index = 0;
  uint8_t state = 0;
  uint8_t speedCharCount = 0;
  int left_target_speed = 0;
  int right_target_speed = 0;
  int signOfSpeed = 1;
  
  for(index = 0; index < buffer.length(); index++)
  {
    if (state == 'L') {
      if (speedCharCount == 3) {
        if (buffer[index] == '+') {
          signOfSpeed = 1;
        } else if (buffer[index] == '-') {
          signOfSpeed = -1;
        } else {
          uint8_t digit = (buffer[index] - '0') * 100;
          left_target_speed = left_target_speed + digit;
          speedCharCount = 2;
        }
      } else if (speedCharCount == 2) {
        uint8_t digit = (buffer[index] - '0') * 10;
        left_target_speed = left_target_speed + digit;
        speedCharCount = 1;
      } else if (speedCharCount == 1) {
        uint8_t digit = (buffer[index] - '0');
        left_target_speed = left_target_speed + digit;
        speedCharCount = 0;
        state = 0;
        leftMotorTargetSpeed = left_target_speed * signOfSpeed;
      } else {
        speedCharCount = 0;
        state = 0;
        signOfSpeed = 1;
      }
    } else if (state == 'R') {
      if (speedCharCount == 3) {
        if (buffer[index] == '+') {
          signOfSpeed = 1;
        } else if (buffer[index] == '-') {
          signOfSpeed = -1;
        } else {
          uint8_t digit = (buffer[index] - '0') * 100;
          right_target_speed = right_target_speed + digit;
          speedCharCount = 2;
        }
      } else if (speedCharCount == 2) {
        uint8_t digit = (buffer[index] - '0') * 10;
        right_target_speed = right_target_speed + digit;
        speedCharCount = 1;
      } else if (speedCharCount == 1) {
        uint8_t digit = (buffer[index] - '0');
        right_target_speed = right_target_speed + digit;
        speedCharCount = 0;
        state = 0;
        rightMotorTargetSpeed = right_target_speed * signOfSpeed;
      } else {
        speedCharCount = 0;
        state = 0;
        signOfSpeed = 1;
      }
    } else if (state == 0) {
      if (buffer[index] == 'L') {
        state = 'L';
        speedCharCount = 3;
        signOfSpeed = 1;
        left_target_speed = 0;
      } else if (buffer[index] == 'R') {
        state = 'R';
        speedCharCount = 3;
        signOfSpeed = 1;
        right_target_speed = 0;
      } else {
        state = 0;
        speedCharCount = 0;
        signOfSpeed = 1;
      }
    }
  }
//  Serial.print("left motor target speed ");
//  Serial.println(left_target_speed);
//  Serial.print("right motor target speed ");
//  Serial.println(right_target_speed);
}
void checkMotorSpeedLimits()
{
    if ((leftMotorTargetSpeed <= -255) || (leftMotorTargetSpeed > 255))
    {
      leftMotorTargetSpeed = 0;
    }
    if ((rightMotorTargetSpeed <= -255) || (rightMotorTargetSpeed > 255))
    {
      rightMotorTargetSpeed = 0;
    }
}

void printMotorDebug()
{
  if (leftMotorCurrentSpeed != leftMotorTargetSpeed)
  {
    Serial.print("Left Motor:  target: ");
    Serial.print(leftMotorTargetSpeed);
    Serial.print(" current: ");
    Serial.println(leftMotorCurrentSpeed);
  }
  if (rightMotorCurrentSpeed != rightMotorTargetSpeed)
  {
    Serial.print("Right Motor:  target: ");
    Serial.print(rightMotorTargetSpeed);
    Serial.print(" current: ");
    Serial.println(rightMotorCurrentSpeed);
  }
}

void leftMotorSetTargetSpeed(int speed)
{
  leftMotorTargetSpeed = speed;
}
void rightMotorSetTargetSpeed(int speed)
{
  rightMotorTargetSpeed = speed;
}
void enableLeftMotor()
{
    digitalWrite(leftMotorEnablePin, HIGH);
}
void enableRightMotor()
{
    digitalWrite(rightMotorEnablePin, HIGH);
}
void disableLeftMotor()
{
    digitalWrite(leftMotorEnablePin, LOW);
}
int adjustLeftMotorSpeed(int currentSpeed, int targetSpeed)
{
  currentSpeed = calculateNextSpeed(currentSpeed,targetSpeed);
  setLeftMotorSpeed(currentSpeed);
  return currentSpeed;
}
int adjustRightMotorSpeed(int currentSpeed, int targetSpeed)
{
  currentSpeed = calculateNextSpeed(currentSpeed,targetSpeed);
  setRightMotorSpeed(currentSpeed);
  return currentSpeed;
}
int calculateNextSpeed(int currentSpeed, int targetSpeed)
{
  if (currentSpeed < targetSpeed)
  {
    currentSpeed = currentSpeed + 1;
  }
  if (currentSpeed > targetSpeed)
  {
    currentSpeed = currentSpeed - 1;
  }
  return currentSpeed;
}
void setLeftMotorSpeed(int currentSpeed)
{
  if (currentSpeed > 0)
  {
    leftMotorForwardSpeed(currentSpeed);
  }
  else
  {
    leftMotorBackwardSpeed(-currentSpeed);
  }
}
void setRightMotorSpeed(int currentSpeed)
{
  if (currentSpeed > 0)
  {
    rightMotorForwardSpeed(currentSpeed);
  }
  else
  {
    rightMotorBackwardSpeed(-currentSpeed);
  }
}

// To drive the motor in H-bridge mode
// the power chip inputs must be opposite polarity
// and the Enable input must be HIGH
void leftMotorForwardSpeed(int speed)
{
  analogWrite(leftMotorFirstPin, speed);
  analogWrite(leftMotorSecondPin, 0);
}
void rightMotorForwardSpeed(int speed)
{
  analogWrite(rightMotorFirstPin, speed);
  analogWrite(rightMotorSecondPin, 0);
}
void leftMotorBackwardSpeed(int speed)
{
    analogWrite(leftMotorFirstPin, 0);
    analogWrite(leftMotorSecondPin, speed);
}
void rightMotorBackwardSpeed(int speed)
{
    analogWrite(rightMotorFirstPin, 0);
    analogWrite(rightMotorSecondPin, speed);
}

void setPwmFrequency(int pin, int divisor) {
// insert code here if needed
}
