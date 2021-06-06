/*
Storm Drain Robot
Copyright 2021  David Perkinson and Kyle Perkinson
 */

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

String readCmdFromSerial()
{
  //uint8_t buffer[32] = "L050R053";
  String buffer = "";
  while (Serial.available() > 0) {
    buffer += (char)Serial.read();
  }
  if (buffer.length() > 0) {
    Serial.println(buffer);
  }
  return buffer;
}

int returnMotorSpeed(String buffer, int start)
{
  int index = 0;
  int targetSpeed = 0;
  uint8_t offset = 0;
  int signOfSpeed = 1;
  int multiplier[3] = {100,10,1};
  
  for(index = start; index < buffer.length(); index++)
  {
    if (buffer[index] == '+') {
      signOfSpeed = 1;
    } else if (buffer[index] == '-') {
      signOfSpeed = -1;
    } else {
      uint8_t digit = (buffer[index] - '0') * multiplier[offset];
      targetSpeed = targetSpeed + digit;
      offset++;
    }
    if (offset > 2) {
      if (targetSpeed <= 100) {
        targetSpeed = targetSpeed * signOfSpeed;
      } else {
        targetSpeed = 0;
      }
      return targetSpeed;
    }
  }    
  if (targetSpeed <= 100) {
    targetSpeed = targetSpeed * signOfSpeed;
  } else {
    targetSpeed = 0;
  }
  return targetSpeed;
}

void readMotorTargetSpeeds()
{
  String buffer = readCmdFromSerial();
  int index = 0;
  
  for(index = 0; index < buffer.length(); index++)
  {
    if (buffer[index] == 'L') {
      leftMotorTargetSpeed = returnMotorSpeed(buffer,(index + 1));
    } else if (buffer[index] == 'R') {
      rightMotorTargetSpeed = returnMotorSpeed(buffer,(index + 1));
    }
  }
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
