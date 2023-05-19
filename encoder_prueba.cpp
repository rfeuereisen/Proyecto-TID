#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define ENCODER_RESOLUTION 360   // Encoder resolution (pulses per revolution)
#define WHEEL_RADIUS       0.05  // Wheel radius in meters
#define WHEEL_BASE         0.1   // Wheelbase in meters

Adafruit_MotorShield motorShield = Adafruit_MotorShield();  // Create an instance of the motor shield
Adafruit_DCMotor *leftMotor = motorShield.getMotor(1);       // Motor on M1
Adafruit_DCMotor *rightMotor = motorShield.getMotor(2);      // Motor on M2

volatile long leftEncoderCount = 0;   // Left motor encoder count
volatile long rightEncoderCount = 0;  // Right motor encoder count

void leftEncoderInterrupt()
{
  if (digitalRead(2) == HIGH)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

void rightEncoderInterrupt()
{
  if (digitalRead(3) == HIGH)
    rightEncoderCount++;
  else
    rightEncoderCount--;
}

void setup()
{
  Serial.begin(9600);

  motorShield.begin();  // Initialize the motor shield
  motorShield.setPWMFreq(1000);  // Set the PWM frequency

  pinMode(2, INPUT_PULLUP);    // Left motor encoder pin
  pinMode(3, INPUT_PULLUP);    // Right motor encoder pin
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderInterrupt, RISING);
}

void moveForward(float distance)
{
  int pulses = distance / (2 * PI * WHEEL_RADIUS) * ENCODER_RESOLUTION;

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  leftMotor->setSpeed(200);
  rightMotor->setSpeed(200);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);

  while (leftEncoderCount < pulses || rightEncoderCount < pulses)
  {
    // Continue moving forward
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void turn(float angle)
{
  float radians = angle * PI / 180.0;
  float distance = WHEEL_BASE * radians;

  int pulses = distance / (2 * PI * WHEEL_RADIUS) * ENCODER_RESOLUTION;

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  if (angle > 0)
  {
    leftMotor->setSpeed(200);
    rightMotor->setSpeed(200);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
  }
  else
  {
    leftMotor->setSpeed(200);
    rightMotor->setSpeed(200);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
  }

  while (leftEncoderCount < pulses || rightEncoderCount < pulses)
  {
    // Continue turning
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop()
{
  // Move forward for 1 meter
  moveForward(1.0);

  // Pause for 1 second
  delay(1000);

    // Turn left by 90 degrees
  turn(-90.0);

  // Pause for 1 second
  delay(1000);

  // Move forward for 0.5 meters
  moveForward(0.5);

  // Pause for 1 second
  delay(1000);

  // Turn right by 45 degrees
  turn(45.0);

  // Pause for 1 second
  delay(1000);

  // Move forward for 0.75 meters
  moveForward(0.75);

  // Pause for 1 second
  delay(1000);

  // Turn left by 135 degrees
  turn(-135.0);

  // Pause for 1 second
  delay(1000);

  // Move forward for 1.5 meters
  moveForward(1.5);

  // Pause for 1 second
  delay(1000);
}
