#include <PID_v1.h>

// Define encoder pins and interrupt numbers
const int encoderAPin = 2;
const int encoderBPin = 3;

// Define motor control pins
const int motorPWMPin = 9;
const int motorDirectionPin = 8;

// Serial communication baud rate
const int serialBaudRate = 115200;

// PID parameters
double kp = 1.0; // Proportional gain
double ki = 0.1; // Integral gain
double kd = 0.01; // Derivative gain

// Constantes
const int encoderResolution = 120;

// Variables
volatile long encoderPos = 0;
double measuredAngle = 0.0;
double setpointAngle = 0.0;
double pwmOutput = 0;

// Initialize PID controller
PID myPID(&measuredAngle, &pwmOutput, &setpointAngle, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(serialBaudRate);
  
  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);
  
  pinMode(motorPWMPin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(encoderAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), updateEncoder, CHANGE);

  // Initialize PID
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Read setpoint angle from serial
  if (Serial.available() >= sizeof(double)) {
    setpointAngle = Serial.readString().toDouble();
  }

  // Update PID and calculate PWM
  myPID.Compute();
  
  // Update motor direction and PWM
  if (pwmOutput > 0) {
    digitalWrite(motorDirectionPin, HIGH);
    analogWrite(motorPWMPin, pwmOutput);
  } else {
    digitalWrite(motorDirectionPin, LOW);
    analogWrite(motorPWMPin, -pwmOutput);
  }

  // Send measured angle via serial
  Serial.write((byte *)&measuredAngle, sizeof(double));
}

void updateEncoder() {
  static uint8_t previousEncoded = 0;
  static int encoderValue = 0;
  int MSB = digitalRead(encoderAPin);
  int LSB = digitalRead(encoderBPin);
  int encoded = (MSB << 1) | LSB;
  int sum = (previousEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--;
  }

  encoderPos = encoderValue;
  measuredAngle = encoderPos * (2 * PI) / encoderResolution; // Convert encoder count to radians
}
