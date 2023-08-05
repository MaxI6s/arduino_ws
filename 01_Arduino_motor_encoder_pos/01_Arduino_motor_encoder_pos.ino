#include <PID_v1.h>

// Define math constant for calculus
// (not sure if it's included in arduino.h)
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// Define encoder pins and interrupt numbers
// Arduino UNO pins with interrupt : 2, 3.
const int encoderAPin = 2;
const int encoderBPin = 3;

// Encoders parameters --> Phase = 4 for quadrature
const int encoderResolution = 120;
const int phase = 4;
const int countPerRevolution = phase * encoderResolution;

// Define motor control pins

// -- TO DO -- 

// Serial communication baud rate
const long serialBaudRate = 115200;

// PID parameters

// -- TO DO -- 

// Initialize PID controller

// -- TO DO -- 

// Variables 

volatile long encoderCount = 0; 
volatile long lastEncoded = 0;
volatile float measuredAngle = 0.0;

void setup() {
  
  Serial.begin(serialBaudRate);

  Serial.println("------ Initialisation ------");
  Serial.print("Baud rate : ");
  Serial.println(serialBaudRate);
  delay(50);

  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);
  delay(50);
  
  Serial.println("------ Pin setup ------");
  Serial.println("Encoder pins must allow interrupt (for arduino UNO : 2, 3)");
  Serial.print("Encoder channel A: ");
  Serial.println(encoderAPin);
  Serial.print("Encoder channel B: ");
  Serial.println(encoderBPin);

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(encoderAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), updateEncoder, CHANGE);
  delay(50);

  initEncoder();
  delay(50);
  
  Serial.println("------ Encoder initialization ------");
  Serial.print("Encoder in position: ");
  Serial.println(lastEncoded, BIN);
  delay(50);
  
  Serial.println("------ START ------");
}

void loop() {
  
}

void updateEncoder(){
  // Read the current state of both encoder
  int MSB = digitalRead(encoderAPin);
  int LSB = digitalRead(encoderBPin);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // TEST TO PRINT SEQUENCE
  /* 
  Serial.print(MSB);
  Serial.print(", ");
  Serial.println(LSB);
  */

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
    encoderCount ++;
  } else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
    encoderCount --;
  }

  lastEncoded = encoded;

  // TEST TO PRINT COUNT
  /*
  Serial.println(encoderCount);
  */
}

void initEncoder(){
  // Read the current state of both encoder
  int MSB = digitalRead(encoderAPin);
  int LSB = digitalRead(encoderBPin);
  lastEncoded = (MSB << 1) | LSB;
}

void calculAngle() {
  measuredAngle = TWO_PI * ((float)encoderCount / countPerRevolution);
}
