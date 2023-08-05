#include <PID_v1.h>


// Define encoder pins and interrupt numbers
// Arduino UNO pins with interrupt : 2, 3.
const int encoderAPin = 2;
const int encoderBPin = 3;

// Encoders parameters
const int encoderResolution = 120;

// Define motor control pins

// -- TO DO -- 

// Serial communication baud rate
const long serialBaudRate = 115200;

// PID parameters

// -- TO DO -- 

// Initialize PID controller

// -- TO DO -- 

// Variables 

volatile int encoderCount = 0; 
volatile int lastEncoded = 0;
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
  
  Serial.println(encoderCount);
  lastEncoded = encoded;
}

void initEncoder(){
  // Read the current state of both encoder
  int MSB = digitalRead(encoderAPin);
  int LSB = digitalRead(encoderBPin);
  lastEncoded = (MSB << 1) | LSB;
}
