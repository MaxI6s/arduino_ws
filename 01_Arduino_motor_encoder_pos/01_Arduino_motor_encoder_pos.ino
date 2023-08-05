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
const int serialBaudRate = 9600;

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

  Serial.println("------ Pin setup ------");
  Serial.println("Encoder pins must allow interrupt (for arduino UNO : 2, 3)");
  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);
  Serial.println("Encoder channel A: " + encoderAPin);
  Serial.println("Encoder channel B: " + encoderBPin);

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(encoderAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), updateEncoder, CHANGE);

  Serial.println("------ Encoder initialization ------");
  initEncoder();
  Serial.print("Encoder in position: ");
  Serial.println(lastEncoded, BIN);

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

  Serial.print(MSB);
  Serial.print(", ");
  Serial.println(LSB);

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
    encoderCount ++;
  } else if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) {
    encoderCount --;
  }

  lastEncoded = encoded;
}

void initEncoder(){
  // Read the current state of both encoder
  int MSB = digitalRead(encoderAPin);
  int LSB = digitalRead(encoderBPin);
  lastEncoded = (MSB << 1) | LSB;
}