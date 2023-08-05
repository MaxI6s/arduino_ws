# 1 "/home/maxime/arduino_dev/arduino_ws/01_Arduino_motor_encoder_pos/01_Arduino_motor_encoder_pos.ino"
# 2 "/home/maxime/arduino_dev/arduino_ws/01_Arduino_motor_encoder_pos/01_Arduino_motor_encoder_pos.ino" 2

//test
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
PID myPID(&measuredAngle, &pwmOutput, &setpointAngle, kp, ki, kd, 0);

void setup() {
  Serial.begin(serialBaudRate);

  pinMode(encoderAPin, 0x0);
  pinMode(encoderBPin, 0x0);

  pinMode(motorPWMPin, 0x1);
  pinMode(motorDirectionPin, 0x1);

  // Attach interrupts for encoder channels
  attachInterrupt(((encoderAPin) == 2 ? 0 : ((encoderAPin) == 3 ? 1 : -1)), updateEncoder, 1);
  attachInterrupt(((encoderBPin) == 2 ? 0 : ((encoderBPin) == 3 ? 1 : -1)), updateEncoder, 1);

  // Initialize PID
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(1);
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
    digitalWrite(motorDirectionPin, 0x1);
    analogWrite(motorPWMPin, pwmOutput);
  } else {
    digitalWrite(motorDirectionPin, 0x0);
    analogWrite(motorPWMPin, -pwmOutput);
  }

  // Send measured angle via serial
  Serial.write("a");
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
  measuredAngle = encoderPos * (2 * 3.1415926535897932384626433832795) / encoderResolution; // Convert encoder count to radians
}
