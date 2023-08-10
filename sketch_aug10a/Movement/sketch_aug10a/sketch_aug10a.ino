#include <SPI.h>

//left = A
//right = B


// Define motor control pins for Motor A
const int motorAEnablePin = 11;   // Enable pin for Motor A
const int motorAPin1 = 10;       // Control pin 1 for Motor A
const int motorAPin2 = 9;       // Control pin 2 for Motor A

// Define motor control pins for Motor B
const int motorBEnablePin = 6;   // Enable pin for Motor B
const int motorBPin1 = 7;        // Control pin 1 for Motor B
const int motorBPin2 = 8;        // Control pin 2 for Motor B

void setup() {
  // Set the motor control pins as outputs for Motor A
  pinMode(motorAEnablePin, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);

  // Set the motor control pins as outputs for Motor B
  pinMode(motorBEnablePin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  
  // Initialize motors to stop
  Serial.begin(115200);
  digitalWrite(motorAEnablePin, LOW);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBEnablePin, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

void Left(int);
void Right(int);
void Forward(int);
void Back(int);

void loop() {
  //10 seconds
  Forward(10000);
}

void Left(int time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}

void Right(int time = 1000)
{
  // Move Motor A forward and Motor B backward
  digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);     // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}

void Forward(int time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}

void Back(int time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}

void Stop(int time = 1000)
{
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
  delay(time);
}
