#include <SPI.h>
#include <Servo.h>

//left = A
//right = B

// Left Wheel : Front
// Define motor control pins for Motor A
const int motorAEnablePin = 11;   // Enable pin for Motor A
const int motorAPin1 = 10;       // Control pin 1 for Motor A
const int motorAPin2 = 9;       // Control pin 2 for Motor A

// Right Wheel : Front
// Define motor control pins for Motor B
const int motorBEnablePin = 7;   // Enable pin for Motor B
const int motorBPin1 = 6;        // Control pin 1 for Motor B
const int motorBPin2 = 8;        // Control pin 2 for Motor 

// Distance Sensors
int TrigPin = 12;
int EchoPin = 13;
  
// Arm Catching Motor
// Define motor control pins for Motor C; The Servo motor
Servo myServo;

// Car Back Bumper Sensor
// Limit Switch
//DEFINE THESE AT SOME POINT YOU BETTER NOT IGNORE THEM IM GONNA MAKE SURE YOU DON'T IGNORE THEM KDAJFLDJFKLDAJFKLDASJFLKDAJFLKDAJFLKDJAFLKDSJAFLDJSAKFDA
//const int limitSwitchPin;

// Distance Values
float lastDistance = 0;
const float goalDistance = 5;

void setup() {
  // Set the motor control pins as outputs for Motor A
  pinMode(motorAEnablePin, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);

  // Set the motor control pins as outputs for Motor B
  pinMode(motorBEnablePin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);

  // Set the motor control pins for Motor C
  myServo.attach(5);

  // Set ultrasonic Sensor pins to In/Out
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  // Initialize motors to stop
  Serial.begin(115200);
  digitalWrite(motorAEnablePin, LOW);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBEnablePin, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
}

void Left(int times, int speeds);
void Right(int times, int speeds);
void Forward(int times, int speeds);
void Back(int times, int speeds);
void LeftBack(int times, int speeds);
void RightBack(int times, int speeds);
void FastLeft(int times, int speeds);
void FastRight(int times, int speeds);
void Stop(int times);
void Catch(int m_Time, float ratioA, float ratioB);

void loop() {
  //digitalWrite(TrigPin, HIGH);
  //digitalWrite(TrigPin, LOW);
  
  delay(3000);
  SimpleCatch();
  delay(1000);
  Deposit();
}

void SimpleCatch()
{  
  myServo.write(90);
  Serial.write("90 ");
  delay(1000);
  //Down
  myServo.write(150);
  Serial.write("160 ");
  delay(400);
  myServo.write(90);
  Serial.write("90 ");
  delay(2000);
  //Up
  myServo.write(50);
  Serial.write("50 ");
  delay(400);
  myServo.write(90);
  Serial.write("90 ");
  delay(4000);
}

void Deposit()
{
  //Up
  myServo.write(90);
  delay(500);
  myServo.write(45);
  delay(450);
  myServo.write(90);
  delay(2000);
  //Down
  myServo.write(155);
  delay(500);
  myServo.write(90);
  delay(500);
}

void CatchTwo(int m_Time = 1000)
{
  
}

//Working
void FastLeft(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  // Move Motor A backward and Motor B forward
  //digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);
  
  //digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);      // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(m_Time);
}

//Working
void FastRight(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  // Move Motor A backward and Motor B forward
  //digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  
  //digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}

void Left(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  //digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  //(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}


void LeftBack(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  // Move Motor A backward and Motor B forward
  //digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  //digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(m_Time);
}


void RightBack(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  // Move Motor A forward and Motor B backward
  //digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, LOW);     // Set direction
  digitalWrite(motorAPin2, HIGH);

  //digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}


void Right(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  //digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  //digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}

//Working
void Forward(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  //digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  //digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(m_Time);
}

//Working
void Back(int m_Time = 1000, int mSpeeds = 100)
{
  analogWrite(motorAEnablePin, mSpeeds);
  analogWrite(motorBEnablePin, mSpeeds);
  
  //digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  //digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}

//Working
void Stop(int m_Time = 1000)
{
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
  delay(m_Time);
}
