#include <SPI.h>
#include <Servo.h>

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

// Define motor control pins for Motor C
Servo myServo;
const int defaultAngle = 90;

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
  myServo.write(defaultAngle);
  
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
void LeftBack(int);
void RightBack(int);
void FastLeft(int);
void FastRight(int);
void Stop(int);

void loop() {
  Left(475);
  Forward(1000);
  Stop(500);
}

void TravelHalfTo(float distance)
{
  //Travel half the distance from the ball to the car
  if(distance == lastDistance/2)
  {
    lastDistance = distance;
    //Scan and Readjust Here
  }

 //If too close to the ball, forward
 if(distance < goalDistance)
 {
  //Move Back 5 ms?
 }
 //If too far from the ball, forward
 if(distance > goalDistance)
 {
  //Move Forward 5 ms?
 }
 //Set this to a Distance Range maybe instead like "if between 2.3 meters and 2.6 meters, the range should be around the size of the ball
 if(distance == goalDistance)
 {
  //Catch
  Catch();
 }
}

void Catch(int time = 1000)
{
  myServo.write(180);
  delay(time);
}

//Working
void FastLeft(int time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);
  
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);      // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}

//Working
void FastRight(int time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}

void Left(int time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}


void LeftBack(int time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}


void RightBack(int time = 1000)
{
  // Move Motor A forward and Motor B backward
  digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, LOW);     // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}


void Right(int time = 1000)
{
  digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}

//Working
void Forward(int time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}

//Working
void Back(int time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}

//Working
void Stop(int time = 1000)
{
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
  delay(time);
}
