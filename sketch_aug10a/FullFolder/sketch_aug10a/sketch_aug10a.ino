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
const int motorBEnablePin = 6;   // Enable pin for Motor B
const int motorBPin1 = 7;        // Control pin 1 for Motor B
const int motorBPin2 = 8;        // Control pin 2 for Motor B

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
  myServo.attach(9);

  // Set the Limit Switch Pins as Input
  //pinMode(limitSwitchPin, INPUT);
  
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
void Catch(int m_Time, float ratioA, float ratioB);

void loop() {
  Catch(1000, 1, 0.18f);
}

void MoveBackUntil(int angle)
{
  while( myServo.read() != angle)
  {
    myServo.write(135);
    Serial.write(myServo.read());
  }
}

void MoveForwardUntil(int angle)
{
  while(myServo.read() != angle)
  {
    myServo.write(45);
    Serial.write(myServo.read());
  }
}

//bool BumperHit => digitalRead(limitSwitchPin) == HIGH ? true : false;

void LightUp()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

//void TravelHalfTo(float distance)
//{
//  //Travel half the distance from the ball to the car
//  if(distance == lastDistance/2)
//  {
//    lastDistance = distance;
//    //Scan and Readjust Here
//  }
//
// //If too close to the ball, forward
// if(distance < goalDistance)
// {
//  //Move Back 5 ms?
// }
// //If too far from the ball, forward
// if(distance > goalDistance)
// {
//  //Move Forward 5 ms?
// }
// //Set this to a Distance Range maybe instead like if between 2.3 meters and 2.6 meters, the range should be around the size of the ball
// if(distance == goalDistance)
// {
//  //Catch
//  Catch(1000);
// }
//}

void Catch(int m_Time = 1000, float ratioA = 1, float ratioB = 1)
{
  myServo.write(90 - ratioA * 90);
  delay(m_Time);
  myServo.write(90 + ratioB * 90);
  delay(m_Time * (1 + ratioB) * 2);
}

void ResetAngle(int m_Time = 1000)
{
  
}

//Working
void FastLeft(int m_Time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);
  
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);      // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(m_Time);
}

//Working
void FastRight(int m_Time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}

void Left(int m_Time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}


void LeftBack(int m_Time = 1000)
{
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(m_Time);
}


void RightBack(int m_Time = 1000)
{
  // Move Motor A forward and Motor B backward
  digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, LOW);     // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}


void Right(int m_Time = 1000)
{
  digitalWrite(motorAEnablePin, LOW); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(m_Time);
}

//Working
void Forward(int m_Time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(m_Time);
}

//Working
void Back(int m_Time = 1000)
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
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
