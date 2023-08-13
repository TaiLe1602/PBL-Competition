#include <SPI.h>
#include <math.h>
#include <Pixy2.h>
Pixy2 pixy2;
float mapfloat(long,long,long,long,long);

const int motorAEnablePin = 11;   // Enable pin for Motor A
const int motorAPin1 = 10;       // Control pin 1 for Motor A
const int motorAPin2 = 9;       // Control pin 2 for Motor A

// Define motor control pins for Motor B
const int motorBEnablePin = 6;   // Enable pin for Motor B
const int motorBPin1 = 7;        // Control pin 1 for Motor B
const int motorBPin2 = 8;        
int TrigPin = 2;
int EchoPin = 3;



float threshold = 0.15;
int basespeed = 127;

int cont = 0;
int signature, x, y, width, height;
float cx, cy, area;
void Left(int time = 1000)
{
  // Move Motor A backward and Motor B forward
  analogWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  analogWrite(motorBEnablePin, 127); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
  }
void MoveForward()
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(1000);
void Right(int time = 1000)
{
  // Move Motor A forward and Motor B backward
  analogWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);     // Set direction
  digitalWrite(motorAPin2, LOW);

  analogWrite(motorBEnablePin, 127); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);      // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}

void Backward(int time = 1000)
{
  analogWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  analogWrite(motorBEnablePin, 127); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}
void Forward(int time = 1000)
{
  analogWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  analogWrite(motorBEnablePin, 127); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  
}


int checkdistance(){
  digitalWrite(TrigPin, HIGH);
  digitalWrite(TrigPin, LOW);
  Serial.print("\n");
  float distance = pulseIn(EchoPin,HIGH)/58.3;
  Serial.print(distance);
  return distance;
}

void Stop(int time = 1000)
{
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
}
void setup() {
  Serial.begin(9600);
  pinMode(motorAEnablePin, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);

  // Set the motor control pins as outputs for Motor B
  pinMode(motorBEnablePin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
     //Arduino 對外啟動距離感測器Trig腳，射出超音波
    checkdistance();
    checkdistance();
  pinMode(TrigPin, OUTPUT);      // Arduino 對外啟動距離感測器Trig腳，射出超音波 
  pinMode(EchoPin, INPUT); 
}

void loop() {
  Forward();
  /*
  if (turn < 0) {
    moveRobot(-80, 170);
  }
  else if (turn > 0) {
    moveRobot(170, -80);
  }
  else {
    moveRobot(70, 70);
  }
  
   float k=48.687;
    float theta = 20;
    float delay1 = theta * k;
    float d_0;
    float d_1;
    float d_2;
    float d_3;
    float min_dist;
    float min_angle;
    float turn_angle;
    while (checkdistance() > 15) {
      Forward();
      }
      Stop();
      d_0 = checkdistance();

      Left(delay1);
      Stop();
      d_1 = checkdistance();
      
      Right(2*delay1);
      Stop();
      d_2 = checkdistance();

      if (d_1 > d_2) {
        d_3 = pow(pow(d_2, 2) + pow(d_1, 2) - 2*d_2*d_1*cos(theta), 0.5);
        min_dist = (d_1 * d_2 * sin(2*theta))/d_3;
        min_angle = acos(min_dist/d_2);
        turn_angle = min_angle + 90;
        Left((int)turn_angle * k);
        Stop();
      }
      else {
        d_3 = pow(pow(d_2, 2) + pow(d_1, 2) - 2*d_2*d_1*cos(theta), 0.5);
        min_dist = (d_1 * d_2 * sin(2*theta))/d_3;
        min_angle = acos(min_dist/d_1);
        turn_angle = min_angle - 90;
        Right((int)turn_angle * k);
        Stop();
      }
  delay(1);*/
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}