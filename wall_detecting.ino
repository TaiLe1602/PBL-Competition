#include <SPI.h>
#include <math.h>
#include <Pixy2.h>
Pixy2 pixy2;
float mapfloat(long,long,long,long,long);

const int motorAEnablePin = 5;   // Enable pin for Motor A
const int motorAPin1 = 11;       // Control pin 1 for Motor A
const int motorAPin2 = 12;       // Control pin 2 for Motor A

// Define motor control pins for Motor B
const int motorBEnablePin = 6;   // Enable pin for Motor B
const int motorBPin1 = 10;        // Control pin 1 for Motor B
const int motorBPin2 = 9;        
int TrigPin = 2;
int EchoPin = 3;


float threshold = 0.15;
int basespeed = 127;
int cont = 0;
int signature, x, y, width, height;
float cx, cy, area;


void Left()
{
  // Move Motor A forward and Motor B backward
  analogWrite(motorAEnablePin, 80); // Enable Motor A
  digitalWrite(motorAPin1, LOW);     // Set direction
  digitalWrite(motorAPin2, HIGH);

  analogWrite(motorBEnablePin, 80); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);      // Set direction
  digitalWrite(motorBPin2, LOW);
}
void Right()
{
  // Move Motor A forward and Motor B backward
  analogWrite(motorAEnablePin, 80); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);     // Set direction
  digitalWrite(motorAPin2, LOW);

  analogWrite(motorBEnablePin, 80); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
}

void Backward()
{
  analogWrite(motorAEnablePin, 80); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  analogWrite(motorBEnablePin, 80); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
}
void Forward(int sp)
{
  analogWrite(motorAEnablePin, sp); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  analogWrite(motorBEnablePin, sp); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
}



int checkdistance(){
  digitalWrite(TrigPin, HIGH);
  digitalWrite(TrigPin, LOW);
  Serial.print("\n");
  float distance = pulseIn(EchoPin,HIGH)/58.3;
  Serial.print(distance);
  return distance;
}

void Stop()
{
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
}
void setup() {
  Serial.begin(9600);
  pixy2.init();
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
float reversefigure(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float pixyCheck() {
  static int i = 0;
  uint16_t blocks;
  char buf[32];
  blocks = pixy2.ccc.getBlocks();
  if (blocks) {
    Serial.print(signature = pixy2.ccc.blocks[0].m_signature);
    Serial.print("\n Making center \n");
    Serial.print(x = pixy2.ccc.blocks[0].m_x);
    Serial.print("\n");
    Serial.print(cx=reversefigure(x,0,320,-1,1));
    Serial.print("\n");
  }
  return cx;
}
float widthCheck() {
  static int i = 0;
  uint16_t blocks;
  char buf[32];
  blocks = pixy2.ccc.getBlocks();
  if (blocks) {
    Serial.print(width = pixy2.ccc.blocks[0].m_width);
    Serial.print("\n");
  }
  return width;
}
int Ultrasonic_distance(){
  digitalWrite(TrigPin,HIGH); //發射超音波
  delay(1);
  digitalWrite(TrigPin,LOW);
  return pulseIn(EchoPin, HIGH)/58.3;
}

void loop() {
  Serial.print(Ultrasonic_distance());
  Serial.print("\n");
  int s = Ultrasonic_distance();
  if(s>15){
    Serial.print(127*sin(0.00922496*s-1.71009)+217);
    Forward(127*sin(0.00922496*s-1.71009)+217);
  }else{
    Stop();
  }
}