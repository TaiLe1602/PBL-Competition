#include <SPI.h>
#include <Pixy2.h>
Pixy2 pixy2;
const int motorAEnablePin = 11;   // Enable pin for Motor A
const int motorAPin1 = 10;       // Control pin 1 for Motor A
const int motorAPin2 = 9;       // Control pin 2 for Motor A

// Define motor control pins for Motor B
const int motorBEnablePin = 6;   // Enable pin for Motor B
const int motorBPin1 = 7;        // Control pin 1 for Motor B
const int motorBPin2 = 8;        // Control pin 2 for Motor B
// Define ultrasonic control pins
int TrigPin = 12;
int EchoPin = 13;
int signature, x, y, width, height;
float cx, cy, area;
int threshold = 0.15;
void Right(int);
void Left(int);
void Forward(int);
void Back(int);
float pixyCheck();
float heightCheck();
void setup() {
  Serial.begin(9600);
  //pixy2.init();
// Define motor control pins for Motor A

  pinMode(motorAEnablePin, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);

  // Set the motor control pins as outputs for Motor B
  pinMode(motorBEnablePin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);

  // Initialize motors to stop
  /*digitalWrite(motorAEnablePin, LOW);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBEnablePin, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);*/
  // Define control pins for ultrasonic device
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}
void loop() {
  while(!blocks){
    int i=0;
    while(i<13){
      Right(500);
      i++;
    }
    if(i==13){
      Forward(2000);
    }
  }
  // rotate to center
  float rotatecenter = pixyCheck();
  while((rotatecenter<=threshold) && (rotatecenter>=threshold)){
    if(rotatecenter<0){
      Right(1000);
    } else {
      Left(1000);
    }
    rotatecenter = pixyCheck();
  }
  // move toward to the ball
  float heightball = heightCheck();
  while(heightball <100){
    Forward(2000);
  }
}
#pragma region wtf
void Ultrasonicdistance(){
  digitalWrite(TrigPin, HIGH);
  digitalWrite(TrigPin, LOW);
  Serial.print("\n");
  float t = pulseIn(EchoPin,HIGH);
  float distance = t/58.3;
  // v= 331+ 0.06*T (T is temperature)
  Serial.print(distance);
  Serial.print("\n");
  delay(1000);
  }

void Left(int time) {
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBEnablePin, LOW); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}
void Right(int time) {
  // Move Motor A forward and Motor B backward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);     // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}
void Forward(int time = 1000) {
  analogWrite(motorAEnablePin, 127);
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  analogWrite(motorBEnablePin, 127);
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}
void Back(int time) {
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}
void Stop(int time) {
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
  delay(time);
}
void TurnRight() {
  // Move Motor A backward and Motor B forward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
}

void TurnLeft()
{
  // Move Motor A forward and Motor B backward
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);     // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
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
}
void MoveBackward()
{
  digitalWrite(motorAEnablePin, HIGH); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);      // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, HIGH); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
}
void Stop() {
  digitalWrite(motorAEnablePin, LOW); // Disable Motor A
  digitalWrite(motorBEnablePin, LOW); // Disable Motor B
}
#pragma endregion

float reversefigure(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
float pixyCheck() {
  static int i = 0;
  uint16_t blocks;
  char buf[32];
  blocks = pixy2.ccc.getBlocks();
  if (blocks) {
    Serial.print(signature = pixy2.ccc.blocks[0].m_signature);
    Serial.print("\n");
    Serial.print(height = pixy2.ccc.blocks[0].m_height);
    Serial.print("\n");
    Serial.print(width = pixy2.ccc.blocks[0].m_width);
    Serial.print("\n");
    Serial.print(x = pixy2.ccc.blocks[0].m_x);
    Serial.print("\n");
    Serial.print(y = pixy2.ccc.blocks[0].m_y);
    Serial.print("\n");
    Serial.print(cx = (x + (width / 2)));
    Serial.print("\n");
    Serial.print(cy = (y + (height / 2)));
    Serial.print("\n");
    Serial.print(cx = reversefigure(cx, 0, 320, -1, 1));
    Serial.print("\n");
    Serial.print(cy = reversefigure(cy, 0, 200, 1, -1));
    Serial.print("\n");
    Serial.print(area = width * height);
    Serial.print("\n");
  }
  return cx;
}
float heightCheck() {
  static int i = 0;
  uint16_t blocks;
  char buf[32];
  blocks = pixy2.ccc.getBlocks();
  if (blocks) {
    Serial.print(signature = pixy2.ccc.blocks[0].m_signature);
    Serial.print("\n");
    Serial.print(height = pixy2.ccc.blocks[0].m_height);
    Serial.print("\n");
    Serial.print(width = pixy2.ccc.blocks[0].m_width);
    Serial.print("\n");
    Serial.print(x = pixy2.ccc.blocks[0].m_x);
    Serial.print("\n");
    Serial.print(y = pixy2.ccc.blocks[0].m_y);
    Serial.print("\n");
    Serial.print(cx = (x + (width / 2)));
    Serial.print("\n");
    Serial.print(cy = (y + (height / 2)));
    Serial.print("\n");
    Serial.print(cx = reversefigure(cx, 0, 320, -1, 1));
    Serial.print("\n");
    Serial.print(cy = reversefigure(cy, 0, 200, 1, -1));
    Serial.print("\n");
    Serial.print(area = width * height);
    Serial.print("\n");
  }
  return height;
}

