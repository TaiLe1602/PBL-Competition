#include <Pixy2.h>
Pixy2 pixy;
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

int catching_distance;

void Left(int);
void Right(int);
void Forward(int);
void Back(int);

void setup() {
  Serial.begin(9600);
  pixy.init();
// Define motor control pins for Motor A

  pinMode(motorAEnablePin, OUTPUT);
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);

  // Set the motor control pins as outputs for Motor B
  pinMode(motorBEnablePin, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);

  // Initialize motors to stop
  digitalWrite(motorAEnablePin, LOW);
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBEnablePin, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
  // Define control pins for ultrasonic device
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(pixyCheck() > 0.1){    
    Right(1000);
  }
  while(pixyCheck() < -0.1){
    Left(1000);
  }
  while(Ultrasonic_distance() > catching_distance){
    Forward(int time = 1000)
  }
}

int Ultrasonic_distance(){
  digitalWrite(TrigPin, HIGH);
  digitalWrite(TrigPin, LOW);
  Serial.print("\n");
  t = pulseIn(EchoPin,HIGH);
  distance = t/58.3;
  return distance();
}

void Left(int time = 1000)
{
  // Move Motor A backward and Motor B forward
  analogWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  analogWrite(motorBEnablePin, 127); // Enable Motor B
  digitalWrite(motorBPin1, HIGH);     // Set direction
  digitalWrite(motorBPin2, LOW);
  delay(time);
}

void Right(int time = 1000)
{
  // Move Motor A forward and Motor B backward
  digitalWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, HIGH);     // Set direction
  digitalWrite(motorAPin2, LOW);

  digitalWrite(motorBEnablePin, 127); // Enable Motor B
  digitalWrite(motorBPin1, LOW);      // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(time);
}

void Forward(int time = 1000)
{
  digitalWrite(motorAEnablePin, 127); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);

  digitalWrite(motorBEnablePin, 127); // Enable Motor B
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

float pixyCheck() {
  uint16_t blocks;
  blocks = pixy.ccc.getBlocks();
  if (blocks) {
    int signature = pixy.ccc.blocks[0].m_signature;
    int height = pixy.ccc.blocks[0].m_height;
    int width = pixy.ccc.blocks[0].m_width;
    int x = pixy.ccc.blocks[0].m_x;
    int y = pixy.ccc.blocks[0].m_y;
    int cx = (x + (width / 2));
    int cy = (y + (height / 2));
    cx = mapfloat(x, 0, 320, -1, 1);
    cy = mapfloat(y, 0, 200, 1, -1);
    int area = width * height;
    return cx;
  }
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}