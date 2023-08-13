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
void Forward()
{
  analogWrite(motorAEnablePin, 80); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  analogWrite(motorBEnablePin, 80); // Enable Motor B
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

void loop() {
  while(!pixy2.ccc.getBlocks()){
    int i;
    for (i = 0; i < 10; i++){
      Right();
      Serial.print("not Found");
      Serial.print("\n");
      delay(150);
      Stop();
      delay(400);
      if(pixy2.ccc.getBlocks()){
        break;
      }
    }
    if(i==10){
      Forward();
      Serial.print("new Position");
      Serial.print("\n");
      delay(500);
      i=0;
    }
  }
  Stop();
  delay(300);
  Serial.print("Found red ball");
  Serial.print("\n");
  // rotate to center
  float rotatecenter = pixyCheck();
  while((rotatecenter<=-threshold) || (rotatecenter>=threshold)){
    if(rotatecenter>0){
      Right();
      delay(50);
      Serial.print("Turn Right: \n");
      rotatecenter = pixyCheck();
      Serial.print(rotatecenter);
      Stop();
      delay(50);
    } else {
      Left();
      delay(50);
      Stop();
      delay(50);
      Serial.print("Turn Left: \n");
    }
    rotatecenter = pixyCheck();
  }
  // move toward to the ball
  Serial.print("Be center");
  float widthball = widthCheck();
  while(widthball <20){
    while((rotatecenter<=-threshold) || (rotatecenter>=threshold)){
    if(rotatecenter<0){
      Right();
      Stop();
      delay(300);
      delay(50);
      Serial.print("Turn Right: \n");
    } else {
      Left();
      delay(50);
      Stop();
      delay(300);
      Serial.print("Turn Left: \n");
    }
    rotatecenter = pixyCheck();
  }
    Forward();
    delay(100);
    Stop();
    delay(100);
    width                                                                                                             ball = widthCheck();
    Serial.print("Height is smaller than 20 \n");
  }
  Stop();
  delay(300);
  while((-heightball*20+200)>0){//decrease velocity from 20 height to 40 height
  analogWrite(motorAEnablePin, (-heightball*20+200)); // Enable Motor A
  digitalWrite(motorAPin1, LOW);      // Set direction
  digitalWrite(motorAPin2, HIGH);
  analogWrite(motorBEnablePin, (-heightball*20+200)); // Enable Motor B
  digitalWrite(motorBPin1, LOW);     // Set direction
  digitalWrite(motorBPin2, HIGH);
  delay(100);
  heightball = widthCheck();
  }
  if(heightball <40){
    Serial.write("error the height is small");
  }
  Stop();
  delay(5000);
}


