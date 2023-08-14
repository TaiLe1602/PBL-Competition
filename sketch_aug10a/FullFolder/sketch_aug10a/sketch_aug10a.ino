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

// Define the pins for the limit switch and an LED (optional)
const int limitSwitchPin = 5;  // Change this to the pin you've connected the limit switch to

// Distance Sensors
int TrigPin = 12;
int EchoPin = 13;
  
// Arm Catching Motor
// Define motor control pins for Motor C; The Servo motor
Servo myServo;
const int servoPin = 5;

// Define Compass Class
//QMC5883LCompass compass;

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
  myServo.attach(servoPin);

  // Set ultrasonic Sensor pins to In/Out
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  // Set the Pins
  pinMode(limitSwitchPin, INPUT);  // Set the limit switch pin as an input
  pinMode(LED_BUILTIN, OUTPUT);         // Set the LED pin as an output
  digitalWrite(LED_BUILTIN, LOW);      // Turn off the LED initially

  // Initialize Compass
  //scompass.init();

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

  
  
  // Read Compass Numbers
  // compass.read();

  // byte a = compass.getAzimuth();
  // // Output here will be a value from 0 - 15 based on the direction of the bearing / azimuth.
  // byte b = compass.getBearing(a);
  
  // Serial.print("B: ");
  // Serial.print(b);
  // Serial.println();
  
  // delay(250);

  // //digitalWrite(TrigPin, HIGH);
  // //digitalWrite(TrigPin, LOW);

  int limitSwitchState = digitalRead(limitSwitchPin);  // Read the state of the limit switch
  
  if (limitSwitchState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on the LED when the limit switch is activated
    Serial.println("Limit switch activated!");
  } else {
    digitalWrite(LED_BUILTIN, LOW);   // Turn off the LED when the limit switch is not activated
  }
  
  // if(Ultrasonic_distance() > catching_distance){
  //   Serial.write("Forward Distance: " + (int)Ultrasonic_distance());
  // }
  
  // delay(3000);
  // SimpleCatch();
  // delay(1000);
  // Deposit();
}

void SimpleCatch()
{  
  //Rest Position
  myServo.write(90);
  Serial.write("90 ");
  delay(1000);
  //Down
  myServo.write(180);
  delay(1000);
  //Up
  myServo.write(125);
  delay(1000);
}

void Deposit()
{
  //Up
  myServo.write(45);
  delay(1000);
  //Rest Position
  myServo.write(90);
  delay(1000);
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
