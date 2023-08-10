int TrigPin = 2;
int EchoPin = 3;
double distance, t;
void setup(){
  Serial.begin(9600);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}
void loop(){
  digitalWrite(TrigPin, HIGH);
  digitalWrite(TrigPin, LOW);
  Serial.print("\n");
  t = pulseIn(EchoPin,HIGH);
  distance = t/58.3;
  Serial.print(distance);
  Serial.print("\n");
  delay(1000);
  }