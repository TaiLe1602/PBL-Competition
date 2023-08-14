#include <SPI.h>
#include <Pixy2.h>

Pixy2 pixy2;

int myPins[6] = {5, 6, 7, 8, 9, 10};
float threshold = 0.15;
int basespeed = 130;

int cont = 0;
int signature, x, y, width, height;
float cx, cy, area;

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pixy2.init();
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
}

void loop() {
  float turn = pixyCheck();
  if (turn > -threshold && turn < threshold) {
    turn = 0;
  }
 // if (turn < 0) {
    //moveRobot(-80, 170);
  //
  //else if (turn > 0) {
  //  moveRobot(170, -80);
  //}
  //else {
   // moveRobot(70, 70);
  //}
  delay(1);
}

float pixyCheck() {
  static int i = 0;
  int j;
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
    Serial.print(cx = mapfloat(cx, 0, 320, -1, 1));
    Serial.print("\n");
    Serial.print(cy = mapfloat(cy, 0, 200, 1, -1));
    Serial.print("\n");
    Serial.print(area = width * height);
    Serial.print("\n");
  }
  else {
    cont += 1;
    if (cont == 100) {
      cont = 0;
      cx = 0;
    }
  }
  return cx;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
