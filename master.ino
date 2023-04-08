#include <Wire.h>

void setup(){
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  Serial.begin(9600);

}
char msg = -1;
void loop() {
  msg = 0x02;
  Wire.beginTransmission(0x08);
  Wire.write(msg);
  Wire.endTransmission();
  delay(200);
  
  msg = 0x03;
  Wire.beginTransmission(0x08);
  Wire.write(msg);
  Wire.endTransmission();
  delay(200);
  
  msg = 0xA4;
  Wire.beginTransmission(0x08);
  Wire.write(msg);
  Wire.endTransmission();
  delay(200);

  msg = 0x55;
  Wire.beginTransmission(0x08);
  Wire.write(msg);
  Wire.endTransmission();
  delay(200);
}
