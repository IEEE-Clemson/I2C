#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Stepper.h>

const int pCell = 28;
const int ledPin = 16;
const int servo1Pin = 2;
const int servo2Pin = 3;


// digital pins for distance_sensor(non pwm)
const int sensorTrig = 37;
const int sensorEcho = 39;

// stepper pins
const int stepperDir = 15;
const int stepperStep = 14;
const int stepsPerRevolution = 63;

//motor pins
const int motorA1 = 21;
const int motorA2 = 20;
const int motorB1 = 19;
const int motorB2 = 18;


Servo servo1;
Servo servo2;

void setup(){
  // I2C setup
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);
  
  // Pins initailization
  pinMode(pCell, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(stepperDir, OUTPUT);
  pinMode(stepperStep, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Motor direction initilize
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, HIGH);
  digitalWrite(motorB2, LOW);
}

void receiveEvent(int x){
  uint8_t msg = Wire.read();
  Serial.println(msg);
  if(msg==0x01){
    red_light();
  }else if(msg==0x02){
    run_servo1();
  }else if(msg==0x03){
    run_servo2();
  }else if((msg & 0x0F) == 0x04){
    carriage_up(msg >> 4);
  }else if((msg  & 0x0F) == 0x05){
    carriage_down(msg>>4);
  }else if((msg & 0x0F) == 0x06){
    move_flicker(16*(msg >> 4));
  }else if(msg == 0x07){
    move_rollers(90);
  }
}

void red_light(){
  int val = analogRead(pCell);
  Wire.write(val>550 ? 0x01 : 0x00);
}

void run_servo1(){
  for(int i=0;i<180;i++){ 
    servo1.write(i);
    delay(10);                    
  }
  for(int i=180;i>0;i--){ 
    servo1.write(i);
    delay(5);                    
  }
  Wire.write(0x04);
}

void run_servo2(){
  for(int i=0;i<180;i++){ 
    servo2.write(i);
    delay(10);                    
  }
  for(int i=180;i>0;i--){ 
    servo2.write(i);
    delay(5);                    
  }
  Wire.write(0x05);
}

void requestEvent(){
  int val = analogRead(pCell);
  Wire.write(val>550 ? 0x01 : 0x00);
}

int disSensor(){
  pinMode(sensorTrig, OUTPUT);
  digitalWrite(sensorTrig, LOW);
  delay(2);
  digitalWrite(sensorTrig, HIGH);
  delay(10);
  digitalWrite(sensorTrig, LOW);
  pinMode(sensorEcho, INPUT);
  
  // distance in cm
  long distance = pulseIn(sensorEcho, HIGH)/58;
  Wire.write((distance<30 && distance >25) ? 0x03 : 0x02);
  delay(100);
  return 0;
}

// move carriage by certain distance revolutions
void carriage_up(uint8_t distance){
  digitalWrite(stepperDir, HIGH);
  for(int i=0;i<distance;i++){
    for (int j=0; j<stepsPerRevolution; j++) {
      digitalWrite(stepperStep, HIGH);
      delay(2);
      digitalWrite(stepperStep, LOW);
      delay(2);
    }
  }
  Wire.write(0x04);
}

void carriage_down(uint8_t distance){
  digitalWrite(stepperDir, LOW);
  for(int i=0;i<distance;i++){
    for (int j=0; j<stepsPerRevolution; j++) {
      digitalWrite(stepperStep, HIGH);
      delay(2);
      digitalWrite(stepperStep, LOW);
      delay(2);
    }
  }
  Wire.write(0x07);
}

void move_flicker(uint8_t speed){
  analogWrite(motorB1, speed%255);
  digitalWrite(motorB2, LOW);
  delay(100);
  Wire.write(0x08);
}

void move_rollers(uint8_t speed){
  analogWrite(motorA1, speed);
  digitalWrite(motorA2, LOW);
  delay(100);
  Wire.write(0x09);
}

void loop(){
  digitalWrite(ledPin, HIGH);
}
