#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <AccelStepper.h>

const int pCell = 28;
const int servo1Pin = 2;
const int servo2Pin = 3;
const int servo3Pin = 4;
const int servo4Pin = 5;

// digital pins for distance_sensor(non pwm)
const int sensorTrig = 37;
const int sensorEcho = 39;

// stepper pins
const int stepperDir = 19;
const int stepperStep = 18;
const int stepsPerRevolution = 63;

//motor pins
const int motorA1 = 9;
const int motorA2 = 8;
const int motorB1 = 7;
const int motorB2 = 6;

// Stepper & servo obj
AccelStepper stepper(AccelStepper::DRIVER, stepperStep, stepperDir);

// 
bool is_stepper = false;
bool is_servo1 = false;
bool is_servo2 = false;
bool is_servo3 = false;
bool is_servo4 = false;
int servo1_pos = 0;
int servo2_pos = 0;
int servo3_pos = 0;
int servo4_pos = 0;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

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
  pinMode(stepperDir, OUTPUT);
  pinMode(stepperStep, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);

  // SETUP STEPPER;
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(2000);
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
  }else if(msg==0x04){
    run_servo3();
  }else if(msg==0x05){
    run_servo4();
  }else if(msg == 0x06){
    move_carriage();
  }else if((msg & 0x0F) == 0x07){
    move_flicker(16*(msg >> 4));
  }else if(msg == 0x08){
    stop_flicker();
  }else if(msg == 0x09){
    move_rollers(90);
  }else if(msg == 0x0A){
    stop_rollers();
  }
}

void red_light(){
  int val = analogRead(pCell);
  Wire.write(val>550 ? 0x01 : 0x00);
  Serial.println(val);
}

void run_servo1(){
  is_servo1 = true;
}

void run_servo2(){
  is_servo1 = true;
}

void run_servo3(){
  is_servo1 = true;
}

void run_servo4(){
  is_servo1 = true;
}

void requestEvent(){
  return ;
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
void move_carriage(){
  is_stepper=true;
}

void move_flicker(uint8_t speed){
  analogWrite(motorB1, speed%255);
  digitalWrite(motorB2, LOW);
  delay(100);
  Wire.write(0x08);
}

void stop_flicker(){
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  Wire.write(0x08);
}

void move_rollers(uint8_t speed){
  analogWrite(motorA1, speed);
  digitalWrite(motorA2, LOW);
  delay(100);
  Wire.write(0x09);
}


void stop_rollers(){
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  Wire.write(0x09);
}

void loop(){
  if(is_servo1){
    if(servo1_pos<360){
      servo1.write((180*(servo1_pos>180))+((-1+2*(servo1_pos<=180))*(servo1_pos%180));     
      servo1_pos++;             
    }
    if(servo1_pos==360){
      Wire.write(0x04);
      servo1_pos = 0;
      is_servo1 = false;
    }
  }
  if(is_servo2){
    if(servo2_pos<360){
      servo2.write((180*(servo2_pos>180))+((-1+2*(servo2_pos<=180))*(servo2_pos%180));     
      servo2_pos++;             
    }
    if(servo2_pos==360){
      Wire.write(0x04);
      servo2_pos = 0;
      is_servo2 = false;
    }
  }
  if(is_servo3){
    if(servo3_pos<360){
      servo3.write((180*(servo3_pos>180))+((-1+2*(servo3_pos<=180))*(servo3_pos%180));     
      servo3_pos++;             
    }
    if(servo3_pos==360){
      Wire.write(0x04);
      servo3_pos = 0;
      is_servo3 = false;
    }
  }
  if(is_servo4){
    if(servo4_pos<360){
      servo4.write((180*(servo4_pos>180))+((-1+2*(servo4_pos<=180))*(servo4_pos%180));     
      servo4_pos++;             
    }
    if(servo4_pos==360){
      Wire.write(0x04);
      servo4_pos = 0;
      is_servo4 = false;
    }
  }
  if(is_stepper){
    stepper.moveTo(-40000);
    while(stepper.currentPosition() > -6500/4) {
      stepper.run();
    }
    stepper.setSpeed(0);
    stepper.setCurrentPosition(-6450/4);
    stepper.moveTo(-6450/4);
    //stepper.runToPosition();
    stepper.moveTo(-6300/4);
    stepper.runToPosition();
    delay(4000);
    
    stepper.moveTo(0);
    stepper.runToPosition();
    delay(10000);
  }
}
