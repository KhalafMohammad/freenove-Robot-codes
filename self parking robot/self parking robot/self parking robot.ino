#include "Servo.h"

#define PIN_SERVO         2    
#define PIN_SONIC_TRIG    7    
#define PIN_SONIC_ECHO    8    
#define PIN_TRACKING_LEFT   A1
#define PIN_TRACKING_CENTER A2
#define PIN_TRACKING_RIGHT  A3
#define PIN_DIRECTION_RIGHT 3
#define PIN_DIRECTION_LEFT  4
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_MOTOR_PWM_LEFT  6
#define MAX_DISTANCE    200   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60) // calculate timeout 
#define SOUND_VELOCITY  340  //soundVelocity: 340m/s

byte servoOffset = 0;    //change the value to Calibrate servo
// u8 distance[4];

Servo servo;


void setup(){
    Serial.begin(9600);
    pinMode(PIN_TRACKING_LEFT, INPUT); 
    pinMode(PIN_TRACKING_RIGHT, INPUT); 
    pinMode(PIN_TRACKING_CENTER, INPUT); 
    pinMode(PIN_SONIC_TRIG, OUTPUT);
    pinMode(PIN_SONIC_ECHO, INPUT); 
    servo.attach(PIN_SERVO);
    servo.write(30);
}


int distance;

void loop(){
  servo.write(30);
  distance = getSonar();
  delay(50);  
  
  if (distance < 30)
  {
    motorRun(100, 100);
  }
  
  else
  {
    motorRun(0,0);
    delay(1000);
    motorRun(170,0);
    delay(1000);
    motorRun(0,0);
    delay(1000);
    servo.write(90);
    distance = getSonar();
    while(1){
      
      if (getSonar() <= 7)
      {
        motorRun(0,0);
        delay(1000);
        motorRun(-170,170);
        delay(840);
        motorRun(0,0);
        delay(1000);
        servo.write(30);
        exit(1);
      }
      else
      {
        motorRun(100, 100);
      }
    }
    
  }

}

void motorRun(int speedl, int speedr) {
  int dirL = 0, dirR = 0;
  if (speedl > 0) {
    dirL = 0;
  }
  else {
    dirL = 1;
    speedl = -speedl;
  }
  if (speedr > 0) {
    dirR = 1;
  }
  else {
    dirR = 0;
    speedr = -speedr;
  }
  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10Î¼s to triger HC_SR04,
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_DISTANCE;
  return distance; // return the distance value
}