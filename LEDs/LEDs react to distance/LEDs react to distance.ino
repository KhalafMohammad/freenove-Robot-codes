
#include <Servo.h>
#include "Freenove_WS2812B_RGBLED_Controller.h"
#define PIN_SERVO           2       

#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5

#define PIN_SONIC_TRIG      7
#define PIN_SONIC_ECHO      8

#define PIN_BATTERY         A0

#define OBSTACLE_DISTANCE   60
#define OBSTACLE_DISTANCE_LOW 30

#define MAX_DISTANCE    200   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s
#define LEDS_COUNT   10  //it defines number of lEDs. 
#define I2C_ADDRESS  0x20

Servo servo;
byte servoOffset = 0;
int speedOffset;//batteryVoltageCompensationToSpeed
Freenove_WS2812B_Controller strip(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB);

void setup() {
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
  servo.attach(PIN_SERVO);
  calculateVoltageCompensation();
}

void loop() {
  updateAutomaticObstacleAvoidance();
}

void updateAutomaticObstacleAvoidance() {
  int distance[3], tempDistance[3][5], sumDisntance;
  static u8 leftToRight = 0, servoAngle = 0, lastServoAngle = 0;
  const u8 scanAngle[2][3] = { {170, 90, 20}, {20, 90, 170} };

  for (int i = 0; i < 3; i++)
  {
    servoAngle = scanAngle[leftToRight][i];
    servo.write(servoAngle);
    if (lastServoAngle != servoAngle) {
      delay(500);
    }
    lastServoAngle = servoAngle;
    for (int j = 0; j < 5; j++) {
      tempDistance[i][j] = getSonar();
      delayMicroseconds(2 * SONIC_TIMEOUT);
      sumDisntance += tempDistance[i][j];
    }
    if (leftToRight == 0) {
      distance[i] = sumDisntance / 5;
    }
    else {
      distance[2 - i] = sumDisntance / 5;
    }
    sumDisntance = 0;
  }
  leftToRight = (leftToRight + 1) % 2;

                                                   //Too little distance ahead

    if (distance[0] > OBSTACLE_DISTANCE_LOW) {                          // obstacle is far
      strip.setLedColor(0, 0, 255, 0);
      strip.setLedColor(1, 0, 255, 0);
      strip.setLedColor(2, 0, 255, 0);
      strip.setLedColor(3, 0, 255, 0);
      strip.setLedColor(4, 0, 255, 0);
      strip.setLedColor(5, 0, 0, 0);
      strip.setLedColor(6, 0, 0, 0);
      strip.setLedColor(7, 0, 0, 0);
      strip.setLedColor(8, 0, 0, 0);
      strip.setLedColor(9, 0, 0, 0);
      strip.show();

    }
    else if (distance[0] <  OBSTACLE_DISTANCE_LOW) {                 //Obstacle is near
    strip.setLedColor(0, 255, 0, 0);
    strip.setLedColor(1, 255, 0, 0);
    strip.setLedColor(2, 255, 0, 0);
    strip.setLedColor(3, 255, 0, 0);
    strip.setLedColor(4, 255, 0, 0);
    strip.setLedColor(5, 0, 0, 0);
    strip.setLedColor(6, 0, 0, 0);
    strip.setLedColor(7, 0, 0, 0);
    strip.setLedColor(8, 0, 0, 0);
    strip.setLedColor(9, 0, 0, 0);        
    strip.show();
    }
    else {
    strip.setAllLedsColor(0, 0, 0);                              //set all LED off .
    strip.show();
    }

    if (distance[2] > OBSTACLE_DISTANCE_LOW) {                   // obstacle is far
    strip.setLedColor(0, 0, 0, 0);
    strip.setLedColor(1, 0, 0, 0);
    strip.setLedColor(2, 0, 0, 0);
    strip.setLedColor(3, 0, 0, 0);
    strip.setLedColor(4, 0, 0, 0);
    strip.setLedColor(5, 0, 255, 0);
    strip.setLedColor(6, 0, 255, 0);
    strip.setLedColor(7, 0, 255, 0);
    strip.setLedColor(8, 0, 255, 0);
    strip.setLedColor(9, 0, 255, 0);
    strip.show();
    }
    else if (distance[2] <  OBSTACLE_DISTANCE_LOW) {              //Obstacle is near
    strip.setLedColor(0, 0, 0, 0);
    strip.setLedColor(1, 0, 0, 0);
    strip.setLedColor(2, 0, 0, 0);
    strip.setLedColor(3, 0, 0, 0);
    strip.setLedColor(4, 0, 0, 0);
    strip.setLedColor(5, 255, 0, 0);
    strip.setLedColor(6, 255, 0, 0);
    strip.setLedColor(7, 255, 0, 0);
    strip.setLedColor(8, 255, 0, 0);
    strip.setLedColor(9, 255, 0, 0);
    strip.show();
    }
    else {
    strip.setAllLedsColor(0, 0, 0);                                //set all LED off .
    strip.show();
    }

}

float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_DISTANCE;
  return distance; // return the distance value
}

void calculateVoltageCompensation() {
  float voltageOffset = 8.4 - getBatteryVoltage();
  speedOffset = voltageOffset * 20;
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


float getBatteryVoltage() {
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = analogRead(PIN_BATTERY);
  float batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
  return batteryVoltage;
}
