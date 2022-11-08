#include <Servo.h>
#define PIN_SERVO           2       

#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5

#define PIN_SONIC_TRIG      7
#define PIN_SONIC_ECHO      8

#define PIN_BATTERY         A0

#define OBSTACLE_DISTANCE   40
#define OBSTACLE_DISTANCE_LOW 15
#define DEBUG

// #define TK_STOP_SPEED 0
// #define TK_FORWARD_SPEED (100 + tk_VoltageCompensationToSpeed)
// #define TK_BACKWARD_SPEED (-100 + tk_VoltageCompensationToSpeed)

// //define different speed levels
// #define TK_TURN_SPEED_LV4 (150 + tk_VoltageCompensationToSpeed)
// #define TK_TURN_SPEED_LV3 (200 + tk_VoltageCompensationToSpeed)
// #define TK_TURN_SPEED_LV2 (-200 + tk_VoltageCompensationToSpeed)
// #define TK_TURN_SPEED_LV1 (-150 + tk_VoltageCompensationToSpeed)

// int tk_VoltageCompensationToSpeed;  //define Voltage Speed Compensation

#define MAX_DISTANCE    200   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

Servo servo;
byte servoOffset = 0;
int speedOffset;//batteryVoltageCompensationToSpeed
u8 distance[4];

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

void setup() {
  Serial.begin(9600); 
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  Serial.begin(9600);
  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
  servo.attach(PIN_SERVO);
  calculateVoltageCompensation();
}

void loop() {
  // int distance[3], tempDistance[3][5], sumDisntance;
  // static u8 leftToRight = 0, servoAngle = 0, lastServoAngle = 0;
  // const u8 scanAngle[2][3] = { {179, 90, 2}, {2, 90, 179} };

  // for (int i = 0; i < 3; i++)
  // {
  //   servoAngle = scanAngle[leftToRight][i];
  //   servo.write(servoAngle);
  //   if (lastServoAngle != servoAngle) {
  //     delay(150);
  //   }
  //   lastServoAngle = servoAngle;
  //   for (int j = 0; j < 5; j++) {
  //     tempDistance[i][j] = getSonar();
  //     delayMicroseconds(2 * SONIC_TIMEOUT);
  //     sumDisntance += tempDistance[i][j];
  //   }
  //   if (leftToRight == 0) {
  //     distance[i] = sumDisntance / 5;
  //   }
  //   else {
  //     distance[2 - i] = sumDisntance / 5;
  //   }
  //   sumDisntance = 0;
  // }
  // leftToRight = (leftToRight + 1) % 2;

  servo.write(179 + servoOffset);
  delay(1000);
  distance[2] = getSonar();
  servo.write(2 + servoOffset);
  delay(1000);
  distance[0] = getSonar();
  servo.write(90 + servoOffset);
  delay(1000);
  distance[1] = getSonar();
  
// ifdef DEBUG
// Serial.print("Distance L / M / R :   ");  //Left/Middle/Right/Middle2
//   for (int i = 0; i < 3; i++) {
//     Serial.print(distance[i]);
//     Serial.print("/");
//   }
//   Serial.print('\n');  //next content will be printed in new line
// enddef 

  if (distance[0] < 40 || distance[2] < 40 || distance[1] < 40) {
    motorRun(0,0);  //car stop
    servo.write(179 + servoOffset);
    delay(1000);
    distance[2] = getSonar();
    servo.write(2 + servoOffset);
    delay(1000);
    distance[0] = getSonar();
    servo.write(90 + servoOffset);
    delay(1000);
    distance[1] = getSonar();

    
      if (distance[0] < distance[2]) {
       motorRun(-(150 + speedOffset), (150 + speedOffset));
        delay(250);
        motorRun(0, 0);
      }
      if (distance[0] > distance[2]) {
        motorRun((150 + speedOffset), -(150 + speedOffset));
        delay(250);
        motorRun(0, 0);
      }
      // if (distance[1] > distance[2]) {
      //   motorRun((150 + speedOffset) ,(150 + speedOffset));
      //   delay(250);
      //   motorRun(0 , 0);
      // }
      if (distance[1] > distance[2] && distance[1] > distance[0]) {
        motorRun((150 + speedOffset) ,(150 + speedOffset));
        delay(150);
        motorRun(0 , 0);
      }
  }
  else if(distance[1]>50 && distance[2] > 50 && distance[0] > 50){
    motorRun(120, 120);
    delay(2000);
    motorRun(200, -200);
    delay(1100);
    motorRun(120,120);
    delay(2000);
    motorRun(0,0);
    delay(90000);
    return (0);
  }
}  
  