#include "Servo.h" //include servo library
#include "Freenove_WS2812B_RGBLED_Controller.h"
#define PIN_SERVO      2
#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_SONIC_TRIG    7
#define PIN_SONIC_ECHO    8
#define PIN_IRREMOTE_RECV 9
#define PIN_SPI_CE      9
#define PIN_SPI_CSN     10
#define PIN_SPI_MOSI    11
#define PIN_SPI_MISO    12
#define PIN_SPI_SCK     13
#define PIN_BATTERY     A0
#define PIN_BUZZER      A0
#define PIN_TRACKING_LEFT A1
#define PIN_TRACKING_CENTER A2
#define PIN_TRACKING_RIGHT  A3
#define MOTOR_PWM_DEAD    10
#define PIN_SERVO 2 //define servo pin


#define MAX_DISTANCE 200 //cm
#define SONIC_TIMEOUT (MAX_DISTANCE*60) // calculate timeout
#define SOUND_VELOCITY 340 //sound Velocity: 340m/s
#define I2C_ADDRESS  0x20
#define LEDS_COUNT   10  //it defines number of lEDs. 

#define TK_STOP_SPEED          0
#define TK_FORWARD_SPEED        (90 + tk_VoltageCompensationToSpeed)

//define different speed levels
int tk_VoltageCompensationToSpeed;  //define Voltage Speed Compensation
#define TK_TURN_SPEED_LV4       (160 + tk_VoltageCompensationToSpeed   )
#define TK_TURN_SPEED_LV3       (130 + tk_VoltageCompensationToSpeed   )
#define TK_TURN_SPEED_LV2       (-120 + tk_VoltageCompensationToSpeed  )
#define TK_TURN_SPEED_LV1       (-140 + tk_VoltageCompensationToSpeed  )

float batteryVoltage = 0;
bool isBuzzered = false;
Servo servo; //create servo object
char servoOffset = 0; //change the value to Calibrate servo
u8 distance[4]; //define an array with type u8(same to unsigned char)
Freenove_WS2812B_Controller strip(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB); //

void setup() {
  pinsSetup(); //set up pins
  getTrackingSensorVal();//Calculate Voltage speed Compensation
  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
  servo.attach(PIN_SERVO); //initialize servo
  servo.write(90 + servoOffset); // change servoOffset to Calibrate servo  
}

void loop() {
  u8 trackingSensorVal = 0;
  trackingSensorVal = getTrackingSensorVal(); //get sensor value

  switch (trackingSensorVal)
  {
    case 0:   //000
      motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED); //car move forward
      break;
    case 7:   //111
      motorRun(TK_STOP_SPEED, TK_STOP_SPEED); //car stop
      break;
    case 1:   //001
      motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1); //car turn
      break;
    case 3:   //011
      motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2); //car turn right
      break;
    case 2:   //010
    case 5:   //101
      motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);  //car move forward
      break;
    case 6:   //110
      motorRun(TK_TURN_SPEED_LV2, TK_TURN_SPEED_LV3); //car turn left
      break;
    case 4:   //100
      motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4); //car turn right
      break;
    default:
      break;
  }
  servo.write(20);
    delay(1000);
    distance[0] = getSonar(); //get ultrasonic value and save it into distance[0]

  servo.write(170);
    delay(1000);
    distance[2] = getSonar();
  
  if (distance[0] > 30){    
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
  }else if (distance[0] < 30) {
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
  if (distance[2] > 30){
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
  }else if (distance[2] < 30) {
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
  }else{
    strip.setAllLedsColor(0, 0, 255);    //set all LED off .
    strip.show();
  }   
}

void tk_CalculateVoltageCompensation() {
  getBatteryVoltage();
  float voltageOffset = 7 - batteryVoltage;
  tk_VoltageCompensationToSpeed = 30 * voltageOffset;
}

//when black line on one side is detected, the value of the side will be 0, or the value is 1
u8 getTrackingSensorVal() {
  u8 trackingSensorVal = 0;
  trackingSensorVal = (digitalRead(PIN_TRACKING_LEFT) == 1 ? 1 : 0) << 2 | (digitalRead(PIN_TRACKING_CENTER) == 1 ? 1 : 0) << 1 | (digitalRead(PIN_TRACKING_RIGHT) == 1 ? 1 : 0) << 0;
  return trackingSensorVal;
}

void pinsSetup() {
  //define motor pin
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  //define ultrasonic moduel pin
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT);
  //define tracking sensor pin
  pinMode(PIN_TRACKING_LEFT, INPUT);
  pinMode(PIN_TRACKING_RIGHT, INPUT);
  pinMode(PIN_TRACKING_CENTER, INPUT);
  setBuzzer(false);
}

float getSonar() {
    unsigned long pingTime;
    float distance;
    digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
    delayMicroseconds(10);
    digitalWrite(PIN_SONIC_TRIG, LOW);
    pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waiting time
    if (pingTime != 0)
        distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
    else
    distance = MAX_DISTANCE;
    return distance; // return the distance value
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
  speedl = constrain(speedl, 0, 255); // speedl absolute value should be within 0~255
  speedr = constrain(speedr, 0, 255); // speedr absolute value should be within 0~255
  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

bool getBatteryVoltage() {
  if (!isBuzzered) {
    pinMode(PIN_BATTERY, INPUT);
    int batteryADC = analogRead(PIN_BATTERY);
    if (batteryADC < 614)    // 3V/12V ,Voltage read: <2.1V/8.4V
    {
      batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
      return true;
    }
  }
  return false;
}

void setBuzzer(bool flag) {
  isBuzzered = flag;
  pinMode(PIN_BUZZER, flag);
  digitalWrite(PIN_BUZZER, flag);
}

void alarm(u8 beat, u8 repeat) {
  beat = constrain(beat, 1, 9);
  repeat = constrain(repeat, 1, 255);
  for (int j = 0; j < repeat; j++) {
    for (int i = 0; i < beat; i++) {
      setBuzzer(true);
      delay(100);
      setBuzzer(false);
      delay(100);
    }
    delay(500);
  }
}

void resetCarAction() {
  motorRun(0, 0);
  setBuzzer(false);
}