#include <Freenove_WS2812B_RGBLED_Controller.h>
#include <IRremote.h>
/**********************************************************************
* Filename    : Multifunctional_IR_Remote_Car.ino
* Product     : Freenove 4WD Car for UNO
* Description : A Multifunctional IR-Remote Car.
* Auther      : www.freenove.com
* Modification: 2019/08/06
**********************************************************************/
#include <Freenove_WS2812B_RGBLED_Controller.h>
////////////////////////////////////////////
////Definitions related to IR-remote
#define IR_UPDATE_TIMEOUT     110
#define IR_CAR_SPEED          150


#define PIN_SERVO     2
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

#define BAT_VOL_STANDARD  7.0

#define IR_REMOTE_KEYCODE_POWER    0xFFA25D
#define IR_REMOTE_KEYCODE_MENU    0xFF629D
#define IR_REMOTE_KEYCODE_MUTE    0xFFE21D
#define IR_REMOTE_KEYCODE_MODE    0xFF22DD
#define IR_REMOTE_KEYCODE_UP      0xFF02FD
#define IR_REMOTE_KEYCODE_BACK    0xFFC23D
#define IR_REMOTE_KEYCODE_LEFT    0xFFE01F
#define IR_REMOTE_KEYCODE_CENTER  0xFFA857
#define IR_REMOTE_KEYCODE_RIGHT   0xFF906F
#define IR_REMOTE_KEYCODE_0       0xFF6897
#define IR_REMOTE_KEYCODE_DOWN    0xFF9867
#define IR_REMOTE_KEYCODE_OK      0xFFB04F
#define IR_REMOTE_KEYCODE_1     0xFF30CF
#define IR_REMOTE_KEYCODE_2     0xFF18E7
#define IR_REMOTE_KEYCODE_3     0xFF7A85
#define IR_REMOTE_KEYCODE_4     0xFF10EF
#define IR_REMOTE_KEYCODE_5     0xFF38C7
#define IR_REMOTE_KEYCODE_6     0xFF5AA5
#define IR_REMOTE_KEYCODE_7     0xFF42BD
#define IR_REMOTE_KEYCODE_8     0xFF4AB5
#define IR_REMOTE_KEYCODE_9     0xFF52AD

IRrecv irrecv(PIN_IRREMOTE_RECV);
decode_results results;
u32 currentKeyCode, lastKeyCode;
bool isStopFromIR = false;
u32 lastIRUpdateTime = 0;

//Definitions related to Led-strip




float batteryVoltage = 0;
bool isBuzzered = false;

void setup() {
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    isStopFromIR = false;
    currentKeyCode = results.value;
    if (currentKeyCode != 0xFFFFFFFF) {
      lastKeyCode = currentKeyCode;
    }
    switch (lastKeyCode) {
      case IR_REMOTE_KEYCODE_UP:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_DOWN:
        motorRun(-IR_CAR_SPEED, -IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_LEFT:
        motorRun(-IR_CAR_SPEED, IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_RIGHT:
        motorRun(IR_CAR_SPEED, -IR_CAR_SPEED);
        break;
      case IR_REMOTE_KEYCODE_CENTER:
        setBuzzer(true);
        break;
      case IR_REMOTE_KEYCODE_0:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(500);
        break;
      case IR_REMOTE_KEYCODE_1:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(600);
        break;
      case IR_REMOTE_KEYCODE_2:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(700);
        break;
      case IR_REMOTE_KEYCODE_3:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(800);
        break;
      case IR_REMOTE_KEYCODE_4:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(900);
        break;
      case IR_REMOTE_KEYCODE_5:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(1000);
        break;
      case IR_REMOTE_KEYCODE_6:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(2000);
        break;
      case IR_REMOTE_KEYCODE_7:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(3000);
        break;
      case IR_REMOTE_KEYCODE_8:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(4000);
        break;
      case IR_REMOTE_KEYCODE_9:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED);
        delay(5000);
        break;
    }
    irrecv.resume(); // Receive the next value
    lastIRUpdateTime = millis();
  }
  else {
    if (millis() - lastIRUpdateTime > IR_UPDATE_TIMEOUT)
    {
      if (!isStopFromIR) {
        isStopFromIR = true;
        motorRun(0, 0);
        setBuzzer(false);
      }
      lastIRUpdateTime = millis();
    }
  }
}

//////////
void pinsSetup() {
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
  pinMode(PIN_TRACKING_LEFT, INPUT); // 
  pinMode(PIN_TRACKING_RIGHT, INPUT); // 
  pinMode(PIN_TRACKING_CENTER, INPUT); // 
  setBuzzer(false);
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
  speedl = constrain(speedl, 0, 255);
  speedr = constrain(speedr, 0, 255);
  if (abs(speedl) < MOTOR_PWM_DEAD && abs(speedr) < MOTOR_PWM_DEAD) {
    speedl = 0;
    speedr = 0;
  }

  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

bool getBatteryVoltage() {
  if (!isBuzzered) {
    pinMode(PIN_BATTERY, INPUT);
    int batteryADC = analogRead(PIN_BATTERY);
    if (batteryADC < 614)   // 3V/12V ,Voltage read: <2.1V/8.4V
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
