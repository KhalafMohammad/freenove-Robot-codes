#include "Servo.h" //include servo library
#include "Freenove_WS2812B_RGBLED_Controller.h"
#include "Freenove_4WD_Car_for_Arduino.h"
#define PIN_SERVO 2 //define servo pin
#define PIN_SONIC_TRIG 7 //define Trig pin
#define PIN_SONIC_ECHO 8 //define Echo pin
#define MAX_DISTANCE 300 //cm
#define SONIC_TIMEOUT (MAX_DISTANCE*60) // calculate timeout
#define SOUND_VELOCITY 340 //sound Velocity: 340m/s
#define I2C_ADDRESS  0x20
#define LEDS_COUNT   10  //it defines number of lEDs. 
#define TK_STOP_SPEED 0
#define TK_FORWARD_SPEED (90 + tk_VoltageCompensationToSpeed )
#define TK_TURN_SPEED_LV4 (180 + tk_VoltageCompensationToSpeed )
#define TK_TURN_SPEED_LV3 (150 + tk_VoltageCompensationToSpeed )
#define TK_TURN_SPEED_LV2 (-140 + tk_VoltageCompensationToSpeed )
#define TK_TURN_SPEED_LV1 (-160 + tk_VoltageCompensationToSpeed )
Servo servo; //create servo object
char servoOffset = 0; //change the value to Calibrate servo
u8 distance[4]; //define an array with type u8(same to unsigned char)
Freenove_WS2812B_Controller strip(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB); //initialization
int tk_VoltageCompensationToSpeed; //define Voltage Speed Compensation


void setup() {
  

    pinsSetup(); //set up pins, from Freenove_4WD_Car_for_Arduino.h library
    getTrackingSensorVal();//Calculate Voltage speed Compensation
    pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
    pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode
    servo.attach(PIN_SERVO); //initialize servo
    servo.write(90 + servoOffset); // change servoOffset to Calibrate servo
     
}

void loop() {
    u8 trackingSensorVal = 0;
    trackingSensorVal = getTrackingSensorVal(); //get sensor value

    servo.write(30);
    delay(1000);
    distance[0] = getSonar(); //get ultrasonic value and save it into distance[0]
    servo.write(90);
    delay(1000);
    distance[1] = getSonar();
    servo.write(150);
    delay(1000);
    distance[2] = getSonar();
    servo.write(90);
    delay(1000);
    distance[1] = getSonar();

    
    Serial.print("Distance L / M / R / M2: "); //Left/Middle/Right/Middle2

    for (int i = 0; i < 4; i++) {
        Serial.print(distance[i]); //print ultrasonic in 45°, 90°, 135°, 90°
        Serial.print("/");
    }

    Serial.print('\n'); //next content will be printed in new line

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

    
    switch (trackingSensorVal)
  {
  case 0: //000
  motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED); //car move forward
  break;
  case 7: //111
  motorRun(TK_STOP_SPEED, TK_STOP_SPEED); //car stop
  break;
  case 1: //001
  motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1); //car turn
  break;
  case 3: //011
  motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2); //car turn right
  break;
  case 2: //010
  case 5: //101
  motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED); //car move forward
  break;
  case 6: //110
  motorRun(TK_TURN_SPEED_LV2, TK_TURN_SPEED_LV3); //car turn left
  break;
  case 4: //100
  motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4); //car turn right
  break;
  default:
  break;
  }    
    
  
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

void tk_CalculateVoltageCompensation() {
getBatteryVoltage(); //from Freenove_4WD_Car_for_Arduino.h library
float voltageOffset = 7 - batteryVoltage;
tk_VoltageCompensationToSpeed = 30 * voltageOffset;
}

u8 getTrackingSensorVal() {
u8 trackingSensorVal = 0;
trackingSensorVal = (digitalRead(PIN_TRACKING_LEFT) == 1 ? 1 : 0) << 2 |
(digitalRead(PIN_TRACKING_CENTER) == 1 ? 1 : 0) << 1 | (digitalRead(PIN_TRACKING_RIGHT) == 1 ?
1 : 0) << 0;
return trackingSensorVal;
}