//#define Serial SerialUSB
//#define USE_USBCON 1

#include "CytronMotorDriver.h"
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int64.h>

CytronMD motor1(PWM_DIR, 9, 8); // PWM 1=Pin 4, DIR1=Pin3
CytronMD motor2(PWM_DIR, 11, 10); // PWM 2=Pin 6, DIR2=Pin5

int encoderR_A = 2;
int encoderR_B = 4;
int encoderL_A = 3;
int encoderL_B = 5;

volatile long pulse_L = 0;
volatile long pulse_R = 0;

#define total 40120  // total pulses
// PC L = 40184 , VM = 41300,40133,40216
// PC R = 40400 , VM = 40041,40142,40491

int pulsesChanged = 0;

//create ros node
ros::NodeHandle nh;

//config message variable
std_msgs::Int64 encL_msg; // message encoder of left wheel
std_msgs::Int64 encR_msg; // message encoder of right wheel

//call publisher
ros::Publisher EncL("Enc_L", &encL_msg); // Publish encoder of left wheel
ros::Publisher EncR("Enc_R", &encR_msg); // Publish encoder of right wheel

void setup() {

  //  Serial.begin(115200);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(EncL);
  nh.advertise(EncR);

  pinMode(encoderL_A, INPUT);
  pinMode(encoderL_B, INPUT);
  pinMode(encoderR_A, INPUT);
  pinMode(encoderR_B, INPUT);

  //  Serial.println("Setup");

  attachInterrupt(digitalPinToInterrupt(encoderL_A), PULSE_L_A_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderR_A), PULSE_R_A_CHANGE, CHANGE);
}

void loop() {
  motor1.setSpeed(50);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(50);   // Motor 1 runs forward at 50% speed.

  if (pulsesChanged != 0) {
    pulsesChanged = 0;
    //    Serial.print("L ");
    //    Serial.print(pulse_L);
    //    Serial.print("  R ");
    //    Serial.println(pulse_R);
    encL_msg.data = pulse_L;
    encR_msg.data = pulse_R;
    //    encL_msg.data = 10;
    //    encR_msg.data = 100;
    EncL.publish( &encL_msg );
    EncR.publish( &encR_msg );
  }
  nh.spinOnce();
  delay(3);

}

void PULSE_L_A_CHANGE() {
  if ( digitalRead(encoderL_B) == 0 ) {
    if ( digitalRead(encoderL_A) == 0 ) {
      // A fell, B is low
      pulse_L--; // moving reverse
    } else {
      // A rose, B is low
      pulse_L++; // moving forward
    }
  } else {
    if ( digitalRead(encoderL_A) == 0 ) {
      // B fell, A is high
      pulse_L++; // moving reverse
    } else {
      // B rose, A is high
      pulse_L--; // moving forward
    }
  }
  pulsesChanged = 1;
}

void PULSE_R_A_CHANGE() {
  if ( digitalRead(encoderR_B) == 0 ) {
    if ( digitalRead(encoderR_A) == 0 ) {
      // A fell, B is low
      pulse_R++; // moving reverse
    } else {
      // A rose, B is low
      pulse_R--; // moving forward
    }
  } else {
    if ( digitalRead(encoderR_A) == 0 ) {
      // B fell, A is high
      pulse_R--; // moving reverse
    } else {
      // B rose, A is high
      pulse_R++; // moving forward
    }
  }
  pulsesChanged = 1;
}
