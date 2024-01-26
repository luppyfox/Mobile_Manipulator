#include "CytronMotorDriver.h"

CytronMD motor_l(PWM_DIR, 9, 8); // PWM 1=Pin 9, DIR1=Pin8
CytronMD motor_r(PWM_DIR, 11, 10); // PWM 2=Pin 11, DIR2=Pin10

int encoderR_A = 2;
int encoderR_B = 4;
int encoderL_A = 3;
int encoderL_B = 5;

volatile long pulse_L = 0;
volatile long pulse_R = 0;

#define total_pulse_L 20181  // total pulses Left wheel
#define total_pulse_R 20225  // total pulses Left wheel

int pulsesChanged = 0;

// globals
long prevT_L = 0;
long prevT_R = 0;

//local
long posPrev_L = 0;
float vlFilt = 0;
float vlPrev = 0;
float vt_L = 0;
long posPrev_R = 0;
float vrFilt = 0;
float vrPrev = 0;
float vt_R = 0;

//pid parameter
float kpL = 37;
float kiL = 0;

float kpR = 10;
float kiR = 0;

float e_L = 0;
float e_R = 0;
float eintegral_L = 0;
float eintegral_R = 0;

void setup() {

  Serial.begin(115200);
  pinMode(encoderL_A, INPUT);
  pinMode(encoderL_B, INPUT);
  pinMode(encoderR_A, INPUT);
  pinMode(encoderR_B, INPUT);

  Serial.println("Setup");

  attachInterrupt(digitalPinToInterrupt(encoderL_A), PULSE_L_A_CHANGE, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(encoderL_B), PULSE_L_B_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderR_A), PULSE_R_A_CHANGE, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(encoderR_B), PULSE_R_B_CHANGE, CHANGE);
}

void loop() {

  if (pulsesChanged != 0) {
    pulsesChanged = 0;
    Serial.print("L ");
    Serial.print(pulse_L);
    Serial.print("  R ");
    Serial.println(pulse_R);
  }

    // read the position and velocity
  long pos_L = 0;
  long pos_R = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos_L = pulse_L;
  pos_R = pulse_R;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT_L = micros();
  float deltaT_L = ((float) (currT_L - prevT_L)) / 1.0e6;
  float velocity_L = (pos_L - posPrev_L) / deltaT_L;
  posPrev_L = pos_L;
  prevT_L = currT_L;

  long currT_R = micros();
  float deltaT_R = ((float) (currT_R - prevT_R)) / 1.0e6;

  float velocity_R = (pos_R - posPrev_R) / deltaT_R;
  posPrev_R = pos_R;
  prevT_R = currT_R;

  // Convert count/s to RPM
  double vl = velocity_L / total_pulse_L * 60.0;
  double vr = velocity_R / total_pulse_R * 60.0;

  // Low-pass filter (25 Hz cutoff)
  vlFilt = 0.854 * vlFilt + 0.0728 * vl + 0.0728 * vlPrev;
  vlPrev = vl;
  vrFilt = 0.854 * vrFilt + 0.0728 * vr + 0.0728 * vrPrev;
  vrPrev = vr;
  vt_L = -5;
  vt_R = -5;
  // Compute the control signal u
  e_L = vt_L + vlFilt;
  eintegral_L = eintegral_L + e_L * deltaT_L;
  e_R = vt_R + vrFilt;
  eintegral_R = eintegral_R + e_R * deltaT_R;

  float ul = kpL * e_L + kiL * eintegral_L ;
  float ur = kpR * e_R + kiR * eintegral_R ;
  Serial.print("ul  ");
  Serial.print(ul);
  Serial.print("  ur  ");
  Serial.print(ur);
  Serial.print("  u R ");
  Serial.print(e_L);
  Serial.print("  u R ");
  Serial.println(e_R);

  // Set the motor speed and direction
  int dir_L = 1;
  if (ul < 0) {
    dir_L = -1;
  }
  int pwr_L = ((int) fabs(ul)) * dir_L;
  if (pwr_L > 255) {
    pwr_L = 255;
  }
  if (pwr_L < -255) {
    pwr_L = -255;
  }

  int dir_R = 1;
  if (ur < 0) {
    dir_R = -1;
  }
  int pwr_R = ((int) fabs(ur)) * dir_R;
  if (pwr_R > 255) {
    pwr_R = 255;
  }
  if (pwr_R < -255) {
    pwr_R = -255;
  }
  
  //  setMotor
  if (vt_L == 0) motor_l.setSpeed(0);
  else motor_l.setSpeed(pwr_L);
  if (vt_R == 0) motor_r.setSpeed(0);
  else motor_r.setSpeed(pwr_R);

  Serial.print("pwm L ");
  Serial.print(pwr_L);
  Serial.print("  pwm R ");
  Serial.println(pwr_R);
  
  delay(1);

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
