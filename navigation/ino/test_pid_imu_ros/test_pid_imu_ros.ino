//#define Serial SerialUSB
//#define USE_USBCON 1

#include "CytronMotorDriver.h"
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

///////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_BNO08x.h>
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  //  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    //    Serial.println("Could not enable stabilized remote vector");
  }
}
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

///////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////
//nav_msgs::Odometry odom_msg;
//ros::Publisher odom_pub("odom", &odom_msg); //For publish value testing
std_msgs::Float32 yaw_msg;
ros::Publisher yaw_pub("yaw_imu", &yaw_msg); //For publish value testing
///////////////////////////////////////////////////////////////////////////////////////

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

  if (!bno08x.begin_I2C(0x4A)) {
    //    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  setReports(reportType, reportIntervalUs);
  delay(10);
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
  ///////////////////////////////////////////////////////////////////////////////////////
  if (bno08x.wasReset()) {
    //    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    yaw_msg.data = 10;//ypr.yaw;
    yaw_pub.publish( &yaw_msg );
  }
  //////////////////////////////////////////////////////////////////////////////////////

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
