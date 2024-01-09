int pulses;                              //Output pulses.
int encoderA = 18;
int encoderB = 19;
//const int pwm = 5;                      //Power of motor.
//const int dir = 4;                       //Direction of the motor.
int pulsesChanged = 0;
#define total 245                        //x1 pulses per rotation.
#define motorSpeed 180                   //Change speed of the motor.
void setup(){
  Serial.begin(115200);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  attachInterrupt(0, A_CHANGE, CHANGE);
  attachInterrupt(1, A_CHANGE, CHANGE);


}//setup

void loop(){
  
  if (pulsesChanged != 0) {
    pulsesChanged = 0;
    Serial.println(pulses);
  }
}

void A_CHANGE(){                                  //Function that to read the pulses in x1.
  if( digitalRead(encoderB) == 0 ) {
    if ( digitalRead(encoderA) == 0 ) {
      // A fell, B is low
      pulses--; // moving reverse
    } else {
      // A rose, B is low
      pulses++; // moving forward
    }
  }
  pulsesChanged = 1;
}
