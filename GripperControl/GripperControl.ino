//Low-level gripper control
//Robert Krug, Todor Stoyanov 13/06/2014

#include <limits.h>

#define POSITION_MODE 0
#define CURRENT_MODE 1
#define BELT_MODE 2
#define NO_MODE 3

const float pi = 3.14159;
const float pwm_resolution = 4095; //PWM resoluion: 12 bit, i.e., 0 - 4095
const float enc_resolution = 4095; //Encoder resolution: 12 bit, i.e., 0 - 4095

//======================= Struct Definitions ======================
struct MotorControlPins {
  int IN1_; //Motor input 1 pin
  int IN2_; //Motor input 2 pin
  int SF_;  //Motor status flag pin
  int EN_;  //Driver board enable pin
  int FB_;  //Analog input pin for current sensing

  MotorControlPins(int IN1, int IN2, int SF, int EN, int FB) : IN1_(IN1), IN2_(IN2), SF_(SF), EN_(EN), FB_(FB) {};
};

struct SensorPins {
  int D_;  //Sensor data pin
  int CK_; //Clock pin
  int S_;  //Select pin

  SensorPins(int D, int CK, int S) : D_(D), CK_(CK), S_(S) {};
};

struct PIDParameters {
  float Kp_;
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]

  PIDParameters(float Kp, float Kd, float Ki, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

struct CurrentSensorStates
{
  float c_;          //sensor reading (filtered)
  float v_;        //converted value
  float alpha_;    //first order filter parameter, 0<=alpha<=1 
  float scale_;    //scale for conversion


  CurrentSensorStates(float c, float v, float alpha, float scale) : c_(c), v_(v), alpha_(alpha), scale_(scale) {};

  void filter(int c){
    c_=alpha_*c_+(1-alpha_)*(float)c; //first order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
  }

  void convertSensorReading(){
    v_=c_*scale_;
  }

};

struct EncoderStates
{
  float t_;        //encoder ticks (filtered)
  int offset_;     //used for zeroing
  float v_;        //converted value
  float dv_;       //time-derivative of the converted value
  int k_;          //rollover counter
  int res_;        //resolution (i.e., number of ticks per revolution)
  float scale_;    //scale factor used for conversion from ticks to value - can be used to lump transmission ratio, radius ...
  float alpha_;    //first order filter parameter, 0<=alpha<=1 

  EncoderStates(int t, int offset, float v, float dv, int k, int res, int scale, float alpha) : t_(t), offset_(offset), v_(v), dv_(dv), k_(k), res_(res), scale_(scale), alpha_(alpha) {};
  void filter(int t) {
    t_=alpha_*t_+(1-alpha_)*(float)t; //first order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
  }
  void convertSensorReading() {
    v_ = 2 * pi * ((t_ - (float)offset_) / (float)res_ + (float)k_) * scale_;
  }; // angle=2*pi*[(reading-offset)/res+k]
};

struct ControlStates
{
  float r_; //setpoint value at current iteration
  float rf_; //final setpoint value
  float ri_; //initial setpoint value
  float e_; //error
  float de_; //error derivative

  float ti_; //time when we initialized motion (seconds)
  float T_; //time for executing the loop

  float u_; //computed control

  float k_; //Motor constant

  bool active_; //flag indicating whether the corresponding controller is active or not

  ControlStates(float r, float rf, float ri, float e, float de, float ti, float T, int u, int k, bool active) : r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), k_(k), active_(active) {};
};

//======================= Global Variables ========================
MotorControlPins* mb1_pins = new MotorControlPins(6, 7, 24, 25, A0);   //Motor belt 1 Arduino pins
PIDParameters* pid_mb1_pc = new PIDParameters(1.0, 0.0, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Position controller PID parameters for motor belt 1
ControlStates* c_b1 = new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, false); //Setpoint and error for drive belt 1
ControlStates* c_b2 = new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, false); //Setpoint and error for drive belt 2
ControlStates* c_oc = new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, false); //Setpoint and error for drive open close

SensorPins* e_b1_pins = new SensorPins(26, 27, 28); //Encoder belt 1 pins
EncoderStates* e_b1_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 1.0, 0.0); //Sensor states for encoder belt 1
EncoderStates* e_b2_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 1.0, 0.0); //Sensor states for encoder belt 2
EncoderStates* e_oc_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 1.0, 0.0); //Sensor states for encoder open close
EncoderStates* e_p1_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 1.0, 0.0); //Sensor states for encoder phalange 1
EncoderStates* e_p2_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 1.0, 0.0); //Sensor states for encoder phalange 2

CurrentSensorStates* curr_b1_s = new CurrentSensorStates(0, 0.0, 0.0, 3.3/4095/0.000525); //current in Milliampere
CurrentSensorStates* curr_b2_s = new CurrentSensorStates(0, 0.0, 0.0, 1); //just for testing
CurrentSensorStates* curr_oc_s = new CurrentSensorStates(0, 0.0, 0.0, 1); //just for testing

int dT = 1000; //Sample time in microseconds
int dT_serial = 100000; //Sample time for the serial connection in microseconds
int t_old, t_new;
int t_old_serial;
int mode = NO_MODE;

//====================  Function Declarations ===========================
void update(); //Simple state machine which reads sensors, computes and sets controls
float pid(float e, float de, PIDParameters* p); //Computes the controls from error e, error derivative de, and controller parameters p)
int actuate(float control, const MotorControlPins* mc_pins); //Send the sign-corrected input to the actuator and read the motor status flag
float minimumJerk(float t0, float t, float q0, float qf); //evaluates a minimum-jerk position trajectory
int readEncoder(const SensorPins* s_pins); //read the current position from the sensor connected to the given pins
int computeEncoderStates(EncoderStates* e_s, const SensorPins* s_pins); //Converts the encoder readings and computes derivatives
byte shiftIn(const SensorPins* s_pins, int readBits); //read in a byte of dapta from the digital input corresponding to the given sensor
void processMessage(); //Process the message comming from the serial connection
float positionControl(ControlStates* c_s, EncoderStates* e_s, MotorControlPins* m_pins, PIDParameters* pid_p); //Compute position control
float currentControl(ControlStates* c_s, CurrentSensorStates* curr_s, MotorControlPins* m_pins, PIDParameters* pid_p);
//======================= Initialization ==========================
void setup() {


  pinMode(mb1_pins->IN1_, OUTPUT);
  pinMode(mb1_pins->IN2_, OUTPUT);
  pinMode(mb1_pins->SF_, INPUT);
  pinMode(mb1_pins->FB_, INPUT);
  pinMode(mb1_pins->EN_, OUTPUT);
  digitalWrite(mb1_pins->EN_, HIGH); //Enable the driver board

  pinMode(e_b1_pins->D_, INPUT);
  pinMode(e_b1_pins->CK_, OUTPUT);
  pinMode(e_b1_pins->S_, OUTPUT);
  digitalWrite(e_b1_pins->CK_, HIGH);   //give some default value
  digitalWrite(e_b1_pins->S_, HIGH);    //give some default value

  analogWriteResolution(12); //sets the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  analogReadResolution(12); //sets the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095

  //Initialize encoder 1 (for belt 1)
  e_b1_s->t_ = readEncoder(e_b1_pins); //Read encoder 1
  e_b1_s->offset_ =  e_b1_s->t_;
  e_b1_s->convertSensorReading();

  t_old = micros();
  t_old_serial = micros();

  Serial.begin(19200); //open a serial connection

  Serial.println("setup done");
}
//============================== Loop ===================================
void loop()
{
  processMessage();
  t_new = micros();
  //Do nothing if the sampling period didn't pass yet
  if (abs(t_new - t_old) < dT)
    return;
  t_old = t_new;


  if (abs(t_new - t_old_serial) > dT_serial) {
    sendStatus();
    t_old_serial = t_new;
  }

  computeEncoderStates(e_b1_s, e_b1_pins); //Compute angle + velocity of encoder belt 1
  curr_b1_s->filter(analogRead(mb1_pins->FB_)); curr_b1_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1
  curr_b2_s->c_=(float)analogRead(mb1_pins->FB_); //DEBUG

  if (mode == POSITION_MODE) {
    update(); //Read sensors, compute and send controls
  } else {
    //  delay(100); Shouldn't be necessary ... the loop is throttled by the sampling time anyway ...
  }


}
//====================  Function Implementations =========================
void update()
{

  //Control Drive 1 if its active
  if (c_b1->active_)
    {
      float u = positionControl(c_b1, e_b1_s, mb1_pins, pid_mb1_pc); //compute controls for drive belt 1
      int sf = actuate(u, mb1_pins); //actuate drive belt 1
    }
}
//--------------------------------------------------------------------------
float positionControl(ControlStates* c_s, EncoderStates* e_s, MotorControlPins* m_pins, PIDParameters* pid_p)
{
  float r = minimumJerk(c_s->ti_, (float)millis(), c_s->T_, c_s->ri_, c_s->rf_); //update the setpoint
  float e = r - e_s->v_; //position error
  float de = (e - c_s->e_) / ((float)dT); //derivative of the position error
  c_s->r_ = r; c_s->e_ = e; c_s->de_ = de; //update the control states for the next iteration

  return pid(e, de, pid_p); //compute control
}
//--------------------------------------------------------------------------
float currentControl(ControlStates* c_s, CurrentSensorStates* curr_s, MotorControlPins* m_pins, PIDParameters* pid_p)
{
  float e = c_s->r_ - curr_s->v_; //current error (setpoint r_ = rf_ = const for current control)
  float de = (e - c_s->e_) / ((float)dT); //derivative of the current error
  c_s->e_ = e; c_s->de_ = de; //update the control states for the next iteration

  return pid(e, de, pid_p) + c_s->k_ * c_s->r_; //compute control with feedforward term
}
//--------------------------------------------------------------------------
float pid(float e, float de, PIDParameters* p)
{
  p->I_ += p->Ki_ * e; //update the integral term
  float u = p->Kp_ * e + p->Kd_ * de + p->I_; //compute the control value

  //clamp the control value and back-calculate the integral term (the latter to avoid windup)
  if (u > p->u_max_)
    {
      p->I_ -= u - p->u_max_;
      u = p->u_max_;
    }
  else if (u < p->u_min_)
    {
      p->I_ += p->u_min_ - u;
      u = p->u_min_;
    }

  return u;
}
//--------------------------------------------------------------------------
int actuate(float control, const MotorControlPins* mc_pins)
{
  //adjust direction depending on the control sign
  c_b1->u_ = (int)(abs(control) + 0.5f);

  if (control > 0)
    {
      analogWrite(mc_pins->IN2_, (int)(abs(control) + 0.5f)); //set the value on the PWM
      analogWrite(mc_pins->IN1_, 0);
    }
  else
    {
      analogWrite(mc_pins->IN1_, (int)(abs(control) + 0.5f)); //set the value on the PWM
      analogWrite(mc_pins->IN2_, 0);
    }

  return digitalRead(mc_pins->SF_); //return the motor status flag
}
//--------------------------------------------------------------------------
float minimumJerk(float t0, float t, float T, float q0, float qf)
{
  if (t > t0 + T)
    t = T + t0; //make sure the ouput stays at qf after T has passed

  return q0 + (qf - q0) * (10 * pow((t - t0) / T, 3) - 15 * pow((t - t0) / T, 4) + 6 * pow((t - t0) / T, 5));
}
//--------------------------------------------------------------------------
byte shiftIn(const SensorPins* s_pins, int readBits)
{
  byte data = 0;
  for (int i = readBits - 1; i >= 0; i--)
    {
      digitalWrite(s_pins->CK_, LOW);
      delayMicroseconds(1);
      digitalWrite(s_pins->CK_, HIGH);
      delayMicroseconds(1);

      byte bit = digitalRead(s_pins->D_);
      //Serial.println(bit, BIN);
      data = data | (bit << i);
    }
  //Serial.print("byte: ");
  //Serial.println(data, BIN);
  return data;
}
//--------------------------------------------------------------------------
int readEncoder(const SensorPins* s_pins)
{
  unsigned int reading = 0;

  //shift in our data
  digitalWrite(s_pins->S_, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(s_pins, 8);
  byte d2 = shiftIn(s_pins, 8);
  byte d3 = shiftIn(s_pins, 2);
  digitalWrite(s_pins->S_, HIGH);

  //get our reading variable
  reading = d1;
  reading = reading << 8;
  reading = reading | d2;

  reading = reading >> 4;

  //check the offset compensation flag: 1 == started up
  if (!((d2 & B00001000) == B00001000))
    reading = -1;

  //check the cordic overflow flag: 1 = error
  if (d2 & B00000100)
    reading = -2;

  //check the linearity alarm: 1 = error
  if (d2 & B00000010)
    reading = -3;

  //check the magnet range: 11 = error
  if ((d2 & B00000001) & (d3 & B10000000))
    reading = -4;

  //add the checksum bit

  return reading;
}
//--------------------------------------------------------------------------
int computeEncoderStates(EncoderStates* e_s, const SensorPins* s_pins)
{
  int t_prev = e_s->t_; //save the previous encoder value
  e_s->filter(readEncoder(s_pins)); //read and filter the new encoder value  (sets e_s->t_)

  if ((e_s->k_ == INT_MAX) ||  (e_s->k_ == INT_MIN))
    e_s->t_ = (-5); //Over/underflow of the rollover count variable

  if (e_s->t_ >= 0)
    {
      //check whether a rollover happened
      if ((e_s->t_ - t_prev) < -e_s->res_ / 2) //if delta is smaller than minus half the resolution -> positive rollover
	e_s->k_++;
      if ((e_s->t_ - t_prev) > e_s->res_ / 2) //if delta is larger than half the resolution -> negative rollover
	e_s->k_--;

      float v_prev = e_s->v_; //save previous sensor value
      e_s->convertSensorReading();//update the sensor value (sets e_s->v_)

      //calculate sensor value derivative
      e_s->dv_ = (e_s->v_ - v_prev) / ((float)dT);

      //Serial.print("Reading: ");
      //Serial.print(e_s->t_, DEC);
      //Serial.print(" Offset: ");
      //Serial.print(e_s->offset_, DEC);
      //Serial.print(" Position: ");
      //Serial.print(e_s->v_, 4);//, DEC);
      //Serial.print(" k: ");
      //Serial.print(e_s->k_, DEC);//, DEC);
      //Serial.print(" Velocity: ");
      //Serial.println(e_s->dv_, 4); //DEC);
    }
  else
    {
      Serial.print("Error in computeEncoderStates(...): ");
      Serial.println(e_s->t_, DEC);
      return e_s->t_;
    }

  return 1;
}
//--------------------------------------------------------------------------
void processMessage() {

  byte b1, b2;
  short target_val = 0;

  if (Serial.available()) {
    delay(10);
    char code[10];
    short i = 0;
    while (Serial.available() && i < 50) {
      code[i++] = Serial.read();
    }

    //Serial.print(code);
    //Serial.print("\r\n");
    if (code[0] == 'P' && code[1] == 'O' && code[2] == 'S') {
      //position mode
      mode = POSITION_MODE;
      b1 = code[3];
      b2 = code[4];
      target_val = b2;
      target_val = target_val << 8;
      target_val = target_val | b1;
      //Serial.println(b1, BIN);
      //Serial.println(b2, BIN);
      c_b1->rf_ = (float)target_val;
      c_b1->ri_ = (float)(*e_b1_s).v_;
      c_b1->ti_ = (float)millis();
      c_b1->T_ = 10000; //Milliseconds
      c_b1->active_ = true;
      //Serial.println("Got a new SETPOINT !!!!!!!!!!!!!!!!!!!!!!!!!!");
      //Serial.println("rf:");
      //Serial.println(c_b1->rf_,4);
      //Serial.println("ri:");
      //Serial.println(c_b1->ri_,4);
      //Serial.println("ti:");
      //Serial.println(c_b1->ti_,4);
      //Serial.println("T:");
      //Serial.println(c_b1->T_,4);
      //Serial.print("\r\n");
    }
    if (code[0] == 'N' && code[1] == 'O' && code[2] == 'N') {
      mode = NO_MODE;
      target_val = 0;
    }
  }
}
void sendStatus() {
  //Serial.print((int) ((*e_oc_s).v_*1000), DEC);
  Serial.print((int) (c_b1->u_), DEC);
  Serial.print(",");
  Serial.print((int) ((*e_p1_s).v_ * 1000), DEC);
  Serial.print(",");
  Serial.print((int) ((*e_p2_s).v_ * 1000), DEC);
  Serial.print(",");
  Serial.print((int) ((*e_b1_s).v_ * 1000), DEC);
  Serial.print(",");
  Serial.print((int) ((*e_b2_s).v_ * 1000), DEC);
  Serial.print(",");
  Serial.print((int)curr_oc_s->v_, DEC);
  Serial.print(",");
  Serial.print((int)curr_b1_s->v_, DEC);
  Serial.print(",");
  Serial.print((int)curr_b2_s->v_, DEC);
  Serial.print("\r\n");
}
//--------------------------------------------------------------------------
