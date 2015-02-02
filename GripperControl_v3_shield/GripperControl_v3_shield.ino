//Low-level gripper control
//Robert Krug, Todor Stoyanov 13/06/2014

#include <limits.h>
#include <DueFlashStorage.h>
#include <ros.h>
#include <velvet_msgs/GripperState.h>
#include <velvet_msgs/SetPos.h>
#include <velvet_msgs/SetCur.h>
#include <velvet_msgs/SetVel.h>
#include <velvet_msgs/SetPID.h>
#include <std_srvs/Empty.h>

#define POSITION_MODE 0
#define VELOCITY_MODE 1
#define CURRENT_MODE 2
#define NO_MODE 3

#define ENCODER_ALPHA 0.7
#define CURRENT_ALPHA 0.99
#define CURRENT_ALPHA_B 0.95

///////////////////////ROS stuff////////////////////////
void setPosCallback(const velvet_msgs::SetPos::Request &req, velvet_msgs::SetPos::Response &res);
void setCurCallback(const velvet_msgs::SetCur::Request &req, velvet_msgs::SetCur::Response &res);
void setVelCallback(const velvet_msgs::SetVel::Request &req, velvet_msgs::SetVel::Response &res);
void setPIDCallback(const velvet_msgs::SetPID::Request &req, velvet_msgs::SetPID::Response &res);
void setZero(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void setOn(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void setOff(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

ros::NodeHandle nh;
velvet_msgs::GripperState state;
ros::Publisher state_publisher("/velvet_state", &state);
ros::ServiceServer<velvet_msgs::SetPos::Request, velvet_msgs::SetPos::Response> pos_server("set_pos", &setPosCallback);
ros::ServiceServer<velvet_msgs::SetCur::Request, velvet_msgs::SetCur::Response> cur_server("set_cur", &setCurCallback);
ros::ServiceServer<velvet_msgs::SetVel::Request, velvet_msgs::SetVel::Response> vel_server("set_vel", &setVelCallback);
ros::ServiceServer<velvet_msgs::SetPID::Request, velvet_msgs::SetPID::Response> pid_server("set_pid", &setPIDCallback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> zer_server("set_zero", &setZero);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> on_server("set_on", &setOn);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> off_server("set_off", &setOff);
////////////////////////////////////////////////////////

DueFlashStorage dueFlashStorage;
uint8_t zeroPositionOCSetFlag = 1;


const float pi = 3.14159;
const float pwm_resolution = 4095; //PWM resoluion: 12 bit, i.e., 0 - 4095
const float belt_limit = 2000; //PWM resoluion: 12 bit, i.e., 0 - 4095
const float enc_resolution = 4095; //Encoder resolution: 12 bit, i.e., 0 - 4095
const float MAX_CURRENT_OC = 1500;
const float MAX_CURRENT_B = 500;
const float VOLTAGE_FACTOR = 0.17; // 4096 * 1000 / 24V //resolution * mAmp / V

int dT = 1000; //Sample time in microseconds
int dT_serial = 10000; //Sample time for the serial connection in microseconds
int t_old, t_new;
int t_old_serial;
int mode = NO_MODE;

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

  PIDParameters(float Kp, float Ki, float Kd, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

struct CurrentSensorStates
{
  float c_;          //sensor reading (filtered)
  float v_;        //converted value
  float alpha_;    //first order filter parameter, 0<=alpha<=1
  float scale_;    //scale for conversion


  CurrentSensorStates(float c, float v, float alpha, float scale) : c_(c), v_(v), alpha_(alpha), scale_(scale) {};

  void filter(int c) {
    c_ = alpha_ * c_ + (1 - alpha_) * (float)c; //first order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
  }

  void convertSensorReading() {
    v_ = c_ * scale_;
  }

};

struct EncoderStates
{
  int raw_ticks_;  //raw encoder ticks
  int offset_;     //used for zeroing
  float p_;        //converted value
  float p_raw_;    //encoder ticks (filtered)
  float dp_;       //time-derivative of the converted value
  int k_;          //rollover counter
  int res_;        //resolution (i.e., number of ticks per revolution)
  float scale_;    //scale factor used for conversion from ticks to value - can be used to lump transmission ratio, radius ...
  float alpha_;    //first order filter parameter, 0<=alpha<=1


  EncoderStates(float p_raw, int offset, float p, float dp, int k, int res, float scale, int raw_ticks, float alpha) : p_raw_(p_raw), offset_(offset), p_(p), dp_(dp), k_(k), res_(res), scale_(scale), raw_ticks_(raw_ticks), alpha_(alpha) {};
  void convertSensorReading(int raw_ticks_new) {
    if ((raw_ticks_new - raw_ticks_) < -res_ / 2) //if delta is smaller than minus half the resolution -> positive rollover
      k_++;
    if ((raw_ticks_new - raw_ticks_) > res_ / 2) //if delta is larger than half the resolution -> negative rollover
      k_--;

    raw_ticks_ = raw_ticks_new;
    p_raw_ = 2 * pi * ((raw_ticks_ - (float)offset_) / (float)res_ + (float)k_) * scale_;
    float p_temp = alpha_ * p_ + (1 - alpha_) * p_raw_; //first order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
    dp_ = (p_temp - p_) / ((float)dT);
    p_ = p_temp;
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

  float R_; //Motor resistance for cc

  bool active_; //flag indicating whether the corresponding controller is active or not
  int mode_; //is it a position or current or velocity controller at the moment?

  ControlStates(float r, float rf, float ri, float e, float de, float ti, float T, int u, int R, bool active, int mode) : r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), R_(R), active_(active), mode_(mode) {};
};

//======================= Global Variables ========================
//BELT1
MotorControlPins* m_b1_pins =   new MotorControlPins(6, 7, 46, 40, A11); //belt left back Motor 1 Arduino pins
ControlStates* c_b1 =  new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15, false, NO_MODE); //Setpoint and error for drive belt 1
PIDParameters* pid_mb1_pc = new PIDParameters(100.0, 0.05, 0.0, belt_limit, -belt_limit, 0.0); //Position controller PID parameters for belt Motor 1
PIDParameters* pid_mb1_cc = new PIDParameters(50.0, 0.0, 15.0, pwm_resolution, -pwm_resolution, 0.0); //Current controller PID parameters for belt Motor 1
PIDParameters* pid_mb1_vc = new PIDParameters(1.0, 0.0, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Velocity controller PID parameters for belt Motor 1
SensorPins* e_b1_pins = new SensorPins(33, 35, 37);
EncoderStates* e_b1_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 14.4, 0, ENCODER_ALPHA); //Sensor states for encoder belt 1

MotorControlPins* m_b3_pins = new MotorControlPins(4, 5, 34, 42, A4); //   //belt left front Motor 1 Arduino pins
ControlStates* c_b3 =  new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15, false, NO_MODE); //Setpoint and error for drive belt 1
PIDParameters* pid_mb3_pc = new PIDParameters(150.0, 0.05, 0.0, belt_limit, -belt_limit, 0.0); //Position controller PID parameters for belt Motor 1
PIDParameters* pid_mb3_cc = new PIDParameters(50.0, 0.0, 15.0, pwm_resolution, -pwm_resolution, 0.0); //Current controller PID parameters for belt Motor 1
PIDParameters* pid_mb3_vc = new PIDParameters(1.0, 0.0, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Velocity controller PID parameters for belt Motor 1
SensorPins* e_b3_pins = new SensorPins(39, 41, 43); 
EncoderStates* e_b3_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, 14.4, 0, ENCODER_ALPHA); //Sensor states for encoder belt 1

///TODO: pins for the sensors and controllers bellow need to be updated
//BELT2
MotorControlPins* m_b2_pins = new MotorControlPins(8, 9, 50, 42, A1);//   //belt right back Motor 1 Arduino pins
ControlStates* c_b2 = new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15, false, NO_MODE); //Setpoint and error for drive belt 2
PIDParameters* pid_mb2_pc = new PIDParameters(150.0, 0.05, 0.0, belt_limit, -belt_limit, 0.0); //Position controller PID parameters for belt Motor 1
PIDParameters* pid_mb2_cc = new PIDParameters(50.0, 0.0, 15.0, pwm_resolution, -pwm_resolution, 0.0); //Current controller PID parameters for belt Motor 2
PIDParameters* pid_mb2_vc = new PIDParameters(1.0, 0.0, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Velocity controller PID parameters for belt Motor 2
SensorPins* e_b2_pins = new SensorPins(24, 26, 28); //
EncoderStates* e_b2_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, -14.4, 0, ENCODER_ALPHA); //Sensor states for encoder belt 2

MotorControlPins* m_b4_pins =  new MotorControlPins(3, 2, 36, 40, A10); //   //belt right front Motor 1 Arduino pins
ControlStates* c_b4 = new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15, false, NO_MODE); //Setpoint and error for drive belt 2
PIDParameters* pid_mb4_pc = new PIDParameters(150.0, 0.05, 0.0, belt_limit, -belt_limit, 0.0); //Position controller PID parameters for belt Motor 1
PIDParameters* pid_mb4_cc = new PIDParameters(50.0, 0.0, 15.0, pwm_resolution, -pwm_resolution, 0.0); //Current controller PID parameters for belt Motor 2
PIDParameters* pid_mb4_vc = new PIDParameters(1.0, 0.0, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Velocity controller PID parameters for belt Motor 2
SensorPins* e_b4_pins = new SensorPins(45, 47, 49);
EncoderStates* e_b4_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, -14.4, 0, ENCODER_ALPHA); //Sensor states for encoder belt 2

//OPEN CLOSE
MotorControlPins* m_oc1_pins = new MotorControlPins(10, 11, 32, 44, A2);//;   //OC Motor 1 Arduino pins
MotorControlPins* m_oc2_pins = new MotorControlPins(12, 13, 48, 44, A0);   //OC Motor 2 Arduino pins
ControlStates* c_oc = new ControlStates(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12, false, true); //Setpoint and error for drive open close
PIDParameters* pid_moc_pc = new PIDParameters(3, 0.1, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Position controller PID parameters for opening Motors
PIDParameters* pid_moc_cc = new PIDParameters(50.0, 0.0, 0.0, pwm_resolution, -pwm_resolution, 0.0); //Current controller PID parameters for opening Motors
SensorPins* e_oc_pins =  new SensorPins(22, 23, 25);
EncoderStates* e_oc_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, -500, 0, ENCODER_ALPHA); //Sensor states for encoder open close
//PHALANGES
SensorPins* e_p1_pins = new SensorPins(27, 29, 31); //;
EncoderStates* e_p1_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, -1.0, 0, ENCODER_ALPHA); //Sensor states for encoder phalange 1
SensorPins* e_p2_pins =  new SensorPins(51, 53, 48);
EncoderStates* e_p2_s = new EncoderStates(0, 0, 0.0 , 0.0, 0, enc_resolution, -1.0, 0, ENCODER_ALPHA); //Sensor states for encoder phalange 2

CurrentSensorStates* curr_b1_s = new CurrentSensorStates(0, 0.0, CURRENT_ALPHA_B, 3.3 / 4095 / 0.000525); //current in Milliampere
CurrentSensorStates* curr_b2_s = new CurrentSensorStates(0, 0.0, CURRENT_ALPHA_B, 3.3 / 4095 / 0.000525); //just for testing
CurrentSensorStates* curr_b3_s = new CurrentSensorStates(0, 0.0, CURRENT_ALPHA_B, 3.3 / 4095 / 0.000525); //current in Milliampere
CurrentSensorStates* curr_b4_s = new CurrentSensorStates(0, 0.0, CURRENT_ALPHA_B, 3.3 / 4095 / 0.000525); //just for testing
CurrentSensorStates* curr_oc1_s = new CurrentSensorStates(0, 0.0, CURRENT_ALPHA_B, 3.3 / 4095 / 0.000525 ); //just for testing
CurrentSensorStates* curr_oc2_s = new CurrentSensorStates(0, 0.0, CURRENT_ALPHA_B, 3.3 / 4095 / 0.000525 ); //just for testing


//====================  Function Declarations ===========================
void update(); //Simple state machine which reads sensors, computes and sets controls
float pid(float e, float de, PIDParameters* p); //Computes the controls from error e, error derivative de, and controller parameters p)
int actuate(float control, const MotorControlPins* mc_pins); //Send the sign-corrected input to the actuator and read the motor status flag
float minimumJerk(float t0, float t, float q0, float qf); //evaluates a minimum-jerk position trajectory
int readEncoder(const SensorPins* s_pins); //read the current position from the sensor connected to the given pins
int computeEncoderStates(EncoderStates* e_s, const SensorPins* s_pins); //Converts the encoder readings and computes derivatives
byte shiftIn(const SensorPins* s_pins, int readBits); //read in a byte of dapta from the digital input corresponding to the given sensor
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorControlPins* m_pins, PIDParameters* pid_p); //Compute position control
void velocityControl(ControlStates* c_s, EncoderStates* e_s, MotorControlPins* m_pins, PIDParameters* pid_p); //Compute velocity control
void currentControl(ControlStates* c_s, CurrentSensorStates* curr_s, MotorControlPins* m_pins, PIDParameters* pid_p);
void setupMotorPins(MotorControlPins* mp); //sets up the pins for a motor
void setupEncoderPins(SensorPins* ep); //sets up the pins for an encoder
void setupEncoderSensor(SensorPins* ep, EncoderStates* es); //connects an encoder sensor to the pins and reads in the first value
void updateState();
//======================= Initialization ==========================
void setup() {

  setupMotorPins(m_b1_pins);
  setupMotorPins(m_b2_pins);
  setupMotorPins(m_b3_pins);
  setupMotorPins(m_b4_pins);
  setupMotorPins(m_oc1_pins);
  setupMotorPins(m_oc2_pins);

  setupEncoderPins(e_b1_pins);
  setupEncoderPins(e_b2_pins);
  setupEncoderPins(e_b3_pins);
  setupEncoderPins(e_b4_pins);
  setupEncoderPins(e_p1_pins);
  setupEncoderPins(e_p2_pins);
  setupEncoderPins(e_oc_pins);

  setupEncoderSensor(e_oc_pins, e_oc_s);
  setupEncoderSensor(e_b1_pins, e_b1_s);
  setupEncoderSensor(e_b2_pins, e_b2_s);
  setupEncoderSensor(e_b3_pins, e_b3_s);
  setupEncoderSensor(e_b4_pins, e_b4_s);
  setupEncoderSensor(e_p1_pins, e_p1_s);
  setupEncoderSensor(e_p2_pins, e_p2_s);

  analogWriteResolution(12); //sets the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  analogReadResolution(12); //sets the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095
  t_old = micros();
  t_old_serial = micros();

  //check if the OC zero position is stored in flash
  zeroPositionOCSetFlag = dueFlashStorage.read(0);
  if (zeroPositionOCSetFlag == 0) {
    //read in the encoder offset!
    byte* b = dueFlashStorage.readAddress(4); // byte array which is read from flash at adress 4
    int encoderFromFlash; // create a temporary struct
    memcpy(&encoderFromFlash, b, sizeof(int)); // copy byte array to temporary struct
    //Serial.print("read encoder form flash");
    //Serial.println(encoderFromFlash, DEC);
    e_oc_s->offset_ = encoderFromFlash;
  }

  //////////////ROS stuff /////////////////////
  nh.initNode();
  nh.advertise(state_publisher);
  nh.advertiseService(pos_server);
  nh.advertiseService(cur_server);
  nh.advertiseService(vel_server);
  nh.advertiseService(pid_server);
  nh.advertiseService(zer_server);
  nh.advertiseService(on_server);
  nh.advertiseService(off_server);
  state.oc.id  = 0;
  state.blb.id = 1;
  state.brb.id = 2;
  state.blf.id = 3;
  state.brf.id = 4;
  state.pl.id = 5;
  state.pr.id = 6;
  state.c_oc.id = 0;
  state.c_blb.id = 1;
  state.c_brb.id = 2;
  state.c_blf.id = 3;
  state.c_brf.id = 4;
  /////////////////////////////////////////////

  //Serial1.println("setup done");
  //int sf;
  //actuate(1000, m_b1_pins);
  //Serial.println(sf,DEC);
  //actuate(1000, m_b2_pins);
  //Serial.println(sf,DEC);
  //actuate(1000, m_b3_pins);
  //Serial.println(sf,DEC);
  //actuate(1000, m_b4_pins);
  //Serial.println(sf,DEC);
  //actuate(1000, m_oc2_pins);
  //Serial.println(sf,DEC);
  //actuate(1000, m_oc1_pins);

}
//============================== Loop ===================================
void loop()
{
  updateState();
  state_publisher.publish(&state);
  nh.spinOnce();
  //processMessage();
  t_new = micros();
  //Do nothing if the sampling period didn't pass yet
  if (abs(t_new - t_old) < dT)
    return;
  t_old = t_new;


  //Serial.println("Help");
  //read encoders
  computeEncoderStates(e_b1_s, e_b1_pins); //Compute angle + velocity of encoder belt 1
  computeEncoderStates(e_b2_s, e_b2_pins); //Compute angle + velocity of encoder belt 2
  computeEncoderStates(e_b3_s, e_b3_pins); //Compute angle + velocity of encoder belt 1
  computeEncoderStates(e_b4_s, e_b4_pins); //Compute angle + velocity of encoder belt 2
  computeEncoderStates(e_p1_s, e_p1_pins); //Compute angle + velocity of encoder phalange 1
  computeEncoderStates(e_p2_s, e_p2_pins); //Compute angle + velocity of encoder phalange 2
  computeEncoderStates(e_oc_s, e_oc_pins); //Compute angle + velocity of encoder open close



  curr_b1_s->filter(analogRead(m_b1_pins->FB_)); curr_b1_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1
  curr_b2_s->filter(analogRead(m_b2_pins->FB_)); curr_b2_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1
  curr_b3_s->filter(analogRead(m_b3_pins->FB_)); curr_b3_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1
  curr_b4_s->filter(analogRead(m_b4_pins->FB_)); curr_b4_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1
  curr_oc1_s->filter(analogRead(m_oc1_pins->FB_)); curr_oc1_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1
  curr_oc2_s->filter(analogRead(m_oc2_pins->FB_)); curr_oc2_s->convertSensorReading(); //read, filter and convert the current sensor reading of belt 1

  //watchdogs on the three currents
  /* if (curr_oc_s->v_ > MAX_CURRENT_OC) {
     digitalWrite(m_oc_pins->EN_, LOW); //Disable the driver board
   }
   if (curr_b1_s->v_ > MAX_CURRENT_B || curr_b2_s->v_ > MAX_CURRENT_B) {
     digitalWrite(m_b1_pins->EN_, LOW); //Disable the driver board
   }*/

  if (mode == POSITION_MODE || mode == VELOCITY_MODE || mode == CURRENT_MODE) {
    update(); //Read sensors, compute and send controls
  } else {
    //do nothing

    //actuate(1000, m_oc_pins);

  }

}
//====================  Setup helper Function Implementations =========================
void setupMotorPins(MotorControlPins* mp)
{
  pinMode(mp->IN1_, OUTPUT);
  pinMode(mp->IN2_, OUTPUT);
  pinMode(mp->SF_, INPUT);
  pinMode(mp->FB_, INPUT);
  pinMode(mp->EN_, OUTPUT);
  digitalWrite(mp->EN_, HIGH); //Enable the driver board
}
//--------------------------------------------------------------------------
void setupEncoderPins(SensorPins* ep)
{
  pinMode(ep->D_, INPUT);
  pinMode(ep->CK_, OUTPUT);
  pinMode(ep->S_, OUTPUT);
  digitalWrite(ep->CK_, HIGH);   //give some default value
  digitalWrite(ep->S_, HIGH);    //give some default value
}
//--------------------------------------------------------------------------
void setupEncoderSensor(SensorPins* ep, EncoderStates* es)
{
  es->raw_ticks_ = readEncoder(ep); //Read encoder 1
  es->offset_ =  es->raw_ticks_;
  es->convertSensorReading(es->raw_ticks_);
}
//====================  Function Implementations =========================
void update()
{
  //Control Drive 1 if its active
  if (c_b1->active_)
  {
    if (c_b1->mode_ == POSITION_MODE) {
      positionControl(c_b1, e_b1_s, m_b1_pins, pid_mb1_pc);//compute the controls for drive belt 1
    }
    else if (c_b1->mode_ == CURRENT_MODE)
    {
      currentControl(c_b1, curr_b1_s, m_b1_pins, pid_mb1_cc);//compute the current controls for drive belt 1
    }
    else {
      c_b1->u_ = 0;
    }
    /*else if (c_b1->mode_ == VELOCITY_MODE)
    {
      velocityControl(c_b1, e_b1_s, m_b1_pins, pid_mb1_vc);
    }*/
    int sf = actuate(c_b1->u_, m_b1_pins); //actuate drive belt 1
  }
  if (c_b2->active_)
  {
    if (c_b2->mode_ == POSITION_MODE)
    {
      positionControl(c_b2, e_b2_s, m_b2_pins, pid_mb2_pc);//compute the controls for drive belt 1
    }
    else if (c_b2->mode_ == CURRENT_MODE)
    {
      currentControl(c_b2, curr_b2_s, m_b2_pins, pid_mb2_cc);//compute the current control for drive belt 1
    }
    else {
      c_b2->u_ = 0;
    }
    /*else if (c_b2->mode_ == VELOCITY_MODE)
    {
      velocityControl(c_b2, e_b2_s, m_b2_pins, pid_mb2_vc);
    }*/
    int sf = actuate(c_b2->u_, m_b2_pins); //actuate drive belt 1
  }

  if (c_b3->active_)
  {
    if (c_b3->mode_ == POSITION_MODE) {
      positionControl(c_b3, e_b3_s, m_b3_pins, pid_mb3_pc);//compute the controls for drive belt 1
    } else if (c_b3->mode_ == CURRENT_MODE)
    {
      currentControl(c_b3, curr_b3_s, m_b3_pins, pid_mb3_cc);//compute the current control for drive belt 1
    }
    else {
      c_b3->u_ = 0;
    }
    int sf = actuate(c_b3->u_, m_b3_pins); //actuate drive belt 1
  }
  if (c_b4->active_)
  {
    if (c_b4->mode_ == POSITION_MODE)
    {
      positionControl(c_b4, e_b4_s, m_b4_pins, pid_mb4_pc);//compute the controls for drive belt 1
    } else if (c_b4->mode_ == CURRENT_MODE)
    {
      currentControl(c_b4, curr_b4_s, m_b4_pins, pid_mb4_cc);//compute the current control for drive belt 1
    } else {
      c_b4->u_ = 0;
    }
    int sf = actuate(c_b4->u_, m_b4_pins); //actuate drive belt 1
  }

  if (c_oc->active_)
  {
    if (c_oc->mode_ == POSITION_MODE)
    {
      positionControl(c_oc, e_oc_s, m_oc1_pins, pid_moc_pc);//compute the controls for drive belt 1
    }
    else if (c_oc->mode_ == CURRENT_MODE)
    {
      currentControl(c_oc, curr_oc1_s, m_oc1_pins, pid_moc_cc);//compute the current controls for drive belt 1
    }
    //mirror output for the two motors
    int sf = actuate(c_oc->u_, m_oc1_pins); //actuate drive belt 1
    sf = actuate(c_oc->u_, m_oc2_pins);
  }
}
//--------------------------------------------------------------------------
void positionControl(ControlStates* c_s, EncoderStates* e_s, MotorControlPins* m_pins, PIDParameters* pid_p)
{
  float r = minimumJerk(c_s->ti_, (float)millis(), c_s->T_, c_s->ri_, c_s->rf_); //update the setpoint
  float e = r - e_s->p_; //position error
  float de = (e - c_s->e_) / ((float)dT); //derivative of the position error
  c_s->r_ = r; c_s->e_ = e; c_s->de_ = de; //update the control states for the next iteration

  c_s->u_ = pid(e, de, pid_p); //compute control
}
//--------------------------------------------------------------------------
void velocityControl(ControlStates* c_s, EncoderStates* e_s, MotorControlPins* m_pins, PIDParameters* pid_p)
{
  float r = minimumJerk(c_s->ti_, (float)millis(), c_s->T_, c_s->ri_, c_s->rf_); //update the setpoint
  float e = r - e_s->dp_; //velocity error
  float de = (e - c_s->e_) / ((float)dT); //derivative of the velocity error
  c_s->r_ = r; c_s->e_ = e; c_s->de_ = de; //update the control states for the next iteration

  c_s->u_ = pid(e, de, pid_p); //compute control
}
//--------------------------------------------------------------------------
void currentControl(ControlStates* c_s, CurrentSensorStates* curr_s, MotorControlPins* m_pins, PIDParameters* pid_p)
{

  float r = minimumJerk(c_s->ti_, (float)millis(), c_s->T_, c_s->ri_, c_s->rf_); //update the setpoint
  c_s->r_ = r;
  float dir =  c_s->r_ >= 0 ? 1 : -1;
  float current = dir * curr_s->v_;

  float e = c_s->r_ - current; //current error (setpoint r_ = rf_ = const for current control)
  float de = (e - c_s->e_) / ((float)dT); //derivative of the current error
  c_s->e_ = e; c_s->de_ = de; //update the control states for the next iteration

  c_s->u_ = pid(e, de, pid_p) + c_s->R_ * c_s->r_ * VOLTAGE_FACTOR; //compute control with feedforward term
  //clamping
  if (c_s->u_ > pid_p->u_max_)
  {
    c_s->u_ = pid_p->u_max_;
  }
  else if (c_s->u_ < pid_p->u_min_)
  {
    c_s->u_ = pid_p->u_min_;
  }

  if (c_s->u_ > 0 && dir < 0)
  {
    c_s->u_ = 0;
  }
  else if (c_s->u_ < 0 && dir > 0)
  {
    c_s->u_ = 0;
  }

}
//--------------------------------------------------------------------------
float pid(float e, float de, PIDParameters* p)
{
  p->I_ += p->Ki_ * e; //update the integral term
  float u = p->Kp_ * e + p->Kd_ * de + p->I_; //compute the control value

  //clamp the control value and back-calculate the integral term (the latter to avoid windup)
  if (u > p->u_max_)
  {
    p->I_ -= p->Ki_ * e; //u - p->u_max_;
    u = p->u_max_;
  }
  else if (u < p->u_min_)
  {
    p->I_ -= p->Ki_ * e; //p->u_min_ - u;
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
  int t_new = readEncoder(s_pins);

  if ((e_s->k_ == INT_MAX) ||  (e_s->k_ == INT_MIN))
    e_s->raw_ticks_ = (-5); //Over/underflow of the rollover count variable

  if (e_s->raw_ticks_ >= 0)
  {
    e_s->convertSensorReading(t_new);  //update the sensor value (sets e_s->v_)   //Serial.print("Reading: ");
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
    /*
    Serial1.print("Error in computeEncoderStates(...): ");
    Serial1.println(e_s->raw_ticks_, DEC);
    Serial1.print("pins ");
    Serial1.println(s_pins->D_, DEC);
    Serial1.println("read from sensor: ");
    Serial1.println(t_new, DEC);
    */
    return e_s->raw_ticks_;
  }

  return 1;
}
//--------------------------------------------------------------------------
void updateState() {

  //encoders
  state.oc.ticks = e_oc_s->raw_ticks_;
  state.oc.val = e_oc_s->p_;
  state.oc.target = c_oc->r_;

  state.blb.ticks = e_b1_s->raw_ticks_;
  state.blb.val = e_b1_s->p_;
  state.blb.target = c_b1->r_;

  state.brb.ticks = e_b2_s->raw_ticks_;
  state.brb.val = e_b2_s->p_;
  state.brb.target = c_b2->r_;

  state.blf.ticks = e_b3_s->raw_ticks_;
  state.blf.val = e_b3_s->p_;
  state.blf.target = c_b3->r_;

  state.brf.ticks = e_b4_s->raw_ticks_;
  state.brf.val = e_b4_s->p_;
  state.brf.target = c_b4->r_;

  state.pl.ticks = e_p1_s->raw_ticks_;
  state.pl.val = e_p1_s->p_;

  state.pr.ticks = e_p2_s->raw_ticks_;
  state.pr.val = e_p2_s->p_;

  //current sensors
  state.c_oc.val = curr_oc1_s->v_;
  state.c_oc.target = c_oc->r_;

  state.c_blb.val = curr_b1_s->v_;
  state.c_blb.target = c_b1->r_;

  state.c_brb.val = curr_b2_s->v_;
  state.c_brb.target = c_b2->r_;

  state.c_blf.val = curr_b3_s->v_;
  state.c_blf.target = c_b3->r_;

  state.c_brf.val = curr_b3_s->v_;
  state.c_brf.target = c_b3->r_;

  //modes
  if (c_oc->mode_ == POSITION_MODE) {
    state.oc_mode = 'P';
  } else {
    state.oc_mode = 'C';
  }
  if (c_b1->mode_ == POSITION_MODE) {
    state.b_mode = 'P';
  } else {
    state.b_mode = 'C';
  }
}

void setPosCallback(const velvet_msgs::SetPos::Request &req, velvet_msgs::SetPos::Response &res) {
  if (req.id == 0) {
    c_oc->rf_ = req.pos;
    c_oc->ri_ = e_oc_s->p_;
    c_oc->ti_ = (float)millis();
    c_oc->T_ = req.time;
    c_oc->active_ = true;
    c_oc->mode_ = POSITION_MODE;

  } else {
    c_b1->rf_ = (float)e_b1_s->p_ + req.pos; //value in mm
    c_b1->ri_ = (float)e_b1_s->p_;
    c_b1->ti_ = (float)millis();
    c_b2->rf_ = (float)e_b2_s->p_ + req.pos;
    c_b2->ri_ = (float)e_b2_s->p_;
    c_b2->ti_ = (float)millis();
    c_b3->rf_ = (float)e_b3_s->p_ + req.pos;
    c_b3->ri_ = (float)e_b3_s->p_;
    c_b3->ti_ = (float)millis();
    c_b4->rf_ = (float)e_b4_s->p_ + req.pos;
    c_b4->ri_ = (float)e_b4_s->p_;
    c_b4->ti_ = (float)millis();

    c_b1->T_ = req.time;
    c_b1->active_ = true;
    c_b1->mode_ = POSITION_MODE;
    c_b2->T_ = req.time;
    c_b2->active_ = true;
    c_b2->mode_ = POSITION_MODE;
    c_b3->T_ = req.time;
    c_b3->active_ = true;
    c_b3->mode_ = POSITION_MODE;
    c_b4->T_ = req.time;
    c_b4->active_ = true;
    c_b4->mode_ = POSITION_MODE;

  }

  mode = POSITION_MODE;
  res.success = true;

}

void setCurCallback(const velvet_msgs::SetCur::Request &req, velvet_msgs::SetCur::Response &res) {
  mode = CURRENT_MODE;
  if (req.id == 0) {
    float dir =  c_oc->r_ >= 0 ? 1 : -1;
    if (c_oc->mode_ != CURRENT_MODE) {
      c_oc->r_ = dir * curr_oc1_s->v_;
      c_oc->mode_ = CURRENT_MODE;
    }
    c_oc->rf_ = req.curr ; //value in mAmp!
    c_oc->ri_ = c_oc->r_; //value in mAmp!
    c_oc->ti_ = (float)millis();
    c_oc->T_ = req.time;
    c_oc->active_ = true;

  } else {
    float dir1 =  c_b1->r_ >= 0 ? 1 : -1;
    float dir2 =  c_b2->r_ >= 0 ? 1 : -1;
    float dir3 =  c_b3->r_ >= 0 ? 1 : -1;
    float dir4 =  c_b4->r_ >= 0 ? 1 : -1;

    if (c_b1->mode_ != CURRENT_MODE) {
      c_b1->r_ = dir1 * curr_b1_s->v_;
      c_b1->mode_ = CURRENT_MODE;
    }
    if (c_b2->mode_ != CURRENT_MODE) {
      c_b2->r_ = dir2 * curr_b2_s->v_;
      c_b2->mode_ = CURRENT_MODE;
    }
    if (c_b3->mode_ != CURRENT_MODE) {
      c_b3->r_ = dir3 * curr_b3_s->v_;
      c_b3->mode_ = CURRENT_MODE;
    }
    if (c_b4->mode_ != CURRENT_MODE) {
      c_b4->r_ = dir2 * curr_b4_s->v_;
      c_b4->mode_ = CURRENT_MODE;
    }

    c_b1->rf_ = req.curr ; //value in mRad!
    c_b1->ti_ = (float)millis();
    c_b1->ri_ = c_b1->r_; //value in mAmp!
    c_b1->T_ = req.time;
    c_b1->active_ = true;

    c_b2->rf_ = req.curr ; //value in mRad!
    c_b2->ti_ = (float)millis();
    c_b2->ri_ = c_b2->r_; //value in mAmp!
    c_b2->T_ = req.time;
    c_b2->active_ = true;

    c_b3->rf_ = req.curr ; //value in mRad!
    c_b3->ti_ = (float)millis();
    c_b3->ri_ = c_b3->r_; //value in mAmp!
    c_b3->T_ = req.time;
    c_b3->active_ = true;

    c_b4->rf_ = req.curr ; //value in mRad!
    c_b4->ti_ = (float)millis();
    c_b4->ri_ = c_b4->r_; //value in mAmp!
    c_b4->T_ = req.time;
    c_b4->active_ = true;
  }
}

void setVelCallback(const velvet_msgs::SetVel::Request &req, velvet_msgs::SetVel::Response &res) {

}

void setPIDCallback(const velvet_msgs::SetPID::Request &req, velvet_msgs::SetPID::Response &res) {
  PIDParameters* target_control;

  if (req.mode == 'P') {
    if (req.id == 0) target_control = pid_moc_pc;
    if (req.id == 1) target_control = pid_mb1_pc;
    if (req.id == 2) target_control = pid_mb2_pc;
    if (req.id == 3) target_control = pid_mb3_pc;
    if (req.id == 4) target_control = pid_mb4_pc;
  } else if (req.mode == 'C') {
    if (req.id == 0) target_control = pid_moc_cc;
    if (req.id == 1) target_control = pid_mb1_cc;
    if (req.id == 2) target_control = pid_mb2_cc;
    if (req.id == 3) target_control = pid_mb3_cc;
    if (req.id == 4) target_control = pid_mb4_cc;
  } else {
    res.success = false;
    return;
  }

  target_control->Kp_ = req.pval;
  target_control->Ki_ = req.ival;
  target_control->Kd_ = req.dval;
  res.success = true;
}

void setZero(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  setupEncoderSensor(e_b1_pins, e_b1_s);
  setupEncoderSensor(e_b2_pins, e_b2_s);
  setupEncoderSensor(e_b3_pins, e_b3_s);
  setupEncoderSensor(e_b4_pins, e_b4_s);
  setupEncoderSensor(e_oc_pins, e_oc_s);

  byte b2[sizeof(int)]; // create byte array to store the struct
  memcpy(b2, &e_oc_s->offset_, sizeof(int)); // copy the struct to the byte array
  dueFlashStorage.write(4, b2, sizeof(int)); // write byte array to flash
  // write 0 to address 0 to indicate that it is not the first time running anymore
  dueFlashStorage.write(0, 0);
}

void setOn(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

  digitalWrite(m_b1_pins->EN_, HIGH); //Enable the driver board
  digitalWrite(m_b2_pins->EN_, HIGH); //Enable the driver board
  digitalWrite(m_oc1_pins->EN_, HIGH); //Enable the driver board

}

void setOff(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  digitalWrite(m_b1_pins->EN_, LOW); //Enable the driver board
  digitalWrite(m_b2_pins->EN_, LOW); //Enable the driver board
  digitalWrite(m_oc1_pins->EN_, LOW); //Enable the driver board

}
//--------------------------------------------------------------------------
