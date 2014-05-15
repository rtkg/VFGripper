//Low-level gripper control
//Robert Krug, Todor Stoyanov 13/06/2014

//======================= Struct Definitions ======================
struct MotorControlPins{
  int IN1_; //Motor input 1 pin
  int IN2_; //Motor input 2 pin
  int SF_;  //Motor status flag pin
  int EN_;  //Driver board enable pin
  int FB_;  //Analog input pin for current sensing 
  int D2_;  //PWM pin

  MotorControlPins(int IN1, int IN2, int SF, int EN, int FB, int D2) : IN1_(IN1), IN2_(IN2), SF_(SF), EN_(EN), FB_(FB), D2_(D2) {};
};

struct SensorPins{
  int D_;  //Sensor data pin
  int CK_; //Clock pin
  int S_;  //Select pin
};

struct PIDParameters{
  float Kp_; 
  float Kd_;
  float Ki_;
  float u_max_; //Maximum controller output (<= max PWM)
  float u_min_; //Minimum controller output [>= -(max PWM)]
  float I_;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]
  
  PIDParameters(float Kp, float Kd, float Ki, float u_max, float u_min, float I) : Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I) {};
};

struct SensorStates
{
  float v_;  //sensor value
  float dv_; //time-derivative of the sensor value
  
  SensorStates(float v, float dv) : v_(v), dv_(dv) {};
};
//======================= Global Variables ========================
float dT = 1000; //Sample time in microseconds
MotorControlPins* m1_pins=new MotorControlPins(24, 25, 26, 27, A0, 6);     //Motor 1 Arduino pins
PIDParameters* pid_m1_pc=new PIDParameters(0.0, 0.0, 0.0, 4095.0, -4095.0, 0.0); //Position controller PID parameters for Motor 1
SensorStates* e1_s=new SensorStates(0.0 ,0.0); //Sensor states for encoder 1

float m1_pc_r=0.0; //Motor 1 position controller setpoint 

//======================= Initialization ==========================
void setup() {
  pinMode((*m1_pins).IN1_,OUTPUT);
  pinMode((*m1_pins).IN2_,OUTPUT);
  pinMode((*m1_pins).SF_,INPUT);
  pinMode((*m1_pins).FB_,INPUT);
  pinMode((*m1_pins).EN_,OUTPUT);
  pinMode((*m1_pins).D2_,OUTPUT);
  
  analogWriteResolution(12); //sets the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  analogReadResolution(12); //sets the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095
  
  Serial.begin(9600); //open a serial connection
  
  digitalWrite((*m1_pins).EN_,HIGH); //Enable the driver board

}
//====================  Function Declarations ===========================
void update(); //Simple state machine which reads sensors, computes and sets controls
float pid(float e, float de, PIDParameters* p); //Computes the controls from error e, error derivative de, and controller parameters p)
int actuate(float control, const MotorControlPins* mc_pins); //Send the sign-corrected input to the actuator and read the motor status flag
void read(SensorStates* s, SensorPins* s_pins);
float minimumJerk(float t0, float t, float q0, float qf); //evaluates a minimum-jerk position trajectory
//============================== Loop ===================================
void loop() 
{
  //Do nothing if the sampling period didn't pass yet   
  if(micros()%dT != 0) 
    return;
   
  update(); //Read sensors, compute and send controls

  //TODO: write stuff to the output serial connection ... could be done with a lower frequency or only when triggering a request

}
//====================  Function Implementations =========================
void update()
{

}
//--------------------------------------------------------------------------
float pid(float e, float de, PIDParameters* p)
{
  (*p).I_+=(*p).Ki_*e; //update the integral term
  float u=(*p).Kp_*e+(*p).Kd_*de+(*p).I_; //compute the control value

  //clamp the control value and back-calculate the integral term (the latter to avoid windup)
  if (u > (*p).u_max_)
    {
      (*p).I_-=u-(*p).u_max_;
      u=(*p).u_max_;
    }
  else if(u < (*p).u_min_)
    {
      (*p).I+=(*p).u_min_-u;
      u=(*p).u_min_;
    }

  return u;
}
//--------------------------------------------------------------------------
int actuate(float control, const MotorControlPins* mc_pins)
{
  //adjust direction depending on the control sign 
  if (control < 0)
    {
    digitalwrite((*mc_pins).IN1_,HIGH);
    digitalWrite(*(mc_pins).IN2_,LOW);
    }
  else
    {
    digitalwrite((*mc_pins).IN1_,LOW);
    digitalWrite(*(mc_pins).IN2_,HIGH);
    }
  analogWrite(*(mc_pins.D2_), (int)(abs(control)+0.5f)); //set the value on the PWM

  return digitalRead(*(mc_pins).SF_); //return the motor status flag
}
//--------------------------------------------------------------------------
void read(SensorStates* s, SensorPins* s_pins)
{

}
//--------------------------------------------------------------------------
float minimumJerk(float t0, float t, float T, float q0, float qf)
{
  return q0+(qf-q0)*(10*pow((t-t0)/T,3)-15*pow((t-t0)/T,4)+6*pow((t-t0)/T,5));
}
//--------------------------------------------------------------------------
