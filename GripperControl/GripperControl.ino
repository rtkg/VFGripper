//Low-level gripper control
//Robert Krug, Todor Stoyanov 13/06/2014

//======================= Struct Definitions ======================
struct MotorControlPins{
  int IN1; //Motor input 1 pin
  int IN2; //Motor input 2 pin
  int SF;  //Motor status flag pin
  int EN;  //Driver board enable pin
  int FB;  //Analog input pin for current sensing 
  int D2;  //PWM pin
};

struct SensorPins{
  int D;  //Sensor data pin
  int CK; //Clock pin
  int S;  //Select pin
};

struct PIDParameters{
  int Kp; 
  int Kd;
  int Ki;
  int u_max; //Maximum controller output (<= max PWM)
  int u_min; //Minimum controller output [>= -(max PWM)]
  int I;     //Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]
};

struct SensorStates
{
  int v;  //sensor value
  int dv; //time-derivative of the sensor value
};
//======================= Global Variables ========================
int dT = 1000; //Sample time in microseconds
MotorControlPins m1_pins={24, 25, 26, 27, A0, 6};     //Motor 1 Arduino pins
PIDParameters pid_m1_pc={0, 0, 0, 4095, -4095, 4095, 0}; //Position controller PID parameters for Motor 1
SensorStates e1_s={0 ,0}; //Sensor states for encoder 1

int m1_pc_r=0; //Motor 1 position controller setpoint 

//======================= Initialization ==========================
void setup() {
  pinMode(m1_pins.IN1,OUTPUT);
  pinMode(m1_pins.IN2,OUTPUT);
  pinMode(m1_pins.SF,INPUT);
  pinMode(m1_pins.FB,INPUT);
  pinMode(m1_pins.EN,OUTPUT);
  pinMode(m1_pins.D2,OUTPUT);
  
  m1_pins.IN1=24;
  
  analogWriteResolution(12); //sets the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  analogReadResolution(12); //sets the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095
  
  Serial.begin(9600); //open a serial connection
  
  digitalWrite(m1_pins.EN,HIGH); //Enable the driver board

}
//====================  Function Declarations ===========================
void update(); //Simple state machine which reads sensors, computes and sets controls
int pid(int e, int de, PIDParameters* p); //Computes the controls from error e, error derivative de, and controller parameters p)
int actuate(int control, const MotorControlPins* mc_pins); //Send the sign-corrected input to the actuator and read the motor status flag
void read(SensorStates* s, SensorPins* s_pins);
//============================== Loop ===================================
void loop() 
{
  //Do nothing if the sampling period didn't pass yet   
  if(micros()%dT != 0) 
    return;
   
  update(); //Read sensors, compute and send controls

  //TODO: write stuff to the output serial connection ... could be done with a lower frequency or only when triggering a request

}
//====================  Function Implementations ===========================
void update()
{

}
//--------------------------------------------------------------------------
int pid(int e, int de, PIDParameters* p)
{
  (*p).I+=(*p).Ki*e; //update the integral term
  int u=(*p).Kp*e+(*p).Kd*de+(*p).I; //compute the control value

  //clamp the control value and the integral term (the latter to avoid reset windup)
  if (u > (*p).u_max)
    {
      (*p).I-=u-(*p).u_max;
      u=(*p).u_max;
    }
  else if(u < (*p).u_min)
    {
      (*p).I+=(*p).u_min-u;
      u=(*p).u_min;
    }

  return u;
}
//--------------------------------------------------------------------------
int actuate(int control, const MotorControlPins* mc_pins)
{
  //adjust direction depending on the control sign 
  if (control < 0)
    {
    digitalwrite((*mc_pins).IN1,HIGH);
    digitalWrite(*(mc_pins).IN2,LOW);
    }
  else
    {
    digitalwrite((*mc_pins).IN1,LOW);
    digitalWrite(*(mc_pins).IN2,HIGH);
    }
  analogWrite(*(mc_pins.D2), abs(control)); //set the value on the PWM

  return digitalRead(*(mc_pins).SF); //return the motor status flag
}
//--------------------------------------------------------------------------
void read(SensorStates* s, SensorPins* s_pins)
{

}
//--------------------------------------------------------------------------
