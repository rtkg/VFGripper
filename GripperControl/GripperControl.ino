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

  SensorPins(int D, int CK, int S) : D_(D), CK_(CK), S_(S) {};
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
  int reading_; //raw sensor value
  float v_;     //sensor value
  float dv_;    //time-derivative of the sensor value
  
  SensorStates(int reading, float v, float dv) : reading_(reading), v_(v), dv_(dv) {};
};
//======================= Global Variables ========================
MotorControlPins* m1_pins=new MotorControlPins(24, 25, 26, 27, A0, 6);     //Motor 1 Arduino pins
PIDParameters* pid_m1_pc=new PIDParameters(0.0, 0.0, 0.0, 4095.0, -4095.0, 0.0); //Position controller PID parameters for Motor 1

SensorPins* e1_pins=new SensorPins(31, 33, 35); //Encoder 1 pins
SensorStates* e1_s=new SensorStates(0, 0.0 ,0.0); //Sensor states for encoder 1

int dT = 1000; //Sample time in microseconds

float m1_pc_r=0.0; //Motor 1 position controller setpoint 

//======================= Initialization ==========================
void setup() {
  pinMode((*m1_pins).IN1_,OUTPUT);
  pinMode((*m1_pins).IN2_,OUTPUT);
  pinMode((*m1_pins).SF_,INPUT);
  pinMode((*m1_pins).FB_,INPUT);
  pinMode((*m1_pins).EN_,OUTPUT);
  pinMode((*m1_pins).D2_,OUTPUT);
  digitalWrite((*m1_pins).EN_,HIGH); //Enable the driver board

  pinMode((*e1_pins).D_,INPUT);
  pinMode((*e1_pins).CK_,OUTPUT);
  pinMode((*e1_pins).S_,OUTPUT);
  digitalWrite((*e1_pins).CK_, HIGH);   //give some default value
  digitalWrite((*e1_pins).S_, HIGH);    //give some default value


  analogWriteResolution(12); //sets the resolution of the analogWrite(...) function to 12 bit, i.e., between 0 - 4095
  analogReadResolution(12); //sets the resolution of the analogRead(...) function to 12 bit, i.e., between 0 - 4095
  
  Serial.begin(19200); //open a serial connection
}
//====================  Function Declarations ===========================
void update(); //Simple state machine which reads sensors, computes and sets controls
float pid(float e, float de, PIDParameters* p); //Computes the controls from error e, error derivative de, and controller parameters p)
int actuate(float control, const MotorControlPins* mc_pins); //Send the sign-corrected input to the actuator and read the motor status flag
void read(SensorStates* s, SensorPins* s_pins);
float minimumJerk(float t0, float t, float q0, float qf); //evaluates a minimum-jerk position trajectory
int readPosition(const SensorPins* s_pins); //read the current position from the sensor connected to the given pins
byte shiftIn(const SensorPins* s_pins, int readBits); //read in a byte of data from the digital input corresponding to the given sensor
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

   Serial.print("Reading: ");
   (*e1_s).reading_ = readPosition(e1_pins); //Read encoder 1
   
   if ((*e1_s).reading_ >= 0)
   {
  //calculate sensor value 
  float v=(float)(*e1_s).reading_ * 0.088; //where is this number coming from ??

  //calculate sensor value derivative
  (*e1_s).dv_ = (v-(*e1_s).dv_)/((float)dT);

  (*e1_s).v_=v; //set new sensor value

      Serial.print("Reading: ");
      Serial.print((*e1_s).reading_, DEC);
      Serial.print(" Position: ");
      Serial.println((*e1_s).v_, 4); //DEC);
      Serial.print(" Velocity: ");
      Serial.println((*e1_s).dv_, 4); //DEC);
   }
   else
   {
      Serial.print("Error: ");
      Serial.println((*e1_s).reading_);
   }
   
   delay(1000);
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
      (*p).I_+=(*p).u_min_-u;
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
    digitalWrite((*mc_pins).IN1_,HIGH);
    digitalWrite((*mc_pins).IN2_,LOW);
    }
  else
    {
    digitalWrite((*mc_pins).IN1_,LOW);
    digitalWrite((*mc_pins).IN2_,HIGH);
    }
  analogWrite((*mc_pins).D2_, (int)(abs(control)+0.5f)); //set the value on the PWM

  return digitalRead((*mc_pins).SF_); //return the motor status flag
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
byte shiftIn(const SensorPins* s_pins, int readBits)
{
  byte data = 0;
  for (int i=readBits-1; i>=0; i--)
  {
    digitalWrite((*s_pins).CK_, LOW);
    delayMicroseconds(1);
    digitalWrite((*s_pins).CK_, HIGH);
    delayMicroseconds(1);

    byte bit = digitalRead((*s_pins).D_);
    //Serial.println(bit, BIN);
    data = data|(bit << i);
  }
  //Serial.print("byte: ");
  //Serial.println(data, BIN);
  return data;
}
//--------------------------------------------------------------------------
int readPosition(const SensorPins* s_pins)
{
  unsigned int position = 0;

  //shift in our data  
  digitalWrite((*s_pins).S_, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(s_pins,8);
  byte d2 = shiftIn(s_pins,8);
  byte d3 = shiftIn(s_pins,2);
  digitalWrite((*s_pins).S_, HIGH);

  //get our position variable
  position = d1;
  position = position << 8;
  position = position|d2;

  position = position >> 4;

  //check the offset compensation flag: 1 == started up
  if (!((d2 & B00001000) == B00001000))
    position = -1;

  //check the cordic overflow flag: 1 = error
  if (d2 & B00000100)
    position = -2;

  //check the linearity alarm: 1 = error
  if (d2 & B00000010)
    position = -3;

  //check the magnet range: 11 = error
  if ((d2 & B00000001) & (d3 & B10000000))
    position = -4;
    
  //add the checksum bit

  return position;
}
//--------------------------------------------------------------------------
