//Low-level gripper control
//Robert Krug, Todor Stoyanov 13/06/2014

#define POSITION_MODE 0
#define CURRENT_MODE 1
#define BELT_MODE 2
#define NO_MODE 3

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
  int offset_;
  float v_;     //sensor value
  float dv_;    //time-derivative of the sensor value
  
  SensorStates(int reading, float v, float dv) : reading_(reading), v_(v), dv_(dv) {};
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
  
  ControlStates(float r, float e, float de, float T) : r_(r), e_(e), de_(de), T_(T){};
};
//======================= Global Variables ========================
MotorControlPins* m1_pins=new MotorControlPins(24, 25, 26, 27, A0, 6);     //Motor 1 Arduino pins
PIDParameters* pid_m1_pc=new PIDParameters(50.0, 0.0, 0.0, 4095.0, -4095.0, 0.0); //Position controller PID parameters for Motor 1
ControlStates* cs1=new ControlStates(0.0, 0.0, 0.0, 0.0); //Setpoint and error for drive 1

SensorPins* e1_pins=new SensorPins(31, 33, 35); //Encoder 1 pins
SensorStates* e1_s=new SensorStates(0, 0.0 ,0.0); //Sensor states for encoder 1

int dT = 10000; //Sample time in microseconds
int t_old, t_new;
int mode = NO_MODE;

bool beltsInited = false;


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
  Serial.println("setup done");
  t_old = micros();
}
//====================  Function Declarations ===========================
void update(); //Simple state machine which reads sensors, computes and sets controls
float pid(float e, float de, PIDParameters* p); //Computes the controls from error e, error derivative de, and controller parameters p)
int actuate(float control, const MotorControlPins* mc_pins); //Send the sign-corrected input to the actuator and read the motor status flag
float minimumJerk(float t0, float t, float q0, float qf); //evaluates a minimum-jerk position trajectory
int readEncoder(const SensorPins* s_pins); //read the current position from the sensor connected to the given pins
void computeEncoderStates(SensorStates* e_s, const SensorPins* s_pins); //Converts the encoder readings and computes derivatives
byte shiftIn(const SensorPins* s_pins, int readBits); //read in a byte of dapta from the digital input corresponding to the given sensor
//============================== Loop ===================================
void loop() 
{
  t_new = micros();
  //Do nothing if the sampling period didn't pass yet   
  if(abs(t_new - t_old) < dT) 
    return;
  t_old = t_new;
  
  processMessage();
  if (mode == POSITION_MODE) {
    update(); //Read sensors, compute and send controls
  } else {
    delay(100);
  }
  
  
  //TODO: write stuff to the output serial connection ... could be done with a lower frequency or only when triggering a request
  
}
//====================  Function Implementations =========================
void update()
{
  computeEncoderStates(e1_s, e1_pins); //Compute angle + velocity of encoder 1
  float r1=minimumJerk((*cs1).ti_,(float)millis()/1000, (*cs1).T_, (*cs1).ri_, (*cs1).rf_);  //update the setpoint for drive 1 
  Serial.println("current setpoint:");
  Serial.println(r1, 4);
  float e1=r1-(*e1_s).v_; //position error for drive 1
  float de1= (e1-(*cs1).e_)/((float)dT); //derivative of the position error for drive 1

  (*cs1).r_=r1; (*cs1).e_=e1; (*cs1).de_=de1; //update the control states for the next iteration 
  
  float u1= pid(e1, de1, pid_m1_pc); //control for drive 1
  actuate(u1, m1_pins); //actuate drive 1
  Serial.println("control output");
  Serial.println(u1, 4);
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
float minimumJerk(float t0, float t, float T, float q0, float qf)
{
  if(T == 0) return 0;
  if(t > t0+T) {
    t=T+t0;
  }
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
int readEncoder(const SensorPins* s_pins)
{
  unsigned int reading = 0;

  //shift in our data  
  digitalWrite((*s_pins).S_, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(s_pins,8);
  byte d2 = shiftIn(s_pins,8);
  byte d3 = shiftIn(s_pins,2);
  digitalWrite((*s_pins).S_, HIGH);

  //get our reading variable
  reading = d1;
  reading = reading << 8;
  reading = reading|d2;

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
void computeEncoderStates(SensorStates* e_s, const SensorPins* s_pins)
{
//  Serial.print("Reading: ");
  (*e_s).reading_ = readEncoder(s_pins); //Read encoder 
  
  if(!beltsInited) {
      (*e_s).offset_ = (*e_s).reading_;
      beltsInited = true;
  }
   
  if ((*e_s).reading_ >= 0)
    {
      //calculate sensor value 
      float v=(float)((*e_s).reading_-(*e_s).offset_) * 0.088; //where is this number coming from ?? -> encoder data sheet

      //calculate sensor value derivative
      (*e_s).dv_ = (v-(*e_s).v_)/((float)dT);

      (*e_s).v_=v; //set new sensor value

      Serial.print("Reading: ");
      Serial.print((*e1_s).reading_, DEC);
      Serial.print("offset: ");
      Serial.print((*e1_s).offset_, DEC);
      Serial.print(" Position: ");
      Serial.println((*e1_s).v_, 4);//, DEC);
      Serial.print(" Velocity: ");
      Serial.println((*e1_s).dv_, 4); //DEC);
    }
  else
    {
      Serial.print("Error: ");
      Serial.println((*e_s).reading_, DEC);
    }
}
void processMessage() {

  byte b1,b2;
  short target_val = 0;
  
  if(Serial.available()) {
    char code[3];  
    for (int i=0; i<3; i++) {
        b1 = Serial.read();
        code[i] = b1;
    }
    //Serial.println(code);
    if(code[0] == 'P' && code[1] == 'O' && code[2] == 'S') {
       //position mode 
       mode = POSITION_MODE;
       b1 = Serial.read();
       b2 = Serial.read();
       target_val = b2;
       target_val = target_val << 8;
       target_val = target_val | b1;
       //Serial.println(b1, BIN);
       //Serial.println(b2, BIN);
       (*cs1).rf_ = (float)target_val;
       (*cs1).ri_ = (float)(*e1_s).v_;
       (*cs1).ti_ = (float)millis()/1000;
       (*cs1).T_ = 10;
       Serial.println("Got a new SETPOINT !!!!!!!!!!!!!!!!!!!!!!!!!!");
       Serial.println("rf:");
       Serial.println((*cs1).rf_,4);
       Serial.println("ri:");
       Serial.println((*cs1).ri_,4);
       Serial.println("ti:");
       Serial.println((*cs1).ti_,4);
       Serial.println("T:");
       Serial.println((*cs1).T_,4);
    }
    if(code[0] == 'N' && code[1] == 'O' && code[2] == 'N') {
       mode = NO_MODE;
       target_val = 0;
    }
  } 
}
//--------------------------------------------------------------------------
