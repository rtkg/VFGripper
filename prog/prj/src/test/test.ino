

//#include "pwm01.h"


//const float pi = 3.14159;
const uint32_t pwm_frequency = 10000;

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
//======================= Global Variables ==========================
MotorControlPins* m1_pins=new MotorControlPins(24, 25, 26, 27, A0, 6);     //Motor 1 Arduino pins

//======================= Initialization ==========================
void setup() {
// pwm_set_resolution(16); //set PWM resolution to 16 bit (0 ... 65535)
 analogWriteResolution(12);
 
  pinMode(m1_pins->IN1_,OUTPUT);
  pinMode(m1_pins->IN2_,OUTPUT);
  pinMode(m1_pins->SF_,INPUT);
  pinMode(m1_pins->FB_,INPUT);
  pinMode(m1_pins->EN_,OUTPUT);
  pinMode(m1_pins->D2_,OUTPUT);
 // pwm_setup(m1_pins->D2_, pwm_frequency, 1); 
  
  
  digitalWrite(m1_pins->EN_,HIGH); //Enable the driver board
  
  //digitalWrite(m1_pins->D2_,HIGH);
  digitalWrite(m1_pins->IN1_,LOW); 
  digitalWrite(m1_pins->IN2_,HIGH); 
  
  analogWrite(2,2050);
  analogWrite(3,2050);
  analogWrite(4,2050);
  analogWrite(5,2050);
  analogWrite(6,2050);
  
//  pwm_write_duty(m1_pins->D2_, 32000); //50% duty cycle  
}
//============================== Loop ===================================
void loop() 
{

    
}

