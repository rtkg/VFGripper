/*!
 * \file
 * \brief   Arduino + ROS code to control DC motor.
 * \author  Krzysztof Kwiecinski supervised by Robert Krug
 * \version 0.1
 * \date    August 2015
 * \pre     roscore
 * \bug     
 * \warning Unfinished current control. Works but not perfect.
 *
 * rosserial_python node
 * Publishes:
 * TODO
 * 
 * Subscribes:
 * TODO
 * Change direction on callback.
 * Change speed on callback.
 * 
 * How to run?
 * 1) roscore 
 * 2) rosrun rosserial_python serial_node.py /dev/ttyACM0
 * 3) rqt (or via console: rostopic pub ... std_msgs/... [--once])
 * 
 * Documentation is done for doxygen.
 * 
 * TODO
 * - finish current control
 * - publisher/subscriber -> services
 */

#define USE_USBCON            // Has to be declared if there are problems with serial communication
#include <ros.h>              // ROS stuff for Arduino
#include <std_msgs/Empty.h>   // Message type
#include <std_msgs/Int8.h>    // Message type
#include <std_msgs/Float32.h> // Message type

/*=============== Declarations of pins ===============*/
const int M1_IN1 = 6;  // Motor input 1
const int M1_IN2 = 7;  // Motor input 2
const int M1_SF  = 24; // Motor status flag
const int M1_FB  = A0; // Analog input A0 for current sensing on Motor 1
const int EN     = 25; // Driver board enable pin
const int M1_D2  = 8;  // PWM pin to control output voltage

const int ledPin = 13; // LED pin to visualize  

/*=============== Constants ===============*/
const int BIT_RESOLUTION = 12; // 12 => [0; 4095], analogWrite(pin, PWM value)
const int PWM_MIN = 0;         // PWM minimum value
const int PWM_MAX = 4095;      // PWM maximum value

const int DELAY_MS = 1; // Delay time [ms]

const float ALPHA_CURRENT = 0.95;   // Value for filtering current
const float DESIRED_CURRENT = 0.02; // Desired current (=> torque)

const float KP = 500.0; // PID value
const float KI = 70.0;  // PID value
const float KD = 1.0;   // PID value
const float DEAD_SPACE = 0.005; // >= 0 TODO
const float DT = 1e-3;  // Time step [s]

const int V_MAX = 24;   // Maximum value for our maxxon motor [V] 

/*=============== Global variables ===============*/
int pwmValue = 0; // [%]
int t_old = 0; //! Timer value for calculating time steps
int t_new = 0; //! Timer value for calculating time steps

/*=============== Functions ===============*/
/*!
 * \brief Converts PWM width from [0; 100]% to PWM range value.
 * 
 * Converts PWM width from [0; 100]% to PWM range value.
 * \param[in] pwm_percent - PWM value in percent between [0; 100]%
 * \return    PWM value between [PWM_MIN, PWM_MAX]
 */
int pwmPercentToVal(const int pwm_percent) {
  return map(constrain(pwm_percent, 0, 100), 0, 100, PWM_MIN, PWM_MAX);
}

/*!
 * \brief Maps float value from one range to another.
 * 
 * Maps float value from one range to another.
 * \param[in] x - value to be mapped
 * \param[in] in_min - lower bound of input range
 * \param[in] in_max - upper bound of input range
 * \param[in] out_min - lower bound of output range
 * \param[in] out_max - upper bound of output range
 * \return    value mapped into output range
 */
float mapFloat(const float x, const float in_min, const float in_max, 
               const float out_min, const float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*============================================================*/
/*!
 * \brief Class defining PID Controller.
 * 
 * Class defining PID Controller.
 */
class PIDController {
  //TODO private
public:
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] Kp - PID proportional part
   * \param[in] Ki - PID integral part
   * \param[in] Kd - PID derivative part
   * \param[in] u_max - maximum value of control variable
   * \param[in] u_min - minimum value of control variable
   * \param[in] I - memory for the integral term
   * \param[in] dead_space - TODO
   */
  PIDController(float Kp, float Ki, float Kd, 
                float u_max, float u_min, 
                float I, float dead_space=-1) : 
    Kp_(Kp), Kd_(Kd), Ki_(Ki), u_max_(u_max), u_min_(u_min), I_(I), dead_space_(dead_space) {};
  
  /*!
   * \brief Calculates PID control variable.
   * 
   * Calculates PID control variable.
   * \param[in] error - error = setpoint - process variable
   * \param[in] d_error - error change in time 
   * \return control variable in range [-PWM_MAX, PWM_MAX]
   */
  float pid(const float error, const float d_error);
  
  float Kp_;          //! PID proportional part
  float Kd_;          //! PID integral part
  float Ki_;          //! PID derivative part
  float u_max_;       //! Maximum controller output (<= V_MAX or PWM_MAX)
  float u_min_;       //! Minimum controller output (>= -(V_MAX or PWM_MAX))
  float I_;           //! Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]
  float dead_space_;  //! TODO +/- error around desired value
};

// TODO
float PIDController::pid(const float error, const float d_error) {
  I_ += Ki_*error;  // Update integral
  float u = Kp_*error + I_ + Kd_*d_error; // Calculate control
  
  // TODO.
  // Do something when dead space is > < 
  if (dead_space_ > 0) { 
    if (abs(error) > dead_space_) {
      u = I_ + Kd_*d_error;
    }
    else {
      int dir = error < 0 ? -1 : 1; // Set direction
      u = Kp_ * (error - dir*dead_space_) + I_ + Kd_*d_error;
    }
  }
  
  // Clamp the control value and recalculate the Integral term (the latter to avoid windup)
  if (u > u_max_) {
    I_ -= Ki_*error;
    u = u_max_;
  } 
  else if (u < u_min_) {
    I_ -= Ki_*error; //p->u_min_ - u;
    u = u_min_;
  }
  
  //u = mapFloat(u, u_min_, u_max_, PWM_MIN, PWM_MAX); // TODO -+ values
  u = mapFloat(u, 0, u_max_, PWM_MIN, PWM_MAX);
  u = constrain(u, PWM_MIN, PWM_MAX);
  return u;  // Return control value (in PWM resolution).
}

/*============================================================*/
/*!
 * \brief Class defining current sensor. 
 * 
 * Class defining current sensor.
 */
class CurrentSensor {
  //TODO public -> private
  //private:
public:
  /*!
   *\brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] alpha - first order filter parameter, 0<=alpha<=1
   * \param[in] sensed_value - value sensed by current sensor [0; PWM_MAX]
   * \param[in] current - current sensed by current sensor [A]
   * \param[in] filtered_current - filtered current [A]
   */ 
  CurrentSensor(float alpha, int sensed_value, float current, float filtered_current) :
    alpha_(ALPHA_CURRENT), sensed_value_(sensed_value), 
    current_(current), filtered_current_(filtered_current) {};
  
  /*!
   * \brief Measure current.
   * 
   * Measure current.
   * \param[in] FB_pin - feedback pin to read analog value of current
   * \post sensed_value_ and current_ have new values
   * \return 
   */
  float senseCurrent(int FB_pin);
  
  /*!
   * \brief Filters measured current.
   * 
   * Filters measured current.
   * \post filtered_current_ has a new value 
   * \return measured current [A]
   */
  float filterCurrent();
  
  const float alpha_;       //! First order filter parameter, 0<=alpha<=1, alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time
  int sensed_value_;        //! analogRead maps input voltages between 0 and 3.3 V into int [0; 4095].
  float current_;           //! Sensed current in [A].
  float filtered_current_;  //! Filtered current in [A].
};

float CurrentSensor::senseCurrent(int FB_pin) {
  sensed_value_ = analogRead(FB_pin); // Map input voltages between 0 and 3.3 V into int [0; 4095]
  // Output provides analog current-sense feedback of approximately 0.525 V per A.
  return current_ =  mapFloat(sensed_value_, PWM_MIN * 1.0, PWM_MAX * 1.0, 0, 3.3) / 0.525; // -> [A]
}

float CurrentSensor::filterCurrent() {
  return filtered_current_ = filtered_current_*alpha_ + current_*(1.0 - alpha_);
}

/*============================================================*/
/*!
 * \brief Class defining current control. 
 * 
 * Class defining current control.
 */
class CurrentControl {
  //TODO private
public:
  /*!
   * \brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] i_d - desired current [A]
   * \param[in] last_error - last error 
   * \param[in] pid - PID controller
   */ 
  CurrentControl(float i_d, float last_error, const PIDController& pid) :  
    i_d_(i_d), last_error_(last_error), pid_(pid) {};
  
  /*!
   * \brief Current control feedback.
   * 
   * Current control feedback.
   * \param[in] current - current current
   */  
  float currentControl(const float current);
  
  float i_d_;         //! Desired current [A]
  float last_error_;  //! Last error (to calculate d_error)
  PIDController pid_; //! PID controller for closed loop
};

// TODO
float CurrentControl::currentControl(const float current) {
  // TODO What about minimum Jerk???
  //float dT = ros::now();
  float error   =  i_d_ - current;              // Current error 
  float d_error = (error - last_error_)/DT;     // Derivative of the current error
  last_error_ = error;                          // Update last error
  return pid_.pid(error, d_error);              // Return control value 
  // TODO + c_s->R_ * c_s->r_ * VOLTAGE_FACTOR; //compute control with feedforward term
}

/*============================================================*/
/*!
 * \brief Class defining motor control pins.
 * 
 * Class defining motor control pins.
 */
class MotorControlPins {
public:
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] IN1 - motor input 1
   * \param[in] IN2 - motor input 2
   * \param[in] SF - motor status flag
   * \param[in] EN - driver board enable pin
   * \param[in] FB - analog input for current sensing
   * \param[in] D2 - disable PWM pin to control output voltage
   */
  MotorControlPins(int IN1, int IN2, int SF, int EN, int FB, int D2) : 
  IN1_(IN1), IN2_(IN2), SF_(SF), EN_(EN), FB_(FB), D2_(D2) {};
  
  int IN1_; //! Motor input 1 pin, controls motor direction
  int IN2_; //! Motor input 2 pin, controls motor direction
  int SF_;  //! Motor status flag
  int EN_;  //! Driver board enable pin
  int FB_;  //! Analog input pin for current feedback from H-Bridge
  int D2_;  //! Disable 2 PWM pin to control output voltage, controls speed
};

/*============================================================*/
/*!
 * \brief Class defining DC Motor. 
 * 
 * Class defining DC Motor.
 */
class Motor {
  //TODO private
public:
  /*!
   * \brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] m_pins - motor control pins
   * \param[in] cs - current sensor
   * \param[in] cc - current control
   */ 
  Motor(const MotorControlPins& m_pins, const CurrentSensor& cs, const CurrentControl& cc) :
    m_pins_(m_pins), cs_(cs), cc_(cc) {};
  
  /*!
   * \brief Drives motor in forward direction. 
   * 
   * Drives motor in forward direction.
   * \post two IN pins have new logic values
   */
  void forward();
  
  /*!
   * \brief Drives motor in reverse direction.
   * 
   * Drives motor in reverse direction.
   * \post two IN pins have new logic values
   */
  void reverse();
  
  /*!
   * \brief Sets PWM (=> speed) for motor.
   * 
   * Sets PWM (=> speed) for motor.
   * \param[in] pwm_val - PWM value [PWM_MIN; PWM_MAX]
   * \post PWM pin has a new analog value
   * \return PWM value [PWM_MIN; PWM_MAX]
   */
  int setPwm(const int pwm_val);
  
  MotorControlPins m_pins_; //! Motor control pins
  CurrentSensor cs_;        //! Current sensor
  CurrentControl cc_;       //! Current control
};

void Motor::forward() {
  digitalWrite(m_pins_.IN1_, HIGH);
  digitalWrite(m_pins_.IN2_, LOW);
}

void Motor::reverse() {
  digitalWrite(m_pins_.IN1_, LOW);
  digitalWrite(m_pins_.IN2_, HIGH);
}

int Motor::setPwm(const int pwm_val) {
  analogWrite(m_pins_.D2_, pwm_val); // TODO Motor::pwm_pin_
  return pwm_val;
}

Motor m1(MotorControlPins(M1_IN1, M1_IN2, M1_SF, EN, M1_FB, M1_D2),
         CurrentSensor(0, 0, 0, ALPHA_CURRENT),
         CurrentControl(DESIRED_CURRENT, 0.0,
                        PIDController(KP, KI, KD, V_MAX, -V_MAX, 0.0, 0.0)));

/*=============== ROS ===============*/
// TODO documentation for this part
ros::NodeHandle nh; //! Node handler

std_msgs::Int8 count_msg; 
ros::Publisher pub_counter("counter", &count_msg);
std_msgs::Float32 current_msg;
ros::Publisher pub_current("current", &current_msg);
std_msgs::Float32 filtered_current_msg;
ros::Publisher pub_filtered_current("filtered_current", &filtered_current_msg);
std_msgs::Float32 error_msg;
ros::Publisher pub_error("error", &error_msg);
std_msgs::Float32 u_msg;
ros::Publisher pub_u("u", &u_msg);
std_msgs::Float32 i_d_msg;
ros::Publisher pub_i_d("i_d", &i_d_msg);

int counter = 0;
void confirmCallback() {
  digitalWrite(ledPin, counter % 2 ? HIGH : LOW); // blink the led
  counter++;
}

// Change direction of the motor when receives a message.
void changeDirectionCallback( const std_msgs::Empty& empty_msg ) {
  if (counter % 2) {
    m1.forward();
  }
  else {
    m1.reverse();
  }
  confirmCallback();
}
ros::Subscriber<std_msgs::Empty> sub_change_dir("change_dir", &changeDirectionCallback);

// TODO remove
int pwm = 0;

/* Change speed of the motor when receives a message.
 * pwm_msg - [0; 100] %
 */
void setPwmCallback( const std_msgs::Int8& pwm_msg ) {
  int pwmValue = pwmPercentToVal(pwm_msg.data);
  analogWrite(M1_D2, pwmValue);
  
  confirmCallback();
}
ros::Subscriber<std_msgs::Int8> sub_set_vel("set_vel", &setPwmCallback);

void setKpCallback( const std_msgs::Float32& Kp_msg ) {
  m1.cc_.pid_.Kp_ = Kp_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kp("set_kp", &setKpCallback);

void setKiCallback( const std_msgs::Float32& Ki_msg ) {
  m1.cc_.pid_.Ki_ = Ki_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_ki("set_ki", &setKiCallback);

void setKdCallback( const std_msgs::Float32& Kd_msg ) {
  m1.cc_.pid_.Kd_ = Kd_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kd("set_kd", &setKdCallback);

void setIdCallback( const std_msgs::Float32& i_d_msg ) {
  m1.cc_.i_d_ = i_d_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_i_d("set_i_d", &setIdCallback);

/*============================================================*/
/*!
 * \brief Sets up code after every reset of the Arduino Board. 
 * 
 * Set up code after every reset of the Arduino Board.
 */
void setup()
{
  /* ROS */
  nh.initNode();
  
  nh.advertise(pub_counter);
  nh.advertise(pub_current);
  nh.advertise(pub_filtered_current);
  nh.advertise(pub_error);
  nh.advertise(pub_u);
  nh.advertise(pub_i_d);
  
  nh.subscribe(sub_set_vel);
  nh.subscribe(sub_set_kp);
  nh.subscribe(sub_set_ki);
  nh.subscribe(sub_set_kd);
  nh.subscribe(sub_set_i_d);
  nh.subscribe(sub_change_dir);
  
  /* Arduino */
  pinMode(m1.m_pins_.IN1_, OUTPUT); // Controls motor direction
  pinMode(m1.m_pins_.IN2_, OUTPUT); // Controls motor direction
  pinMode(m1.m_pins_.SF_,  INPUT);  // Motor Status flag from H-Bridge
  pinMode(m1.m_pins_.FB_,  INPUT);  // Current feedback from H-Bridge
  pinMode(m1.m_pins_.EN_,  OUTPUT); // Enables motor
  pinMode(m1.m_pins_.D2_,  OUTPUT); // Controls speed
  //VDD = Arduino 3.3 V
  
  /* Set default states */
  digitalWrite(m1.m_pins_.EN_,  HIGH);
  digitalWrite(m1.m_pins_.IN1_, HIGH);
  digitalWrite(m1.m_pins_.IN2_, LOW);
  pinMode(ledPin, OUTPUT);
  
  /* Set PWM resolution */
  analogWriteResolution(BIT_RESOLUTION);
  analogReadResolution(BIT_RESOLUTION);
  
  m1.setPwm(20);
  
  /* Open a serial connection */
  //Serial.begin(57600);
}

/*============================================================*/
/*! 
 * \brief Runs repeatedly code in the loop. 
 * 
 * Runs repeatedly code in the loop. 
 */
void loop()
{
  current_msg.data = m1.cs_.senseCurrent(m1.m_pins_.FB_);
  pub_current.publish( &current_msg );
  filtered_current_msg.data = m1.cs_.filterCurrent();
  pub_filtered_current.publish( &filtered_current_msg );
  count_msg.data = counter;
  pub_counter.publish( &count_msg );
  u_msg.data =  m1.setPwm(m1.cc_.currentControl(m1.cs_.filtered_current_));
  pub_u.publish( &u_msg );
  error_msg.data = m1.cc_.last_error_;
  pub_error.publish( &error_msg );
  i_d_msg.data = m1.cc_.i_d_;
  pub_i_d.publish( &i_d_msg );
  
  nh.spinOnce();
  delay(DELAY_MS);
}