/*!
 * \file
 * \brief   Arduino + ROS code to control DC motor.
 * \author  Krzysztof Kwiecinski supervised by Robert Krug
 * \version 0.1
 * \date    August 2015
 * \pre     roscore
 * \bug     
 * \warning TODO Feedforward term - verify (V_MAX?)
 *               add jerk, 
 *               H-Bridge changes - test, 
 *               PID Calibration, 
 *               frequency PWM
 *               class SensorPins,
 *
 * rosserial_python node
 * Publishes:
 * TODO
 * 
 * Subscribes:
 * TODO
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
 * - private classes
 */

#define USE_USBCON            // Has to be declared if there are problems with serial communication
#include <ros.h>              // ROS stuff for Arduino

#include <std_msgs/Empty.h>   // Message type
#include <std_msgs/Int8.h>    // Message type
#include <std_msgs/Float32.h> // Message type

#include <cmath>
#include <limits.h>
#include <limits>

// TODO How to include them?
/*
#include <velvet_msgs/GripperState.h>
#include <velvet_msgs/GripperState.h>
#include <velvet_msgs/SetPos.h>
#include <velvet_msgs/SetCur.h>
#include <velvet_msgs/SetVel.h>
#include <velvet_msgs/SetPID.h>
#include <std_srvs/Empty.h>
*/

/*=============== Declarations of pins ===============*/
const int M1_IN1 = 6;  //! Motor input 1
const int M1_IN2 = 7;  //! Motor input 2
const int M1_SF  = 24; //! Motor status flag
const int M1_FB  = A0; //! Analog input A0 for current sensing on Motor 1
const int EN     = 25; //! Driver board enable pin
const int M1_D2  = 8;  //! PWM pin to control output voltage

// TODO Check!!!!!!!!!!!
const int E1_CSn = 26;  //! Chip select pin
const int E1_DO  = 27;  //! Sensor data output pin
const int E1_CLK = 28;  //! Clock pin

const int LED_PIN = 13; //! LED pin to visualize  

/*=============== Controller modes ===============*/
enum ControlMode {
  POSITION_MODE, //! Position controller
  VELOCITY_MODE, //! Velocity controller 
  CURRENT_MODE,  //! Current controller
  NO_MODE        //! Without controller
};

/*=============== Constants ===============*/
const int BIT_RESOLUTION = 12; //! 12 => [0; 4095], analogWrite(pin, PWM value)
const int PWM_MIN = 0;         //! PWM minimum value
const int PWM_MAX = 4095;      //! PWM maximum value

const int DELAY = 1; // Delay time [ms]

const float ALPHA_CURRENT = 0.99;   //! Value for filtering current
const float DESIRED_CURRENT = 0.015; //! Desired current (=> torque)

const float ALPHA_ENCODER = 0.7; //! TODO
const float ENCODER_RESOLUTION = 4095; //! Encoder resolution: 12 bit, i.e., 0 - 4095 (=> 0.0879 deg)
const float SCALE_ENCODER = 1; //! TODO

const float T_JERK = 0.0;       //! Time for executing the loop

const float KP = 1000000.0; //! PID value
const float KI = 10.0;   //! PID value
const float KD = 0.02;   //! PID value
const float DEAD_SPACE = 0.0; //(1-ALPHA_CURRENT)*DESIRED_CURRENT; //! Deadband around setpoint
const float DT = 1e-3;  //! Time step [s]

const int   V_MAX = 24;   //! Maximum value for our maxon motor [V] 
const float V_MIN = 4.4;  //! Minimum value for our maxon motor to overcome inner resistance [V]
const int OFFSET  = static_cast<int>(mapFloat(V_MIN, 0.0, V_MAX, PWM_MIN, PWM_MAX)); //! V_MIN converted to PWM value
const float R_MOTOR = 7.25; //! Terminal resistance of the motor

/*=============== Global variables ===============*/
int pwm_duty_cycle = 0; //! PWM duty cycle [%]
int t_old = 0; //! Timer value for calculating time steps
int t_new = 0; //! Timer value for calculating time steps
int offset = OFFSET; //! Offset for PWM value

/*=============== Functions ===============*/
/*!
 * TODO  /* Set PWM resolution 
 */
void setUpPwm() {
  analogWriteResolution(BIT_RESOLUTION);
  analogReadResolution(BIT_RESOLUTION);
}

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

// TODO Use this class
// TODO Put it into [Current, ?]Controller??? I think so
/*============================================================*/
/*!
 * \brief Class defining Control Variables.
 * 
 * Class defining Control Variables.
 */
class ControlStates
{
public:
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] r - setpoint (reference) value at current iteration
   * \param[in] rf - final setpoint value
   * \param[in] ri - initial setpoint value
   * \param[in] e - error
   * \param[in] de - error derivative
   * \param[in] ti - time when we initialized motion [s]
   * \param[in] T - time for executing the loop
   * \param[in] u - computed control
   * \param[in] active - flag indicating whether the corresponding controller is active or not
   */
  ControlStates(float r, float rf, float ri, float e, float de, 
                float ti, float T, int u, bool active) : 
  r_(r), rf_(rf), ri_(ri), e_(e), de_(de), ti_(ti), T_(T), u_(u), active_(active) {};
  
  /*!
   * \brief Calculates minimum jerk trajectory point.
   * 
   * Calculates minimum jerk trajectory point. 
   * Allows to avoid step response and hence to obtain smoother trajectory.
   * The smaller T, the more rapid response. 
   * Similar to step response achieved by higher order systems.
   * 
   * \return minimum jerk trajectory point
   */
  float minimumJerk(float t);
  
  float r_;  //! Setpoint (reference) value at current iteration
  float rf_; //! Final setpoint value
  float ri_; //! Initial setpoint value
  float e_;  //! Error
  float de_; //! Error derivative
  
  float ti_; //! Time when we initialized motion [s]
  float T_;  //! Time for executing the loop
  
  int u_; //! Computed control
  
  bool active_; //! Flag indicating whether the corresponding controller is active or not
};

float ControlStates::minimumJerk(float t) {
  if (t > ti_ + T_) {
    t = T_ + ti_;       // Make sure the ouput stays at qf after T has passed
  }
  // Return smoother value
  // TODO What with dividing by 0?
  return ri_ + (rf_-ri_)*(10*pow((t-ti_)/T_, 3) - 15*pow((t-ti_)/T_, 4) + 6*pow((t-ti_)/T_, 5));
}

/*============================================================*/
/*!
 * \brief Class defining PID Controller.
 * 
 * Class defining PID Controller.
 */
class PIDController {
public:
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] Kp - PID proportional part
   * \param[in] Ki - PID integral part
   * \param[in] Kd - PID derivative part
   * \param[in] u_min - minimum value of control variable
   * \param[in] u_max - maximum value of control variable
   * \param[in] I - memory for the integral term
   * \param[in] dead_space - dead area around setpoint 
   */
  PIDController(float Kp, float Ki, float Kd, 
                float u_min, float u_max, 
                float I, float dead_space=-1) : 
    Kp_(Kp), Kd_(Kd), Ki_(Ki), u_min_(u_min), u_max_(u_max), I_(I), dead_space_(dead_space) {};
  
  /*!
   * \brief Calculates PID control variable.
   * 
   * Calculates PID control variable.
   * Computes the controls from error e, error derivative de and PID parameters.
   * \param[in] error - error = setpoint - process variable
   * \param[in] d_error - error change in time 
   * \return control variable in range [-PWM_MAX, PWM_MAX]
   */
  float pid(const float error, const float d_error);
  
  float Kp_;          //! PID proportional part
  float Kd_;          //! PID integral part
  float Ki_;          //! PID derivative part
  float u_min_;       //! Minimum controller output
  float u_max_;       //! Maximum controller output 
  float I_;           //! Serves as memory for the integral term [i.e., I=dT*(Ki*e_0, ... , Ki*e_t)]
  float dead_space_;  //! Calculated output must leave the deadband before the actual output will change; >=0
};

float PIDController::pid(const float error, const float d_error) {
  float u;                          // Control variable
  I_ += error;                      // Update integral
  /* TODO Add this part??? Set boundaries */
  /*
  if (I_ > I_max_ || I < I_min_) {  
    constrain(I_, I_min_, I_max_);  // Keep the integral within some boundaries
  }
 */ 
  if ( !(dead_space_> 0) ) {              // If there is no deadband
    u = Kp_*error + Ki_*I_ + Kd_*d_error; // Calculate control
  }
  // TODO I don't fully understand this part ;/
  else {                             // If there is a deadband 
    if (abs(error) < dead_space_) {  // And process variable is inside it
      u = Ki_*I_ + Kd_*d_error;      // Then update control value without P-term (just ID) TODO why???
    }
    else {                           // If PV is outside deadband
                                     // Then update CV with smaller error TODO why??? and all PID terms 
      u = Kp_ * (error - (error<0 ? -1 : 1)*dead_space_) + Ki_*I_ + Kd_*d_error;
    }
  }
  
  // Clamp the CV and recalculate the Integral term (the latter to avoid windup)
  if ((u > u_max_) || (u < u_min_)) {     // If CV is bigger/smaller than max/min feasible value
    I_ -= error;                      // Back-calculate the I-term to avoid wind up
    u = constrain(u, u_min_, u_max_); // Clamp the CV
  }
  
  return u;  // Return control value (in +/- PWM resolution).
}

/*============================================================*/
/*!
 * \brief Class defining current sensor. 
 * 
 * Class defining current sensor.
 */
class CurrentSensor {
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
    alpha_(alpha), sensed_value_(sensed_value), 
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
   * \pre current_ has to be measured before
   * \post filtered_current_ has a new value 
   * \return measured current [A]
   */
  float filterCurrent();
  
  float alpha_;             //! First order filter parameter, 0<=alpha<=1, alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time
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
public:
  /*!
   * \brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] r_motor - terminal resistance of the motor [ohm]
   * \param[in] cs - control states of the current control 
   * \param[in] pid - PID controller
   */ 
  CurrentControl(float r_motor, const ControlStates& cs, const PIDController& pid) :  
  r_motor_(r_motor), cs_(cs), pid_(pid) {};
  
  /*!
   * \brief Current control feedback.
   * 
   * Current control feedback.
   * \param[in] current - current current
   */  
  float currentControl(const float current);
  
  float r_motor_;     //! Terminal resistance of the motor [ohm] (to calculate the feedforward term)
  ControlStates cs_;  //! Control states of the current control
  PIDController pid_; //! PID controller for closed loop
};

// TODO
float CurrentControl::currentControl(const float current) {
  // cs_.r_ = cs_.minimumJerk(static_cast<float>(millis())); // Set a new desired current [A] (filtered) TODO nan!!!
  //TODO ??? ??? And later part with dir on v3?
  //float dir =  c_s->r_ >= 0 ? 1 : -1;
  //float current = dir * curr_s->v_;
  cs_.de_ = (cs_.e_ - (cs_.r_-current));  // Derivative of the current error (already a bit filtered)
  //cs_.de_ = ALPHA_ERROR*cs_.de_ + (1-ALPHA_ERROR)*(cs_.e_ - (cs_.r_-current)); // TODO Or filtereven more?
  cs_.e_ =  cs_.r_ - current;             // Current error
  // TODO Maybe add set point weighting?
  cs_.u_ = pid_.pid(cs_.e_, cs_.de_);     // Set a new control value
  
  float feedforward = r_motor_ * current; // V = L*di/dt + RI + E  
                                          // We hold the motor => w=0 => E=0; di/dt == 0
                                          // => V = RI
  mapFloat(feedforward, 0, 1.0*V_MAX, 1.0*PWM_MIN, 1.0*PWM_MAX); // Map from Voltage to PWM range TODO correct???
  // TODO do we need 1.0*???
  cs_.u_ += feedforward;  // Compute control with feedforward term // TODO shouldn't be +/-???
  
  //u_>=0 ? u_+=offset : u_-=offset;              // TODO Add resistance of the motor                      
  //constrain(u_, static_cast<int>(pid_.u_min_), static_cast<int>(pid_.u_max_));      // TODO Clamp after that
  
  constrain(cs_.u_, static_cast<int>(pid_.u_min_), static_cast<int>(pid_.u_max_)); // Clamp
  return cs_.u_;    // Return CV (can be negative!)
}

/*============================================================*/
/*!
 * \brief Class defining control.
 * 
 * Class defining control.
 */
class Control {
public:
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] mode - position/current/velocity controller
   * \param[in] cc - current control
   */
  Control(ControlMode mode, const CurrentControl& cc) :
  mode_(mode), cc_(cc) {};
  
  ControlMode mode_;    //! Position/current/velocity controller at the moment 
  CurrentControl cc_;   //! Current control
};

/*============================================================*/
/*!
 * \brief Class defining sensor pins.
 * 
 * Class defining sensor pins.
 */
class SensorPins {
public:
  int DO_;  //! Sensor data output pin
  int CLK_; //! Clock pin
  int CSn_;  //! Chip select pin
  
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] DO - sensor data output pin
   * \param[in] CLK - clock pin
   * \param[in] CSn - chip select pin
   */
  SensorPins(int DO, int CLK, int CSn) : 
    DO_(DO), CLK_(CLK), CSn_(CSn) {};
    
  /*!
   * TODO
   */
  void setUp();  
    
  /*!
   * \brief Read in a byte (or less bits) of data from the digital input corresponding to the given sensor.
   * 
   * Read in a byte (or less bits) of data from the digital input corresponding to the given sensor.
   * \param[in] readBits - number of bits to be read
   * \return Byte containing all read bits.
   */
  byte shiftIn(int readBits); 
};

void SensorPins::setUp() {
  pinMode(DO_,  INPUT);
  pinMode(CLK_, OUTPUT);
  pinMode(CSn_, OUTPUT);
  
  digitalWrite(CLK_, HIGH);  // Give some default value
  digitalWrite(CSn_,  HIGH); // Give some default value
}

byte SensorPins::shiftIn(int readBits) {
  byte data = 0;
  for (int i = readBits - 1; i >= 0; i--) {
    // Each subsequent rising CLK edge shifts out one bit of data
    digitalWrite(CLK_, LOW);
    delayMicroseconds(1);
    digitalWrite(CLK_, HIGH);
    delayMicroseconds(1);
    
    byte bit = digitalRead(DO_);
    data = data | (bit << i); 
  }
  return data;
}

/*============================================================*/
/*!
 * \brief Class defining encoder.
 * 
 * Class defining encoder.
 */
class Encoder {
public: 
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] res - resolution (i.e., number of ticks per revolution)
   * \param[in] scale - scale factor used for conversion from ticks to value - can be used to lump transmission ratio, radius ...
   * \param[in] alpha - first order filter parameter, 0<=alpha<=
   * \param[in] s_pins - sensor pins of the encoder
   */
  Encoder(int res, float scale, float alpha, SensorPins s_pins);   
  
  /*!
   * \brief Do the first reading and set offset. 
   * 
   * Do the first reading and set offset.
   */
  void setUp();
  
  //TODO
  int getRawTicks();
  
  /*!
   * \brief Read the current position from the sensor.
   * 
   * Read the current position from the sensor.
   * \post k_ may have a new value
   * \post raw_ticks_ has a new value
   * \return Reading of the encoder (ticks) or a failure status flag
   */
  void readEncoder(); 
  
  /*!
   * \brief TODO 
   */ 
  void convertSensorReading();
  
  /*!
   * \brief Converts the encoder readings and computes derivatives.
   * 
   * Converts the encoder readings and computes derivatives.
   * \return Raw ticks > 0 or a status flag -5
   */
  int computeEncoder(); 
  
  // TODO check types!!!
  int raw_ticks_;  //! Raw encoder ticks [0; 4095]
  float p_raw_;    //! Converted value (scale_)
  float p_;        //! Filtered converted value 
  float dp_;       //! Time-derivative of the converted value
  int k_;          //! Rollover counter TODO what's that? 0/1?
  int rev_;        //! Number of revolutions
  int res_;        //! Resolution (i.e., number of ticks per revolution)
  float scale_;    //! Scale factor used for conversion from ticks to value - can be used to lump transmission ratio, radius ...
  int offset_;     //! Used for zeroing
  float alpha_;    //! First order filter parameter, 0<=alpha<=1
  SensorPins s_pins_; //! Sensor pins
  
  void convertSensorReadingR(int raw_ticks_new);
  void setupEncoderSensorR();
  int computeEncoderStatesR();
  int readEncoderR();
};

Encoder::Encoder(int res, float scale, float alpha, SensorPins s_pins) :   
  raw_ticks_(0.0),
  p_raw_(0.0),
  p_(0.0),
  dp_(0.0),
  k_(0),
  rev_(0),
  res_(res), 
  scale_(scale),
  offset_(0.0),
  alpha_(alpha), 
  s_pins_(s_pins)
  {};

void Encoder::setUp() {
  offset_ = getRawTicks();
  readEncoder();
  convertSensorReading();
}

int Encoder::getRawTicks() {
  unsigned int reading = 0;
  
  // If CSn changes to logic low, Data Out (DO) will change from high impedance (tri-state) to logic high 
  // and the read-out will be initiated.
  digitalWrite(s_pins_.CSn_, LOW);
  //Propagation delay 384μs (slow mode) 96μs (fast mode) 
  // System propagation delay absolute output : delay of ADC, DSP and absolute interface 
  delayMicroseconds(1);
  // Shift in our data (read: 18bits ( 12bits data + 6 bits status)
  byte d1 = s_pins_.shiftIn(8);
  byte d2 = s_pins_.shiftIn(8);
  byte d3 = s_pins_.shiftIn(2);
  // A subsequent measurement is initiated by a “high” pulse at CSn with a minimum duration of tCSn
  digitalWrite(s_pins_.CSn_, HIGH);
  
  // Get our reading variable
  reading = d1;
  reading = reading << 8;
  reading = reading | d2; 
  // The first 12 bits are the angular information D[11:0]
  reading = reading >> 4;
  
  // The subsequent 6 bits contain system information, 
  // about the validity of data such as OCF, COF, LIN, Parity and Magnetic Field status.
  if (!((d2 & B00001000) == B00001000)) { // Check the offset compensation flag: 1 == started up
    reading = -1; // TODO enum
  }
  if (d2 & B00000100) { // Check the cordic overflow flag: 1 = error
    reading = -2;
  }
  if (d2 & B00000010) { // Check the linearity alarm: 1 = error
    reading = -3;
  }
  if ((d2 & B00000001) & (d3 & B10000000)) { // Check the magnet range: 11 = error
    reading = -4;
  }
  //add the checksum bit TODO
  
  return static_cast<int>(reading); 
}

void Encoder::readEncoder() {
  int reading = getRawTicks();

  // Update raw_ticks_ and rollover
  // TODO What is rollover? Only gives 0/1
  if (reading - raw_ticks_ < -res_ / 2) { 
    k_++; // Delta is smaller than minus half the resolution -> positive rollover
  }
  if (reading - raw_ticks_ > res_ / 2) {
    k_--; // Delta is larger than half the resolution -> negative rollover
  }
  
  // Here it's number of revolutions FIXME
  if (reading * raw_ticks_ < 0) { // Different signs 
    if (reading > 0) { 
      rev_++; 
    }
    else {
      rev_--; 
    }
  }
  
  raw_ticks_ = reading; // Update ticks counter
}

void Encoder::convertSensorReading() {
  // angle = 2*pi*[(reading-offset)/res+k]
  p_raw_ = 2.0*M_PI * ((static_cast<float>(raw_ticks_) - static_cast<float>(offset_)) 
           / static_cast<float>(res_) 
           + static_cast<float>(k_)) 
           * scale_; 
  float p_tmp = alpha_*p_ + (1-alpha_)*p_raw_; // First order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
  
  float dT=1000; // [us] FIXME
  float dp_raw = (p_tmp - p_) / ((float)dT*1e-6);
  
  dp_ = alpha_*dp_ + (1-alpha_)*dp_raw;
  p_ = p_tmp;
}; 

int Encoder::computeEncoder() {
  readEncoder();
  if ((k_ == std::numeric_limits<int>::max()) ||  (k_ == std::numeric_limits<int>::min())) {
    raw_ticks_ = (-5); // Over/underflow of the rollover count variable TODO enum
  }
  else {                     // Everything's OK
    convertSensorReading();  // Update the sensor value 
  }
  return raw_ticks_;
}

void Encoder::setupEncoderSensorR()
{
  raw_ticks_ = readEncoderR(); //Read encoder 1
  offset_    = raw_ticks_;
  convertSensorReadingR(raw_ticks_);
}
int Encoder::readEncoderR()
{
  unsigned int reading = 0;
  
  //shift in our data
  digitalWrite(s_pins_.CSn_, LOW);
  delayMicroseconds(1);
  byte d1 = s_pins_.shiftIn(8);
  byte d2 = s_pins_.shiftIn(8);
  byte d3 = s_pins_.shiftIn(2);
  digitalWrite(s_pins_.CSn_, HIGH);
  
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
void Encoder::convertSensorReadingR(int raw_ticks_new) {
  if ((raw_ticks_new - raw_ticks_) < -res_ / 2) // delta is smaller than minus half the resolution -> positive rollover
    k_++;
  if ((raw_ticks_new - raw_ticks_) > res_ / 2) // delta is larger than half the resolution -> negative rollover
    k_--;
  
  // TODO what's this part?
  raw_ticks_ = raw_ticks_new;
  p_raw_ = 2 * M_PI * ((raw_ticks_ - (float)offset_) / (float)res_ + (float)k_) * scale_;
  float p_temp = alpha_ * p_ + (1 - alpha_) * p_raw_; //first order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
  float dT=1000;
  float dp_raw = (p_temp - p_) / ((float)dT*1e-6);
  dp_ = alpha_ * dp_ + (1 - alpha_) * dp_raw;
  p_ = p_temp;
}; // angle=2*pi*[(reading-offset)/res+k]
int Encoder::computeEncoderStatesR()
{
  int t_new = readEncoderR();
  
  if ((k_ == INT_MAX) ||  (k_ == INT_MIN))
    raw_ticks_ = (-5); //Over/underflow of the rollover count variable
    
    if (raw_ticks_ >= 0)
    {
      convertSensorReadingR(t_new);  //update the sensor value (sets e_s->v_)   //Serial.print("Reading: ");
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
       *    Serial1.print("Error in computeEncoderStates(...): ");
       *    Serial1.println(e_s->raw_ticks_, DEC);
       *    Serial1.print("pins ");
       *    Serial1.println(s_pins->D_, DEC);
       *    Serial1.println("read from sensor: ");
       *    Serial1.println(t_new, DEC);
       */
      return raw_ticks_;
    }
    
    return 1;
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
  
  /*!
   * TODO
   */
  void setUp();
  
  int IN1_; //! Motor input 1 pin, controls motor direction
  int IN2_; //! Motor input 2 pin, controls motor direction
  int SF_;  //! Motor status flag
  int EN_;  //! Driver board enable pin
  int FB_;  //! Analog input pin for current feedback from H-Bridge
  int D2_;  //! Disable 2 PWM pin to control output voltage, controls speed
};

void MotorControlPins::setUp() {
  pinMode(IN1_, OUTPUT); // Controls motor direction
  pinMode(IN2_, OUTPUT); // Controls motor direction
  pinMode(SF_,  INPUT);  // Motor Status flag from H-Bridge
  pinMode(FB_,  INPUT);  // Current feedback from H-Bridge
  pinMode(EN_,  OUTPUT); // Enables motor
  pinMode(D2_,  OUTPUT); // Controls speed
  
  digitalWrite(EN_,  HIGH); // Enables the driver board
  
  digitalWrite(IN1_, HIGH); // Default state
  digitalWrite(IN2_, LOW);
}

/*============================================================*/
/*!
 * \brief Class defining DC Motor. 
 * 
 * Class defining DC Motor.
 */
class Motor {
public:
  /*!
   * \brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] m_pins - motor control pins
   * \param[in] curr_s - current sensor
   * \param[in] e - encoder
   * \param[in] c - control of the motor
   */ 
  Motor(const MotorControlPins& m_pins, const CurrentSensor& curr_s, 
        const Encoder& e, const Control& c) :
    m_pins_(m_pins), curr_s_(curr_s), e_(e), c_(c) {};
  
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
  
  /*! 
   * \brief Adjust direction depending on the control varaible sign and set speed.
   * 
   * Adjust direction depending on the control varaible sign and set speed.
   * Send the sign-corrected input to the actuator and read the motor status flag.
   * \param[in] cv - control variable
   * \return true - status flag HIGH -> good
   *         false - status flag LOW -> bad
   */
  bool actuate(const float cv); 
  
  MotorControlPins m_pins_; //! Motor control pins
  CurrentSensor curr_s_;    //! Current sensor
  Encoder e_;               //! Encoder
  Control c_;               //! Control of the motor
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
  analogWrite(m_pins_.D2_, pwm_val); 
  return pwm_val;
}

bool Motor::actuate(const float cv) {
  // Set direction
  if (cv > 0.0) {
    forward();
  }
  else {
    reverse();
  }
  setPwm(static_cast<int>(abs(cv) + 0.5f)); // Set speed
  return digitalRead(m_pins_.SF_); // Return status flag 
}

/*=============== Variable motor definition ===============*/
Motor m1(MotorControlPins(M1_IN1, M1_IN2, M1_SF, EN, M1_FB, M1_D2),
         CurrentSensor(ALPHA_CURRENT, 0, 0, 0),
         Encoder(ENCODER_RESOLUTION, /*TODO*/SCALE_ENCODER, ALPHA_ENCODER,
                 SensorPins(E1_DO, E1_CLK, E1_CSn)), //TODO Change???
         Control(CURRENT_MODE, 
                 CurrentControl(R_MOTOR,
                                ControlStates(DESIRED_CURRENT/*0.0*/, DESIRED_CURRENT, 0.0, 0.0, 0.0, 0.0, T_JERK, 0, true), // TODO
                                PIDController(KP, KI, KD, -PWM_MAX, PWM_MAX, 0.0, 0.0)
                                )
                 )
         );

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
std_msgs::Float32 integral_msg;
ros::Publisher pub_integral("integral", &integral_msg);

std_msgs::Float32 enc_dp_msg;
ros::Publisher pub_enc_dp("enc_dp", &enc_dp_msg);
std_msgs::Float32 enc_p_msg;
ros::Publisher pub_enc_p("enc_p", &enc_p_msg);
std_msgs::Float32 enc_rev_msg;
ros::Publisher pub_enc_rev("enc_rev", &enc_rev_msg);
std_msgs::Float32 enc_raw_ticks_msg;
ros::Publisher pub_enc_raw_ticks("enc_raw_ticks", &enc_raw_ticks_msg);
std_msgs::Float32 enc_p_raw_msg;
ros::Publisher pub_enc_p_raw("enc_p_raw", &enc_p_raw_msg);
std_msgs::Float32 enc_k_msg;
ros::Publisher pub_enc_k("enc_k", &enc_k_msg);
std_msgs::Float32 enc_res_msg;
ros::Publisher pub_enc_res("enc_res", &enc_res_msg);
std_msgs::Float32 enc_scale_msg;
ros::Publisher pub_enc_scale("enc_scale", &enc_scale_msg);
std_msgs::Float32 enc_offset_msg;
ros::Publisher pub_enc_offset("enc_offset", &enc_offset_msg);
std_msgs::Float32 enc_alpha_msg;
ros::Publisher pub_enc_alpha("enc_alpha", &enc_alpha_msg);

int counter = 0;
void confirmCallback() {
  digitalWrite(LED_PIN, counter % 2 ? HIGH : LOW); // blink the led
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
  m1.c_.cc_.pid_.Kp_ = Kp_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kp("set_kp", &setKpCallback);

void setKiCallback( const std_msgs::Float32& Ki_msg ) {
  m1.c_.cc_.pid_.Ki_ = Ki_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_ki("set_ki", &setKiCallback);

void setKdCallback( const std_msgs::Float32& Kd_msg ) {
  m1.c_.cc_.pid_.Kd_ = Kd_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kd("set_kd", &setKdCallback);

void setIdCallback( const std_msgs::Float32& i_d_msg ) {
  //m1.c_.cc_.cs_.rf_ = i_d_msg.data; // TODO
  m1.c_.cc_.cs_.r_ = i_d_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_i_d("set_i_d", &setIdCallback);

void setOffsetCallback( const std_msgs::Float32& offset_msg ) {
  offset = offset_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_offset("set_offset", &setOffsetCallback);

void setDeadSpaceCallback( const std_msgs::Float32& dead_msg ) {
  m1.c_.cc_.pid_.dead_space_ = dead_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_dead("set_dead", &setDeadSpaceCallback);

void setAlphaCallback( const std_msgs::Float32& alpha_msg ) {
  m1.curr_s_.alpha_ = alpha_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_alpha("set_alpha", &setAlphaCallback);

void setUpRos(ros::NodeHandle & node_handler) {
  node_handler.advertise(pub_counter);
  node_handler.advertise(pub_current);
  node_handler.advertise(pub_filtered_current);
  node_handler.advertise(pub_error);
  node_handler.advertise(pub_u);
  node_handler.advertise(pub_i_d);
  node_handler.advertise(pub_integral);
  
  node_handler.advertise(pub_enc_alpha);
  node_handler.advertise(pub_enc_dp);
  node_handler.advertise(pub_enc_k);
  node_handler.advertise(pub_enc_offset);
  node_handler.advertise(pub_enc_p);
  node_handler.advertise(pub_enc_p_raw);
  node_handler.advertise(pub_enc_raw_ticks);
  node_handler.advertise(pub_enc_res);
  node_handler.advertise(pub_enc_rev);
  node_handler.advertise(pub_enc_scale);
  
  node_handler.subscribe(sub_set_vel);
  node_handler.subscribe(sub_set_kp);
  node_handler.subscribe(sub_set_ki);
  node_handler.subscribe(sub_set_kd);
  node_handler.subscribe(sub_set_i_d);
  node_handler.subscribe(sub_set_offset);
  node_handler.subscribe(sub_set_dead);
  node_handler.subscribe(sub_set_alpha);
  node_handler.subscribe(sub_change_dir);
}

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
  setUpRos(nh);
  
  /* Arduino */
  m1.m_pins_.setUp();
  m1.e_.s_pins_.setUp();
  m1.e_.setupEncoderSensorR(); // FIXME
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  setUpPwm();
  m1.setPwm(2000);
  
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
  current_msg.data = m1.curr_s_.senseCurrent(m1.m_pins_.FB_);
  pub_current.publish( &current_msg );
  filtered_current_msg.data = m1.curr_s_.filterCurrent();
  pub_filtered_current.publish( &filtered_current_msg );
  u_msg.data =  m1.c_.cc_.currentControl(m1.curr_s_.filtered_current_);
  pub_u.publish( &u_msg );
  
  m1.actuate(u_msg.data);
  //m1.setPwm(3000);
  
  error_msg.data = m1.c_.cc_.cs_.e_;
  pub_error.publish( &error_msg );
  i_d_msg.data = m1.c_.cc_.cs_.r_;
  pub_i_d.publish( &i_d_msg );
  integral_msg.data = m1.c_.cc_.pid_.I_;
  pub_integral.publish( &integral_msg );
  
  m1.e_.computeEncoderStatesR();
  
  enc_dp_msg.data = m1.e_.dp_;
  pub_enc_dp.publish( &enc_dp_msg );
  enc_p_msg.data = m1.e_.p_;
  pub_enc_p.publish( &enc_p_msg );
  enc_raw_ticks_msg.data = m1.e_.raw_ticks_;
  pub_enc_raw_ticks.publish( &enc_raw_ticks_msg );
  enc_rev_msg.data = m1.e_.rev_; 
  pub_enc_rev.publish( &enc_rev_msg );
  enc_k_msg.data = m1.e_.k_;
  pub_enc_k.publish( &enc_k_msg );
  enc_scale_msg.data = m1.e_.scale_;
  pub_enc_scale.publish( &enc_scale_msg );
  enc_alpha_msg.data = m1.e_.alpha_;
  pub_enc_alpha.publish( &enc_alpha_msg );
  enc_offset_msg.data = m1.e_.offset_;
  pub_enc_offset.publish( &enc_offset_msg );
  enc_p_raw_msg.data = m1.e_.p_raw_;
  pub_enc_p_raw.publish( &enc_p_raw_msg );
  enc_res_msg.data = m1.e_.res_;
  pub_enc_res.publish( &enc_res_msg );
  
  count_msg.data = counter;
  pub_counter.publish( &count_msg );
  
  nh.spinOnce();
  delay(DELAY);
}