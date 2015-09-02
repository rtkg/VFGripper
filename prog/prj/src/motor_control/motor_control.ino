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
 *
 * rosserial_python node

 * How to run?
 * 1) roscore 
 * 2) rosrun rosserial_python serial_node.py /dev/ttyACM0
 * 3) rqt (or via console: rostopic pub ... std_msgs/... [--once])
 * 
 * Documentation is done for doxygen.
 * \note Set up must NOT be in constructors but in setup()!
 * \note constrain() sometimes doesn't work!
 *
 * TODO
 * - publisher/subscriber -> services
 * - constantly measure current and encoder!
 * - velocity control
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

const int E1_CSn = 26;  //! Chip select pin
const int E1_DO  = 27;  //! Sensor data output pin
const int E1_CLK = 28;  //! Clock pin

const int LED_PIN = 13; //! LED pin to visualize  

/*=============== Constants ===============*/
const int BIT_RESOLUTION = 12; //! 12 => [0; 4095], analogWrite(pin, PWM value)
const int PWM_MIN = 0;         //! PWM minimum value
const int PWM_MAX = 4095;      //! PWM maximum value

const int DELAY = 1; // Delay time [ms]

const float ALPHA_CURRENT = 0.99;   //! Value for filtering current
const float DESIRED_CURRENT = 0.015; //! Desired current (=> torque) [mA]
const float DESIRED_POSITION = 2000.0; //! Desired position [ticks]
const float DESIRED_VELOCITY = 100.0; //! Desired position [TODO]

const float ALPHA_ENCODER = 0.7; //! Value for filtering position
const float ENCODER_RESOLUTION = 4095; //! Encoder resolution: 12 bit, i.e., 0 - 4095 (=> 0.0879 deg)
const float SCALE_ENCODER = 14.4; //! TODO

const float T_JERK = 100.0;       //! Time for executing the loop [ms]

const float KP_CURR = 1500000.0; //! PID value
const float KI_CURR = 1400.0;   //! PID value
const float KD_CURR = 100000.0;   //! PID value

const float KP_POS = 10.0; //! PID value
const float KI_POS = 0.05;   //! PID value
const float KD_POS = 100.0;   //! PID value

// TODO
const float KP_VEL = 10.0; //! PID value
const float KI_VEL = 0.0;   //! PID value
const float KD_VEL = 0.0;   //! PID value

const float DEAD_SPACE_CURR = (1-ALPHA_CURRENT)*DESIRED_CURRENT; //! Deadband around setpoint

const int   V_MAX = 24;   //! Maximum value for our maxon motor [V] 
const float V_MIN = 4.4;  //! Minimum value for our maxon motor to overcome inner resistance [V]
const int OFFSET  = static_cast<int>(mapFloat(V_MIN, 0.0, V_MAX, PWM_MIN, PWM_MAX)); //! V_MIN converted to PWM value
const float R_MOTOR = 7.25; //! Terminal resistance of the motor

/*=============== Time variables ===============*/
const float DT = 1e-3;       //! Time step [s]
const int dT = 1000;         //! Sample time [us] 
const int dT_serial = 75000; //! Sample time for the serial connection [us]
int t_old = 0;        //! Timer value for calculating time steps
int t_new = 0;        //! Timer value for calculating time steps
int t_old_serial = 0; //! Timer value for calculating time steps

/*=============== Global variables ===============*/
int pwm_duty_cycle = 0; //! PWM duty cycle [%]
int offset_motor = 200;//OFFSET; //! Offset for PWM value to avoid the area where motor does not move at all TODO

/*=============== Functions ===============*/
/*!
 * \brief Sets up PWM write and read resolution. 
 * 
 * Sets up PWM write and read resolution.
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
 * \brief Constrain value between range.
 * 
 * Constrain value between range.
 * Use this one instead of constrain because constrain() doesn't always work! (why?)
 * \param[in, out] val - value to clamp into range [min; max]
 * \param[in] min - lower bound
 * \param[in] max - upper bound
 */
void clamp(int & val, int min, int max) {
  if (val < min) {
    val = min;
  }
  else if (val > max) {
    val = max;
  }
}

/*!
 * \brief Constrain value between range.
 * 
 * Constrain value between range.
 * Use this one instead of constrain because constrain() doesn't always work! (why?)
 * \param[in, out] val - value to clamp into range [min; max]
 * \param[in] min - lower bound
 * \param[in] max - upper bound
 */
void clamp(float & val, float min, float max) {
  if (val < min) {
    val = min;
  }
  else if (val > max) {
    val = max;
  }
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
   * \param[in] rf - final setpoint value
   * \param[in] ri - initial setpoint value
   * \param[in] ti - time when we initialized motion [s]
   * \param[in] T - time for executing the loop
   * \param[in] active - flag indicating whether the corresponding controller is active or not
   */
  ControlStates(float ri, float rf, float ti, float T, bool active);
  
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
  
  float ri_; //! Initial setpoint value
  float rf_; //! Final setpoint value
  float r_;  //! Setpoint (reference) value at current iteration
  float e_;  //! Error
  float de_; //! Error derivative
  float ti_; //! Time when we initialized motion [s]
  float T_;  //! Time for executing the loop
  int u_; //! Computed control
  bool active_; //! Flag indicating whether the corresponding controller is active or not
};

ControlStates::ControlStates(float ri, float rf, float ti, float T, bool active) : 
  ri_(ri),
  rf_(rf), 
  r_(rf_),
  e_(0.0),
  de_(0.0),
  ti_(ti), 
  T_(T),  
  u_(0.0),
  active_(active)
  {};

float ControlStates::minimumJerk(float t) {
  if (t > ti_ + T_) {
    t = T_ + ti_;       // Make sure the ouput stays at qf after T has passed
  }
  // Return smoother value
  return ( T_ > 0 ? 
                    ( ri_ + (rf_-ri_)*(10*pow((t-ti_)/T_, 3) - 15*pow((t-ti_)/T_, 4) + 6*pow((t-ti_)/T_, 5)) ) :
                      rf_);
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
   * \param[in] dead_space - dead area around setpoint 
   */
  PIDController(float Kp, float Ki, float Kd, 
                float u_min, float u_max, float dead_space=-1);
  
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

PIDController::PIDController(float Kp, float Ki, float Kd, 
              float u_min, float u_max, float dead_space) : 
              Kp_(Kp), 
              Kd_(Kd), 
              Ki_(Ki), 
              u_min_(u_min), 
              u_max_(u_max), 
              I_(0.0), 
              dead_space_(dead_space) 
              {};
              
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
    clamp(u, u_min_, u_max_);         // Clamp the CV
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
   */ 
  CurrentSensor(float alpha);
  
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

CurrentSensor::CurrentSensor(float alpha) :
  alpha_(alpha), 
  sensed_value_(0), 
  current_(0.0), 
  filtered_current_(0.0) 
  {};

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
  float currentControl(float current);
  
  float r_motor_;     //! Terminal resistance of the motor [ohm] (to calculate the feedforward term)
  ControlStates cs_;  //! Control states of the current control
  PIDController pid_; //! PID controller for closed loop
};

float CurrentControl::currentControl(float current) {
   cs_.r_ = cs_.minimumJerk(static_cast<float>(millis())); // Set a new desired current [A] (filtered)
  
  float dir =  (cs_.r_ >= 0 ? 1 : -1);  // If current reference is <0 (due to jerk) 
  current *= dir;                       // We keep or change the sign
  
  cs_.de_ = (cs_.e_ - (cs_.r_-current));  // Derivative of the current error (already a bit filtered)
  //cs_.de_ = ALPHA_ERROR*cs_.de_ + (1-ALPHA_ERROR)*(cs_.e_ - (cs_.r_-current)); // TODO Or filter even more?
  cs_.e_ =  cs_.r_ - current;             // Current error
  // TODO Maybe add set point weighting?
  
  cs_.u_ = pid_.pid(cs_.e_, cs_.de_);     // Set a new control value
  float feedforward = r_motor_ * current; // V = L*di/dt + RI + E  
                                          // We hold the motor => w=0 => E=0; di/dt == 0
                                          // => V = RI
  mapFloat(feedforward, 0, 1.0*V_MAX, 1.0*PWM_MIN, 1.0*PWM_MAX); // Map from Voltage to PWM range TODO correct?
  // TODO do we need 1.0*???
  cs_.u_ += feedforward;  // Compute control with feedforward term // TODO shouldn't be +/-???
  
  clamp(cs_.u_, static_cast<int>(pid_.u_min_), static_cast<int>(pid_.u_max_)); // Clamp
  // We don't want it to rotate in the other direction
  if (cs_.u_ > 0 && dir < 0) {
    cs_.u_ = 0; // So just don't move
  }
  else if (cs_.u_ < 0 && dir > 0) {
    cs_.u_ = 0;
  }
  
  return cs_.u_;    // Return CV
}

/*============================================================*/
/*!
 * \brief Class defining position control. 
 * 
 * Class defining position control.
 */
class PositionControl {
public:
  /*!
   * \brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] cs - control states of the position control 
   * \param[in] pid - PID controller
   */ 
  PositionControl(const ControlStates& cs, const PIDController& pid) :  
  cs_(cs), pid_(pid) {};
  
  /*!
   * \brief Position control feedback.
   * 
   * Position control feedback.
   * \param[in] position - current position
   */  
  float positionControl(const float position);
  
  ControlStates cs_;  //! Control states of the position control
  PIDController pid_; //! PID controller for closed loop
};

float PositionControl::positionControl(const float position) {
  cs_.r_ = cs_.minimumJerk(static_cast<float>(millis())); // Set a new desired position [TODO] (filtered)
  
  cs_.de_ = (cs_.e_ - (cs_.r_-position));  // Derivative of the current error (already a bit filtered)
  //cs_.de_ = ALPHA_ERROR*cs_.de_ + (1-ALPHA_ERROR)*(cs_.e_ - (cs_.r_-position)); // TODO Or filter even more?
  cs_.e_ =  cs_.r_ - position;             // Current error
  // TODO Maybe add set point weighting?
  
  cs_.u_ = pid_.pid(cs_.e_, cs_.de_);     // Set a new control value
  if (cs_.u_ >= 0) { // Add offset to overcome the motor inner resistance TODO 
    cs_.u_ += offset_motor;
  }
  else {
    cs_.u_ -= offset_motor;
  }
  clamp(cs_.u_, static_cast<int>(pid_.u_min_), static_cast<int>(pid_.u_max_));
 
  return cs_.u_;    // Return CV
}

/*============================================================*/
/*!
 * \brief Class defining velocity control. 
 * 
 * Class defining velocity control.
 */
class VelocityControl {
public:
  /*!
   * \brief Parametrized constructor. 
   *
   * Parametrized constructor.
   * \param[in] cs - control states of the velocity control 
   * \param[in] pid - PID controller
   */ 
  VelocityControl(const ControlStates& cs, const PIDController& pid) :  
  cs_(cs), pid_(pid) {};
  
  /*!
   * \brief Velocity control feedback.
   * 
   * Velocity control feedback.
   * \param[in] velocity - current velocity
   */  
  float velocityControl(const float velocity);
  
  ControlStates cs_;  //! Control states of the velocity control
  PIDController pid_; //! PID controller for closed loop
};

float VelocityControl::velocityControl(const float velocity) {
  cs_.r_ = cs_.minimumJerk(static_cast<float>(millis())); // Set a new desired velocity [TODO] (filtered)
  
  cs_.de_ = (cs_.e_ - (cs_.r_-velocity));  // Derivative of the current error (already a bit filtered)
  //cs_.de_ = ALPHA_ERROR*cs_.de_ + (1-ALPHA_ERROR)*(cs_.e_ - (cs_.r_-velocity)); // TODO Or filter even more?
  cs_.e_ =  cs_.r_ - velocity;             // Current error
  // TODO Maybe add set point weighting?
  
  cs_.u_ = pid_.pid(cs_.e_, cs_.de_);     // Set a new control value
  clamp(cs_.u_, static_cast<int>(pid_.u_min_), static_cast<int>(pid_.u_max_)); // Clamp
  
  return cs_.u_;    // Return CV
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
   * \brief Enum defining control modes.
   * 
   * Enum defining control modes.
   */
  enum ControlMode {
    POSITION_MODE = 1, //! Position controller
    VELOCITY_MODE = 2, //! Velocity controller 
    CURRENT_MODE = 0,  //! Current controller
    NO_MODE = 3        //! Without controller
  };
  
  /*!
   * \brief Parametrized constructor.
   * 
   * Parametrized constructor.
   * \param[in] mode - position/current/velocity controller
   * \param[in] cc - current control
   * \param[in] pc - position control
   * \param[in] vc - velocity control
   */
  Control(ControlMode mode, const CurrentControl& cc, const PositionControl& pc, const VelocityControl& vc) :
  mode_(mode), cc_(cc), pc_(pc), vc_(vc) {};
  
  ControlMode mode_;    //! Position/current/velocity controller at the moment 
  CurrentControl cc_;   //! Current control
  PositionControl pc_;  //! Position control
  VelocityControl vc_;  //! Velocity control
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
   * \brief Set up directions and default states of the pins.
   * 
   * Set up directions and default states of the pins.
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
  
  /*!
   * \brief Checks even parity of the bits.
   * 
   * Checks even parity of the bits.
   * \param[in] d1 - first byte of data
   * \param[in] d2 - second byte of data
   * \param[in] d3 - third byte of data
   * \return true - when there is even parity,
   *         false - otherwise
   */
  bool evenParity(const byte d1, const byte d2, const byte d3);
  
  /*!
   * \brief Check if readings are correct.
   * 
   * Check if readings are correct.
   * 6 bits contain system information about the validity of data such as 
   * OCF, COF, LIN, Parity and Magnetic Field status.
   * \param[in] d1 - first byte of data
   * \param[in] d2 - second byte of data
   * \param[in] d3 - third byte of data
   * \return status flag < 0 when there's something wrong,
   *         0 otherwise 
   */
  int checkReading(const byte d1, const byte d2, const byte d3);
  
  /*!
   * \brief Check encoder raw ticks position. 
   * 
   * Check encoder raw ticks position.
   * \return raw_ticks_ [0; 4095]
   */
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
   * \brief Converts ticks to angle in radians multiplied by scaling factor. 
   * 
   * Converts ticks to angle in radians multiplied by scaling factor.
   * \post p_raw, p_, dp_ have new values
   */ 
  void convertSensorReading();
  
  /*!
   * \brief Converts the encoder readings and computes derivatives.
   * 
   * Converts the encoder readings and computes derivatives.
   * \return Raw ticks > 0 or a status flag -5
   */
  int computeEncoder(); 
  
  enum Flags {
    OCF = -1, //! Offset Compensation Finished
    COF = -2, //! Cordic Overflow
    LIN = -3, //! Linearity Alarm
    MAG = -4, //! Magnetic field is <45mT or >75mT. It is still possible to operate the AS5045 in this range, but not recommended
    MagINC = -5, //! Distance decrease; push-function. This state is dynamic and only active while the magnet is moving towards the chip.
    MagDEC = -6, //! Distance increase; pull-function. This state is dynamic and only active while the magnet is moving away from the chip.
    PARITY = -7  //! Even checksum of bits 1:15
  };
  
  int raw_ticks_;  //! Raw encoder ticks [0; 4095]
  float p_raw_;    //! Converted value (scale_)
  float p_;        //! Filtered converted value 
  float dp_;       //! Time-derivative of the converted value
  int k_;          //! Rollover counter (number of revolutions)
  int res_;        //! Resolution (i.e., number of ticks per revolution)
  float scale_;    //! Scale factor used for conversion from ticks to value - can be used to lump transmission ratio, radius ...
  int offset_;     //! Used for zeroing
  float alpha_;    //! First order filter parameter, 0<=alpha<=1
  SensorPins s_pins_; //! Sensor pins
};

Encoder::Encoder(int res, float scale, float alpha, SensorPins s_pins) :   
  raw_ticks_(0.0),
  p_raw_(0.0),
  p_(0.0),
  dp_(0.0),
  k_(0),
  res_(res), 
  scale_(scale),
  offset_(0.0),
  alpha_(alpha), 
  s_pins_(s_pins)
  {};

void Encoder::setUp() {
  raw_ticks_ = getRawTicks();
  offset_ = raw_ticks_;
  readEncoder();
  convertSensorReading();
}

bool Encoder::evenParity(const byte d1, const byte d2, const byte d3) {
  int sum = 0;
  int i = 0;
  // Count 1 bits
  for (i=0; i<8; i++) {
    if ((1 << i) & d1) sum++;
  }
  for (i=0; i<8; i++) {
    if ((1 << i) & d2) sum++;
  }
  for (i=7; i>=1; i--) {
    if ((1 << i) & d3) sum++;
  }
  // Check parity bit according to the sum bits 1:17
  if ( (B00000001 & d3) ) {
    if (sum%2 == 1) return true; // Odd sum => parity bit = 1
  }
  else {
    if (sum%2 == 0) return true; // Even sum => parity bit = 0
  }
  return false; // Wrong parity bit
}

int Encoder::checkReading(const byte d1, const byte d2, const byte d3) {
  if (!((d2 & B00001000) == B00001000)) { // Check the offset compensation flag: 1 == started up
    return OCF; 
  }
  if (d2 & B00000100) { // Check the cordic overflow flag: 1 = error
    return COF;
  }
  if (d2 & B00000010) { // Check the linearity alarm: 1 = error
    return LIN;
  }
  if ((d2 & B00000001) && (d3 & B10000000)) { // 16bit MagINC, 17bit MagDEC
    return MAG;                               // Check the magnet range: 11 = error
  }
  else { 
    if (d2 & B00000001) { // 16bit MagINC
      return MagINC;                            
    }
    else if (d3 & B10000000) { // 17bit MagDEC
      return MagDEC;                            
    }
  }
  if (!evenParity(d1, d2, d3)) {
    return PARITY;
  }
  return 0;
}

int Encoder::getRawTicks() {
  unsigned int reading = 0;
  
  // If CSn changes to logic low, Data Out (DO) will change from high impedance (tri-state) to logic high 
  // and the read-out will be initiated.
  digitalWrite(s_pins_.CSn_, LOW);
  //Propagation delay 384μs (slow mode) 96μs (fast mode) 
  // System propagation delay absolute output : delay of ADC, DSP and absolute interface 
  delayMicroseconds(384); // FIXME
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
  // Return value or flag
  int a = -1;
  return ( a = checkReading(d1, d2, d3) >= 0 ? static_cast<int>(reading) : a ); 
}

void Encoder::readEncoder() {
  int reading = getRawTicks();
  if (reading >= 0) { // Everything's OK
    if (reading - raw_ticks_ < -res_ / 2) { 
      k_++; // Delta is smaller than minus half the resolution -> positive rollover
    }
    if (reading - raw_ticks_ > res_ / 2) {
      k_--; // Delta is larger than half the resolution -> negative rollover
    }
  }
  raw_ticks_ = reading; // Update ticks counter with value or flag
}

void Encoder::convertSensorReading() {
  // angle = 2*pi*[(reading-offset)/res+k]
  p_raw_ = 2.0*M_PI * ((static_cast<float>(raw_ticks_) - static_cast<float>(offset_)) 
           / static_cast<float>(res_) 
           + static_cast<float>(k_)) 
           * scale_; 
  float p_tmp = alpha_*p_ + (1-alpha_)*p_raw_; // First order low-pass filter (alpha = 1/(1+2*pi*w*Td), w=cutoff frequency, Td=sampling time)
  float dp_raw = (p_tmp - p_); 
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
   * \brief Set up directions and default states of the pins.
   * 
   * Set up directions and default states of the pins.
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
   * \brief Check control method and do it.
   * 
   * Check control method and do it.
   * \return control value
   */ 
  int control();
  
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

int Motor::control() {
  if (c_.mode_ == Control::CURRENT_MODE) {
    curr_s_.senseCurrent(m_pins_.FB_);
    curr_s_.filterCurrent();
    return c_.cc_.currentControl(curr_s_.filtered_current_);
  }
  else if (c_.mode_ == Control::POSITION_MODE) {
    e_.computeEncoder();
    return c_.pc_.positionControl(e_.p_);
  }
  else if (c_.mode_ == Control::VELOCITY_MODE) {
    e_.computeEncoder();
    return c_.vc_.velocityControl(e_.dp_);
  }
  return 0;
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
         CurrentSensor(ALPHA_CURRENT),
         Encoder(ENCODER_RESOLUTION, SCALE_ENCODER, ALPHA_ENCODER,
                 SensorPins(E1_DO, E1_CLK, E1_CSn)), 
         Control(Control::POSITION_MODE, 
                 CurrentControl(R_MOTOR,
                                ControlStates(0.0, DESIRED_CURRENT, 0.0, T_JERK, true), 
                                PIDController(KP_CURR, KI_CURR, KD_CURR, -PWM_MAX, PWM_MAX, 0.0)
                                ),
                 PositionControl(ControlStates(0.0, DESIRED_POSITION, 0.0, T_JERK, false), // FIXME true/false
                                 PIDController(KP_POS, KI_POS, KD_POS, -PWM_MAX, PWM_MAX, 0.0)
                                ),
                 VelocityControl(ControlStates(0.0, DESIRED_VELOCITY, 0.0, T_JERK, false),
                                 PIDController(KP_VEL, KI_VEL, KD_VEL, -PWM_MAX, PWM_MAX, 0.0)
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
std_msgs::Float32 ref_msg;
ros::Publisher pub_ref("ref", &ref_msg);
std_msgs::Float32 integral_msg;
ros::Publisher pub_integral("integral", &integral_msg);

std_msgs::Float32 enc_dp_msg;
ros::Publisher pub_enc_dp("enc_dp", &enc_dp_msg);
std_msgs::Float32 enc_p_msg;
ros::Publisher pub_enc_p("enc_p", &enc_p_msg);
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

void setKpCurrCallback( const std_msgs::Float32& Kp_msg ) {
  m1.c_.cc_.pid_.Kp_ = Kp_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kp_curr("set_kp_curr", &setKpCurrCallback);

void setKiCurrCallback( const std_msgs::Float32& Ki_msg ) {
  m1.c_.cc_.pid_.Ki_ = Ki_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_ki_curr("set_ki_curr", &setKiCurrCallback);

void setKdCurrCallback( const std_msgs::Float32& Kd_msg ) {
  m1.c_.cc_.pid_.Kd_ = Kd_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kd_curr("set_kd_curr", &setKdCurrCallback);

void setKpPosCallback( const std_msgs::Float32& Kp_msg ) {
  m1.c_.pc_.pid_.Kp_ = Kp_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kp_pos("set_kp_pos", &setKpPosCallback);

void setKiPosCallback( const std_msgs::Float32& Ki_msg ) {
  m1.c_.pc_.pid_.Ki_ = Ki_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_ki_pos("set_ki_pos", &setKiPosCallback);

void setKdPosCallback( const std_msgs::Float32& Kd_msg ) {
  m1.c_.pc_.pid_.Kd_ = Kd_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kd_pos("set_kd_pos", &setKdPosCallback);

void setKpVelCallback( const std_msgs::Float32& Kp_msg ) {
  m1.c_.vc_.pid_.Kp_ = Kp_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kp_vel("set_kp_vel", &setKpVelCallback);

void setKiVelCallback( const std_msgs::Float32& Ki_msg ) {
  m1.c_.vc_.pid_.Ki_ = Ki_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_ki_vel("set_ki_vel", &setKiVelCallback);

void setKdVelCallback( const std_msgs::Float32& Kd_msg ) {
  m1.c_.vc_.pid_.Kd_ = Kd_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_kd_vel("set_kd_vel", &setKdVelCallback);

void setRefCallback( const std_msgs::Float32& ref_msg ) {
  switch (m1.c_.mode_) {
    case Control::CURRENT_MODE:
      m1.c_.cc_.cs_.ri_ = m1.c_.cc_.cs_.r_; 
      m1.c_.cc_.cs_.rf_ = ref_msg.data;
      m1.c_.cc_.cs_.ti_ = static_cast<float>(millis());
      break;
    case Control::POSITION_MODE:
      m1.c_.pc_.cs_.ri_ = m1.c_.pc_.cs_.r_; 
      m1.c_.pc_.cs_.rf_ = ref_msg.data;
      m1.c_.pc_.cs_.ti_ = static_cast<float>(millis());
    case Control::VELOCITY_MODE:
      m1.c_.vc_.cs_.ri_ = m1.c_.vc_.cs_.r_; 
      m1.c_.vc_.cs_.rf_ = ref_msg.data;
      m1.c_.vc_.cs_.ti_ = static_cast<float>(millis());
    default:
      break;
  }
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_ref("set_ref", &setRefCallback);

void setOffsetMotorCallback( const std_msgs::Float32& offset_msg ) {
  offset_motor = offset_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_offset("set_offset", &setOffsetMotorCallback);

void setDeadSpaceCallback( const std_msgs::Float32& dead_msg ) {
  m1.c_.cc_.pid_.dead_space_ = dead_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_dead("set_dead", &setDeadSpaceCallback);

void setAlphaCurrCallback( const std_msgs::Float32& alpha_msg ) {
  m1.curr_s_.alpha_ = alpha_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_alpha_curr("set_alpha_curr", &setAlphaCurrCallback);

void setAlphaEncCallback( const std_msgs::Float32& alpha_msg ) {
  m1.e_.alpha_ = alpha_msg.data;
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_alpha_enc("set_alpha_enc", &setAlphaEncCallback);

// TODO active = true/false
void setModeCallback( const std_msgs::Float32& mode_msg ) {
  switch (static_cast<int>(mode_msg.data)) {
    case 0 : 
      m1.c_.mode_ = Control::CURRENT_MODE;
      break;
    case 1 : 
      m1.c_.mode_ = Control::POSITION_MODE;
      break;
    case 2 : 
      m1.c_.mode_ = Control::VELOCITY_MODE;
      break;
    default:
      m1.c_.mode_ = Control::NO_MODE;
  }
  confirmCallback();
}
ros::Subscriber<std_msgs::Float32> sub_set_mode("set_mode", &setModeCallback);

void setUpRos(ros::NodeHandle & node_handler) {
  node_handler.advertise(pub_counter);
  node_handler.advertise(pub_current);
  node_handler.advertise(pub_filtered_current);
  node_handler.advertise(pub_error);
  node_handler.advertise(pub_u);
  node_handler.advertise(pub_ref);
  node_handler.advertise(pub_integral);
  
  node_handler.advertise(pub_enc_alpha);
  node_handler.advertise(pub_enc_dp);
  node_handler.advertise(pub_enc_k);
  node_handler.advertise(pub_enc_offset);
  node_handler.advertise(pub_enc_p);
  node_handler.advertise(pub_enc_p_raw);
  node_handler.advertise(pub_enc_raw_ticks);
  node_handler.advertise(pub_enc_res);
  node_handler.advertise(pub_enc_scale);
  
  node_handler.subscribe(sub_set_vel);
  
  node_handler.subscribe(sub_set_kp_curr);
  node_handler.subscribe(sub_set_ki_curr);
  node_handler.subscribe(sub_set_kd_curr);
  node_handler.subscribe(sub_set_kp_pos);
  node_handler.subscribe(sub_set_ki_pos);
  node_handler.subscribe(sub_set_kd_pos);
  node_handler.subscribe(sub_set_kp_vel);
  node_handler.subscribe(sub_set_ki_vel);
  node_handler.subscribe(sub_set_kd_vel);
  
  node_handler.subscribe(sub_set_ref);
  node_handler.subscribe(sub_set_offset);
  node_handler.subscribe(sub_set_dead);
  node_handler.subscribe(sub_set_alpha_curr);
  node_handler.subscribe(sub_set_alpha_enc);
  node_handler.subscribe(sub_change_dir);
  node_handler.subscribe(sub_set_mode);
}

/*!
 * \brief Publishes interestind data from motor.
 * 
 * Publishes interestind data from motor.
 */
void publishEverything() {
  pub_u.publish( &u_msg );
  current_msg.data = m1.curr_s_.current_;
  pub_current.publish( &current_msg );
  filtered_current_msg.data = m1.curr_s_.filtered_current_;
  pub_filtered_current.publish( &filtered_current_msg );
  
  if (m1.c_.mode_ == Control::CURRENT_MODE) {
    error_msg.data = m1.c_.cc_.cs_.e_;
    ref_msg.data = m1.c_.cc_.cs_.r_;
    integral_msg.data = m1.c_.cc_.pid_.I_;
  }
  else if (m1.c_.mode_ == Control::POSITION_MODE) {
    error_msg.data = m1.c_.pc_.cs_.e_;
    ref_msg.data = m1.c_.pc_.cs_.r_;
    integral_msg.data = m1.c_.pc_.pid_.I_;
  }
  else if (m1.c_.mode_ == Control::VELOCITY_MODE) {
    error_msg.data = m1.c_.vc_.cs_.e_;
    ref_msg.data = m1.c_.vc_.cs_.r_;
    integral_msg.data = m1.c_.vc_.pid_.I_;
  }
  
  pub_error.publish( &error_msg );
  pub_ref.publish( &ref_msg );
  pub_integral.publish( &integral_msg );
  
  enc_dp_msg.data = m1.e_.dp_;
  pub_enc_dp.publish( &enc_dp_msg );
  enc_p_msg.data = m1.e_.p_;
  pub_enc_p.publish( &enc_p_msg );
  enc_raw_ticks_msg.data = m1.e_.raw_ticks_;
  pub_enc_raw_ticks.publish( &enc_raw_ticks_msg );
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
  m1.e_.setUp(); 
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  setUpPwm();
}

/*============================================================*/
/*! 
 * \brief Runs repeatedly code in the loop. 
 * 
 * Runs repeatedly code in the loop. 
 */
void loop() 
{
  // Publishing takes too much time so spin and check if we should publish
  t_new = micros();
  nh.spinOnce();
  if (abs(t_new - t_old_serial) > dT_serial) {
    publishEverything();
    t_old_serial = t_new;
  }
  
  // Control motor all the time
  u_msg.data = m1.control();
  m1.actuate(u_msg.data);
}