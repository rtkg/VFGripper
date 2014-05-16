#include "pwm01.h"

void setup() 
{
    uint32_t  pwm_duty = 32767;
    uint32_t  pwm_freq1 = 2;  
    uint32_t  pwm_freq2 = 5000;

    // Set PWM Resolution
    pwm_set_resolution(16);  

    // Setup PWM Once (Up to two unique frequencies allowed
    //-----------------------------------------------------    
    pwm_setup( 6, pwm_freq1, 1);  // Pin 6 freq set to "pwm_freq1" on clock A
    pwm_setup( 7, pwm_freq2, 2);  // Pin 7 freq set to "pwm_freq2" on clock B
    pwm_setup( 8, pwm_freq2, 2);  // Pin 8 freq set to "pwm_freq2" on clock B
    pwm_setup( 9, pwm_freq2, 2);  // Pin 9 freq set to "pwm_freq2" on clock B
      
    // Write PWM Duty Cycle Anytime After PWM Setup
    //-----------------------------------------------------    
    pwm_write_duty( 6, pwm_duty );  // 50% duty cycle on Pin 6
    pwm_write_duty( 7, pwm_duty );  // 50% duty cycle on Pin 7
    pwm_write_duty( 8, pwm_duty );  // 50% duty cycle on Pin 8
    pwm_write_duty( 9, pwm_duty );  // 50% duty cycle on Pin 9

    delay(30000);  // 30sec Delay; PWM signal will still stream
        
    // Force PWM Stop On All Pins
    //-----------------------------    
    pwm_stop( 6 );
    pwm_stop( 7 );
    pwm_stop( 8 );
    pwm_stop( 9 );
}

void loop() 
{  
}