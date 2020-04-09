/**
 * @file balboa_motor.h
 * @brief Motor device driver for the Balboa 32u4 dual motor driver
 * @author Victor Yu
 */

#ifndef BALBOA_MOTOR_H
#define BALBOA_MOTOR_H

#include <Balboa32U4.h>

namespace Balboa
{

    struct MotorParams
    {
        Balboa32U4Motors *motor_ptr;
        int max_command; //  0-300
        float no_load_rps; // max speed with no load [rad/s]
    };
    
    class Motor
    {
    public:
        /*
         * @brief Class constructor
         * @param dir_pin_ the direction pin 
         * @param pwm_pin the PWM pin
         */
         Motor(MotorParams p)
            : max_command_(p.max_command),
              no_load_rps_(p.no_load_rps),
              command_(0)
         {
         }

        int convert_command(float value)
        {
            // input value is limited to range [-1.0, 1.0] and then mapped to range [-max_command_, +max_command_]
            float factor = max(min(value, 1.0f), -1.0f);
            return (int)(max_command_ * factor);
        }

        void set_left_speed(float value)
        {
          int command = 
        }

        unsigned int get_command() { return command_; }
        int get_max_command() { return max_command_; }
        float get_no_load_rps() { return no_load_rps_; }
         
    private:
        const int dir_pin_;
        const int pwm_pin_;
        const int max_command_;
        const float no_load_rps_;

        unsigned int command_;
    };
};

#endif
