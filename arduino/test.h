#ifndef TEST_H
#define TEST_H

extern unsigned long curr_millis;
extern unsigned long prev_serial_millis;
extern unsigned long prev_control_millis;
extern unsigned long start_millis;
extern const unsigned long serial_period;
extern const unsigned long control_period;
extern const unsigned long enc_reset_period;
extern int left_enc_count;
extern int right_enc_count;
extern int prev_left_enc_count;
extern int prev_right_enc_count;
extern int enc_iter;
extern float enc_cpr;
extern float Ts;
extern float no_load_radps;

//#include <Balboa32U4.h>

namespace Balboa
{
void TestEncoderClass(Balboa32U4Encoders &encoders)
{

    if (curr_millis - prev_serial_millis >= serial_period)
    {   
            
        left_enc_count = encoders.getCountsLeft();
        right_enc_count = encoders.getCountsRight();

        if (left_enc_count != prev_left_enc_count)
        {
            Serial.print("Left encoder count (prev) = ");
            Serial.println(prev_left_enc_count);
            Serial.print("Left encoder count = ");
            Serial.println(left_enc_count);
            prev_left_enc_count = left_enc_count;
        }

        if (right_enc_count != prev_right_enc_count)
        {
            Serial.print("Right encoder count (prev) = ");
            Serial.println(prev_right_enc_count);
            Serial.print("Right encoder count = ");
            Serial.println(right_enc_count);
            prev_right_enc_count = right_enc_count;
        }

        prev_serial_millis = curr_millis;
    }

//    if (curr_millis > enc_iter*enc_reset_period)
//    {
//      Serial.println("Resetting.");
//      Serial.println(enc_iter*enc_reset_period);
//      encoders.getCountsAndResetLeft();
//      encoders.getCountsAndResetRight();
//      enc_iter = enc_iter + 1;
//    }
}

void TestMotorOpenLoop(Balboa32U4Motors &motors, int max_command, char side)
{

    float command = 0;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 2000)
        {
            command = 0 * max_command;
//          command = 150;   
        }
        else if (curr_millis < 4000)
        {
            command = 0.5 * max_command;
//            command = 300;   
        }
        else if (curr_millis < 6000)
        {
            command = 1.0 * max_command;
//            command = 400;
        }
        else if (curr_millis < 8000)
        {
            command = 0.5 * max_command;
//            command = 300;
        }
        else if (curr_millis < 10000)
        {
            command = 0.0 * max_command;
//            command = 150;
        }
        else
          command = 0;

        if (side == 'L')
          motors.setLeftSpeed(command);
        else if (side == 'R')
          motors.setRightSpeed(command);
        else
        {
          motors.setLeftSpeed(command);
          motors.setRightSpeed(command);
        }
        prev_control_millis = curr_millis;
        Serial.println(command);
    }
}

void TestMotorPositionCommand(Balboa32U4Motors &motors, Balboa32U4Encoders &encoders, int enc_count, int speed, char side)
{

    float command = 0;
    float tol = 20;

    if (side == 'L')
    {
      while (abs(enc_count-encoders.getCountsLeft()) > tol)
      {
        if (enc_count > left_enc_count)
          motors.setLeftSpeed(speed);
        else
          motors.setLeftSpeed(-speed);
      }
      motors.setLeftSpeed(0);
    }else if (side == 'R')
    {
      while (abs(enc_count-encoders.getCountsRight()) > tol)
      {
        if (enc_count > right_enc_count)
          motors.setRightSpeed(speed);
        else
          motors.setRightSpeed(-speed);
      }
      motors.setRightSpeed(0);
    }
    
}

void TestMotorPositionControl(Balboa32U4Motors &motors, Balboa32U4Encoders &encoders, PidController &pid, float factor, char side)
{

    float setpoint = 0;
    float max_range = enc_cpr;
    float control = 0;
    float curr_pos = 0;
    float max_motor_command = 300;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000)
        {
            setpoint = 0 * factor;   
        }
        else if (curr_millis < 8000)
        {
            setpoint = 0.5 * factor;
        }
        else if (curr_millis < 12000)
        {
            setpoint = 1.0 * factor;
        }
        else if (curr_millis < 16000)
        {
            setpoint = 0.5 * factor;
        }
        else
        {
            setpoint = 0.0 * factor;
        }

        if (side == 'L')
        {
          curr_pos = encoders.getCountsLeft() / max_range;
          control = pid.ComputeControl(setpoint, curr_pos, 0);
          motors.setLeftSpeed(control*max_motor_command);
        }
        else if (side == 'R')
        {
          curr_pos = encoders.getCountsRight() / max_range;
          control = pid.ComputeControl(setpoint, curr_pos, 0);
          motors.setRightSpeed(control*max_motor_command);
        }

        prev_control_millis = curr_millis;

        if (curr_millis - prev_serial_millis > serial_period)
        {
          Serial.print(1.0);
          Serial.print(" ");
          Serial.print(-1.0);
          Serial.print(" ");
          Serial.print(setpoint);
          Serial.print(" ");
          Serial.print(curr_pos);
          Serial.print(" ");
          Serial.println(control);
          prev_serial_millis = curr_millis;
        }

    }

}

void TestMotorVelocityStep(Balboa32U4Motors &motors, Balboa32U4Encoders &encoders, PidController &pid, EncoderVelocity &enc_vel, float dir, float u_ff, char side)
{

    float setpoint = 0;
    float vel_radps = 0;
    float control = 0;
    float max_motor_command = 300;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 5000)
        {
            setpoint = 0;   
        }
        else if (curr_millis < 10000)

        {
            setpoint = 0.4 * dir;
        }
        else if (curr_millis < 15000)
        {
            setpoint = 0.8 * dir;
        }
        else if (curr_millis < 20000)
        {
            setpoint = 0.4 * dir;
        }
        else
        {
            setpoint = 0;
        }
        
        u_ff = u_ff*setpoint;
        
        if (side == 'L')
        {
          vel_radps = enc_vel.ComputeVelocity(encoders.getCountsLeft()); // velocity [rad/s]
          control = pid.ComputeControl(setpoint, vel_radps/no_load_radps, u_ff);
          motors.setLeftSpeed(control*max_motor_command);
        }
        else if (side == 'R')
        {
          vel_radps = enc_vel.ComputeVelocity(encoders.getCountsRight()); // velocity [rad/s]
          control = pid.ComputeControl(setpoint, vel_radps/no_load_radps, u_ff);
          motors.setRightSpeed(control*max_motor_command);
        }

        prev_control_millis = curr_millis;

        if (curr_millis - prev_serial_millis > serial_period)
        {
          Serial.print(1.0);
          Serial.print(" ");
          Serial.print(-1.0);
          Serial.print(" ");
          Serial.print(setpoint);
          Serial.print(" ");
          Serial.print(vel_radps/no_load_radps);
          Serial.print(" ");
          Serial.print(pid.u_p_);
          Serial.print(" ");
          Serial.print(pid.u_i_);
          Serial.print(" ");
          Serial.println(pid.u_d_);
//          Serial.print(" ");
//          Serial.println(control);
          prev_serial_millis = curr_millis;
        }
    }
}

void TestMotorVelocitySine(Balboa32U4Motors &motors, Balboa32U4Encoders &encoders, PidController &pid, EncoderVelocity &enc_vel, float dir, float u_ff, char side)
{

    float sin_freq = 0.5; // (Hz)
    float setpoint = dir * 0.5 * sin(2.0 * PI * sin_freq * curr_millis / 1000.0);
    float vel_radps = 0;
    float control = 0;
    float max_motor_command = 300;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000)
        {
            setpoint = 0;   
        }
        else if (curr_millis > 12000)
        {
            setpoint = 0;
        }

        if (side == 'L')
        {
          vel_radps = enc_vel.ComputeVelocity(encoders.getCountsLeft()); // velocity [rad/s]
          control = pid.ComputeControl(setpoint, vel_radps/no_load_radps, u_ff);
          motors.setLeftSpeed(control*max_motor_command);
        }
        else if (side == 'R')
        {
          vel_radps = enc_vel.ComputeVelocity(encoders.getCountsRight()); // velocity [rad/s]
          control = pid.ComputeControl(setpoint, vel_radps/no_load_radps, u_ff);
          motors.setRightSpeed(control*max_motor_command);
        }

        prev_control_millis = curr_millis;

        if (curr_millis - prev_serial_millis > serial_period)
        {
          Serial.print(1.0);
          Serial.print(" ");
          Serial.print(-1.0);
          Serial.print(" ");
          Serial.print(setpoint);
          Serial.print(" ");
          Serial.print(vel_radps/no_load_radps);
          Serial.print(" ");
          Serial.println(control);
          prev_serial_millis = curr_millis;
        }
    }
}

void TestMaxSpeed(Balboa32U4Motors &motors, Balboa32U4Encoders &encoders, EncoderVelocity &enc_vel, float motor_cmd, char side)
{

    float vel_radps = 0;
    while (curr_millis < 5000)
    {
      if (curr_millis - prev_control_millis >= control_period)
      {
        if (side == 'L')
        {
          motors.setLeftSpeed(motor_cmd);
          vel_radps = enc_vel.ComputeVelocity(encoders.getCountsLeft()); // velocity [rad/s]
        }
        else if (side == 'R')
        {
          motors.setRightSpeed(motor_cmd);
          vel_radps = enc_vel.ComputeVelocity(encoders.getCountsRight()); // velocity [rad/s]
        }
        prev_control_millis = curr_millis;
      }
      
      if (curr_millis - prev_serial_millis >= serial_period)
      {
        Serial.print("Velocity (rad/s) = ");
        Serial.println(vel_radps);
        prev_serial_millis = curr_millis;
      }

      curr_millis = millis() - start_millis;
    }
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
}

//void TestDiffSteerForward(DiffSteer &robot)
//{
//    if (curr_millis - prev_control_millis >= control_period)
//    {
//        if (curr_millis < 4000) { robot.Drive(0, 0); }
//        else if (curr_millis < 6000) { robot.Drive(robot.get_max_vel()/4, 0); } // Forward
//        else if (curr_millis < 8000) { robot.Drive(robot.get_max_vel()/2, 0); } // Forward
//        else if (curr_millis < 10000) { robot.Drive(robot.get_max_vel()/4, 0); } // Forward
//        else if (curr_millis < 11000) { robot.Drive(0, 0); } // Stop
//        else if (curr_millis < 13000) { robot.Drive(-robot.get_max_vel()/4, 0); } // Backward
//        else if (curr_millis < 15000) { robot.Drive(-robot.get_max_vel()/2, 0); } // Backward
//        else if (curr_millis < 17000) { robot.Drive(-robot.get_max_vel()/4, 0); } // Backward
//        else { robot.Drive(0, 0); }
//    }
//}
//
//void TestDiffSteerRotate(DiffSteer &robot)
//{
//    if (curr_millis - prev_control_millis >= control_period)
//    {
//        if (curr_millis < 4000) { robot.Drive(0, 0); }
//        else if (curr_millis < 6000) { robot.Drive(0, robot.get_max_ang_vel()/4); } // Forward
//        else if (curr_millis < 8000) { robot.Drive(0, robot.get_max_ang_vel()/2); } // Forward
//        else if (curr_millis < 10000) { robot.Drive(0, robot.get_max_ang_vel()/4); } // Forward
//        else if (curr_millis < 11000) { robot.Drive(0, 0); } // Stop
//        else if (curr_millis < 13000) { robot.Drive(0, -robot.get_max_ang_vel()/4); } // Backward
//        else if (curr_millis < 15000) { robot.Drive(0, -robot.get_max_ang_vel()/2); } // Backward
//        else if (curr_millis < 17000) { robot.Drive(0, -robot.get_max_ang_vel()/4); } // Backward
//        else { robot.Drive(0, 0); }
//    }
//}
//
//void TestDiffSteerCircle(DiffSteer &robot, float radius, float dir)
//{
//    float vel = robot.get_max_vel()/3;
//    float ang_vel = vel / radius * dir;
//    if (curr_millis - prev_control_millis >= control_period)
//    {
//        if (curr_millis < 4000) { robot.Drive(0, 0); }
//        else if (curr_millis < 16000) { robot.Drive(vel, ang_vel); }
//        else { robot.Drive(0, 0); }
//    }
//}

}


#endif
