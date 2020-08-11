#ifndef ROBOT_H
#define ROBOT_H

#include <Balboa32U4.h>

namespace Balboa
{
  struct RobotParams
  {
    int max_motor_command; // 0-300 for 0-100% PWM duty cycle
    float max_left_motor_radps; // max no load speed of left motor [rad/s]
    float max_right_motor_radps; // max no load speed of left motor [rad/s]
    float wheelbase; // [m]
    float wheel_radius; // [m]
    FiltDeriv left_radps;
    FiltDeriv right_radps;
    PidParams left_pid_params;
    PidParams right_pid_params;
    int left_pos_deadband;
    int left_neg_deadband;
    int right_pos_deadband;
    int right_neg_deadband;
  }

  struct DiffDriveWheelVel
  {
    // struct to hold left and right wheel velocity variables
    float left; // left wheel velocity [m/s]
    float right;// right wheel velocity [m/s]
  };

  class Robot
  {
    public:
      Robot(RobotParams p, Balboa32U4Motors motors, Balboa32U4Encoders encoders) :
        wheelbase_(p.wheelbase), wheel_radius(p.wheel_radius),
        motors_(motors), encoders_(encoders),
        left_radps_(p.left_radps), right_radps_(p.right_radps),
      {
        max_v_ = min(p.max_left_motor_radps, p.max_right_motor_radps) * wheel_radius_; // maximum forward velocity [m/s]
        max_w_ = max_v_ / (wheel_base_ / 2); // maximum angular velocity [rad/s]
        left_pid_ = PidController(p.left_pid_params);
        right_pid_ = PidController(p.right_pid_params);
      }

    void Drive(float v, float w)
    {
      // Process encoder measurements
      left_encoder_.ProcessMeasurement();
      right_encoder_.ProcessMeasurement();

      curr_pos = encoders.getCountsLeft()/enc_cpr*2*PI; // current position [rad]
      filt_vel = fd.ComputeDeriv(curr_pos); // normalized velocity
      control = pid.ComputeControl(setpoint, filt_vel, 0);
      motors.setLeftSpeed(control*max_motor_command);
  
      // Calculate the current wheel velocities
//      float curr_left_wheel_vel = left_encoder_.get_vel_rps() * wheel_radius_;
//      float curr_right_wheel_vel = right_encoder_.get_vel_rps() * wheel_radius_;
  
      // Compute the desired wheel velocities
      DiffDriveWheelVel des_wheel_vel = UniToDiff(v, w);
  
      // Compute controller commands based on desired and current wheel velocities
      float left_motor_command = left_pid_.ComputeCommand(des_wheel_vel.left, curr_left_wheel_vel);
      float right_motor_command = right_pid_.ComputeCommand(des_wheel_vel.right, curr_right_wheel_vel);
  
      // Send controller commands to each motor
      motors_.setLeftSpeed(ctrl_to_motor_cmd(left_ctrl_out));
      motors_.setRightSpeed(ctrl_to_motor_cmd(right_ctrl_out));
//      left_motor_.set_command(left_motor_command);
//      right_motor_.set_command(right_motor_command);
    }

    int ctrl_to_motor_cmd(float value)
    {
      // Maps controller output [-1.0, 1.0] to motor command [-max_command_, +max_command_]
      float factor = max(min(value, 1.0f), -1.0f);
      return (int)(max_command_ * factor);
    }

    DiffDriveWheelVel UniToDiff(float v, float w)
    {
      // This function ensures that w is respected as best as possible
      // by scaling v.
      // v: desired robot forward velocity [m/s]
      // w: desired robot angular velocity [rad/s]

      DiffDriveWheelVel dd_vel = {0.0, 0.0};

      if (!ensure_w_)
      {
          dd_vel.right = (v + w * wheelbase_ / 2.0);
          dd_vel.left = (v - w * wheelbase_ / 2.0);
          return dd_vel;
      }
      
      // 1. Limit v and w to be within the possible range
      float lim_w = max(min(w, max_w_), -max_w_);
      float lim_v = max(min(v, max_v_), -max_v_);

      // 2. Compute left and right wheel velocities required to achieve limited v and w
      float lim_rwheel_v = (lim_v + lim_w * wheelbase_ / 2.0);
      float lim_lwheel_v = (lim_v - lim_w * wheelbase_ / 2.0);

      // 3. Find max and min of the limited wheel velocities
      float max_lim_wheel_v = max(lim_rwheel_v, lim_lwheel_v);
      float min_lim_wheel_v = min(lim_rwheel_v, lim_lwheel_v);

      // 4. Shift limited wheel velocities if they exceed the maximum wheel velocity
      if (max_lim_wheel_v > max_v_)
      {
          dd_vel.right = lim_rwheel_v - (max_lim_wheel_v - max_v_);
          dd_vel.left = lim_lwheel_v - (max_lim_wheel_v - max_v_);
      }
      else if (min_lim_wheel_v < -max_v_) 
      {
          dd_vel.right = lim_rwheel_v - (min_lim_wheel_v + max_v_);
          dd_vel.left = lim_lwheel_v - (min_lim_wheel_v + max_v_);
      }
      else
      {
          dd_vel.right = lim_rwheel_v;
          dd_vel.left = lim_lwheel_v;
      }

      return dd_vel;
    }
      
    private:
      float wheelbase_;
      float wheel_radius_;
      float max_v_; // maximum forward velocity [m/s]
      float max_w_; // maximum angular velocity [rad/s]
      bool ensure_w_;
      Balboa32U4Motors motors_;
      Balboa32U4Encoders encoders_;
      signed long left_enc_count;
      signed long prev_left_enc_count;
      signed long right_enc_count;
      signed long prev_right_enc_count;
  }
}
