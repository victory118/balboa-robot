#include <Balboa32U4.h>
#include "pid_controller.h"
#include "encoder_velocity.h"
#include "test.h"

Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4ButtonA buttonA;

bool ran_once = false;
unsigned long prev_control_millis;
unsigned long prev_serial_millis;
unsigned long curr_millis;
unsigned long start_millis;
int left_enc_count;
int right_enc_count;
int prev_left_enc_count;
int prev_right_enc_count;
int enc_iter = 1;
float enc_cpr = 1323; // gearbox = 51.45:1, plastic gear = 45:21, enc. resolution = 12 cpr; 51.45*45/21*12 = 1323
// https://www.pololu.com/blog/662/how-to-make-a-balboa-robot-balance-part-1-selecting-mechanical-parts
// https://www.pololu.com/docs/0J70/all#3.4 
const unsigned long enc_reset_period = 3000;
const unsigned long control_period = 5; // control loop period [millis]
const unsigned long serial_period = 100; // period for printing to terminal for debugging [millis]
float Ts = control_period / 1000.0; // control sample time [sec]
float wheel_radius = 0.04; // [m] 80mm diameter
//https://www.pololu.com/product/1434
float no_load_radps = 5500/enc_cpr*2*PI; // [counts per second]

// Initialize PID controller parameter struct
//{float Kp, float Ki, float Kd, float Tf, float Ts, float pos_deadband, float neg_deadband}
Balboa::PidParams left_pid_p = {1.0, 0.0, 0.0, 0.0, Ts, 0.0, 0.0};
Balboa::PidController left_pid = Balboa::PidController(left_pid_p);
Balboa::PidParams right_pid_p = {1.0, 0.0, 0.0, 0.0, Ts, 0.0, 0.0};
Balboa::PidController right_pid = Balboa::PidController(right_pid_p);

// Initialize FiltDeriv object to calculate the filtered velocity
//EncoderVelocity(float Tf, float Ts, float counts_per_rev)
Balboa::EncoderVelocity left_vel(5*Ts, Ts, enc_cpr);
Balboa::EncoderVelocity right_vel(5*Ts, Ts, enc_cpr);

void setup() {

  // Wait for the user to press button A.
  buttonA.waitForButton();

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(1000);

  start_millis = millis();
  curr_millis = start_millis;
  prev_serial_millis = start_millis;
  prev_control_millis = start_millis;
  Serial.begin(9600);       // initialize Serial Communication
}

void loop() {
  
  curr_millis = millis() - start_millis;
  // Test maximum command to motors: 300
//  Balboa::TestMotorOpenLoop(motors, 300, 'L'); // Passed!
//  Balboa::TestMotorOpenLoop(motors, -300, 'L'); // Passed!
//  Balboa::TestMotorOpenLoop(motors, 300, 'R'); // Passed!
//  Balboa::TestMotorOpenLoop(motors, -300, 'R'); // Passed!

  // Verify encoder counts per revolution: 1323 seems correct
//  Balboa::TestEncoderClass(encoders);
//  Balboa::TestMotorPositionCommand(motors, encoders, 1323, 100, 'L'); // Passed
//  Balboa::TestMotorPositionCommand(motors, encoders, -1323, 100, 'L'); // Passed
//  Balboa::TestMotorPositionCommand(motors, encoders, 1323, 100, 'R'); // Passed
//  Balboa::TestMotorPositionCommand(motors, encoders, -1323, 100, 'R'); // Passed

  // Find deadband
//  motors.setLeftSpeed(40);
//  motors.setLeftSpeed(-43);
//  motors.setRightSpeed(30);
//  motors.setRightSpeed(-35);

  // Motor position control
    
//    left_pid.set_gains(2.0, 1.0, 0.1, 10.0 * Ts);
//    left_pid.set_deadband(40/300, -43/300);
//    Balboa::TestMotorPositionControl(motors, encoders, left_pid, 1, 'L'); // Passed!

//    right_pid.set_gains(2.0, 1.0, 0.5, 10.0 * Ts);
//    right_pid.set_deadband(30/300, -35/300);
//    Balboa::TestMotorPositionControl(motors, encoders, right_pid, 1, 'R'); // Passed!
  
  // Find maximum speed of each motor to add feedforward to PID controller
//    TestMaxSpeed(Balboa32U4Motors &motors, Balboa32U4Encoders &encoders, EncoderVelocity &enc_vel, float motor_cmd, char side)
//  Balboa::TestMaxSpeed(motors, encoders, left_vel, 300, 'L'); // 29 rad/s * 0.04 m = 1.2 m/s
//  Balboa::TestMaxSpeed(motors, encoders, left_vel, -300, 'L'); // 30 rad/s * 0.04 m = 1.16 m/s
//  Balboa::TestMaxSpeed(motors, encoders, right_vel, 300, 'R'); // 28.5 rad/s * 0.04 m = 1.14 m/s
//  Balboa::TestMaxSpeed(motors, encoders, right_vel, -300, 'R'); // 28 rad/s * 0.04 m = 1.12 m/s

   // Motor velocity control

   // Step velocity commands with PID only
    
//  left_pid.set_gains(5.0, 0.5, 0.2, 10.0*Ts);
//  Balboa::TestMotorVelocityStep(motors, encoders, left_pid, left_vel, 1, 0, 'L'); // Passed!
//  left_pid.set_gains(5.0, 0.5, 0.2, 10.0*Ts);
//  Balboa::TestMotorVelocityStep(motors, encoders, left_pid, left_vel, -1, 0, 'L'); // Passed!

//  right_pid.set_gains(5.0, 0.5, 0.2, 10.0*Ts);
//  Balboa::TestMotorVelocityStep(motors, encoders, right_pid, right_vel, 1, 0, 'R'); // Passed!
//  right_pid.set_gains(5.0, 0.5, 0.2, 10.0*Ts);
//  Balboa::TestMotorVelocityStep(motors, encoders, right_pid, right_vel, -1, 0, 'R'); // Passed!

  // Step velocity commands with PID and feedforward
//  left_pid.set_gains(2.0, 0.5, 0, 15.0*Ts);
//  Balboa::TestMotorVelocityStep(motors, encoders, left_pid, left_vel, 1, 1, 'L'); // Passed!
//  left_pid.set_gains(2.0, 0, 0.1, 15.0*Ts);
//  Balboa::TestMotorVelocityStep(motors, encoders, left_pid, left_vel, 1, 1, 'L'); // Passed!

  right_pid.set_gains(2.0, 0, 0.1, 15.0*Ts);
  Balboa::TestMotorVelocityStep(motors, encoders, right_pid, right_vel, -1, 1, 'R'); // Passed!
}
