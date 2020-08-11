#ifndef FILT_DERIV_H
#define FILT_DERIV_H

namespace Balboa
{
    
    class EncoderVelocity
    {
    public:
        EncoderVelocity(float Tf, float Ts, float counts_per_rev)
            : Tf_(Tf), Ts_(Ts), counts_per_rev_(counts_per_rev),
              prev_cnt_(0), vel_radps_filt_(0), prev_vel_radps_filt_(0), vel_radps_(0)
        {
          alpha_ = Ts_ / (Ts_ + Tf_); // derivate low pass filter coefficient
          // 0: one time step delay, 0-1: low-pass, 1: no filtering
          // alpha = Ts / (Ts + Tf) - typically around 0.1
          // alpha = 0: one time step delay, 0-1: low-pass, 1: no filtering
          // Tf should be about 10*Ts
          // higher alpha -> lower Tf -> higher cut-off freq. -> less filtering
        }

        float ComputeVelocity(int curr_cnt)
        {
            int delta_cnt = curr_cnt - prev_cnt_; // difference between current and previous encoder counts
            float delta_rad = delta_cnt / counts_per_rev_ * 2.0 * PI; // difference in radians [rad/s]
            vel_radps_ = delta_rad / Ts_;
            
            vel_radps_filt_ = (1 - alpha_) * prev_vel_radps_filt_ + alpha_ * vel_radps_; // Filter measurement

            prev_cnt_ = curr_cnt; // update encoder count
            prev_vel_radps_filt_ = vel_radps_filt_;

            return vel_radps_filt_;
        }

        void Reset()
        {
            prev_cnt_ = 0;
            vel_radps_ = 0;
            vel_radps_filt_ = 0;
            prev_vel_radps_filt_ = 0;
        }

        int prev_cnt_; // previous encoder count
        float vel_radps_; //
        float vel_radps_filt_;
        float prev_vel_radps_filt_;
        
    private:
        float Ts_; // sample time [sec]
        float Tf_; // derivate low pass filter time constant
        float alpha_; // derivative low pass filter coefficient
        float counts_per_rev_; 

    };
};

#endif
