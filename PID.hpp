/**
 * @file PID.h
 * @author Chancelor Cuddeback
 * @brief Proportional Integral Derivative Controller
 * @date 2023-06-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef PID_H
#define PID_H
#include "ControllerBase.hpp"

class PID : ControllerBase
{
    public:
        PID(double p_gain, double i_gain, double d_gain, double delta_t);
        ~PID();

        double step(double cmd, double measurement);
        void reset();
    private:
        double _integral_val;
        double _prev_input;
        const double _delta_t;
};

PID::PID(double p_gain, double i_gain, double d_gain, double delta_t);

double step(float cmd, float measurement)
{
    float error_val = cmd - measurement;  
    _integral_val += error_val ;     
    float de_dt = 0;
    de_dt = -(measurement - _prev_input) * (1 / _delta_t);
    _prev_de_dt = de_dt;

    _prev_pid_time = now;
    _prev_input = measurement;

    float p = p_gain * error_val;  
    float i = i_gain * _integral_val;
    float d = d_gain * de_dt; 
    
    return p + i + d;
}

#endif // PID_H