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

/**
 * @brief The proportional integral derivative controller, PID, uses the desired value and the measured value to drive a system to the 
 * desired value. The PID controller is a feedback controller that uses the error between the desired value and the measured value to 
 * calculate the output. The output is calculated by multiplying the error by the proportional gain, the integral of the error by the
 * integral gain, and the derivative of the error by the derivative gain. This controller does not require a model of the system and
 * must be tuned to achieve the desired performance. 
 * 
 */
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
        const double _p_gain, _i_gain, _d_gain;
};

#endif // PID_H