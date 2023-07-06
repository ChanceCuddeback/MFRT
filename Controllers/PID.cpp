#include "PID.hpp"

/**
 * @brief Construct a new PID::PID object
 * 
 * @param p_gain Proportional gain
 * @param i_gain Integral gain
 * @param d_gain Derivative gain
 * @param delta_t Delta T in seconds
 */
PID::PID(double p_gain, double i_gain, double d_gain, double delta_t) :
    _p_gain(p_gain),
    _i_gain(i_gain),
    _d_gain(d_gain),
    _delta_t(delta_t),
    _integral_val(0.0),
    _prev_input(0.0)
{
}

/**
 * @brief Destroy the PID::PID object
 * 
 */
PID::~PID()
{
}

/**
 * @brief Step the PID controller
 * 
 * @param cmd Desired value
 * @param measurement Measured value
 * @return double Output of the PID controller
 */
double PID::step(double cmd, double measurement)
{
    // Calculate the error
    const double error{cmd - measurement};
    // Calculate the derivative
    const double derivative{(measurement - _prev_input) / _delta_t};
    // Calculate the integral
    _integral_val += error * _delta_t;
    // Calculate the output
    const double output{(_p_gain * error) + (_i_gain * _integral_val) + (_d_gain * derivative)};
    // Update the previous input
    _prev_input = measurement;
    // Return the output
    return output;
}

/**
 * @brief Reset the state of the PID controller to zero. This does not change the gains or the time step. 
 * 
 */
void PID::reset()
{
    _integral_val = 0.0;
    _prev_input = 0.0;
}