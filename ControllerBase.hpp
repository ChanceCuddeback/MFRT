/**
 * @file ControllerBase.hpp
 * @author Chancelor Cuddeback
 * @brief Base for controllers
 * @date 2023-06-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H


class ControllerBase 
{
    public:
        ControllerBase();
        virtual ~ControllerBase();

        virtual double step(double cmd, double measurement) = 0;
        virtual void reset() = 0;
};

#endif // CONTROLLER_BASE_H