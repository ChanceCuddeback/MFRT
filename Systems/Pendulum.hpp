/**
 * @file Pendulum.hpp
 * @author Chancelor Cuddeback
 * @brief Rigid body, single DOF, point mass pendulum; with and without damping
 * @date 2023-07-05
 * 
 */
#ifndef PENDULUM_H
#define PENDULUM_H
#include <map>
#include <string>

class Pendulum
{
    public:
        Pendulum(double mass, double length, double gravity, double damping, double angle, double angular_velocity);

        ~Pendulum();

        void step(double cmd, double delta_t);
        void reset();

        std::map<std::string, double> getState() const;

    private:
        const double _mass;
        const double _length;
        const double _gravity;
        const double _damping;

        double _angle;
        double _angular_velocity;
};

#endif // PENDULUM_H