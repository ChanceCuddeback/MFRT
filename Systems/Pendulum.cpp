#include "Pendulum.hpp"
#include <cmath> // sin

Pendulum::Pendulum(double mass, double length, double gravity, double damping, double angle = 0.0, double angular_velocity = 0.0) :
    _mass(mass),
    _length(length),
    _gravity(gravity),
    _damping(damping),
    _angle(0.0),
    _angular_velocity(0.0)
{
}

Pendulum::~Pendulum()
{
}

void Pendulum::step(double cmd, double delta_t)
{
    const double torque{cmd - (_damping * _angular_velocity)};
    const double angular_acceleration{(torque - (_mass * _gravity * _length * sin(_angle))) / (_mass * _length * _length)};
    _angular_velocity += angular_acceleration * delta_t;
    _angle += _angular_velocity * delta_t;
}
void Pendulum::reset()
{
    _angle = 0.0;
    _angular_velocity = 0.0;
}

std::map<std::string, double> Pendulum::getState() const
{
    return {{"Angle", _angle}, {"Angular Velocity", _angular_velocity}};
}