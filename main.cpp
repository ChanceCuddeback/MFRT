// Runs a simulation to test the PID controller and pendulum system. 

#include "Systems/Pendulum.hpp"
#include "Controllers/PID.hpp"

#include <iostream>
#include <string>
#include <vector>


static const double dt{0.01};
static const double g{9.81};

int main()
{
    // Define pendulum params
    const double mass{1.0};
    const double length{1.0};
    const double damping{0.1};
    const double initial_angle{2.0};
    const double initial_angular_velocity{0.0};
    // Initialize the pendulum
    Pendulum pendulum = Pendulum(mass, length, g, damping, initial_angle, initial_angular_velocity);

    // Define controller params
    const double p_gain{0.0};
    const double i_gain{0.0};
    const double d_gain{0.0};
    // Initialize the controller
    PID pid = PID(p_gain, i_gain, d_gain, dt);

    // Define the simulation parameters
    const double simulation_time{10.0};
    const int num_steps{static_cast<int>(simulation_time / dt)};
    
    // Initialize the simulation
    double cmd{0.0};
    double measurement{0.0};
    double output{0.0};
    std::vector<double> time;
    std::vector<double> angle;
    std::vector<double> angular_velocity;
    std::vector<double> command;
    std::vector<double> output_vector;
    for (int i{0}; i < num_steps; ++i)
    {
        // Step the controller
        output = pid.step(cmd, measurement);
        // Step the system
        pendulum.step(output, dt);

        // Save the data
        std::map<std::string, double> state = pendulum.getState();
        time.push_back(i * dt);
        angle.push_back(state["Angle"]);
        angular_velocity.push_back(state["Angular Velocity"]);
        command.push_back(cmd);
        output_vector.push_back(output);
    }
    
    // Plot the results
    // plt::figure();
    // plt::subplot(2, 1, 1);
    // plt::plot(time, angle);
    // plt::plot(time, command);
    // plt::title("Angle");
    // plt::subplot(2, 1, 2);
    // plt::plot(time, angular_velocity);
    // plt::plot(time, output_vector);
    // plt::title("Angular Velocity");
    // plt::show();
    
    return 0;
}