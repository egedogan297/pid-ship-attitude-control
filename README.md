# Spacecraft Attitude (Tilt) Control Simulator

As a Dead Space fan, after building a basic PID rocket simulator, I thought I could adapt the same PID logic to a spacecraft — specifically a planet-cracker ship. This project simulates a large ship correcting its tilt angle using differential thrust from two sets of thrusters (left and right), visualized as an artificial horizon display.

## What It Does

The ship starts tilted at 50 degrees and uses a PID controller to level itself back to 0 degrees (horizontal). Gravity constantly pulls the ship off-balance (modeled as an inverted pendulum problem), so the thrusters have to continuously correct for it. The simulation is displayed from a rear view, similar to an aircraft's artificial horizon indicator.

## Components

- **Ship class**: rotational physics model (angle, angular velocity, angular acceleration)
- **PID class**: computes a correction value based on the tilt error
- **simulation class**: runs the control loop, splits the PID output into left/right thrust, and visualizes it with turtle graphics
- Final result plotted with matplotlib

## Key Concepts

- Moment of Inertia (rotational equivalent of mass)
- Torque (rotational equivalent of force)
- Differential thrust: splitting a single PID correction value into two opposing thruster outputs to create rotation
- Inverted pendulum dynamics: how gravity can actively work against stability instead of helping it
- Radian vs degree consistency in physics calculations
- Damping and oscillation in PID tuning (avoiding overshoot)

## Credit

This project builds on the PID logic from my [rocket height control simulator](link-to-other-repo), adapted from a rotational (torque-based) system I designed myself, inspired by the Dead Space universe.
