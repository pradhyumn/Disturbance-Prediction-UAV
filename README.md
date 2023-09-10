# Disturbance-Prediction-UAV

This work provides the code for a finite-time stable disturbance observer design for the discretized dynamics of an unmanned vehicle in three-dimensional translational and rotational motion. The dynamics of this vehicle is discretized using a Lie group variational integrator as a grey box dynamics model that also accounts for unknown additive disturbance force and torque. Therefore, the input-state dynamics is partly known. The unknown dynamics is lumped into a single disturbance force and a single disturbance torque, both of which are estimated using the disturbance observer we design. This disturbance observer is finite-time stable (FTS) and works like a real-time machine learning scheme for the unknown dynamics.

This work was presented in American Control Conference 22 held in Atlanta, Georgia in May, 2022. The link to the full paper is [here](https://ieeexplore.ieee.org/abstract/document/9867308).

The control system architecture is shown below with the disturbance observer in the feedback loop to estimate the force and torque disturbances. The position and attitude controller in loop with the DO are also developed with asymptotic stability in mind.

![Control Systems Architecture](Results/ControlSystemsArchitecture.png)


Dynamic turbulent wind effects are difficult to model exactly, even with the use of detailed CFD analysis and simplifying assumptions for the wind fields. The uncertainties that exist in a realistic wind field contain a variety of different atmospheric sources like turbulence, vortex, gust and shear. Despite this, a combination of sinusoidal wave-forms are generally sufficient to capture the dominant characteristics that exist in a realistic wind field. To this avail


