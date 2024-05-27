 # THINGS TO DO
# Change data plotter to hold the the new state variables, not just z position but a z and theta
# Have animation draw new box

import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('../..')  # add parent directory
import pendulumParam as P
from signalGenerator import signalGenerator
from pendulumAnimation import pendulumAnimation
from dataPlotter import dataPlotter
from pendulumDynamics import pendulumDynamics
from pendulumController import pendulumController
from UnscentedKalmanFilter import UnscentedKalmanFilter

# instantiate reference input classes
reference = signalGenerator(amplitude=0.0, frequency=0.02, y_offset=0.0)
torque = signalGenerator(amplitude=0.0, frequency=0.001)
noise = signalGenerator(amplitude=0.005, frequency=0.001)
pendulum = pendulumDynamics(alpha=0.0)
#controller = pendulumController()
ukf = UnscentedKalmanFilter()

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = pendulumAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop

    # Propagate dynamics at rate Ts
    t_next_plot = t + P.t_plot
    while t < t_next_plot:
        # Place to account for a non-zero control term, for now, we are just measuring a system
        # as is
        u = 0

        # Propagate true dynamics, output the true state and "true" radar measurement
        x_true, R_true = pendulum.update(u)

        # Add artificial noise to the range measurement
        n = noise.random(t)
        R_meas = R_true + n

        # Use noisy range measurement to estimate the state using an unscented kalman filter
        estimates, cov_estimate = ukf.update(R_meas)

        # Update simulation time
        t = t + P.Ts


    # update animation
    animation.update(pendulum.state)
    dataPlot.update(t, R_true, R_meas, pendulum.state, estimates)

    #plt.show()
    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
