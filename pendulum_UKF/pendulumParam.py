# Pendulum Kalman Filter Parameter File
import numpy as np

# Desired state, of the form
# des_state = np.array([[0.0],
#                       [0.0]])

# Physical parameters of the mass known to the controller
m = 1.0 + np.random.uniform() * 0.1  # mass of pendulum bob kg
l = 1.5    # length of pendulum
g = 10  # gravity m/s^2
h = 5.0  # distance from radar to the top of the ceiling

# Initial Conditions
theta0 = np.pi/12 # initial angle of pendulum
thetadot0 = 0.0   # initial angular velocity of pendulum


# Simulation Parameters
t_start = 0  # Start time of simulation
t_end = 100.0  # End time of simulation
Ts = 0.02  # sample time for simulation
t_plot = 0.01  # the plotting and animation is updated at this rate




