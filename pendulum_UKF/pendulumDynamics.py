import numpy as np
import random
import sys
sys.path.append('../..')  # add parent directory
import pendulumParam as P


class pendulumDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.theta0],  # initial cube position
            [P.thetadot0] # initial theta dot
        ])

        # Mass of the ball, kg
        self.m = P.m * (1. + alpha * (2. * np.random.rand() - 1.))

        # Length of the pendulum
        self.l = P.l * (1. + alpha * (2. * np.random.rand() - 1.))

        # the gravity constant is well known, so we don't change it.
        self.g = P.g

        # sample rate at which the dynamics are propagated
        self.Ts = P.Ts

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = self.saturate(u)

        self.rk4_step(u)  # propagate the state by one time sample
        x_true = self.h()  # return the corresponding output

        R_true = np.sqrt((P.h - P.l)**2 + (P.l*np.sin(x_true.item(0)))**2)

        return x_true, R_true

    def f(self, state, u):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability
        theta = state.item(0)
        thetadot = state.item(1)

        thetaddot = -self.g/self.l * (np.sin(theta))
        xdot = np.array([[thetadot],
                         [thetaddot]])

        return xdot

    def h(self):
        # return the output equations
        # could also use input u if needed
        theta = self.state.item(0)
        thetadot = self.state.item(1)
        y = np.array([[theta],
                      [thetadot]])

        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u):

        return u
