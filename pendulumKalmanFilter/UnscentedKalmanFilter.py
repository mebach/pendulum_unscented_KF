import numpy as np
import pendulumParam as P
from MerweScaledSigmaPoints import MerweScaledSigmaPoints


class UnscentedKalmanFilter:
    def __init__(self):
        # Initial state and covariance estimate
        self.x = np.array([P.theta0,
                          P.thetadot0])

        # Number of dimensions
        self.n = len(self.x)

        self.P = np.eye(self.n) * 1e-3

        self.x_prior = np.array([[0.0], [0.0]])
        self.P_prior = np.eye(self.n) * 0.0

        self.x_post = np.array([[0.0], [0.0]])
        self.P_post = np.eye(self.n) * 0.0

        self.z = 0.0

        # Process noise covariance matrix
        self.Q = np.eye(2) * 1e-5

        # Measurement noise covariance
        self.R = np.array([[0.1]])

        # Kalman gain | dimension of states x dimension of measurements
        self.K = np.zeros((2, 1))

        # Innovation residual | dimension of measurements
        y = np.zeros(1)

        # System Uncertainty | dim of measurements x dim of measurements
        self.S = np.zeros((1, 1))

        # Inverse System Uncertainty | dim of measurements x dim of measurements
        self.SI = np.zeros((1, 1))

        # Sigma point parameters
        self.alpha = 1e-3
        self.beta = 2.0
        self.kappa = 0.0

        # sigma points object
        self.points = MerweScaledSigmaPoints(n=self.n, alpha=self.alpha, beta=self.beta,
                                             kappa=self.kappa)

        self.sigmas   = np.zeros((5, 2)) # number of sigma points x number of states
        self.sigmas_f = np.zeros((5, 2)) # number of sigma points x number of states
        self.sigmas_h = np.zeros((5, 1)) # number of sigma points x number of measurements

        # Weights for the sigma points for the mean and covariance respectively
        self.Wm = self.points.Wm
        self.Wc = self.points.Wc

        # Rate at which to propagate dynamics
        self.dt = P.Ts

    def f(self, x):
        y1 = x[0] + x[1] * self.dt
        y2 = x[1] + (-P.g/P.l * np.sin(x[0]) * self.dt)
        return np.array([y1, y2])


    def h(self, x):
        r = np.sqrt(P.h**2 + P.l**2 - 2*P.h*P.l*np.cos(x[0]))
        return r

    def unscented_transform_prior(self, sigmas, Wm, Wc):
        # kmax, n = self.sigmas.shape
        new_mean = np.dot(Wm, sigmas)
        y = sigmas - new_mean[np.newaxis, :]
        new_Cov = np.dot(y.T, np.dot(np.diag(Wc), y)) + self.Q
        return new_mean, new_Cov

    def unscented_transform_measurement(self, sigmas, Wm, Wc):
        measurement_mean = np.dot(Wm, sigmas)
        y = sigmas - measurement_mean
        new_Cov = np.dot(y.T, np.dot(np.diag(Wc), y)) + self.R
        return measurement_mean, new_Cov

    def cross_variance(self, x, z, sigmas_f, sigmas_h):
        Pxz = np.zeros((sigmas_f.shape[1], sigmas_h.shape[1]))
        N = sigmas_f.shape[0]
        for i in range(N):
            dx = sigmas_f[i] - x
            dz = sigmas_h[i] - z
            Pxz += self.Wc[i] * np.outer(dx, dz)
        return Pxz

    def update(self, z):

        ### Main Filter function ###

        # Compute sigma points based on current mean and covariance state estimate
        self.sigmas = self.points.sigma_points(self.x, self.P)

        # Pass sigma points through process model
        for i in range(np.shape(self.sigmas)[0]):
            self.sigmas_f[i] = self.f(self.sigmas[i])

        # Pass process sigmas through the unscented transform to find the mean and cov prior
        self.x_prior, self.P_prior = self.unscented_transform_prior(self.sigmas_f, self.Wm, self.Wc)

        # Pass process sigmas through the measurement function
        for i in range(np.shape(self.sigmas_f)[0]):
            self.sigmas_h[i] = self.h(self.sigmas_f[i])

        # Calculate new mean and covariance of the predicted measurement
        zp, self.S = self.unscented_transform_measurement(self.sigmas_h, self.Wm, self.Wc)
        self.SI = np.linalg.inv(self.S)

        # print('zp: ', zp)
        # print('S: ', self.S)
        # print('SI: ', self.SI)

        # Calculate cross variance of the state and the measurements
        Pxz = self.cross_variance(self.x_prior, zp, self.sigmas_f, self.sigmas_h)
        # print("Pxz: ", Pxz)

        # Calculate the Kalman gain
        self.K = np.dot(Pxz, self.SI)

        # Calculate innovation residual | incoming measurement subtracted by the "predicted"
        # measurement
        self.y = z - zp
        # print("y: ", self.y)

        # Calculate an overall estimate of the mean and covariance of the system
        self.x_post = self.x_prior + np.dot(self.K, self.y)
        self.P_post = self.P_prior - np.dot(self.K, np.dot(self.S, self.K.T))

        self.x = self.x_post
        self.P = self.P_post
        print(self.x)
        print(self.P)

        return self.x_post, self.P_post



