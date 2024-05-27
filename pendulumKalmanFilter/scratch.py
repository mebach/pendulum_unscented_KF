from MerweScaledSigmaPoints import MerweScaledSigmaPoints
import numpy as np
import pendulumParam as P

points = MerweScaledSigmaPoints(n=2, alpha=0.1, beta=2.0, kappa=0.0)
x = np.array([P.theta0, P.thetadot0])
P = np.eye(2) * 1e-3


sigmas = points.sigma_points(x, P)
print(np.size(sigmas))
print(points.Wm)
print(points.Wc)