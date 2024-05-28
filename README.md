# Unscented Kalman Filter Exercise 

**To run the program, simply run main.py.**

The following project is an exercise in applying an Unscented Kalman Filter to a nonlinear system for state estimation. 

The system is a simple pendulum in free motion (no control / actuation inputs). It is assumed that there are no damping forces of any kind. The desired states that we wish to estimate are the angle and angle rate of the pendulum. The small-angle assumption is NOT used; thus, the dynamics of the system are nonlinear. 

In this exercise, I have also assumed that the state of the system cannot be measured directly; rather, in this exercise, I assume that there is a radar at some known distance below the pendulum measuring the range to the mass on the end of the pendulum. I assume the radar is able to obtain a range measurement irrespective of the angle of the pendulum. Since the parameters of the system (the length of the pendulum, the distance from where the pendulum hangs to the radar, etc.) are known, the angle of the pendulum can be ascertained.

Because both the process model (the dynamic equations of motion) and the measurement model (the transformation from range to an angle) are both highly nonlinear functions, a conventional linear Kalman Filter is insufficient to measure and estimate the system. So, an Unscented Kalman Filter has been implemented. 

Running main.py will bring up an animation of the pendulum system in motion as well as a pseudo-live data plotter. Filter performance can be inspected qualitatively from the data plots. 
