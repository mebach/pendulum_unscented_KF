import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import sys
sys.path.append('../..')  # add parent directory
import numpy as np
import pendulumParam as P


class pendulumAnimation:
    '''
        Create pendulum animation
    '''
    def __init__(self):
        self.flagInit = True                  # Used to indicate initialization
        self.fig, self.ax = plt.subplots()    # Initializes a figure and axes object
        self.handle = []                      # Initializes a list object that will
                                              # be used to contain handles to the
                                              # patches and line objects.
        self.length = P.l
        plt.axis([0, 2*P.l, -5*P.l, P.l]) # Change the x,y axis limits

        plt.plot([0, 2*P.l], [-P.h+P.l, -P.h+P.l], 'k--')    # Draw ground
        plt.plot([0, 2*P.l], [P.l, P.l], 'black')



    def update(self, u):
        # Process inputs to function
        theta = u.item(0)

        self.drawPendulum(theta)
        self.ax.axis('equal')

        # After each function has been called, initialization is over.
        if self.flagInit == True:
            self.flagInit = False

    def drawPendulum(self, theta):

        x = [P.l, P.l+P.l*np.sin(theta)]
        y = [P.l, P.l-P.l*np.cos(theta)]
        xy = (x[1], y[1])
        xy_radar = (P.l, -P.h+P.l)
        phi = np.arctan2(P.l*np.sin(theta), (P.h-P.l))


        if self.flagInit == True:
            # Create the Rectangle patch and append its handle
            # to the handle list
            line, = self.ax.plot(x, y, lw=2, c='black')

            self.handle.append(mpatches.Circle(xy, radius=0.1, fc='limegreen'))
            self.handle.append(mpatches.Rectangle(xy_radar, 0.1, 0.5, fc='blue', ec='black'))
            self.ax.add_patch(self.handle[0])
            self.ax.add_patch(self.handle[1])
            self.handle.append(line)  # Add the patch to the axes

        else:
            self.handle[0].center = xy
            self.handle[1].set_angle(-phi*180/np.pi)
            self.handle[2].set_xdata(x)         # Update line
            self.handle[2].set_ydata(y)



