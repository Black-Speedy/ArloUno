import numpy as np

def wrapTheta(theta):
        if theta > np.pi * 2:
            return theta - np.pi * 2
        elif theta < 0:
            return theta + np.pi * 2
        else:
            return theta
        
pos1 = (0, 0)
pos2 = (1, 1)

selfTheta = np.pi/2

ydiff = pos2[1] - pos1[1]
xdiff = pos2[0] - pos1[0]

print(ydiff, xdiff)

theta = np.arctan2(ydiff, xdiff) - selfTheta
theta = wrapTheta(theta)

print(theta)

print(np.degrees(theta))
