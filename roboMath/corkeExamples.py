import roboticstoolbox
import spatialmath
import spatialgeometry
import roboMath
import numpy as np
import scipy as sp
import scipy.linalg as lalg
from spatialmath.base import rot2, trplot2, transl2, trot2
import matplotlib.pyplot as plt


R:np.matrix = roboMath.rot_mat2(45, "deg")
print(R)
print(R.A)

trplot2(R)

#plt.show()

d = np.linalg.det(R)
print(d)

R = roboMath.rot_mat2(0.3)

# This yields a skewed symmetrical matrix of the angle in radians
L = lalg.logm(R)
print(L)
# Exponentiating yields the original rot matrix. See Corke p.36-37
orig = lalg.expm(L)
print(orig)

T = transl2(1, 2)
print("T\n", T)
A = trot2(30, "deg")
print("A\n", A)
TA = T @ A
print("TA\n", TA)