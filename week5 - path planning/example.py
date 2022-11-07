#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

# Target poses
x1 = 0
y1 = 0
theta1 = 0 * np.pi / 180

x2= 20
y2= 50
theta2 = 75 * np.pi / 180

# Coefficients
A  = np.array([ [1, x1, x1**2, x1**3],
                [1, x2, x2**2, x2**3],
                [0, 1,   2*x1, 3*x1**2],
                [0, 1,   2*x2, 3*x2**2]
                ])

b = np.array([ [y1],
               [y2],
               [np.tan(theta1)],
               [np.tan(theta2)],               
               ])

a_coef = np.linalg.inv(A) @ b

a0 = a_coef[0];
a1 = a_coef[1];
a2 = a_coef[2];
a3 = a_coef[3];

# Path
X = np.linspace(x1, x2, 1000, endpoint= True);
Y = a3 * X**3 +  a2 * X**2 + a1*X + a0


# plt.clf()
# plt.plot(X,Y)
# maxLim = max(abs(x1), abs(y1), abs(x2), abs(y2) ) 
# plt.xlim([-1.05*maxLim, 1.05*maxLim])
# plt.ylim([-1.05*maxLim, 1.05*maxLim])
# plt.quiver(x1, y1, np.cos(theta1), np.sin(theta1 ))
# plt.quiver(x2, y2, np.cos(theta2 ), np.sin(theta2 ))



# Archlength
S = np.linspace(0,0, X.size) # initilize an array
S[0] = 0
for i in range(1, X.size):
    dX = X[i] - X[i-1]
    dY = Y[i] - Y[i-1]
    dS = np.sqrt( dX**2 + dY**2)
    S[i] = S[i-1] + dS


#  Parametrization
def parametrize(amax, Vmax, S):
    D = S[-1]
    t1 = Vmax / amax
    tf = D/Vmax + t1
    
    S1 = 0.5*amax*t1**2
    S2 = D - S1
    S3 = D
    
    T = np.linspace(0,0, S.size)
    for i in range(S.size):
        s = S[i]
        if s <= S1:
            T[i] = np.sqrt( 2 * s / amax)
            
        elif s > S1 and s <= S2:
            T[i] = t1 + (s-S1)/Vmax
            
        elif s > S2 and s <= S3:
            T[i] = tf - np.sqrt(2*(D-s)/amax)
    
    return T
    

amax = 5
Vmax = 10
T = parametrize(amax, Vmax, S)

# plt.clf()
# plt.plot(T, S)

# Velocity
def trapvel(amax, Vmax, D, T):
    t1 = Vmax / amax
    tf = D /Vmax + t1
    
    V = np.linspace(0,0, T.size)
    for i in range(T.size):
        t = T[i]
        if t <= t1:
            V[i] = amax*t
            
        elif t > t1 and t <= tf - t1:
            V[i] = Vmax
            
        elif t > (tf-t1) and t <= tf:
            V[i] = Vmax - amax*(t-tf + t1)
            
        else:
            V[i] = 0
            
    return V
  
    
V = trapvel(amax, Vmax, S[-1], T)

# plt.clf()
# plt.plot(T, V)

# Theta and W
Q = np.linspace(0,0, T.size)
W = np.linspace(0,0, T.size)
Q[0] = theta1
W[0] = 0
for i in range(1, X.size):
    dY = Y[i] - Y[i-1]
    dX = X[i] - X[i-1]
    Q[i] = np.arctan2(dY, dX)
    W[i] = (Q[i] - Q[i-1]) / (T[i] - T[i-1]) 
    
# plt.clf()
# plt.plot(T,Q)

# Commands
# These  values are not equally spaced in time
# We want to use an equally spaced time so that we can send it in a loop.
rate = 50
time_step = 1/rate
T_cmd = np.arange(0, T[-1], time_step)

from scipy.interpolate import interp1d
V_of_t = interp1d(T, V, kind='cubic')
W_of_t = interp1d(T, W, kind='cubic')

V_cmd = V_of_t(T_cmd)
W_cmd = W_of_t(T_cmd)


'''
'''