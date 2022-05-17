from casadi import *
import numpy as np
from pylab import plot, step, figure, legend, show, spy, quiver, scatter

opti = Opti()
ti = 0
# tf = opti.variable(1)
xi = 0  # init pos
xf = 100  # final desired pos
xdoti = 0  # init vel
xdotf = 0  # final des vel

N = 20  # num time steps
X = opti.variable(2, N + 1)  #
U = opti.variable(1, N)
T = opti.variable()
t = np.linspace(0, 1, N + 1)
# ddynamic constrains
# return Xdot
f = lambda x, u: vertcat(x[1], u)

# define length of control interval
dt = T / N

for k in range(N):  # loop over control intervals
    # Integrate
    x_next = X[:, k] + f(X[:, k], U[:, k]) * dt
    opti.subject_to(X[:, k + 1] == x_next)
"""
for k in range(N): # loop over control intervals
    # Runge-Kutta 4 integration
    k1 = f(X[:,k],         U[:,k])
    k2 = f(X[:,k]+dt/2*k1, U[:,k])
    k3 = f(X[:,k]+dt/2*k2, U[:,k])
    k4 = f(X[:,k]+dt*k3,   U[:,k])
    x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
    opti.subject_to(X[:,k+1]== x_next) # close the gaps
"""
opti.subject_to(opti.bounded(-2, X[1, :], 2))  # limit velocity -1 to 1
opti.subject_to(X[0, 0] == xi)  # Initial position
opti.subject_to(X[1, 0] == xdoti)  # Initial velocity
opti.subject_to(X[0, -1] == xf)  # final pos
opti.subject_to(opti.bounded(-1, U[0, :], 1))
opti.subject_to(T >= 0)
# opti.subject_to(X[1,-1] == xdotf) #final velocity

# set initial guess
for i in range(N + 1):

    opti.set_initial(X[0, i], (xf - xi) / (N + 1 - i))  # intial pos guess

# Objective functions
opti.minimize(T)

# set initial control input
# opti.set_initial(U, np.repeat(1, N))

# set some solver settings
opti.solver("ipopt")
sol = opti.solve()

scatter(t * sol.value(T), sol.value(X[0, :]), label="pos traj")
scatter(t * sol.value(T), sol.value(X[1, :]), label="vel traj")
legend()
show()

"""
Max car speed (boosting): 2300 uu/s
Supersonic speed threshold: 2200 uu/s
Max driving speed (forward and backward) with no boost: 1410 uu/s
Maximum car angular acceleration:
Yaw: 9.11 radians/s^2
Pitch: 12.46 radians/s^2
Roll: 38.34 radians/s^2
Maximum car angular velocity: 5.5 radians/s
Boost acceleration: 991.666 uu/s^2
"""
