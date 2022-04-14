import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np
from casadi import *

from src.utilities.lin_alg_utils import LinAlgUtils as lau
from src.robot.state import State
from src.robot.model import Model

# from src.robot.bot import Bot


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

opti = Opti()
ti = 0  # starting time
xi = 0  # init x pos
xf = 1000  # final desired x pos
yi = 0  # init y pos
yf = 1000  # final desired x pos
zi = 1000  # init y pos
zf = 700  # final desired x pos
xdoti = 0  # init x vel
ydoti = 0  # init y vel
zdoti = 0  # init z vel
e0i = 1
e1i = 0
e2i = 0
e3i = 1
pitchdoti = 0
rolldoti = 0
yawdoti = 0
g = 0  # Gravity

# Max values
v_dotMax = 66.67  # uu/s^2
vMax = 2300  # uu/s
omega_max = 5.5  # rad/s, same for roll, pitch, yaw
phi_omegadot_max = 38.34  # roll rad/s
theta_omegadot_max = 12.46  # pitch rad/s
psi_omegadot_max = 9.11  # yaw rad/s
boost_max = 915.666
throttle_max = 66.667
# brake_max = -3500

N = 20  # num time steps
X = opti.variable(
    13, N + 1
)  # x, xdot, y, ydot vmag, theta, thetadot,psi, psidot, phi, phidot
U = opti.variable(4, N)  # angular accelerations, Thrust
T = opti.variable()
t = np.linspace(0, 1, N + 1)

dt = T / N

# constraints
opti.subject_to(T >= 0)

# intial conditions
opti.subject_to(X[0, 0] == xi)  # Initial position
opti.subject_to(X[1, 0] == yi)  # Initial position
opti.subject_to(X[2, 0] == zi)  # Initial position
opti.subject_to(X[3, 0] == xdoti)  # Initial velocity
opti.subject_to(X[4, 0] == ydoti)  # Initial velocity
opti.subject_to(X[5, 0] == zdoti)  # Initial velocity
opti.subject_to(X[6, 0] == e0i)  # Initial e0
opti.subject_to(X[7, 0] == e1i)  # Initial e1
opti.subject_to(X[8, 0] == e2i)  # Initial e2
opti.subject_to(X[9, 0] == e3i)  # Initial e3
opti.subject_to(X[10, 0] == pitchdoti)  # Initial velocity
opti.subject_to(X[11, 0] == rolldoti)  # Initial velocity
opti.subject_to(X[12, 0] == yawdoti)  # Initial velocity

# def
# qT = Bot()
# qT.set_state(X[:, 0])
# modelT = Model(dt=1 / 120, q=qT)

# final conditions
opti.subject_to(X[0, -1] == xf)  # Initial position x
opti.subject_to(X[1, -1] == yf)  # Initial position y
opti.subject_to(X[2, -1] == zf)  # Initial position z
opti.subject_to(X[3, -1] == 0)  # Initial velocity x
opti.subject_to(X[4, -1] == 0)  # Initial velocity y
opti.subject_to(X[5, -1] == 0)  # Initial velocity z
# opti.subject_to(X[6, -1] == e0f)  # Initial quat e0
# opti.subject_to(X[7, -1] == e1f)  # Initial quat e1
# opti.subject_to(X[8, -1] == e2f)  # Initial quat e2
# opti.subject_to(X[9, -1] == e3f)  # Initial quat e3
# opti.subject_to(X[10, -1] == pitchdotf)  # Initial pitch omega
# opti.subject_to(X[11, -1] == rolldotf)  # Initial roll omega
# opti.subject_to(X[12, -1] == yawdotf)  # Initial yaw omega

# limiting conditions inputs
opti.subject_to(opti.bounded(-throttle_max, U[0, :], boost_max))
opti.subject_to(opti.bounded(-phi_omegadot_max, U[1, :], phi_omegadot_max))
opti.subject_to(opti.bounded(-theta_omegadot_max, U[2, :], theta_omegadot_max))
opti.subject_to(opti.bounded(-psi_omegadot_max, U[3, :], psi_omegadot_max))

# limiting conditions
opti.subject_to(opti.bounded(-omega_max, X[10, :], omega_max))
opti.subject_to(opti.bounded(-omega_max, X[11, :], omega_max))
opti.subject_to(opti.bounded(-omega_max, X[12, :], omega_max))
opti.subject_to(opti.bounded(-vMax, X[3, :], vMax))
opti.subject_to(opti.bounded(-vMax, X[4, :], vMax))
opti.subject_to(opti.bounded(-vMax, X[5, :], vMax))

for i in range(N + 1):

    opti.set_initial(X[0, i], (xf - xi) / (N + 1 - i))  # intial pos guess
    opti.set_initial(X[1, i], (yf - yi) / (N + 1 - i))  # intial pos guess
    opti.set_initial(X[2, i], (zf - zi) / (N + 1 - i))  # intial pos guess

"""
# lamda function
f = lambda x, u: model.q_dot
"""
# define length of control interval
dt = (T) / N  # length of a control interval

state = State(
    x=xi,
    y=yi,
    z=zi,
    x_dot=xdoti,
    y_dot=ydoti,
    z_dot=zdoti,
    e0=e0i,
    e1=e1i,
    e2=e2i,
    e3=e3i,
    phi_dot=rolldoti,
    theta_dot=pitchdoti,
    psi_dot=yawdoti,
)

model = Model(dt=dt, q=state)
f = lambda x, u: model.q_dot(x, u)

for k in range(N):  # loop over control intervals
    # Integrate
    x_next = X[:, k] + f(X[:, k], U[:, k]) * dt
    opti.subject_to(X[:, k + 1] == x_next)

# https://github.com/casadi/casadi/issues/2762

"""
for k in range(N):  # loop over control intervals
    # Runge-Kutta 4 integration
    k1 = f(X[:, k], U[:, k])
    k2 = f(X[:, k] + dt / 2 * k1, U[:, k])
    k3 = f(X[:, k] + dt / 2 * k2, U[:, k])
    k4 = f(X[:, k] + dt * k3, U[:, k])
    x_next = X[:, k] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    opti.subject_to(X[:, k + 1] == x_next)  # close the gaps
"""

# for k in range(N):  # loop over control intervals
#     modelT.step(U[:, k])

# Objective functions
opti.minimize(T)  # minimizing time

# set some solver settings
opti.solver("ipopt")
sol = opti.solve()

# sol.value(X)
# sol.value(U)
