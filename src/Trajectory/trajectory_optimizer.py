import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

from casadi import *
import matplotlib.pyplot as plt
from src.utilities.history import History

# from pylab import plot, step, figure, legend, show, spy, quiver, scatter


class TrajectoryOptimizer:
    def __init__(self):
        self.opti = Opti()  # optimizer
        self.opti_g = Opti()  # optimizer

        # Direct collocation

        # Initial Conditions
        # Bot
        self.t_i = 0  # initial time [s]
        self.y_i = -1500  # initial y-position [uu]
        self.z_i = 600  # initial z-position [uu]
        self.y_dot_i = 0  # initial y-velocity [uu/s]
        self.z_dot_i = 0  # initial z-velocity [uu/s]
        self.theta_i = 0  # np.pi / 2  # initial pitch [rad]
        self.theta_dot_i = 0  # initial pitch rate [rad/s]

        # Final Conditions
        self.theta_f = -np.pi / 2  # - np.pi / 8  # final desired pitch rate [rad/s]

        # Enviornment
        self.g = -660  # acceleration due to gravity [uu/s^2]

        # Ball
        self.y_b_i = 1500  # initial y-position [uu]
        self.z_b_i = 1000  # initial z-position [uu]
        self.y_b_dot_i = -750  # initial y-velocity [uu/s]
        self.z_b_dot_i = 1000  # initial z-velocity [uu/s]

        # Final Conditions
        # Bot
        # self.y_f = 1000  # desired final y-position [uu]
        # self.z_f = 500  # desired final z-position [uu]
        # self.y_dot_f = 0  # desired final y-velocity [uu/s]
        # self.z_dot_f = 0  # desired final z-velocity [uu/s]

        # self.theta_dot_f = 0  # final desired pitch rate [rad/s]

        # Max Values
        self.v_max = 2000  # maxium velocity [uu/s]
        self.omega_max = 5.5  # maximum angular velocity [rad/s]
        self.theta_ddot_max = 12.46  # maximum pitch acceleration [rad/s]
        self.boost_max = 900  # 915.666  # 991.666  # maximum boost acceleration [rad/s]
        self.throttle_max = 66.67  # maximum throttle acceleration [uu/s^2]

        self.v_b_max = 6000  # Maxium velocity [uu/s]

        """Projectile motion guess"""
        self.z_max = 2044  # ceiling
        self.z_min = 0
        self.y_max = 5120
        self.y_min = -5120

        # self.guess_z = 0.5 * self.g * self.t_guess**2 + self.z_b_dot_i * self.t_guess + self.z_b_i
        self.guess_a = 0.5 * self.g
        self.guess_b = self.z_b_dot_i
        self.guess_c = self.z_b_i
        self.t_guess = (
            self.guess_b + (self.guess_b**2 - 4 * self.guess_a * self.guess_c) ** 0.5
        ) / (-2 * self.guess_a)

        self.y_max_guess = self.t_guess * self.y_b_dot_i

        self.t_unit = np.linspace(0, 1, 50)
        self.t_list_guess = self.t_unit * self.t_guess
        self.z_b_list_guess = []
        self.guess_break_game = False

        for i in range(50):
            self.z_b_guess = (
                0.5 * self.g * self.t_list_guess[i] ** 2
                + self.z_b_dot_i * self.t_list_guess[i]
                + self.z_b_i
            )
            self.z_b_list_guess.append(self.z_b_guess)

        if self.z_b_guess > max(self.z_b_list_guess):
            self.guess_break_game = True
        # elif self.z_b_guess < self.z_min:
        #    self.guess_break_game = True

        """
        if self.y_max_guess < self.y_min:
            self.guess_break_game = True
        elif self.y_max_guess > self.y_max:
            self.guess_break_game = True
        """

        # solver tuning variables
        self.n = 50  # number of timesteps

        """Intial Optimized Guess"""
        self.q_g = self.opti_g.variable(4, self.n + 1)  # state of bot
        self.b_g = self.opti_g.variable(4, self.n + 1)  # state of ball
        self.u_g = self.opti_g.variable(2, self.n)  # inputs(ay, az)
        self.t_g = self.opti_g.variable()  # time
        self.dt_g = self.t_g / self.n
        self.time_g = np.linspace(0, 1, self.n + 1)
        self.time_u_g = np.linspace(0, 1, self.n)
        self.set_constraints_g()
        self.guess2_g()
        self.integrate_rk42_g()
        self.opti_g.solver("ipopt")
        self.sol_g = self.opti_g.solve()
        # print(self.opti_g.debug.show_infeasibilities())

        # Intial positions from the first optimzed guess
        self.t_f_guess = self.sol_g.value(self.t_g[-1])
        self.y_f_guess = self.sol_g.value(self.q_g[0, -1])
        self.z_f_guess = self.sol_g.value(self.q_g[1, -1])
        self.y_dot_f_guess = self.sol_g.value(self.q_g[2, -1])
        self.z_dot_f_guess = self.sol_g.value(self.q_g[3, -1])
        # theta = np.arctan2(self.sol_g.value(self.q_g[1, -1]), self.sol_g.value(self.q_g[0, -1]))
        """
        self.y_b_f_guess = self.sol_g.value(self.b_g[0, -1])
        self.z_b_f_guess = self.sol_g.value(self.b_g[1, -1])
        self.y_b_dot_f_guess = self.sol_g.value(self.b_g[2, -1])
        self.z_b_dot_f_guess = self.sol_g.value(self.b_g[3, -1])
        """
        # Check if its doable
        if max(self.sol_g.value(self.q_g[0, :])) > self.y_max:
            self.guess_break_game = True
        elif min(self.sol_g.value(self.q_g[0, :])) < self.y_min:
            self.guess_break_game = True
        if max(self.sol_g.value(self.q_g[1, :])) > self.z_max:
            self.guess_break_game = True
        elif min(self.sol_g.value(self.q_g[1, :])) < self.z_min:
            self.guess_break_game = True
        if max(self.sol_g.value(self.b_g[0, :])) > self.y_max:
            self.guess_break_game = True
        elif min(self.sol_g.value(self.b_g[0, :])) < self.y_min:
            self.guess_break_game = True
        if max(self.sol_g.value(self.b_g[1, :])) > self.z_max:
            self.guess_break_game = True
        elif min(self.sol_g.value(self.b_g[1, :])) < self.z_min:
            self.guess_break_game = True

        """Actual Optimizer"""
        # self.no_boost_ratio = 0.04  #% of the time step w/o boost
        self.no_boost_ratio = 0.04  #% of the time step w/o boost
        self.no_boost_time = int(
            self.n * self.no_boost_ratio
        )  # ratio of the totatl time with no boost inputs
        self.no_boost_time = 2
        # self.pos_cf_w = 0  # weight of the error of position(1e-2 seems to work well)
        self.col_cf_w = 5e0  # weight of the error of the head on collision(at 1e-2)
        # self.v_cf_w = 1e-1  # weight of the error velocity being in the opposity directions(to be fixed)

        # can implement final velocity of the ball
        # demo. Graph of actual traj, Graph of desire traj, gooy to implement traj

        self.y_b_f = self.y_b_i + self.y_b_dot_i * self.t_f_guess
        self.z_b_f = (
            self.z_b_i
            + self.z_b_dot_i * self.t_f_guess
            + 0.5 * self.g * self.t_f_guess**2
        )

        # dimensions
        self.radius_b = 92.75  # ball radius [uu]
        self.radius_r = (
            120 / 2
        )  # robot radius [uu] https://onedrive.live.com/view.aspx?resid=F0182A0BAEBB5DFF!14583&ithint=file%2cxlsx&app=Excel&authkey=!ALu0cMkDZDoWOws

        # solver variables
        self.q = self.opti.variable(6, self.n + 1)  # state
        self.b = self.opti.variable(4, self.n + 1)  # state of ball
        self.u = self.opti.variable(2, self.n)  # inputs (roll_acceleration, omega)
        self.t = self.opti.variable()  # time
        self.dt = self.t / self.n
        self.time = np.linspace(0, 1, self.n + 1)
        self.time_u = np.linspace(0, 1, self.n)
        self.set_constraints()
        self.guess3()

        self.integrate_rk42()

        self.opti.solver("ipopt")
        # print(self.opti.debug.value)
        # print(self.opti.debug.value(self.q, opti.initial()))

        self.sol = self.opti.solve()
        # print(self.opti.debug.show_infeasibilities())

    def set_constraints(self):
        """
        Sets optimizer constraints
        """
        # Time
        self.opti.subject_to(self.t >= 0.1)

        # Initial Conditions of the bot
        self.opti.subject_to(self.q[0, 0] == self.y_i)  # initial y-position
        self.opti.subject_to(self.q[1, 0] == self.z_i)  # initial z-position
        self.opti.subject_to(self.q[2, 0] == self.y_dot_i)  # initial y-velocity
        self.opti.subject_to(self.q[3, 0] == self.z_dot_i)  # initial z-velocity
        self.opti.subject_to(self.q[4, 0] == self.theta_i)  # initial theta
        # self.opti.subject_to(self.q[4, 0] == getTheta(self.e0_i, self.e1_i, self.e2_i, self.e3_i)) #initial x component of orientation quaternion
        self.opti.subject_to(self.q[5, 0] == self.theta_dot_i)  # initial pitch rate

        # Initial Conditions of the ball
        self.opti.subject_to(self.b[0, 0] == self.y_b_i)  # initial y-position
        self.opti.subject_to(self.b[1, 0] == self.z_b_i)  # initial z-position
        self.opti.subject_to(self.b[2, 0] == self.y_b_dot_i)  # initial y-velocity
        self.opti.subject_to(self.b[3, 0] == self.z_b_dot_i)  # initial z-velocity

        # Bouds of the arena
        self.opti.subject_to(self.q[0, :] < self.y_max)  # y-position max
        self.opti.subject_to(self.q[1, :] < self.z_max)  # z-position max
        self.opti.subject_to(self.b[0, :] < self.y_max)  # y-position max
        self.opti.subject_to(self.b[1, :] < self.z_max)  # z-position max

        self.opti.subject_to(self.q[0, :] > self.y_min)  # y-position max
        self.opti.subject_to(self.q[1, :] > self.z_min)  # z-position max
        self.opti.subject_to(self.b[0, :] > self.y_min)  # y-position max
        self.opti.subject_to(self.b[1, :] > self.z_min)  # z-position max

        # Final Conditions
        # minimize
        """
        self.opti.subject_to((self.q[0, -1] == self.b[0, -1]))  # final y-position
        self.opti.subject_to((self.q[1, -1] == self.b[1, -1]))  # final z-position
        self.opti.minimize(self.t)
        """
        # self.pos_cf = self.pos_cf_w * ((self.q[0, -1] - self.b[0, -1]) ** 2 + (self.q[1, -1] - self.b[1, -1]) ** 2)
        self.col_cf = (
            self.col_cf_w
            * (
                (self.q[0, -1] + (cos(self.q[4, -1]) * self.radius_r))
                - (self.b[0, -1] + (-cos(self.q[4, -1]) * self.radius_b))
            )
            ** 2
            + (
                (self.q[1, -1] + (sin(self.q[4, -1]) * self.radius_r))
                - (self.b[1, -1] + (-sin(self.q[4, -1]) * self.radius_b))
            )
            ** 2
        )  # cost function associacted with hitting the ball at its center
        # self.opti.minimize(self.t + self.pos_cf)
        # self.opti.minimize(self.t + self.col_cf + self.pos_cf)
        self.opti.minimize(self.t + self.col_cf)

        # Do-able collision
        # self.opti.subject_to(self.q[2, -1] - self.b[2, -1] >= 0)  # final y-velocity
        # self.opti.subject_to(self.q[3, -1] - self.b[3, -1] >= 0)  # final z-velocity

        # final theta
        self.opti.subject_to(self.q[4, -1] == self.theta_f)  # final theta

        # self.opti.subject_to(self.q[5, -1] == self.theta_dot_f)  # final pitch rate
        # Control Inputs
        self.opti.subject_to(
            self.opti.bounded(0, self.u[0, :], self.boost_max)
        )  # boost acceleration
        self.opti.subject_to(
            self.opti.bounded(-self.theta_ddot_max, self.u[1, :], self.theta_ddot_max)
        )  # pitch acceleration

        self.opti.subject_to(
            self.opti.bounded(0, self.u[0, -self.no_boost_time :], 0.00001)
        )  # no boost for the last no boost amount of timesteps

        # Limiting Conditions
        self.opti.subject_to(
            self.opti.bounded(-self.omega_max, self.q[5, :], self.omega_max)
        )
        # self.opti.subject_to(self.opti.bounded(-self.v_max, self.q[2:4, :], self.v_max))
        # v_car = ((self.q[2] ** 2) + (self.q[3] ** 2)) ** (1 / 2)
        # v_car = sumsqr(self.q[2:4])
        """
        self.opti.subject_to(self.opti.bounded(-(self.v_max), self.q[2, :], self.v_max))
        self.opti.subject_to(self.opti.bounded(-(self.v_max), self.q[3, :], self.v_max))
        """
        """
        self.opti.subject_to(
            self.opti.bounded(
                0, sqrt(self.q[3, :] ** 2 + self.q[2, :] ** 2), self.v_max
            )
        )
        """

        self.v_mag = self.opti.variable(1, self.n + 1)
        """
        for i in range(self.n +1):
            self.opti.subject_to(
                self.v_mag[0, i] == self.q[2, i] ** 2 + self.q[3, i] ** 2
            )
            self.opti.subject_to(
                self.opti_g.bounded(0, self.v_mag[0, i], self.v_max**2)
            )  # velocity mag
        """

        self.opti.subject_to(self.v_mag[0, :] == self.q[2, :] ** 2 + self.q[3, :] ** 2)
        self.opti.subject_to(
            self.opti_g.bounded(0, self.v_mag[0, :], self.v_max**2)
        )  # velocity mag

        # self.opti.subject_to(self.opti.bounded(-self.v_max, self.q[2:4, :], self.v_max))
        # self.opti.subject_to(v_car <= 20)
        # self.opti.subject_to(sumsqr(self.q[2:4]) <= 10)
        # v_ball = (self.b[2] ** 2 + self.b[3] ** 2) ** (1 / 2)
        # self.opti.subject_to(
        #     self.opti.bounded(-self.v_b_max, self.b[2:4, :], self.v_b_max)
        # )
        # self.opti.subject_to(self.opti.bounded(0, v_ball, self.v_b_max))

        self.opti.subject_to(
            self.radius_b + self.radius_r
            <= sqrt(
                (self.q[0, :] - self.b[0, :]) ** 2 + (self.q[1, :] - self.b[1, :]) ** 2
            )
        )

    def guess3(self):
        for i in range(self.n + 1):
            self.opti.set_initial(
                self.q[0, i], fabs(self.y_b_f - self.y_i) / (self.n + 1 - i)
            )  # path in the y-position guess
            self.opti.set_initial(
                self.q[1, i], fabs(self.z_b_f - self.z_i) / (self.n + 1 - i)
            )  # pathe in the z-position guess

        self.opti.set_initial(self.t, self.t_f_guess)
        self.opti.set_initial(self.q[0, -1], self.y_f_guess)
        self.opti.set_initial(self.q[1, -1], self.z_f_guess)
        self.opti.set_initial(self.q[2, -1], self.y_dot_f_guess)
        self.opti.set_initial(self.q[3, -1], self.z_dot_f_guess)

        """
        self.opti.set_initial(self.b[0, -1], self.y_b_f_guess)
        self.opti.set_initial(self.b[1, -1], self.z_b_f_guess)
        self.opti.set_initial(self.b[2, -1], self.y_b_dot_f_guess)
        self.opti.set_initial(self.b[3, -1], self.z_b_dot_f_guess)
        """

    def q_dot(self, q, u):
        T_theta = 12.14599781908070  # torque coefficient for pitch

        D_theta = -2.798194258050845  # drag coefficient for pitch

        p = q[:2]  # position of the bot in the yz world plane
        v = q[2:4]  # velocity of the bot in the yz world plane
        theta = q[4]  # pitch of the bot
        omega = q[5]  # angular velocity of the bot
        # pitch_abs = fabs(u[1])
        omega_dot = u[1] + D_theta * omega  # + (D_theta * (1 - fabs(u[1]))) * omega
        theta_dot = omega * D_theta  # uneeded
        # v_dot = vertcat(acos(theta) * u[0], asin(theta) * u[0])
        v_dot = vertcat(cos(theta) * u[0], sin(theta) * u[0] + self.g)

        q_dot = vertcat(
            v[0],
            v[1],
            v_dot[0],
            v_dot[1],
            omega,  # theta_dot,
            omega_dot,
        )
        return q_dot

    def b_dot(self, b):
        p_b = b[:2]  # position of the ball
        v_b = b[2:4]  # velocity of the ball

        v_dot_b = vertcat(0, self.g)  # ball velocity in z coordinate

        b_dot = vertcat(v_b[0], v_b[1], v_dot_b[0], v_dot_b[1])
        return b_dot

        for k in range(self.n):  # loop over control intervals
            # Runge-Kutta 4 integration
            k1 = self.q_dot(self.q[:, k], self.u[:, k])
            k2 = self.q_dot(self.q[:, k] + self.dt * 0.5 * k1, self.u[:, k])
            k3 = self.q_dot(self.q[:, k] + self.dt * 0.5 * k2, self.u[:, k])
            k4 = self.q_dot(self.q[:, k] + self.dt * k3, self.u[:, k])
            x_next_q = self.q[:, k] + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self.opti.subject_to(self.q[:, k + 1] == x_next_q)

    def integrate_euler2(self):
        for k in range(self.n):  # loop over control intervals
            x_next_q = self.q[:, k] + self.q_dot(self.q[:, k], self.u[:, k]) * self.dt
            self.opti.subject_to(self.q[:, k + 1] == x_next_q)

            x_next_b = self.b[:, k] + self.b_dot(self.b[:, k]) * self.dt
            self.opti.subject_to(self.b[:, k + 1] == x_next_b)

    def integrate_rk42(self):
        for k in range(self.n):  # loop over control intervals
            # Runge-Kutta 4 integration
            k1 = self.q_dot(self.q[:, k], self.u[:, k])
            k2 = self.q_dot(self.q[:, k] + self.dt * 0.5 * k1, self.u[:, k])
            k3 = self.q_dot(self.q[:, k] + self.dt * 0.5 * k2, self.u[:, k])
            k4 = self.q_dot(self.q[:, k] + self.dt * k3, self.u[:, k])
            x_next_q = self.q[:, k] + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self.opti.subject_to(self.q[:, k + 1] == x_next_q)

            k1_b = self.b_dot(self.b[:, k])
            k2_b = self.b_dot(self.b[:, k] + self.dt * 0.5 * k1_b)
            k3_b = self.b_dot(self.b[:, k] + self.dt * 0.5 * k2_b)
            k4_b = self.b_dot(self.b[:, k] + self.dt * k3_b)
            x_next_b = self.b[:, k] + self.dt / 6 * (k1_b + 2 * k2_b + 2 * k3_b + k4_b)
            self.opti.subject_to(self.b[:, k + 1] == x_next_b)

    def set_constraints_g(self):
        """
        Sets optimizer constraints
        """
        # Time
        self.opti_g.subject_to(self.t_g >= 0.1)

        # Initial Conditions of the bot
        # self.opti_g.set_initial(self.t_g, 2)
        self.opti_g.subject_to(self.q_g[0, 0] == self.y_i)  # initial y-position
        self.opti_g.subject_to(self.q_g[1, 0] == self.z_i)  # initial z-position
        self.opti_g.subject_to(self.q_g[2, 0] == self.y_dot_i)  # initial y-velocity
        self.opti_g.subject_to(self.q_g[3, 0] == self.z_dot_i)  # initial z-velocity

        # Initial Conditions of the ball
        self.opti_g.subject_to(self.b_g[0, 0] == self.y_b_i)  # initial y-position
        self.opti_g.subject_to(self.b_g[1, 0] == self.z_b_i)  # initial z-position
        self.opti_g.subject_to(self.b_g[2, 0] == self.y_b_dot_i)  # initial y-velocity
        self.opti_g.subject_to(self.b_g[3, 0] == self.z_b_dot_i)  # initial z-velocity

        # Final Conditions
        """
        self.pos_cf_g = self.pos_cf_w * (
            (self.q_g[0, -1] - self.b_g[0, -1]) ** 2
            + (self.q_g[1, -1] - self.b_g[1, -1]) ** 2
        )  # cost function assocaited with distance between
        """
        # self.pos_cf = self.pos_cf_w * ((self.q[0, -1] - self.b[0, -1]) ** 2 + (self.q[1, -1] - self.b[1, -1]) ** 2)
        # self.opti_g.minimize(self.t_g + self.pos_cf_g)
        self.opti_g.minimize(self.t_g)
        self.opti_g.subject_to(self.q_g[0, -1] == self.b_g[0, -1])
        self.opti_g.subject_to(self.q_g[1, -1] == self.b_g[1, -1])
        # Control Inputs

        self.u_mag_g = self.opti_g.variable(1, self.n + 1)
        self.v_mag_g = self.opti_g.variable(1, self.n + 1)
        for i in range(self.n):
            self.opti_g.subject_to(
                self.u_mag_g[0, i] == self.u_g[0, i] ** 2 + self.u_g[1, i] ** 2
            )
            self.opti_g.subject_to(
                self.opti_g.bounded(0, self.u_mag_g[0, i], self.boost_max**2)
            )  # boost mag
            self.opti_g.subject_to(
                self.v_mag_g[0, i] == self.q_g[2, i] ** 2 + self.q_g[3, i] ** 2
            )
            self.opti_g.subject_to(self.v_mag_g[0, i] <= self.v_max**2)
        # self.opti_g.subject_to(self.opti_g.bounded(-500, self.u_g[0], 500))
        # self.opti_g.subject_to(self.opti_g.bounded(-500, self.u_g[1], 500))

        # self.opti_g.subject_to(sumsqr(self.q_g[2:4]) <= self.v_max)
        # self.opti_g.subject_to(
        #    self.opti_g.bounded(-self.v_b_max, self.b_g[2:4, :], self.v_b_max)
        # )
        # self.v_mag = self.opti_g.variable(1, self.n + 1)
        # self.opti_g.subject_to(
        #     self.v_mag[0, :] == self.q_g[2, :] ** 2 + self.q_g[3, :] ** 2
        # )
        # self.opti_g.subject_to(
        #     self.opti_g.bounded(0, self.v_mag[0, :], self.v_max**2)
        # )  # velocity mag

    def guess2_g(self):
        """
        TODO
        """
        self.t_f_g = self.t_guess / 2  # guess of the final time[s]
        self.y_b_f = self.y_b_i + self.y_b_dot_i * self.t_f_g
        self.z_b_f = (
            self.y_b_i
            + self.y_b_dot_i * self.t_f_g
            + self.z_b_i
            + self.z_b_dot_i * self.t_f_g
            + 0.5 * self.g * self.t_f_g**2
        )

        for i in range(self.n + 1):
            self.opti_g.set_initial(
                self.q_g[0, i], fabs(self.y_b_f - self.y_i) / (self.n + 1 - i)
            )  # y-position guess
            self.opti_g.set_initial(
                self.q_g[1, i], fabs(self.z_b_f - self.z_i) / (self.n + 1 - i)
            )  # z-position guess

    def q_dot_g(self, q_g, u_g):
        p_g = q_g[:2]  # position of the bot in the yz world plane
        v_g = q_g[2:4]  # velocity of the bot in the yz world plane

        # v_dot = vertcat(acos(theta) * u[0], asin(theta) * u[0])
        v_dot_g = vertcat(u_g[0], u_g[1] + self.g)

        q_dot_g = vertcat(
            v_g[0],
            v_g[1],
            v_dot_g[0],
            v_dot_g[1],
        )
        return q_dot_g

    def b_dot_g(self, b):
        p_b = b[:2]  # position of the ball
        v_b = b[2:4]  # velocity of the ball

        v_dot_b = vertcat(0, self.g)  # ball velocity in z coordinate

        b_dot = vertcat(v_b[0], v_b[1], v_dot_b[0], v_dot_b[1])
        return b_dot

    def integrate_rk42_g(self):
        for k in range(self.n):  # loop over control intervals
            # Runge-Kutta 4 integration
            k1 = self.q_dot_g(self.q_g[:, k], self.u_g[:, k])
            k2 = self.q_dot_g(self.q_g[:, k] + self.dt_g * 0.5 * k1, self.u_g[:, k])
            k3 = self.q_dot_g(self.q_g[:, k] + self.dt_g * 0.5 * k2, self.u_g[:, k])
            k4 = self.q_dot_g(self.q_g[:, k] + self.dt_g * k3, self.u_g[:, k])
            x_next_q = self.q_g[:, k] + self.dt_g / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self.opti_g.subject_to(self.q_g[:, k + 1] == x_next_q)

            k1_b = self.b_dot_g(self.b_g[:, k])
            k2_b = self.b_dot_g(self.b_g[:, k] + self.dt_g * 0.5 * k1_b)
            k3_b = self.b_dot_g(self.b_g[:, k] + self.dt_g * 0.5 * k2_b)
            k4_b = self.b_dot_g(self.b_g[:, k] + self.dt_g * k3_b)
            x_next_b = self.b_g[:, k] + self.dt_g / 6 * (
                k1_b + 2 * k2_b + 2 * k3_b + k4_b
            )
            self.opti_g.subject_to(self.b_g[:, k + 1] == x_next_b)


if __name__ == "__main__":
    to = TrajectoryOptimizer()
    to.q_dot(to.q[:, 0], to.u[:, 0])

    """
    plt.figure()
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.q[0, :]), label="bot y position"
    )
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.q[1, :]), label="bot z position"
    )
    plt.legend()

    plt.figure()
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.q[2, :]), label="bot y velocity"
    )
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.q[3, :]), label="bot z velocity"
    )
    plt.legend()
    """
    """
    plt.figure()
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.b[0, :]), label="ball y position"
    )
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.b[1, :]), label="ball z position"
    )
    plt.legend()

    plt.figure()
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.b[2, :]), label="ball y velocity"
    )
    plt.plot(
        to.time * to.sol.value(to.t), to.sol.value(to.b[3, :]), label="ball z velocity"
    )
    plt.legend()
    """
    plt.figure()
    plt.plot(to.time_u * to.sol.value(to.t), to.sol.value(to.u[0, :]), label="boost")
    plt.legend()

    plt.figure()
    plt.plot(
        to.time_u * to.sol.value(to.t),
        to.sol.value(to.u[1, :]),
        label="angle acceleration input",
    )
    plt.legend()

    plt.figure()
    plt.plot(
        to.time * to.sol.value(to.t),
        to.sol.value(to.q[4, :]) * 180 / np.pi,
        label="theta",
    )
    plt.legend()
    """
    plt.figure()
    plt.plot(
        to.sol.value(to.b[0, :]),
        to.sol.value(to.b[1, :]),
        label="ball y vs z",
    )

    plt.plot(
        to.sol.value(to.q[0, :]),
        to.sol.value(to.q[1, :]),
        label="bot y vs z",
    )
    """
    radius_total = 120 + 92.75
    radius_total = to.radius_b + to.radius_r
    ball_y0 = to.sol.value(to.b[0, -1])
    ball_z0 = to.sol.value(to.b[1, -1])

    bot_y0 = to.sol.value(to.q[0, -1])
    bot_z0 = to.sol.value(to.q[1, -1])

    angle = np.linspace(0, 2 * np.pi, 360)

    bot_y = to.radius_r * cos(angle) + bot_y0
    bot_z = to.radius_r * sin(angle) + bot_z0

    ball_y = to.radius_b * cos(angle) + ball_y0
    ball_z = to.radius_b * sin(angle) + ball_z0

    """
    dot_y = 1 * sin(angle)
    dot_z = 1 * sin(angle)
    
    plt.figure()
    plt.plot(
        bot_y,
        bot_z,
        label="ball ig-model",
    )

    plt.plot(
        ball_y,
        ball_z,
        label="bot ig-model",
    )

    plt.plot(
        dot_y + ball_y0,
        ball_z0 + dot_z,
        label="bot center",
    )
    plt.plot(
        bot_y0 + dot_y,
        bot_z0 + dot_y,
        label="bot center",
    )
    axes = plt.gca()
    axes.set_aspect("equal")
    """

    plt.figure()
    plt.plot(
        to.time * to.sol.value(to.t),
        to.sol.value(to.v_mag[0, :]) ** 0.5,
        label="v_mag",
    )

    plt.figure()
    plt.plot(
        to.sol_g.value(to.b_g[0, :]),
        to.sol_g.value(to.b_g[1, :]),
        label="ball y vs z",
    )

    plt.plot(
        to.sol_g.value(to.q_g[0, :]),
        to.sol_g.value(to.q_g[1, :]),
        label="bot y vs z",
    )

    plt.plot(
        to.sol_g.value(to.b_g[0, :]),
        to.sol_g.value(to.b_g[1, :]),
        label="ball traj y vs z",
    )

    plt.plot(
        to.sol_g.value(to.q_g[0, :]),
        to.sol_g.value(to.q_g[1, :]),
        label="bot traj y vs z",
    )
    plt.title("First guess Trajectory")

    plt.figure()
    plt.plot(
        to.sol.value(to.b[0, :]),
        to.sol.value(to.b[1, :]),
        label="ball y vs z",
        color="Orange",
    )

    plt.plot(
        to.sol.value(to.q[0, :]),
        to.sol.value(to.q[1, :]),
        label="bot y vs z",
        color="Blue",
    )

    plt.plot(bot_y, bot_z, label="ball ig-model", color="Blue")

    plt.plot(ball_y, ball_z, label="bot ig-model", color="Orange")

    plt.title("Final guess Trajectory at " + str(to.theta_f * 180 / np.pi) + " degrees")
    # axes = plt.gca()
    # axes.set_aspect("equal")

    plt.figure()
    plt.plot(to.time * to.sol.value(to.t), to.sol.value(to.q[2, :]), label="car y vel")

    plt.plot(to.time * to.sol.value(to.t), to.sol.value(to.q[3, :]), label="car z vel")

    print(to.sol.value(to.b[2:4, -1]))
    print(to.sol.value(to.q[2:4, -1]))

    print(to.sol.value(to.q[2, -1]) - to.sol.value(to.b[2, -1]))
    print(to.sol.value(to.q[3, -1]) - to.sol.value(to.b[3, -1]))

    history = History()
    t = np.array(to.time * to.sol.value(to.t))
    q = np.array(to.sol.value(to.q))
    b = np.array(to.sol.value(to.b))
    u = np.array(to.sol.value(to.u))
    history.append_many_trajectory(
        t=t,
        q=q,
        b=b,
        u=u,
    )

    history.save_tj()
    np.savetxt(
        "C:/Users/Student/Documents/RLBot_IS/trajectory-optimization/data/input_data.csv",
        u,
        delimiter=", ",
        fmt="% s",
    )
    print(to.t_guess)
    print(to.sol_g.value(to.t_g))
    print(to.sol.value(to.t))

    plt.legend()
    plt.show()

    # print(to.t_f)

"""
Notes

Upper bound for t is about 5s(4.9s) if you assume you throw a ball at z = 0 where the apex is at the ceiling z = 2044.
"""
