from casadi import *
import matplotlib.pyplot as plt

# from pylab import plot, step, figure, legend, show, spy, quiver, scatter


class TrajectoryOptimizer:
    def __init__(self):
        self.opti = Opti()  # optimizer

        # Initial Conditions
        # Bot
        self.t_i = 0  # initial time [s]
        self.y_i = -1000  # initial y-position [uu]
        self.z_i = 0  # initial z-position [uu]
        self.y_dot_i = 0  # initial y-velocity [uu/s]
        self.z_dot_i = 0  # initial z-velocity [uu/s]
        self.theta_i = np.pi / 2  # initial pitch [rad]
        # self.e0_i = 0.00001  # initial x component of orientation quaternion
        # self.e1_i = 0.00001  # initial y component of orientation quaternion
        # self.e2_i = 0.00001  # initial z component of orientation quaternion
        # self.e3_i = 1  # initial w component of orientation quaternion
        self.theta_dot_i = 0  # initial pitch rate [rad/s]

        # Enviornment
        self.g = -650  # acceleration due to gravity [uu/s^2]

        # Ball
        self.y_b_i = 1000  # initial y-position [uu]
        self.z_b_i = 250  # initial z-position [uu]
        self.y_b_dot_i = -200  # initial y-velocity [uu/s]
        self.z_b_dot_i = 5000  # initial z-velocity [uu/s]
        self.theta_b_i = 0  # initial pitch [rad]
        self.theta_dot_b_i = 0  # initial pitch rate [rad/s]

        # Final Conditions
        """
        # Bot
        self.y_f = 1000  # desired final y-position [uu]
        self.z_f = 500  # desired final z-position [uu]
        self.y_dot_f = 0  # desired final y-velocity [uu/s]
        self.z_dot_f = 0  # desired final z-velocity [uu/s]
        self.theta_f = 0  # final desired pitch rate [rad/s]
        self.theta_dot_f = 0  # final desired pitch rate [rad/s]
        """

        # Max Values
        self.v_max = 2300  # maxium velocity [uu/s]
        self.omega_max = 5.5  # maximum angular velocity [rad/s]
        self.theta_ddot_max = 12.46  # maximum pitch acceleration [rad/s]
        self.boost_max = 915.666  # maximum boost acceleration [rad/s]
        self.throttle_max = 66.67  # maximum throttle acceleration [uu/s^2]

        self.v_b_max = 6000  # Maxium velocity [uu/s]

        # solver tuning variables
        self.pos_e_w = 1e-2
        self.t_f = 12  # guess of the final time[s]
        """
        self.c = ((self.y_i - self.y_b_i) ** 2 + (self.z_i - self.z_b_i) ** 2) ** 0.5
        self.d = (
            (self.y_dot_i - self.y_b_dot_i) ** 2 + (self.z_dot_i - self.z_b_dot_i) ** 2
        )
        # self.a = ((self.boost_max - self.g) ** 2 + self.g**2) ** 0.5
        self.a = self.boost_max
        self.guess_factor = -0.9
        self.t_f = (
            self.guess_factor
            * (-self.d - (self.d**2 - 4 * self.a * self.c) ** 0.5)
            / (2 * self.a)
        )  # guess of the final time[s]
        """
        self.y_b_f = self.y_b_i + self.y_b_dot_i * self.t_f
        self.z_b_f = (
            self.z_b_i + self.z_b_dot_i * self.t_f + 0.5 * self.g * self.t_f**2
        )

        # dimensions
        self.radius_b = 92.75  # ball radius [uu]

        # solver variables
        self.n = 50  # number of timesteps
        self.q = self.opti.variable(6, self.n + 1)  # state
        self.b = self.opti.variable(4, self.n + 1)  # state of ball
        self.u = self.opti.variable(2, self.n)  # inputs (roll_acceleration, omega)
        self.t = self.opti.variable()  # time
        self.dt = self.t / self.n
        self.time = np.linspace(0, 1, self.n + 1)
        self.time_u = np.linspace(0, 1, self.n)
        self.set_constraints()
        self.guess2()

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

        # Final Conditions
        # minimize
        """
        self.opti.subject_to((self.q[0, -1] == self.b[0, -1]))  # final y-position
        self.opti.subject_to((self.q[1, -1] == self.b[1, -1]))  # final z-position
        self.opti.minimize(self.t)
        """
        self.opti.minimize(
            self.t
            + self.pos_e_w
            * (
                (self.q[0, -1] - self.b[0, -1]) ** 2
                + (self.q[1, -1] - self.b[1, -1]) ** 2
            )
        )

        # self.opti.subject_to(self.q[2, -1] == self.y_dot_f)  # final y-velocity
        # self.opti.subject_to(self.q[3, -1] == self.z_dot_f)  # final z-velocity
        # self.opti.subject_to(self.q[4, -1] == self.z_f)  # final z-position
        # self.opti.subject_to(self.q[5, -1] == self.theta_dot_f)  # final pitch rate
        # Control Inputs
        self.opti.subject_to(
            self.opti.bounded(0, self.u[0, :], self.boost_max)
        )  # boost acceleration
        self.opti.subject_to(
            self.opti.bounded(-self.theta_ddot_max, self.u[1, :], self.theta_ddot_max)
        )  # pitch acceleration

        # Limiting Conditions
        self.opti.subject_to(
            self.opti.bounded(-self.omega_max, self.q[5, :], self.omega_max)
        )
        self.opti.subject_to(self.opti.bounded(-self.v_max, self.q[2:4, :], self.v_max))
        self.opti.subject_to(
            self.opti.bounded(-self.v_b_max, self.b[2:4, :], self.v_b_max)
        )

    def guess(self):
        """
        TODO
        """
        for i in range(self.n + 1):
            self.opti.set_initial(
                self.q[0, i], fabs(self.y_f - self.y_i) / (self.n + 1 - i)
            )  # y-position guess
            self.opti.set_initial(
                self.q[1, i], fabs(self.z_f - self.z_i) / (self.n + 1 - i)
            )  # z-position guess

    def guess2(self):
        """
        TODO
        """
        for i in range(self.n + 1):
            self.opti.set_initial(
                self.q[0, i], fabs(self.y_b_f - self.y_i) / (self.n + 1 - i)
            )  # y-position guess
            self.opti.set_initial(
                self.q[1, i], fabs(self.z_b_f - self.z_i) / (self.n + 1 - i)
            )  # z-position guess

    def q_dot(self, q, u):
        T_theta = 12.14599781908070  # torque coefficient for pitch

        D_theta = -2.798194258050845  # drag coefficient for pitch

        p = q[:2]  # position of the bot in the yz world plane
        v = q[2:4]  # velocity of the bot in the yz world plane
        theta = q[4]  # pitch of the bot
        omega = q[5]  # angular velocity of the bot

        omega_dot = u[1] * T_theta
        theta_dot = omega * D_theta  # uneeded
        # v_dot = vertcat(acos(theta) * u[0], asin(theta) * u[0])
        v_dot = vertcat(cos(theta) * u[0], sin(theta) * u[0] + self.g)

        q_dot = vertcat(
            v[0],
            v[1],
            v_dot[0],
            v_dot[1],
            theta_dot,
            omega_dot,
        )
        return q_dot

    def b_dot(self, b):
        p_b = b[:2]  # position of the ball
        v_b = b[2:4]  # velocity of the ball

        v_dot_b = vertcat(0, self.g)  # ball velocity in z coordinate

        b_dot = vertcat(v_b[0], v_b[1], v_dot_b[0], v_dot_b[1])
        return b_dot

    def integrate_euler(self):
        for k in range(self.n):  # loop over control intervals
            x_next_q = self.q[:, k] + self.q_dot(self.q[:, k], self.u[:, k]) * self.dt
            self.opti.subject_to(self.q[:, k + 1] == x_next_q)

    def integrate_rk4(self):
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

    def getTheta(e0, e1, e2, e3):
        t2 = +2.0 * (e3 * e1 - e2 * e0)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        theta = np.arcsin(t2)
        return theta


if __name__ == "__main__":
    to = TrajectoryOptimizer()
    # to.q_dot(to.q[:, 0], to.u[:, 0])

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

    # print(to.sol.value(to.b))
    print(to.t_f)

    plt.legend()
    plt.show()
