import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import matplotlib.pyplot as plt

from src.utilities.history import History


class Plotter(History):
    def __init__(self):
        load_data = self.load()

        self.t_history = load_data[0]

        self.x_history = load_data[1]
        self.y_history = load_data[2]
        self.z_history = load_data[3]
        self.x_dot_history = load_data[4]
        self.y_dot_history = load_data[5]
        self.z_dot_history = load_data[6]
        self.e0_history = load_data[7]
        self.e1_history = load_data[8]
        self.e2_history = load_data[9]
        self.e3_history = load_data[10]
        self.phi_dot_history = load_data[11]
        self.theta_dot_history = load_data[12]
        self.psi_dot_history = load_data[13]

        self.x_m_history = load_data[14]
        self.y_m_history = load_data[15]
        self.z_m_history = load_data[16]
        self.x_dot_m_history = load_data[17]
        self.y_dot_m_history = load_data[18]
        self.z_dot_m_history = load_data[19]
        self.e0_m_history = load_data[20]
        self.e1_m_history = load_data[21]
        self.e2_m_history = load_data[22]
        self.e3_m_history = load_data[23]
        self.phi_dot_m_history = load_data[24]
        self.theta_dot_m_history = load_data[25]
        self.psi_dot_m_history = load_data[26]

    def __call__(self):
        plt.figure()
        plt.plot(self.t_history, self.x_history, color="red", label="x_bot")
        plt.plot(self.t_history, self.y_history, color="green", label="y_bot")
        plt.plot(self.t_history, self.z_history, color="blue", label="z_bot")

        plt.plot(
            self.t_history,
            self.x_m_history,
            color="red",
            linestyle="dashed",
            label="x_model",
        )

        plt.plot(
            self.t_history,
            self.y_m_history,
            color="green",
            linestyle="dashed",
            label="y_model",
        )

        plt.plot(
            self.t_history,
            self.z_m_history,
            color="blue",
            linestyle="dashed",
            label="z_model",
        )

        plt.xlim(5.25, 10)
        plt.ylim(0, 1000)
        plt.legend()
        plt.show()


if __name__ == "__main__":
    plotter = Plotter()
    plotter()
