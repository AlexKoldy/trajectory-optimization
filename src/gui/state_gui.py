# TODO: Fix paths
import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from gui import Ui_Dialog
from src.robot.state import State
from src.communications.client import Client
from src.communications.comms_protocol import CommsProtocol
from src.utilities.lin_alg_utils import LinAlgUtils as lau


class StateGUI(Ui_Dialog):
    def __init__(self):
        pass

    def setupUi(self, Dialog):
        super().setupUi(Dialog)

        def send_initial_state():
            """
            Sends the game state specified in the GUI via the client
            """
            x = float(self.x.text())
            y = float(self.y.text())
            z = float(self.z.text())
            x_dot = float(self.x_dot.text())
            y_dot = float(self.y_dot.text())
            z_dot = float(self.z_dot.text())
            phi = float(self.phi.text())
            theta = float(self.theta.text())
            psi = float(self.psi.text())
            phi_dot = float(self.phi_dot.text())
            theta_dot = float(self.theta_dot.text())
            psi_dot = float(self.psi_dot.text())
            g = float(self.g.text())
            e0_des = float(self.e0_des.text())
            e1_des = float(self.e1_des.text())
            e2_des = float(self.e2_des.text())
            e3_des = float(self.e3_des.text())
            P_quat = float(self.P_quat.text())
            P_omega = float(self.P_omega.text())
            e0, e1, e2, e3 = lau.euler_to_quaternion(phi=phi, theta=theta, psi=psi)
            q_bot = State(
                x=x,
                y=y,
                z=z,
                x_dot=x_dot,
                y_dot=y_dot,
                z_dot=z_dot,
                e0=e0,
                e1=e1,
                e2=e2,
                e3=e3,
                phi_dot=phi_dot,
                theta_dot=theta_dot,
                psi_dot=psi_dot,
            )
            quat_des = np.array([e0_des, e1_des, e2_des, e3_des])
            controller_coefficients = [P_omega, P_quat]
            c = Client()
            data = []
            data.append(np.array2string(q_bot()))
            data.append(np.array2string(quat_des))
            data.append(controller_coefficients)
            data.append(str(g))
            data = "".join(map(str, data))
            c.send_message(CommsProtocol.types["modify state"], data)

        self.send_button.clicked.connect(send_initial_state)
