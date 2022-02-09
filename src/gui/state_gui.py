#TODO: Fix paths
import sys
sys.path.append('C:/Users/Student/Documents/RLBot_IS/trajectory-optimization')

import numpy as np

from gui import Ui_Dialog
from src.robot.state import State
from src.communications.client import Client
from src.communications.comms_protocol import CommsProtocol

class StateGUI(Ui_Dialog):
    def __init__(self):
        pass
    def setupUi(self, Dialog):
        super().setupUi(Dialog)

        def send_initial_state():
            x = float(self.lineEdit.text())
            y = float(self.lineEdit_2.text())
            z = float(self.lineEdit_3.text())
            x_dot = float(self.lineEdit_4.text())
            y_dot = float(self.lineEdit_5.text())
            z_dot = float(self.lineEdit_6.text())
            phi = float(self.lineEdit_7.text())
            theta = float(self.lineEdit_8.text())
            psi = float(self.lineEdit_9.text())
            phi_dot = float(self.lineEdit_10.text())
            theta_dot = float(self.lineEdit_11.text())
            psi_dot = float(self.lineEdit_12.text())
            print(x)
            print(y)
            print(z)
            print(x_dot)
            print(y_dot)
            print(z_dot)
            print(phi)
            print(theta)
            print(psi)
            print(phi_dot)
            print(theta_dot)
            print(psi_dot)
            #e0, e1, e2, e3 = blah blah
            '''
            q = State(
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
            
            c = Client()
            c.send_message(CommsProtocol.types["initialize state"], np.array2string(q()))      
            ''' 
        self.sendButton.clicked.connect(send_initial_state)

