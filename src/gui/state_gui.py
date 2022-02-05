#TODO: Fix paths
import sys
sys.path.append('C:/Users/Student/Documents/RLBot_IS/trajectory-optimization')

from PyQt5 import QtWidgets
import sys
import numpy as np

from gui import Ui_Dialog
from src.robot.state import State
from src.communications.client import Client
from src.communications.comms_protocol import CommsProtocol

class StateGUI(Ui_Dialog):
    def __init__(self):
        #super(StateGUI, self).__init__()
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
            phi_dot = float(self.lineEdit_7.text())
            theta_dot = float(self.lineEdit_8.text())
            psi_dot = float(self.lineEdit_9.text())
            e0 = float(self.lineEdit_10.text())
            e1 = float(self.lineEdit_11.text())
            e2 = float(self.lineEdit_12.text())
            e3 = float(self.lineEdit_13.text())
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
        self.Save.accepted.connect(send_initial_state)


class ApplicationWindow(QtWidgets.QDialog):
    def __init__(self):
        super(ApplicationWindow, self).__init__()

        self.ui = StateGUI()
        self.ui.setupUi(self)

def main():
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()