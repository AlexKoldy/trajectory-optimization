import sys
sys.path.append('C:/Users/Student/Documents/RLBot_IS/trajectory-optimization')

from PyQt5 import QtWidgets

from state_gui import StateGUI

class ApplicationWindow(QtWidgets.QDialog):
    def __init__(self):
        super(ApplicationWindow, self).__init__()
        self.ui = StateGUI()
        self.ui.setupUi(self)