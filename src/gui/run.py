import sys
sys.path.append('C:/Users/Student/Documents/RLBot_IS/trajectory-optimization')

from PyQt5 import QtWidgets

from application_window import ApplicationWindow

def main():
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    app.exec_()

if __name__ == "__main__":
    while(True):
        main()