# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(400, 319)
        self.Save = QtWidgets.QDialogButtonBox(Dialog)
        self.Save.setGeometry(QtCore.QRect(50, 270, 341, 32))
        self.Save.setOrientation(QtCore.Qt.Horizontal)
        self.Save.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.Save.setObjectName("Save")
        self.lineEdit = QtWidgets.QLineEdit(Dialog)
        self.lineEdit.setGeometry(QtCore.QRect(60, 40, 113, 20))
        self.lineEdit.setObjectName("lineEdit")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(20, 10, 121, 21))
        self.label.setObjectName("label")
        self.lineEdit_2 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_2.setGeometry(QtCore.QRect(60, 60, 113, 20))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_3 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_3.setGeometry(QtCore.QRect(60, 80, 113, 20))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.lineEdit_4 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_4.setGeometry(QtCore.QRect(60, 100, 113, 20))
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.lineEdit_5 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_5.setGeometry(QtCore.QRect(60, 120, 113, 20))
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.lineEdit_6 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_6.setGeometry(QtCore.QRect(60, 140, 113, 20))
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.lineEdit_8 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_8.setGeometry(QtCore.QRect(60, 280, 113, 20))
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.lineEdit_7 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_7.setGeometry(QtCore.QRect(60, 160, 113, 20))
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.lineEdit_9 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_9.setGeometry(QtCore.QRect(60, 180, 113, 20))
        self.lineEdit_9.setObjectName("lineEdit_9")
        self.lineEdit_10 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_10.setGeometry(QtCore.QRect(60, 240, 113, 20))
        self.lineEdit_10.setObjectName("lineEdit_10")
        self.lineEdit_11 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_11.setGeometry(QtCore.QRect(60, 200, 113, 20))
        self.lineEdit_11.setObjectName("lineEdit_11")
        self.lineEdit_12 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_12.setGeometry(QtCore.QRect(60, 220, 113, 20))
        self.lineEdit_12.setObjectName("lineEdit_12")
        self.lineEdit_13 = QtWidgets.QLineEdit(Dialog)
        self.lineEdit_13.setGeometry(QtCore.QRect(60, 260, 113, 20))
        self.lineEdit_13.setObjectName("lineEdit_13")
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(10, 40, 47, 21))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(10, 60, 47, 21))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(10, 80, 47, 21))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setGeometry(QtCore.QRect(10, 100, 47, 21))
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(Dialog)
        self.label_6.setGeometry(QtCore.QRect(10, 120, 47, 21))
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(Dialog)
        self.label_7.setGeometry(QtCore.QRect(10, 140, 47, 21))
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(Dialog)
        self.label_8.setGeometry(QtCore.QRect(10, 200, 47, 21))
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(Dialog)
        self.label_9.setGeometry(QtCore.QRect(10, 160, 47, 21))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(Dialog)
        self.label_10.setGeometry(QtCore.QRect(10, 180, 47, 21))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(Dialog)
        self.label_11.setGeometry(QtCore.QRect(10, 260, 47, 21))
        self.label_11.setObjectName("label_11")
        self.label_12 = QtWidgets.QLabel(Dialog)
        self.label_12.setGeometry(QtCore.QRect(10, 220, 47, 21))
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(Dialog)
        self.label_13.setGeometry(QtCore.QRect(10, 240, 47, 21))
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(Dialog)
        self.label_14.setGeometry(QtCore.QRect(10, 280, 47, 21))
        self.label_14.setObjectName("label_14")

        self.retranslateUi(Dialog)
        self.Save.accepted.connect(Dialog.accept) # type: ignore
        self.Save.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.lineEdit.setText(_translate("Dialog", "0"))
        self.label.setText(_translate("Dialog", "Intial State of the Robot"))
        self.lineEdit_2.setText(_translate("Dialog", "0"))
        self.lineEdit_3.setText(_translate("Dialog", "0"))
        self.lineEdit_4.setText(_translate("Dialog", "0"))
        self.lineEdit_5.setText(_translate("Dialog", "0"))
        self.lineEdit_6.setText(_translate("Dialog", "0"))
        self.lineEdit_8.setText(_translate("Dialog", "0"))
        self.lineEdit_7.setText(_translate("Dialog", "0"))
        self.lineEdit_9.setText(_translate("Dialog", "0"))
        self.lineEdit_10.setText(_translate("Dialog", "0"))
        self.lineEdit_11.setText(_translate("Dialog", "0"))
        self.lineEdit_12.setText(_translate("Dialog", "0"))
        self.lineEdit_13.setText(_translate("Dialog", "0"))
        self.label_2.setText(_translate("Dialog", "Pos X"))
        self.label_3.setText(_translate("Dialog", "Pos Y"))
        self.label_4.setText(_translate("Dialog", "Pos Z"))
        self.label_5.setText(_translate("Dialog", "Vel X"))
        self.label_6.setText(_translate("Dialog", "Vel Y"))
        self.label_7.setText(_translate("Dialog", "Vel Z"))
        self.label_8.setText(_translate("Dialog", "Yaw"))
        self.label_9.setText(_translate("Dialog", "Roll"))
        self.label_10.setText(_translate("Dialog", "Pitch"))
        self.label_11.setText(_translate("Dialog", "c"))
        self.label_12.setText(_translate("Dialog", "a"))
        self.label_13.setText(_translate("Dialog", "b"))
        self.label_14.setText(_translate("Dialog", "d"))
