
from PyQt5 import QtCore, QtGui, QtWidgets

#GUI designed in QtDesigner
class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(406, 316)
        Dialog.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.progressBar = QtWidgets.QProgressBar(Dialog)
        self.progressBar.setGeometry(QtCore.QRect(180, 10, 211, 61))
        self.progressBar.setStyleSheet("")
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.lcdNumber = QtWidgets.QLCDNumber(Dialog)
        self.lcdNumber.setGeometry(QtCore.QRect(180, 250, 171, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.lcdNumber.setFont(font)
        self.lcdNumber.setStyleSheet("")
        self.lcdNumber.setObjectName("lcdNumber")
        self.progressBar_2 = QtWidgets.QProgressBar(Dialog)
        self.progressBar_2.setGeometry(QtCore.QRect(180, 90, 211, 61))
        self.progressBar_2.setProperty("value", 24)
        self.progressBar_2.setObjectName("progressBar_2")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(360, 270, 61, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(10, 20, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(10, 100, 161, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setGeometry(QtCore.QRect(10, 260, 161, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.progressBar_3 = QtWidgets.QProgressBar(Dialog)
        self.progressBar_3.setGeometry(QtCore.QRect(180, 170, 211, 61))
        self.progressBar_3.setProperty("value", 24)
        self.progressBar_3.setObjectName("progressBar_3")
        self.label_6 = QtWidgets.QLabel(Dialog)
        self.label_6.setGeometry(QtCore.QRect(10, 180, 161, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label.setText(_translate("Dialog", "C°"))
        self.label_3.setText(_translate("Dialog", "Air humidity"))
        self.label_4.setText(_translate("Dialog", "Soil humidity"))
        self.label_5.setText(_translate("Dialog", "Temperature"))
        self.label_6.setText(_translate("Dialog", "Lighting"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())


