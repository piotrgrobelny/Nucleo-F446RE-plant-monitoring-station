from PyQt5 import QtWidgets, QtCore
import gui
import read_data

class MainWindow(QtWidgets.QWidget):
    def __init__(self, parent = None):
        QtWidgets.QWidget.__init__(self, parent)
        self.ui = gui.Ui_Dialog()
        self.ui.setupUi(self)
        #timer which counts time to update data, set for one second (1000ms)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateData)
        self.timer.start(1000)

    #update values from sensors
    def updateData(self):
        data = read_data.read_string()
        self.ui.lcdNumber.display(data[1]) #temperature
        self.ui.progressBar_3.setValue(int(data[2]+20)) #light inensity but sensor doesn't work properly
        self.ui.progressBar.setValue(int(data[0]))  #air humidity
        self.ui.progressBar_2.setValue(int(data[3])) #soil humidity

if __name__ == "__main__":
    read_data.save_data()
    import sys
    app = QtWidgets.QApplication(sys.argv)
    c = MainWindow()
    c.show()
    sys.exit(app.exec_())