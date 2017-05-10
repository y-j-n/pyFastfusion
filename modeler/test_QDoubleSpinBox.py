#!/usr/bin/python
# -*- coding: utf-8 -*-

# http://stackoverflow.com/questions/20898897/label-displays-sum-of-two-qspinbox-python-pyside

import sys
from PySide import QtGui, QtCore

class Example(QtGui.QWidget):

    def __init__(self):
        super(Example, self).__init__()

        self.initUI()

    def initUI(self):

        #Add all GUI Elements to Class
        self.amountLabel = QtGui.QLabel('Amount')
        self.counterLabel = QtGui.QLabel('Counter')
        self.totalLabel = QtGui.QLabel('Total')
        self.amountSpin = QtGui.QSpinBox()
        self.counterSpin = QtGui.QSpinBox()

        self.totalOutput = QtGui.QLabel('0')

        grid = QtGui.QGridLayout()
        grid.setSpacing(0)

        grid.addWidget(self.amountLabel, 3, 0)
        grid.addWidget(self.counterLabel, 3, 1)
        grid.addWidget(self.totalLabel, 3, 2)

        grid.addWidget(self.amountSpin, 4, 0)
        grid.addWidget(self.counterSpin, 4, 1)
        grid.addWidget(self.totalOutput, 4, 2)

        self.setLayout(grid)

        # ACTIONS
        self.amountSpin.valueChanged[str].connect(self.onChanged)
        self.counterSpin.valueChanged[str].connect(self.onChanged)

        self.setGeometry(800, 400, 250, 80)
        self.setWindowTitle('Simple Calculator')
        self.show()

    def onChanged(self, val):
        #we ignore the val and just get the values directly from our spinboxes
        sum = self.amountSpin.value() + self.counterSpin.value()
        #and display them
        self.totalOutput.setText(str(sum))
        self.totalOutput.adjustSize()

def main():

    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()