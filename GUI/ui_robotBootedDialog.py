# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'F:\Gilles\Universiteit\5e Master 2\MasterProef\Thesis_SoftwarePrograms\Galileo_LearningAlgorithm_v1\PC_End_Galileo_Interaction_Program_v1\QtDesign_GUIs\RobotBootedDialog.ui'
#
# Created: Tue Jun  9 16:01:49 2015
#      by: PyQt4 UI code generator 4.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_dialogRobotBooted(object):
    def setupUi(self, dialogRobotBooted):
        dialogRobotBooted.setObjectName(_fromUtf8("dialogRobotBooted"))
        dialogRobotBooted.resize(400, 300)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(dialogRobotBooted.sizePolicy().hasHeightForWidth())
        dialogRobotBooted.setSizePolicy(sizePolicy)
        dialogRobotBooted.setMinimumSize(QtCore.QSize(400, 300))
        self.verticalLayout = QtGui.QVBoxLayout(dialogRobotBooted)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.lblMessage = QtGui.QLabel(dialogRobotBooted)
        self.lblMessage.setObjectName(_fromUtf8("lblMessage"))
        self.verticalLayout.addWidget(self.lblMessage)
        self.cmbtnUseRobotParams = QtGui.QCommandLinkButton(dialogRobotBooted)
        self.cmbtnUseRobotParams.setAutoDefault(False)
        self.cmbtnUseRobotParams.setObjectName(_fromUtf8("cmbtnUseRobotParams"))
        self.verticalLayout.addWidget(self.cmbtnUseRobotParams)
        self.cmbtnUsePCParameters = QtGui.QCommandLinkButton(dialogRobotBooted)
        self.cmbtnUsePCParameters.setDefault(True)
        self.cmbtnUsePCParameters.setObjectName(_fromUtf8("cmbtnUsePCParameters"))
        self.verticalLayout.addWidget(self.cmbtnUsePCParameters)
        self.cmbtnLoadRobotState = QtGui.QCommandLinkButton(dialogRobotBooted)
        self.cmbtnLoadRobotState.setAutoDefault(False)
        self.cmbtnLoadRobotState.setObjectName(_fromUtf8("cmbtnLoadRobotState"))
        self.verticalLayout.addWidget(self.cmbtnLoadRobotState)

        self.retranslateUi(dialogRobotBooted)
        QtCore.QMetaObject.connectSlotsByName(dialogRobotBooted)

    def retranslateUi(self, dialogRobotBooted):
        dialogRobotBooted.setWindowTitle(_translate("dialogRobotBooted", "PRACAM-Globe just booted", None))
        self.lblMessage.setText(_translate("dialogRobotBooted", "<html><head/><body><p><span style=\" font-size:12pt;\">The robot just booted.</span></p><p><span style=\" font-size:12pt;\">Which parameters would you want to use?</span></p></body></html>", None))
        self.cmbtnUseRobotParams.setText(_translate("dialogRobotBooted", "Robot parameters", None))
        self.cmbtnUsePCParameters.setText(_translate("dialogRobotBooted", "PC parameters", None))
        self.cmbtnLoadRobotState.setText(_translate("dialogRobotBooted", "Load a saved robot state", None))

