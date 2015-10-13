# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'F:\Gilles\Universiteit\5e Master 2\MasterProef\Thesis_SoftwarePrograms\Galileo_LearningAlgorithm_v1\PC_End_Galileo_Interaction_Program_v1\QtDesign_GUIs\LoadRobotStateDialog.ui'
#
# Created: Wed Apr 08 15:21:59 2015
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

class Ui_LoadRobotDialog(object):
    def setupUi(self, LoadRobotDialog):
        LoadRobotDialog.setObjectName(_fromUtf8("LoadRobotDialog"))
        LoadRobotDialog.resize(500, 400)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(LoadRobotDialog.sizePolicy().hasHeightForWidth())
        LoadRobotDialog.setSizePolicy(sizePolicy)
        LoadRobotDialog.setMinimumSize(QtCore.QSize(400, 300))
        LoadRobotDialog.setToolTip(_fromUtf8(""))
        self.verticalLayout = QtGui.QVBoxLayout(LoadRobotDialog)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.lblSelectRobotState = QtGui.QLabel(LoadRobotDialog)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lblSelectRobotState.setFont(font)
        self.lblSelectRobotState.setTextFormat(QtCore.Qt.AutoText)
        self.lblSelectRobotState.setObjectName(_fromUtf8("lblSelectRobotState"))
        self.verticalLayout_4.addWidget(self.lblSelectRobotState)
        self.lstAvailableRobotStates = QtGui.QListWidget(LoadRobotDialog)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lstAvailableRobotStates.sizePolicy().hasHeightForWidth())
        self.lstAvailableRobotStates.setSizePolicy(sizePolicy)
        self.lstAvailableRobotStates.setMinimumSize(QtCore.QSize(300, 200))
        self.lstAvailableRobotStates.setObjectName(_fromUtf8("lstAvailableRobotStates"))
        self.verticalLayout_4.addWidget(self.lstAvailableRobotStates)
        self.verticalLayout.addLayout(self.verticalLayout_4)
        self.line = QtGui.QFrame(LoadRobotDialog)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.verticalLayout.addWidget(self.line)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.btnLoadRobotState = QtGui.QPushButton(LoadRobotDialog)
        self.btnLoadRobotState.setCheckable(False)
        self.btnLoadRobotState.setAutoDefault(True)
        self.btnLoadRobotState.setObjectName(_fromUtf8("btnLoadRobotState"))
        self.horizontalLayout.addWidget(self.btnLoadRobotState)
        spacerItem = QtGui.QSpacerItem(10, 20, QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.chkOverwriteRobotState = QtGui.QCheckBox(LoadRobotDialog)
        self.chkOverwriteRobotState.setChecked(True)
        self.chkOverwriteRobotState.setObjectName(_fromUtf8("chkOverwriteRobotState"))
        self.horizontalLayout.addWidget(self.chkOverwriteRobotState)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.btnCancel = QtGui.QPushButton(LoadRobotDialog)
        self.btnCancel.setObjectName(_fromUtf8("btnCancel"))
        self.horizontalLayout.addWidget(self.btnCancel)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.lblSelectRobotState.setBuddy(self.lstAvailableRobotStates)

        self.retranslateUi(LoadRobotDialog)
        QtCore.QObject.connect(self.btnCancel, QtCore.SIGNAL(_fromUtf8("clicked()")), LoadRobotDialog.reject)
        QtCore.QMetaObject.connectSlotsByName(LoadRobotDialog)

    def retranslateUi(self, LoadRobotDialog):
        LoadRobotDialog.setWindowTitle(_translate("LoadRobotDialog", "Load a robot state", None))
        self.lblSelectRobotState.setText(_translate("LoadRobotDialog", "&Select the desired robot state to load:", None))
        self.lstAvailableRobotStates.setToolTip(_translate("LoadRobotDialog", "Choose the robot state to load", None))
        self.btnLoadRobotState.setToolTip(_translate("LoadRobotDialog", "Loads the selected state in the robot and updates the GUI parameters", None))
        self.btnLoadRobotState.setText(_translate("LoadRobotDialog", "&Load", None))
        self.chkOverwriteRobotState.setText(_translate("LoadRobotDialog", "&Overwrite optimization state", None))
        self.btnCancel.setToolTip(_translate("LoadRobotDialog", "Cancel loading", None))
        self.btnCancel.setText(_translate("LoadRobotDialog", "Cancel", None))

