from PyQt4.QtCore import *
from PyQt4.QtGui import *
import ui_robotBootedDialog
from settingsPC import SettingsPC
from loadRobotStateDialog import LoadRobotStateDialog

class RobotBootedDialog(QDialog, ui_robotBootedDialog.Ui_dialogRobotBooted):
    """This is a dialog screen, shown when the robot boots to choose which parameters to use"""

    def __init__(self, bluetoothLock, bluetoothSocket, modelPCLock, modelPC, logger, parent = None):
        super(RobotBootedDialog, self).__init__(parent)
        self.setupUi(self)

        self.logger = logger
        self.bluetoothLock = bluetoothLock
        self.bluetoothSocket = bluetoothSocket
        self.modelPCLock = modelPCLock
        self.modelPC = modelPC

    #endFct __init__

    @pyqtSlot("bool")
    def on_cmbtnUseRobotParams_clicked(self, pressed):
        self.logger.debug("RobotBootedDialog: User chooses to use the parameters of robot.")
        # ask robot to upload its parameters:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.DownloadStatus(SettingsPC.AvailableRobotParametersToDownload)
        self.bluetoothLock.release()
        self.done(3)


    @pyqtSlot("bool")
    def on_cmbtnUsePCParameters_clicked(self, pressed):
        self.logger.debug("RobotBootedDialog: User chooses to use the parameters of PC.")
        # filter parameters out of modelPC
        self.modelPCLock.acquire()
        modelPCDict = self.modelPC.__dict__
        self.modelPCLock.release()
        # NOTE: PSOstate is not integrated with gui
        parametersToSetDict = {variableName: modelPCDict[variableName] for variableName in SettingsPC.AllowedParametersToSet}  

        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(parametersToSetDict.keys(), parametersToSetDict.values())
        self.bluetoothLock.release()
        self.done(4)

    @pyqtSlot("bool")
    def on_cmbtnLoadRobotState_clicked(self, pressed):
        # request the available robot states to load
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SendMessage(SettingsPC.BT_getAvailableRobotStates, "")
        self.bluetoothLock.release()
        # wait in the QDialog for the up-to-date loadable robot states
        dialogWindow = LoadRobotStateDialog(self.bluetoothLock, self.bluetoothSocket, self.logger, self)
        dialogWindow.show()
        res = dialogWindow.exec_()

        if res == 0:
            # user did not select to load a robot state
            return  # let user rechoose its action
        else:
            # robot is loading a new state
            self.logger.debug("RobotBootedDialog: User chooses to load a saved robotstate.")
            self.done(5)  # return to mainwindow

    #endFct on_cmbtnLoadRobotState_clicked
