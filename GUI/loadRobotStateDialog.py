from PyQt4.QtCore import *
from PyQt4.QtGui import *
import ui_loadRobotStateDialog
from settingsPC import SettingsPC

class LoadRobotStateDialog(QDialog, ui_loadRobotStateDialog.Ui_LoadRobotDialog):
    """This is a dialog screen to choose a robot state to load"""

    def __init__(self, bluetoothLock, bluetoothSocket, logger, parent=None):
        super(LoadRobotStateDialog, self).__init__(parent)
        self.setupUi(self)
        self.RobotStatesReceived = False
        self.bluetoothLock = bluetoothLock
        self.bluetoothSocket = bluetoothSocket
        self.logger = logger
        
        loadingWidget = QListWidgetItem("... Loading robot states ...", parent=self.lstAvailableRobotStates)
        loadingWidget.setTextAlignment(Qt.AlignHCenter)
        # set Loading not enabled until RobotStatesReceived and an item selected
        self.btnLoadRobotState.setEnabled(False)
        self.btnCancel.setDefault(True)

        # connect receiving signal
        self.connect(bluetoothSocket, self.bluetoothSocket.availableRobotStateToLoadReceived, self.ReceiveAvailableRobotStates)
        
        
    @pyqtSlot("bool")
    def on_btnLoadRobotState_clicked(self, pressed):
        selectedItem = self.lstAvailableRobotStates.selectedItems()
        if len(selectedItem) == 0:
            self.logger.error("LoadrobotStateDialog:on_btnLoadRobotState_clicked: No item selected!")
            return
        self.logger.debug("Loading: {0} with overwrite = {1}".format(str(selectedItem[0].text()), self.chkOverwriteRobotState.isChecked()))

        self.bluetoothLock.acquire()
        # BT: call robot to load selected state
        self.bluetoothSocket.SendMessage(SettingsPC.BT_loadRobotState, {"filename": str(selectedItem[0].text()), "overwriteState": self.chkOverwriteRobotState.isChecked()})
        # BT: call robot to upload its new state
        self.bluetoothSocket.DownloadStatus(SettingsPC.AvailableRobotParametersToDownload)
        self.bluetoothLock.release()

        #  close dialog
        self.accept()

    @pyqtSlot()
    def on_lstAvailableRobotStates_itemSelectionChanged(self):
        if self.RobotStatesReceived:
            # once an item selected: make loading available:
            self.btnCancel.setDefault(False)
            self.btnLoadRobotState.setEnabled(True)
            self.btnLoadRobotState.setDefault(True)

    #@pyqtSlot(list)
    def ReceiveAvailableRobotStates(self, availableRobotStatesList):
        "Slot to handle received robotstate list"
        self.lstAvailableRobotStates.clear()

        # add all listitems to the listwidget
        for i in range(0, len(availableRobotStatesList)):
            QListWidgetItem(availableRobotStatesList[i], self.lstAvailableRobotStates)
        self.lstAvailableRobotStates.sortItems(order= Qt.DescendingOrder)
        self.RobotStatesReceived = True
        self.btnLoadRobotState.setEnabled(False)
