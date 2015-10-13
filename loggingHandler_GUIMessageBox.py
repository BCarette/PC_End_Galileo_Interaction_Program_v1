import logging
import time
from bluetoothSocketPC import BluetoothSocketPC
from PyQt4.QtCore import *
from modelPC import ModelPC

class LoggingHandler_GUIMessageBox(logging.Handler):
    """
    Handler to log messages to the robot control Main window message box
    
    :var modelPCLock: Lock, to avoid concurrent use of the modelPC object
    :var modelPC: ModelPC, contains updateGUIMessageBox: SIGNAL, (Message to print, logging level, isPCLogMessage)
    """

    def __init__(self, parent = None):
        logging.Handler.__init__(self)
        # signal to emit to notify GUI is found in helper class, because a logging.Handler overloads the emit method
        self.signalHolder = LoggingHandler_SignalHolder()

    def emit(self, record):
        # record.message is the log message
        self.signalHolder.emit(self.signalHolder.updateGUIMessageBox, "{0}::({1},{2})::{3}".format(record.levelname, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(record.created)), int(record.created % 1 * 1000), record.getMessage()),
                  record.levelno, True)

#endClass LoggingHandler_GUIMessageBox

class LoggingHandler_SignalHolder(QObject):
    """Helper class for LoggingHandler_GUIMessageBox, stores its signals"""

    def __init__(self, parent = None):
        super(LoggingHandler_SignalHolder, self).__init__(parent)
        self.updateGUIMessageBox = SIGNAL("updateGUIMessageBox(QString, int, bool)")