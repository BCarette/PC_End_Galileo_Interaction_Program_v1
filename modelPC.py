import threading
import logging
from TrackingProperties import TrackingProperties
from numpy import *
from PyQt4.QtCore import *
import time
import sys
from settingsPC import SettingsPC


class ModelPC(QObject):
    """description of class"""
    # shared data
    # GUI custom signals
    #GuiProgressBarValueChanged = pyqtSignal(int, name="guiProgressBarValueChanged")
    

    def __init__(self, logger, bluetoothLock, bluetoothSocket, trackingPropertiesLock, modelPCLock, GUIfinishedEvent, stopTrackingEvent, guiInitedEvent, loggingSignalHolder, parent = None):
        #threading.Thread.__init__(self)
        super(ModelPC, self).__init__(parent)
        
        # thread inits
        self.logger = logger
        self.bluetoothLock = bluetoothLock
        self.bluetoothSocket = bluetoothSocket
        self.trackingPropertiesLock = trackingPropertiesLock
        self.GUIfinishedEvent = GUIfinishedEvent
        self.modelPCLock = modelPCLock
        self.guiInitedEvent = guiInitedEvent

        # logging signal connection variables:
        self.loggingSignalHolder = loggingSignalHolder

        # create signals:
        self.changed_ProgressbarValue = SIGNAL("changed_ProgressbarValue(int)")
        self.changed_MotorSpeedPointsList = SIGNAL("changed_MotorSpeedPointsList")  # short circuited signal
        self.changed_MotorIntervalLengthsList = SIGNAL("changed_MotorIntervalLengthsList")  # short circuited signal
        self.changed_PSOSettings = SIGNAL("changed_PSOSettings")  # dict with only those settings that needs to be changed # short circuited signal
        self.changed_MaxMotorIntervalLength = SIGNAL("changed_MaxMotorIntervalLength(double)")
        self.changed_OperationCycleTime = SIGNAL("changed_OperationCycleTime(int)")
        self.changed_SaveIntermediateStates = SIGNAL("changed_SaveIntermediateStates(bool)")
        self.changed_IntermediateSaveRobotStateEvaluationThreshold = SIGNAL("changed_IntermediateSaveRobotStateEvaluationThreshold(int)")
        self.changed_EvaluationMetricToUse = SIGNAL("changed_EvaluationMetricToUse(int)")
        self.changed_SaveRobotStateOnInterruption = SIGNAL("changed_SaveRobotStateOnInterruption(bool)")
        self.changed_RobotOperationType = SIGNAL("changed_RobotOperationType(QString)")
        self.changed_StartPosMinRectSize = SIGNAL("changed_StartPosMinRectSize(int)")

        self.stopRobotOperationCycle = SIGNAL("stopRobotOperationCycle(bool)")  # signal to GUI that it should stop the robot operation cycle
        self.updateStatusMessage_Backend = SIGNAL("updateStatusMessage_Backend(QString)")


        # evaluation data
        self.evaluationThread = None  # to check if evaluationThread still running
        self.stopTrackingEvent = stopTrackingEvent

        # translating dict of parameter names with their corresponding setter methods:
        self.setterMethodsDict = {"saveRobotStateOnInterruption" : self.Change_SaveRobotStateOnInterruption, "robotOperationType" : self.Change_RobotOperationType, "PSOsettings" : self.Change_PSOSettings, 
                                  "PSOstate" : self.Change_PSOstate, "evaluationMetricToUse" : self.Change_EvaluationMetricToUse, "saveIntermediateStates" : self.Change_SaveIntermediateStates,
                                  "intermediateSaveRobotStateEvaluationThreshold" : self.Change_IntermediateSaveRobotStateEvaluationThreshold, "operationCycleTime" : self.Change_OperationCycleTime,
                                  "maxMotorIntervalLength" : self.Change_MaxMotorIntervalLength, "startPosMinRectSize" : self.Change_StartPosMinRectSize, "speedPointsList" : self.Change_MotorSpeedPointsList,
                                  "intervalLengthsList" : self.Change_MotorIntervalLengthsList}

        ## NOTE: these next default settings are overwritten by downloading all parameters from robot 
        # robot general settings:
        self.saveRobotStateOnInterruption = True

        # robot algorithm settings:
        self.robotOperationType = "Manual"
        self.PSOsettings = {"swarmsize": 10, "omega": 0.5, "phig": 0.5, "phip": 0.5, "maxiter": 1000, "minstep": 1e-3, "minfunc": 1}
        self.PSOstate = {"overwriteState": False, "S": 0, "D": 0, "x": array([]), "v": array([]), "p": array([]), "fp": array([]), "g": [], "fg": 0, "optimizationIteration": 0}

        # robot evaluation settings:
        self.evaluationMetricToUse = "invTotalDistance_LeavingRect"
        self.saveIntermediateStates = False
        self.intermediateSaveRobotStateEvaluationThreshold = 0
        self.operationCycleTime = 30  # int
        self.maxMotorIntervalLength = 30
        self.startPosMinRectSize = 50

        # robot motor operation data
        self.speedPointsList = zeros((3,9), dtype = int16)
        self.intervalLengthsList = zeros((3,9), dtype = float64)

        # robot state variables?s
        return
    #endFct __init__

    def Change_MotorSpeedPointsList(self, newMotorSpeedPointsList):
        "Change values in ModelPC and update GUI"
        self.speedPointsList = newMotorSpeedPointsList.copy()
        self.emit(self.changed_MotorSpeedPointsList, newMotorSpeedPointsList)
    #endFct Change_MotorSpeedPointsList

    def Change_MotorIntervalLengthsList(self, newMotorIntervalLengthsList):
        "Change values in ModelPC and update GUI"
        #self.logger.debug("ModelPC:Change_MotorIntervalLenthsList called with value = {0}".format(newMotorIntervalLengthsList))
        self.intervalLengthsList = newMotorIntervalLengthsList.copy()
        self.emit(self.changed_MotorIntervalLengthsList, newMotorIntervalLengthsList)
    #endFct Change_MotorIntervalLengthsList

    def Change_PSOSettings(self, newPSOsettings):
        "Change values in ModelPC and update GUI"
        self.PSOsettings.update(newPSOsettings)
        self.emit(self.changed_PSOSettings, newPSOsettings.copy())
    #endFct Change_PSOSettings

    def Change_MaxMotorIntervalLength(self, newMaxLength):
        "Change values in ModelPC and update GUI"
        self.maxMotorIntervalLength = newMaxLength
        self.emit(self.changed_MaxMotorIntervalLength, newMaxLength)
    #endFct Change_MaxMotorIntervalLength
    
    def Change_OperationCycleTime(self, newTime):
        "Change values in ModelPC and update GUI"
        self.operationCycleTime = newTime
        self.emit(self.changed_OperationCycleTime, newTime)
    #endFct Change_OperationCycleTime

    def Change_SaveIntermediateStates(self, newValue):
        "Change values in ModelPC and update GUI"
        self.saveIntermediateStates = newValue
        self.emit(self.changed_SaveIntermediateStates, newValue)
    #endFct Change_SaveIntermediateStates

    def Change_IntermediateSaveRobotStateEvaluationThreshold(self, newValue):
        "Change values in ModelPC and update GUI"
        self.intermediateSaveRobotStateEvaluationThreshold = newValue
        self.emit(self.changed_IntermediateSaveRobotStateEvaluationThreshold, newValue)
    #endFct Change_SaveIntermediateStates

    
    def Change_EvaluationMetricToUse(self, newValue):
        "Change values in ModelPC and update GUI"
        # check if valid metric:
        if newValue not in SettingsPC.AvailableEvaluationMetrics:
            self.logger.error("ModelPC:Change_EvaluationMetricToUse: Trying to set a new evaluation metric {0}, but is not found in the available metrics list.".format(newValue))
            return
        self.evaluationMetricToUse = newValue
        self.emit(self.changed_EvaluationMetricToUse, SettingsPC.AvailableEvaluationMetrics.index(newValue))
    #endFct Change_EvaluationMetricToUse

    def Change_SaveRobotStateOnInterruption(self, newValue):
        "Change values in ModelPC and update GUI"
        self.saveRobotStateOnInterruption = newValue
        self.emit(self.changed_SaveRobotStateOnInterruption, newValue)
    #endFct Change_SaveRobotStateOnInterruption

    def Change_RobotOperationType(self, newValue):
        "Change values in ModelPC and update GUI"
        self.robotOperationType = newValue
        self.emit(self.changed_RobotOperationType, newValue)
    #endFct Change_RobotOperationType

    def Change_PSOstate(self, newDict):
        "Change values of PSOstate, NOT YET IMPLEMENTED IN GUI"
        self.PSOstate.update(newDict)
        # EVT: update GUI
    #endFct Change_PSOstate

    def Change_StartPosMinRectSize(self, newValue):
        "Change values in ModelPC and update GUI"
        self.startPosMinRectSize = newValue
        self.emit(self.changed_StartPosMinRectSize, newValue)
    #endFct Change_StartPosMinRectSize


    def Stop_RobotOperationCycle(self):
        "Triggers stop operation cyle action in GUI"
        self.emit(self.stopRobotOperationCycle, True)
    #endFct Stop_RobotOperationCycle

    def UpdateGUI_StatusMessage(self, text):
        "Set a new status message in GUI from backend"
        self.emit(self.updateStatusMessage_Backend, text)
    #endFct UpdateGUI_StatusMessage