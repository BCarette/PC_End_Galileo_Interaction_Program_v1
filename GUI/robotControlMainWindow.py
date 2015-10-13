from PyQt4.QtCore import *
from PyQt4.QtGui import *
import ui_robotControlMainWindow
from loadRobotStateDialog import LoadRobotStateDialog
from robotBootedDialog import RobotBootedDialog
from settingsPC import SettingsPC
from modelPC import ModelPC
from mainPC import MainPC
from TrackingProperties import TrackingProperties
from Tracking import Tracking
import re
import time
import datetime
from numpy import *
import logging
import pyqtgraph as pg

class RobotControlMainWindow(QMainWindow, ui_robotControlMainWindow.Ui_RobotControlMainWindow):
    """This is the main window of the GUI controlling the ball robot"""

    def __init__(self, modelPC, parent=None):
        super(RobotControlMainWindow, self).__init__(parent)
        # init generated gui:
        self.setupUi(self)

        # block sending signals of items being programmatically set here:
        self.ddlTracking_EvaluationMetricToUse.blockSignals(True)
        self.ddlTracking_FrameToShow.blockSignals(True)

        self.logger = modelPC.logger
        self.modelPC = modelPC
        self.modelPCLock = modelPC.modelPCLock
        self.bluetoothLock = modelPC.bluetoothLock
        self.bluetoothSocket = modelPC.bluetoothSocket
        self.trackingPropertiesLock = modelPC.trackingPropertiesLock
        self.stopTrackingEvent = modelPC.stopTrackingEvent
        self.state_StartbtnActivated = False
        self.bluetoothLock.acquire()
        self.robotBootedWaitingOnUserChoice = self.bluetoothSocket.robotBootedWaitingOnUserChoice
        self.ackAckStartNewEvaluationReceived = self.bluetoothSocket.ackAckStartNewEvaluationReceived
        self.bluetoothLock.release()
        
        # translating dictionairies
        self.possibleOperationTypes = {"action_Manual_operation": "Manual", "action_Particle_Swarm_operation" : "PSO"}
        self.possiblePSOSettings = {"spnbx_OptSet_Swarmsize":  "swarmsize", "dblspnbx_OptSet_Omega": "omega", "dblspnbx_OptSet_Phip": "phip", "dblspnbx_OptSet_Phig": "phig", "spnbx_OptSet_MaxIterations": "maxiter",
                                    "dblspnbx_OptSet_Minstep": "minstep", "dblspnbx_OptSet_Minfunc": "minfunc"}
        self.allPSOSettingsFields = {"swarmsize" : self.spnbx_OptSet_Swarmsize, "omega" : self.dblspnbx_OptSet_Omega, "phip" : self.dblspnbx_OptSet_Phip, "phig" : self.dblspnbx_OptSet_Phig,
                                     "maxiter" : self.spnbx_OptSet_MaxIterations, "minstep" : self.dblspnbx_OptSet_Minstep, "minfunc" : self.dblspnbx_OptSet_Minfunc}
        self.allOperationTypeFields = {"Manual" : self.action_Manual_operation, "PSO" : self.action_Particle_Swarm_operation}

        # collect all motorIntervalLength variable pointers in a list:
        self.allMotorIntervalLengthFields = [self.dblspnbxMotor1Interval1, self.dblspnbxMotor1Interval2, self.dblspnbxMotor1Interval3, self.dblspnbxMotor1Interval4, self.dblspnbxMotor1Interval5,
                                             self.dblspnbxMotor1Interval6, self.dblspnbxMotor1Interval7, self.dblspnbxMotor1Interval8, self.dblspnbxMotor1Interval9, self.dblspnbxMotor2Interval1,
                                             self.dblspnbxMotor2Interval2, self.dblspnbxMotor2Interval3, self.dblspnbxMotor2Interval4, self.dblspnbxMotor2Interval5, self.dblspnbxMotor2Interval6,
                                             self.dblspnbxMotor2Interval7, self.dblspnbxMotor2Interval8, self.dblspnbxMotor2Interval9, self.dblspnbxMotor3Interval1, self.dblspnbxMotor3Interval2,
                                             self.dblspnbxMotor3Interval3, self.dblspnbxMotor3Interval4, self.dblspnbxMotor3Interval5, self.dblspnbxMotor3Interval6, self.dblspnbxMotor3Interval7,
                                             self.dblspnbxMotor3Interval8, self.dblspnbxMotor3Interval9]
        self.allMotorSpeedFields = [self.spnbxMotor1Speed1, self.spnbxMotor1Speed2, self.spnbxMotor1Speed3, self.spnbxMotor1Speed4, self.spnbxMotor1Speed5, self.spnbxMotor1Speed6, self.spnbxMotor1Speed7,
                                    self.spnbxMotor1Speed8, self.spnbxMotor1Speed9, self.spnbxMotor2Speed1, self.spnbxMotor2Speed2, self.spnbxMotor2Speed3, self.spnbxMotor2Speed4, self.spnbxMotor2Speed5,
                                    self.spnbxMotor2Speed6, self.spnbxMotor2Speed7, self.spnbxMotor2Speed8, self.spnbxMotor2Speed9, self.spnbxMotor3Speed1, self.spnbxMotor3Speed2, self.spnbxMotor3Speed3,
                                    self.spnbxMotor3Speed4, self.spnbxMotor3Speed5, self.spnbxMotor3Speed6, self.spnbxMotor3Speed7, self.spnbxMotor3Speed8, self.spnbxMotor3Speed9]

        # set cursor log message box to the end:
        self.txtEditLogMessages.moveCursor(QTextCursor.End)
        # bundle operationTypes in a group
        self.setOperationTypeGroup = QActionGroup(self)
        self.setOperationTypeGroup.addAction(self.action_Manual_operation)
        self.setOperationTypeGroup.addAction(self.action_Particle_Swarm_operation)
        self.action_Manual_operation.setChecked(True)

        # fill dropdownlists
        self.ddlTracking_FrameToShow.addItems(SettingsPC.AvailableFramesToView)
            # set default selected:
        self.ddlTracking_FrameToShow.setCurrentIndex(SettingsPC.AvailableFramesToView.index("contourFilters_on_res_MorphOp"))
        self.ddlTracking_EvaluationMetricToUse.addItems(SettingsPC.AvailableEvaluationMetrics)
            # set default selected:
        self.ddlTracking_EvaluationMetricToUse.setCurrentIndex(SettingsPC.AvailableEvaluationMetrics.index("invTotalDistance_LeavingRect"))

        # create widgets in statusbar:
        self.lblStatusBar_RobotStatus = QLabel("No robot connected", self)
        self.lblStatusBar_RobotStatus.setMargin(5)
        
        self.lblStatusBar_TrackingStatus = QLabel("Not tracking", self)
        self.lblStatusBar_RobotStatus.setMargin(5)

        self.lblStatusBar_Messages = QLabel("", self)
        self.lblStatusBar_Messages.setMargin(5)

        self.statusbar.addPermanentWidget(self.lblStatusBar_RobotStatus, 1)
        self.statusbar.addPermanentWidget(self.lblStatusBar_TrackingStatus, 1)
        self.statusbar.addPermanentWidget(self.lblStatusBar_Messages, 2)
        
        # button start opertions only enabled when robot ready:
        self.btnStartStop.setEnabled(False)
        # don't allow actions before robot booted:
        self.action_Download_status.setEnabled(False)  # state here does not depend on btnStopRestartCycle
        self.action_Set_parameters.setEnabled(False)
        self.action_Save_robot_state.setEnabled(False)  # state here does not depend on btnStopRestartCycle
        self.action_Load_robot_state.setEnabled(False)
        self.action_Shutdown_robot.setEnabled(False)
        # NOTE: settings are enabled at start-up to interact with modelPC, but bluetooth messages to robot are disabled!

        # set visibilities on defaults:
        self.spnbx_Tracking_IntermediateStateSavingThreshold.setEnabled(self.modelPC.saveIntermediateStates)

        # connect custom signals
        try:
            self.connect(modelPC, modelPC.changed_ProgressbarValue, self.Change_ProgressbarValue)
            self.connect(modelPC, modelPC.changed_MotorSpeedPointsList, self.Change_MotorSpeedPointsList)
            self.connect(modelPC, modelPC.changed_MotorIntervalLengthsList, self.Change_MotorIntervalLengthsList)
            self.connect(modelPC, modelPC.changed_PSOSettings, self.Change_PSOSettings)
            self.connect(modelPC, modelPC.changed_MaxMotorIntervalLength, self.Change_MaxMotorIntervalLength)
            self.connect(modelPC, modelPC.changed_OperationCycleTime, self.Change_OperationCycleTime)
            self.connect(modelPC, modelPC.changed_SaveIntermediateStates, self.Change_SaveIntermediateStates)
            self.connect(modelPC, modelPC.changed_IntermediateSaveRobotStateEvaluationThreshold, self.Change_IntermediateSaveRobotStateEvaluationThreshold)
            self.connect(modelPC, modelPC.changed_EvaluationMetricToUse, self.Change_EvaluationMetricToUse)
            self.connect(modelPC, modelPC.changed_SaveRobotStateOnInterruption, self.Change_SaveRobotStateOnInterruption)
            self.connect(modelPC, modelPC.changed_RobotOperationType, self.Change_RobotOperationType)
            self.connect(modelPC, modelPC.changed_StartPosMinRectSize, self.Change_StartPosMinRectSize)

            # connect logger signals:
            self.connect(modelPC.loggingSignalHolder, modelPC.loggingSignalHolder.updateGUIMessageBox, self.UpdateMessageBox)
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.updateGUIMessageBox, self.UpdateMessageBox)

            # connect update status signals:
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.updateRobotStatus, self.UpdateRobotStatusLabel)
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.updateStatusMessage, self.UpdateStatusMessageLabel)
            self.connect(modelPC, modelPC.updateStatusMessage_Backend, self.UpdateStatusMessageLabel)

            # connect notify signals from bluetooth class:
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.notifyRobotStartable, self.Change_RobotStartable)
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.startNewTrackingCycle, self.StartNewEvaluation)
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.notifyRobotReadyToRestartCycle, self.Change_RobotReadyToRestart)
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.updateProgressBar, self.Change_ProgressbarValue)
            
            # robot booted:
            self.connect(modelPC.bluetoothSocket, modelPC.bluetoothSocket.startRobotBootedDialog, self.StartRobotBootedDialog)

            # connect action trigger signals from backend to frontend:
            self.connect(modelPC, modelPC.stopRobotOperationCycle, self.on_btnStopRestartCycle_clicked)
            
            
        except:
            self.logger.error("Exception occurred in main GUI trying to connect signals")
            # evt: rethrow

        # connect GUI signals
        # operationType signals
        self.connect(self.setOperationTypeGroup, SIGNAL("triggered(QAction*)"), self.guiOperationTypeChanged)
        # motorspeed signals
        self.connect(self.spnbxMotor1Speed1, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed2, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed3, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed4, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed5, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed6, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed7, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed8, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor1Speed9, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed1, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed2, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed3, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed4, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed5, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed6, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed7, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed8, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor2Speed9, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed1, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed2, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed3, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed4, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed5, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed6, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed7, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed8, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        self.connect(self.spnbxMotor3Speed9, SIGNAL("valueChanged(int)"), self.guiMotorSpeedChanged)
        # motor intervalLength signals
        self.connect(self.dblspnbxMotor1Interval1, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval2, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval3, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval4, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval5, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval6, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval7, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval8, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor1Interval9, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval1, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval2, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval3, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval4, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval5, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval6, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval7, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval8, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor2Interval9, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval1, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval2, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval3, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval4, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval5, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval6, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval7, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval8, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        self.connect(self.dblspnbxMotor3Interval9, SIGNAL("valueChanged(double)"), self.guiMotorIntervalLengthChanged)
        # PSO settings
        self.connect(self.spnbx_OptSet_Swarmsize, SIGNAL("valueChanged(int)"), self.guiPSOSettingsChanged)
        self.connect(self.dblspnbx_OptSet_Omega, SIGNAL("valueChanged(double)"), self.guiPSOSettingsChanged)
        self.connect(self.dblspnbx_OptSet_Phip, SIGNAL("valueChanged(double)"), self.guiPSOSettingsChanged)
        self.connect(self.dblspnbx_OptSet_Phig, SIGNAL("valueChanged(double)"), self.guiPSOSettingsChanged)
        self.connect(self.spnbx_OptSet_MaxIterations, SIGNAL("valueChanged(int)"), self.guiPSOSettingsChanged)
        self.connect(self.dblspnbx_OptSet_Minstep, SIGNAL("valueChanged(double)"), self.guiPSOSettingsChanged)
        self.connect(self.dblspnbx_OptSet_Minfunc, SIGNAL("valueChanged(double)"), self.guiPSOSettingsChanged)
        # general optimization setting: max motor IntervalLength has custom Slot
        # tracking settings
        self.connect(self.spnbx_Tracking_ColorFilter_HueMin, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ColourFilter_HueChanged)
        self.connect(self.spnbx_Tracking_ColorFilter_HueMax, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ColourFilter_HueChanged)
        self.connect(self.spnbx_Tracking_ColorFilter_SatMin, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ColourFilter_SaturationChanged)
        self.connect(self.spnbx_Tracking_ColorFilter_SatMax, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ColourFilter_SaturationChanged)
        self.connect(self.spnbx_Tracking_ColorFilter_ValueMin, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ColourFilter_ValueChanged)
        self.connect(self.spnbx_Tracking_ColorFilter_ValueMax, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ColourFilter_ValueChanged)
        self.connect(self.spnbx_Tracking_ContourRadiusMin, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ContourRadiusChanged)
        self.connect(self.spnbx_Tracking_ContourRadiusMax, SIGNAL("valueChanged(int)"), self.guiTrackingSettings_ContourRadiusChanged)


        # visualisations inits:
        #DEBUG
        self.grphcsViewMotor1.setBackground("w")
        self.grphcsViewMotor2.setBackground("w")
        self.grphcsViewMotor3.setBackground("w")
        #pg.setConfigOption('background', 'w')
        #pg.setConfigOption('foreground', 'b')
        #self.grphcsViewMotor1.plotItem.setLabel("bottom", "time", "s")
        self.grphcsViewMotor1.plotItem.setLabel("left", "Motor 1")
        self.grphcsViewMotor2.plotItem.setLabel("left", "Motor 2")
        self.grphcsViewMotor3.plotItem.setLabel("left", "Motor 3")
        #self.grphcsViewMotor1.setYRange(-3200,3200,0)
        #self.grphcsViewMotor1.plotItem.plot([1,2,3,4,5], [5,4,3,2,1], symbol = "o", symbolSize = 5)
        self.allMotorVisualisations = [self.grphcsViewMotor1, self.grphcsViewMotor2, self.grphcsViewMotor3]
        # update visualisations:
        self.updateGraphicsViewMotor(0)
        self.updateGraphicsViewMotor(1)
        self.updateGraphicsViewMotor(2)

        # unblock blocked signals for initializations:
        self.ddlTracking_EvaluationMetricToUse.blockSignals(False)
        self.ddlTracking_FrameToShow.blockSignals(False)

        # Gui inited:
        self.logger.debug("GUI inited")
        modelPC.guiInitedEvent.set()

    def blockedRobotSettingsSignals(self, block):
        "Block/unblock signals from robotsettings fields"
        for element in self.allMotorIntervalLengthFields:
            element.blockSignals(block)
        for element in self.allMotorSpeedFields:
            element.blockSignals(block)
        for element in self.allPSOSettingsFields.values():
            element.blockSignals(block)
        # elements from Tracking, evaluation settings:
        self.spnbx_Tracking_OperationCycleTime.blockSignals(block)
        self.chkTracking_SaveIntermediateStates.blockSignals(block)
        self.spnbx_Tracking_IntermediateStateSavingThreshold.blockSignals(block)
        self.ddlTracking_EvaluationMetricToUse.blockSignals(block)
        self.spnbx_Tracking_StartPosMinRectSize.blockSignals(block)
        self.chkTracking_SaveStateOnInterruption.blockSignals(block)
    #endFct blockedRobotSettingsSignals

    def updateGraphicsViewMotor(self, motorNr):
        "Update the graph of motor signal of motorNr"
        self.modelPCLock.acquire()
        motorSpeedPoints = self.modelPC.speedPointsList[motorNr, :].tolist()
        #motorTimePoints = self.modelPC.intervalLengthsList[motorNr, :].copy()
        allmotorTimePoints = self.modelPC.intervalLengthsList.copy()
        self.modelPCLock.release()
        
        ### delete any double zero intervalpoints:
        motorTimePoints = allmotorTimePoints[motorNr, :].tolist()

        operatingTimePoints = []
        operatingSpeedPoints = []

        if max(motorTimePoints) < 1e-10:
            # all interval lengths are zero => make motor idle all the time
            operatingSpeedPoints = [0,0]
            operatingTimePoints = [self.spnbx_Tracking_OperationCycleTime.value(),0]
        else:
            # given speedpoints contain at least one non-zero interval time
            previousTime = motorTimePoints[0]
            checkFirstPointToInsert = False if previousTime > 0 else True
            i = -1
            if previousTime > 0:
                # starting interval is nonzero => directly store it
                operatingSpeedPoints = [motorSpeedPoints[0]]
                operatingTimePoints = [motorTimePoints[0]]
                i = 1
            else: 
                #wait until the end to decide to insert the first interval or not
                # look for first non-zero interval point:
                for j in range(1, len(motorTimePoints)):
                    if motorTimePoints[j] > 0:
                        i = j
                        break
                #endFor searching first next non-zero interval

            # i is set to first interval after startingPoint which needs to be added
            while i < len(motorSpeedPoints):
                # append current point
                operatingSpeedPoints.append(motorSpeedPoints[i])
                operatingTimePoints.append(motorTimePoints[i])
                if motorTimePoints[i] > 0:
                    # inserted a non-zero interval => move to next point:
                    # if this is the last point of motorSpeedPoints => checkFirstPointToInsert
                    if (i == len(motorSpeedPoints) - 1) and checkFirstPointToInsert:
                        # last motor speed points interval is non-zero => insert also the first interval if zero::
                        operatingSpeedPoints.insert(0, motorSpeedPoints[0])
                        operatingTimePoints.insert(0, motorTimePoints[0])
                                           
                else:
                    # current interval has zero length => look for next non-zero interval point
                    nextNonzeroIntervalIndex = -1
                    for j in range(i + 1, len(motorTimePoints)):
                        if motorTimePoints[j] > 0:
                            nextNonzeroIntervalIndex = j
                            break
                    #endFor searching first next non-zero interval

                    if nextNonzeroIntervalIndex > -1:
                        # next non-zero interval found:
                        # move while loop to next non-zero interval point
                        i = nextNonzeroIntervalIndex   
                        continue

                    else:
                        # no next non-zero interval anymore in list
                        # check if starting point is also zero interval point
                        #if checkFirstPointToInsert:
                            # motorSpeedPoints does not end with a non-zero interval
                            # => neglect starting point
                        # end of processing speedPoints reached:
                        break
                    #endIf next non-zero interval
                #endIf motorIntervalLength[i]
                i += 1
            #endWhile i loop
        ###

        allmotorTimePoints[allmotorTimePoints < 1e-3] = 1e-3
        maxSumTime = amax(allmotorTimePoints.sum(1))

        operatingTimePointsArray = array(operatingTimePoints, dtype = float)
        operatingTimePointsArray[ operatingTimePointsArray < 1e-3] = 1e-3
        operatingTimePoints = operatingTimePointsArray.tolist()
        #motorTimePoints = allmotorTimePoints[motorNr, :].tolist()
        #motorTimePoints[motorTimePoints < 1e-3] = 1e-3  # don't plot multiple points on the same x-value
        #motorTimePoints = motorTimePoints.tolist()
        
        # adapt motor signals to be plotted:
        plotSpeedPoints = [operatingSpeedPoints[0]]  #array([], dtype = int16)
        plotTimePoints = [float(0)]  #array([], float)
        plottedTime = float(0)
        
        ## check if current array is in fact the longest:
        #if (abs(sum(operatingTimePoints) - maxSumTime) < 1e-10) or sum(operatingTimePoints) > maxSumTime :
        #    plotSpeedPoints = operatingSpeedPoints
        #    plotTimePoints = cumsum(operatingTimePoints)
        #    plottedTime = maxSumTime
        ##endIf
        
        indexOrigArray = -1
        indexOrigArrayNext = 0        
        while plottedTime < maxSumTime:
            indexOrigArray = indexOrigArray + 1 if indexOrigArray < (len(operatingTimePoints) - 1) else 0
            indexOrigArrayNext = indexOrigArrayNext + 1 if indexOrigArrayNext < (len(operatingTimePoints) - 1) else 0
            
            if plottedTime + operatingTimePoints[indexOrigArray] < maxSumTime:
                # plotting still within max time => just append
                plotSpeedPoints.append(operatingSpeedPoints[indexOrigArrayNext])
                plotTimePoints.append(plottedTime + operatingTimePoints[indexOrigArray])  # cumsum here
                plottedTime += operatingTimePoints[indexOrigArray]
            else:
                # next point would lie outside max time => interpolate:
                plotSpeedPoints.append(interp(maxSumTime, [plottedTime, plottedTime + operatingTimePoints[indexOrigArray]], [operatingSpeedPoints[indexOrigArray], operatingSpeedPoints[indexOrigArrayNext]]))
                plotTimePoints.append(maxSumTime)
                plottedTime = maxSumTime
            #endIf appending arrays

        #endWhile constructing plotting arrays

        # perform plotting
        self.allMotorVisualisations[motorNr].plotItem.clear()
        self.allMotorVisualisations[motorNr].plotItem.plot(plotTimePoints, plotSpeedPoints, symbol = "o", symbolSize = 5)
        #self.logger.debug("newGraph = \n{0}\n{1}".format(plotTimePoints, plotSpeedPoints))
    #endFct updateGraphicsViewMotor

    @pyqtSlot("bool")
    def on_btnStartStop_clicked(self, pressed):
        if pressed:
            # notify robot to start:
            self.bluetoothLock.acquire()
            self.bluetoothSocket.SendMessage(SettingsPC.BT_startOperations, "")
            self.bluetoothLock.release()
            self.logger.info("RobotControlMainWindow:on_btnStartStop_clicked: Start robot operations.")
            # motor operations start
            self.btnStartStop.setText("Stop")
            self.btnStopRestartCycle.setEnabled(True)

        else:
            # notify robot to stop:
            self.bluetoothLock.acquire()
            self.bluetoothSocket.SendMessage(SettingsPC.BT_stopOperations, "")
            self.bluetoothLock.release()
            self.logger.info("RobotControlMainWindow:on_btnStartStop_clicked: Stop robot operations.")
            # stop running evaluations:
            self.stopTrackingEvent.set()
            self.btnStartStop.setEnabled(False)  # wait on robot ReadyToStart before enabling start button
            self.state_StartbtnActivated = False
            self.btnStartStop.setText("Start")
            self.btnStopRestartCycle.setEnabled(False)
            self.btnStopRestartCycle.setChecked(False)
            self.btnStopRestartCycle.setText("Stop Cycle")
        #endIf pressed

        # enable/disable some input fields according to operating robot or not:
        self.grpBxMotor1Params.setEnabled(not pressed)
        self.grpBxMotor2Params.setEnabled(not pressed)
        self.grpBxMotor3Params.setEnabled(not pressed)
        self.grpbx_PSOSettings.setEnabled(not pressed)
        self.grpbx_Tracking_EvaluationSettings.setEnabled(not pressed)
        # enable/disable menu bar items:
        self.action_Download_status.setEnabled(not pressed)  # state here does not depend on btnStopRestartCycle
        self.action_Set_parameters.setEnabled(not pressed)
        self.action_Save_robot_state.setEnabled(not pressed)  # state here does not depend on btnStopRestartCycle
        self.action_Load_robot_state.setEnabled(not pressed)
        self.action_Shutdown_robot.setEnabled(not pressed)
        self.action_Manual_operation.setEnabled(not pressed)
        self.action_Particle_Swarm_operation.setEnabled(not pressed)
        self.action_Tracking_Testmode.setEnabled(not pressed)
        # disable signalling from setting fields wich are disabled:
        self.blockedRobotSettingsSignals(pressed)

    #endFct on_btnStartStop_clicked


    @pyqtSlot("bool")
    def on_btnStopRestartCycle_clicked(self, pressed):
        if pressed:
            # notify robot to stop its operation cycle:
            self.bluetoothLock.acquire()
            self.bluetoothSocket.SendMessage(SettingsPC.BT_stopCycle, "")
            self.bluetoothLock.release()
            self.logger.info("RobotControlMainWindow:on_btnStopRestartCycle_clicked: Stopping robot operation cycle")
            # stop running evaluations:
            self.stopTrackingEvent.set()
            self.btnStopRestartCycle.setText("Restart Cycle")
            self.btnStopRestartCycle.setEnabled(False)
            self.btnStopRestartCycle.setChecked(True) # button should be pressed when operationCyle stopped
            # wait for bluetoothSocket signal that robot is ready to restart before enabling the restart button
        else:
            # notify robot to restart
            self.bluetoothLock.acquire()
            refTimestamp = self.bluetoothSocket.robotReadyToRestartCycle_ReferenceTimestamp
            self.bluetoothSocket.SendMessage(SettingsPC.BT_restartCycle, refTimestamp)
            self.bluetoothLock.release()
            self.logger.info("RobotControlMainWindow:on_btnStopRestartCycle_clicked: Restarting robot operation cycle with referenceNr = {0}".format(refTimestamp))
            self.btnStopRestartCycle.setText("Stop Cycle")
        #endIf pressing button
        # enable/disable menu bar items:
        self.action_Download_status.setEnabled(self.btnStopRestartCycle.isChecked())  # state here does not depend on btnStartStop
        self.action_Save_robot_state.setEnabled(self.btnStopRestartCycle.isChecked())  # state here does not depend on btnStartStop
    #endFct on_btnStopRestartCycle_clicked

    @pyqtSlot("int")
    def Change_ProgressbarValue(self, value):
        " Change the value of the progress bar"
        if value < 0 or value > 100:
            self.logger.error("RobotControlMainWindow:guiProgressBarValueChanged: received a value outside bounds: {0}".format(value))
        # else: set value progress bar
        self.progressBarRobotCycle.setValue(value)
        #self.logger.debug("RobotControlMainWindow:Change_ProgressbarValue with value = {0}".format(value))
    #endFct Change_ProgressbarValue

    @pyqtSlot("QAction*")
    def guiOperationTypeChanged(self, senderAction):
        "Menubar -> Operaton types -> Set robot operation type"
        senderObjectName = str(senderAction.objectName())

        self.modelPCLock.acquire()
        # check if setting really changed value:
        if self.possibleOperationTypes[senderObjectName] == self.modelPC.robotOperationType:
            self.modelPCLock.release()
            return  # value did not change
        self.modelPC.robotOperationType = self.possibleOperationTypes[senderObjectName]
        self.modelPCLock.release()
        # notify robot about change
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["robotOperationType"], [self.possibleOperationTypes[senderObjectName]])
        self.bluetoothLock.release()
    #endFct guiOperationTypeChanged

    @pyqtSlot("int")
    def guiMotorSpeedChanged(self, value):
        " Handles changed motorspeed spinboxes in the GUI"
        signalSender = self.sender()
        # determine which parameter changed:
        matchObj = re.match(r"^spnbxMotor(?P<motorNr>\d)Speed(?P<speedNr>\d)", str(signalSender.objectName()))
        if matchObj is None:
            self.logger.error("RobotControlMainWindow:guiMotorSpeedChanged: Could not relate received sender {0} to a motorspeed parameter".format(str(signalSender.objectName())))
            return
        #endIf no matchObj
        # indexing in python start from 0 => subtract 1 for indexing
        motorNr = int(matchObj.group("motorNr")) - 1
        speedNr = int(matchObj.group("speedNr")) - 1
        # set new speed value:
        self.modelPCLock.acquire()
        # test first if it really is a new value:
        if self.modelPC.speedPointsList[[motorNr], [speedNr]] == value:
            # motorspeed is not new => is changed programmatically
            self.modelPCLock.release()
            return  # value did not change
        self.modelPC.speedPointsList[motorNr, speedNr] = value
        speedPointsList = self.modelPC.speedPointsList
        self.modelPCLock.release()
        # notify robot about change
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["speedPointsList"], [speedPointsList])
        self.bluetoothLock.release()

        # update visualisations: 
        self.updateGraphicsViewMotor(motorNr)
    #endFct guiMotorSpeedChanged

    @pyqtSlot("double")
    def guiMotorIntervalLengthChanged(self, value):
        " Handles changed interval length spinboxes in the GUI"
        signalSender = self.sender()
        # determine which parameter changed:
        matchObj = re.match(r"^dblspnbxMotor(?P<motorNr>\d)Interval(?P<intervalNr>\d)", str(signalSender.objectName()))
        if matchObj is None:
            self.logger.error("RobotControlMainWindow:guiMotorIntervalLengthChanged: Could not relate received sender {0} to an interval length parameter".format(str(signalSender.objectName())))
            return
        #endIf no matchObj
        # indexing in python start from 0 => subtract 1 for indexing
        motorNr = int(matchObj.group("motorNr")) - 1
        intervalNr = int(matchObj.group("intervalNr")) - 1
        # set new speed value:
        self.modelPCLock.acquire()
        # test first if it really is a new value:
        if abs(self.modelPC.intervalLengthsList[[motorNr], [intervalNr]] - value) < 1e-3:
            # intervalLength is not new => is changed programmatically
            self.modelPCLock.release()
            return  # value did not change
        self.modelPC.intervalLengthsList[motorNr, intervalNr] = value
        motorIntervalLengthsList = self.modelPC.intervalLengthsList
        self.modelPCLock.release()
        # notify robot about change
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["intervalLengthsList"], [motorIntervalLengthsList])
        self.bluetoothLock.release()

        # update visualisation: 
        self.updateGraphicsViewMotor(motorNr)

    #endFct guiMotorIntervalLengthChanged

    @pyqtSlot("int")
    @pyqtSlot("double")
    def guiPSOSettingsChanged(self, value):
        "Handles changed settings of PSO in the GUI, except maximum motor interval length."
        senderObjectName = str(self.sender().objectName())
        senderParameterName = self.possiblePSOSettings[senderObjectName]

        self.modelPCLock.acquire()
        # check if setting really changed: (could be changed programmatically)
        if abs(self.modelPC.PSOsettings[senderParameterName] - value) < 1e-10:
            self.modelPCLock.release()
            return # value did not change

        self.modelPC.PSOsettings[senderParameterName] = value
        PSOsettings = self.modelPC.PSOsettings
        self.modelPCLock.release()
        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["PSOsettings"], [PSOsettings])
        self.bluetoothLock.release()
    #endFct guiPSOSettingsChanged

    @pyqtSlot("double")
    def on_dblspnbx_OptSet_MaxMotorIntervalLength_valueChanged(self, value):
        "Handles optimization settings changes of maximum motor interval length"
        # check first if value really changed or is set programmatically:
        self.modelPCLock.acquire()
        if abs(self.modelPC.maxMotorIntervalLength - value) < 1e-10:
            self.modelPCLock.release()
            return # value did not change
        self.modelPC.maxMotorIntervalLength = value
        self.modelPCLock.release()
        # change maxima of motor parameters:
        for spnbxInterval in self.allMotorIntervalLengthFields:
            spnbxInterval.setMaximum(value)

        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["maxMotorIntervalLength"], [value])
        self.bluetoothLock.release()
    #endFct on_dblspnbx_OptSet_MaxMotorIntervalLength_valueChanged

    @pyqtSlot("int")
    def guiTrackingSettings_ColourFilter_HueChanged(self, value):
        "Handles changing of tracking settings about the colour filter: min Hue and max Hue"
        # determine if min or max value is signalled:
        senderObjectName = str(self.sender().objectName())
        minMatchObj = re.search(r"Min", senderObjectName)
        parameterIndex = 0 if minMatchObj is not None else 1

        # check if value really changed or set programmatically:
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.Hue[parameterIndex] == value:
            self.trackingPropertiesLock.release()
            return # value did not change
        TrackingProperties.Hue[parameterIndex] = value
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct guiTrackingSettings_ColourFilter_HueChanged

    @pyqtSlot("int")
    def guiTrackingSettings_ColourFilter_SaturationChanged(self, value):
        "Handles changing of tracking settings about the colour filter: min Saturation and max Saturation"
        # determine if min or max value is signalled:
        senderObjectName = str(self.sender().objectName())
        minMatchObj = re.search(r"Min", senderObjectName)
        parameterIndex = 0 if minMatchObj is not None else 1

        # check if value really changed or set programmatically:
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.Saturation[parameterIndex] == value:
            self.trackingPropertiesLock.release()
            return # value did not change
        TrackingProperties.Saturation[parameterIndex] = value
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct guiTrackingSettings_ColourFilter_SaturationChanged

    @pyqtSlot("int")
    def guiTrackingSettings_ColourFilter_ValueChanged(self, value):
        "Handles changing of tracking settings about the colour filter: min Value and max Value"
        # determine if min or max value is signalled:
        senderObjectName = str(self.sender().objectName())
        minMatchObj = re.search(r"Min", senderObjectName)
        parameterIndex = 0 if minMatchObj is not None else 1

        # check if value really changed or set programmatically:
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.Value[parameterIndex] == value:
            self.trackingPropertiesLock.release()
            return # value did not change
        TrackingProperties.Value[parameterIndex] = value
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct guiTrackingSettings_ColourFilter_ValueChanged

    @pyqtSlot("int")
    def on_spnbx_Tracking_MorphologicalElSize_valueChanged(self, value):
        "Handles changing of tracking settings its morphological element size for the morphological opening operation"
        # check if value really changed
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.MorphologicalElementDiam == value:
            self.trackingPropertiesLock.release()
            return  # value did not change
        TrackingProperties.MorphologicalElementDiam = value
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct on_spnbx_Tracking_MorphologicalElSize_valueChanged

    @pyqtSlot("int")
    def guiTrackingSettings_ContourRadiusChanged(self, value):
        "Handles changing of tracking settings, thresholds for detected circles: ContourRadiusMin and ContourRadiusMax"
        senderObjectName = str(self.sender().objectName())
        minMatchObj = re.search(r"Min", senderObjectName)
        parameterIndex = 0 if minMatchObj is not None else 1

        # check if value really changed or set programmatically:
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.ContourRadius[parameterIndex] == value:
            self.trackingPropertiesLock.release()
            return # value did not change
        TrackingProperties.ContourRadius[parameterIndex] = value
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct guiTrackingSettings_ContourRadiusChanged

    @pyqtSlot("int")
    def on_spnbx_Tracking_deltaRadius_valueChanged(self, value):
        "Handles changing of tracking settings its extra mask radius around found contours in the image"
        # check if value really changed
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.DeltaRadius == value:
            self.trackingPropertiesLock.release()
            return  # value did not change
        TrackingProperties.DeltaRadius = value
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct on_spnbx_Tracking_deltaRadius_valueChanged

    @pyqtSlot("int")
    def on_ddlTracking_FrameToShow_currentIndexChanged(self, value):
        "Handles changing of frame to view during tracking"
        selectedFrame = SettingsPC.AvailableFramesToView[value]
        # check if value really changed:
        self.trackingPropertiesLock.acquire()
        if TrackingProperties.FrameToShow == selectedFrame:
            self.trackingPropertiesLock.release()
            return   # Value did not change
        TrackingProperties.FrameToShow = selectedFrame
        TrackingProperties.NewSettingsPushed = True
        self.trackingPropertiesLock.release()
    #endFct on_ddlTracking_FrameToShow_currentIndexChanged

    @pyqtSlot("int")
    def on_spnbx_Tracking_OperationCycleTime_valueChanged(self, value):
        "Handles changing of evaluation setting: Operation cycle time"
        # check if value really changed:
        self.modelPCLock.acquire()
        if self.modelPC.operationCycleTime == value:
            self.modelPCLock.release()
            return # value did not change
        self.modelPC.operationCycleTime = value
        self.modelPCLock.release()
        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["operationCycleTime"], [value])
        self.bluetoothLock.release()
    #endFct on_spnbx_Tracking_OperationCycleTime_valueChanged

    @pyqtSlot("int")
    def on_chkTracking_SaveIntermediateStates_stateChanged(self, value):
        "Handles changing of evaluation setting: Save intermediate state in optimization operation"
        booleanValue = value == 2
        self.spnbx_Tracking_IntermediateStateSavingThreshold.setEnabled(booleanValue)
        # check if value really changed:
        self.modelPCLock.acquire()
        if self.modelPC.saveIntermediateStates == booleanValue:
            self.modelPCLock.release()
            return  # value did not change
        self.modelPC.saveIntermediateStates = booleanValue
        self.modelPCLock.release()
        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["saveIntermediateStates"], [booleanValue])
        self.bluetoothLock.release()
    #endFct on_chkTracking_SaveIntermediateStates_stateChanged

    @pyqtSlot("int")
    def on_spnbx_Tracking_IntermediateStateSavingThreshold_valueChanged(self, value):
        "Handles changing of evaluation setting: intermediate save threshold"
        # check if value really changed:
        self.modelPCLock.acquire()
        if self.modelPC.intermediateSaveRobotStateEvaluationThreshold == value:
            self.modelPCLock.release()
            return # value did not change
        self.modelPC.intermediateSaveRobotStateEvaluationThreshold = value
        self.modelPCLock.release()
        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["intermediateSaveRobotStateEvaluationThreshold"], [value])
        self.bluetoothLock.release()
    #endFct on_spnbx_Tracking_IntermediateStateSavingThreshold_valueChanged

    @pyqtSlot("int")
    def on_ddlTracking_EvaluationMetricToUse_currentIndexChanged(self, value):
        "Handles which metric to use to evaluate a recorded tracking cycle"
        selectedMetric = SettingsPC.AvailableEvaluationMetrics[value]
        # check if value really changed:
        self.modelPCLock.acquire()
        if self.modelPC.evaluationMetricToUse == selectedMetric:
            self.modelPCLock.release()
            return   # Value did not change
        self.modelPC.evaluationMetricToUse = selectedMetric
        self.modelPCLock.release()
        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["evaluationMetricToUse"], [selectedMetric])
        self.bluetoothLock.release()
    #endFct on_ddlTracking_EvaluationMetricToUse_currentIndexChanged

    @pyqtSlot("int")
    def on_spnbx_Tracking_StartPosMinRectSize_valueChanged(self, value):
        "Handles changing of evaluation settings: size of the rectangle around the robot starting position that the robot needs to escape"
        # check if value really changed
        self.modelPCLock.acquire()
        if self.modelPC.startPosMinRectSize == value:
            self.modelPCLock.release()
            return  # value did not change
        self.modelPC.startPosMinRectSize = value
        self.modelPCLock.release()
    #endFct on_spnbx_Tracking_StartPosMinRectSize_valueChanged

    @pyqtSlot("int")
    def on_chkTracking_SaveStateOnInterruption_stateChanged(self, value):
        "Handles changing of evaluation setting: Save robot state when its operations are interrupted"
        booleanValue = value == 2
        # check if value really changed:
        self.modelPCLock.acquire()
        if self.modelPC.saveRobotStateOnInterruption == booleanValue:
            self.modelPCLock.release()
            return  # value did not change
        self.modelPC.saveRobotStateOnInterruption = booleanValue
        self.modelPCLock.release()
        # notify robot about change:
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(["saveRobotStateOnInterruption"], [booleanValue])
        self.bluetoothLock.release()
    #endFct on_chkTracking_SaveStateOnInterruption_stateChanged

    #@pyqtSlot("numpy.ndarray")
    def Change_MotorSpeedPointsList(self, newArray):
        "Update changes in GUI"
        flatValueArray = newArray.flat
        for i in range(0, len(self.allMotorSpeedFields)):
            self.allMotorSpeedFields[i].setValue(flatValueArray[i])

        # update visualisations:
        self.updateGraphicsViewMotor(0)
        self.updateGraphicsViewMotor(1)
        self.updateGraphicsViewMotor(2)
    #endFct Change_MotorSpeedPointsList

    #@pyqtSlot("numpy.ndarray")
    def Change_MotorIntervalLengthsList(self, newArray):
        "Update changes in GUI"
        # Check if new values within bounds of GUI fields:
        maxGuiBound = self.dblspnbxMotor1Interval1.maximum()
        newUpperBound = newArray.max() if not (newArray <= maxGuiBound).all() else -1
        if newUpperBound != -1:
            self.logger.warning("RobotControlMainWindow:Change_MotorIntervalLengthsList: Some new values in the list are larger than currently set maximum => change upper bound to: {0}".format(newUpperBound))
            self.dblspnbx_OptSet_MaxMotorIntervalLength.setValue(newUpperBound)

        flatValueArray = newArray.flat
        for i in range(0, len(self.allMotorIntervalLengthFields)):
            if newUpperBound != -1:
                self.allMotorIntervalLengthFields[i].setMaximum(newUpperBound)
            self.allMotorIntervalLengthFields[i].setValue(flatValueArray[i])

        # update visualisations: 
        self.updateGraphicsViewMotor(0)
        self.updateGraphicsViewMotor(1)
        self.updateGraphicsViewMotor(2)
    #endFct Change_MotorSpeedPointsList

    #@pyqtSlot(dict)
    def Change_PSOSettings(self, updatedDict):
        "Update changes in GUI"
        for key, value in updatedDict.items():
            self.allPSOSettingsFields[key].setValue(value)
    #endFct Change_PSOSettings

    @pyqtSlot("double")
    def Change_MaxMotorIntervalLength(self, value):
        "Update changes in GUI"
        self.dblspnbx_OptSet_MaxMotorIntervalLength.setValue(value)
        # change maxima of motor parameters:
        for spnbxInterval in self.allMotorIntervalLengthFields:
            spnbxInterval.setMaximum(value)
    #endFct Change_MaxMotorIntervalLength

    @pyqtSlot("int")
    def Change_OperationCycleTime(self, value):
        "Update changes in GUI"
        self.spnbx_Tracking_OperationCycleTime.setValue(value)
    #endFct Change_OperationCycleTime
    
    @pyqtSlot("bool")
    def Change_SaveIntermediateStates(self, value):
        "Update changes in GUI"
        self.chkTracking_SaveIntermediateStates.setChecked(value)
    #endFct Change_SaveIntermediateStates

    @pyqtSlot("int")
    def Change_IntermediateSaveRobotStateEvaluationThreshold(self, value):
        "Update changes in GUI"
        self.spnbx_Tracking_IntermediateStateSavingThreshold.setValue(value)
    #endFct Change_IntermediateSaveRobotStateEvaluationThreshold

    
    @pyqtSlot("int")
    def Change_EvaluationMetricToUse(self, value):
        "Update changes in GUI"
        self.ddlTracking_EvaluationMetricToUse.setCurrentIndex(value)
    #endFct Change_EvaluationMetricToUse
    
    @pyqtSlot("bool")
    def Change_SaveRobotStateOnInterruption(self, value):
        "Update changes in GUI"
        self.chkTracking_SaveStateOnInterruption.setChecked(value)
    #endFct Change_SaveRobotStateOnInterruption

    @pyqtSlot("QString")
    def Change_RobotOperationType(self, Name):
        "Update changes in GUI"
        self.allOperationTypeFields[str(Name)].setChecked(True)
    #endFct Change_RobotOperationType

    @pyqtSlot("int")
    def Change_StartPosMinRectSize(self, value):
        "Update changes in GUI"
        self.spnbx_Tracking_StartPosMinRectSize.setValue(value)
    #endFct Change_StartPosMinRectSize

    @pyqtSlot("bool")
    def on_action_Download_status_triggered(self, checked):
        "Perform action when demanding to get the parameters from the robot"
        self.bluetoothLock.acquire()
        self.bluetoothSocket.DownloadStatus(SettingsPC.AvailableRobotParametersToDownload)
        self.bluetoothLock.release()
    #endFct on_action_Download_status_triggered

    @pyqtSlot("bool")
    def on_action_Set_parameters_triggered(self, checked):
        "Perform action when demanding to set all the parameters in the GUI to the robot"
        # filter parameters out of modelPC
        self.modelPCLock.acquire()
        modelPCDict = self.modelPC.__dict__
        self.modelPCLock.release()
        # NOTE: PSOstate is not integrated with gui
        parametersToSetDict = {variableName: modelPCDict[variableName] for variableName in SettingsPC.AllowedParametersToSet}  

        self.bluetoothLock.acquire()
        self.bluetoothSocket.SetParameters(parametersToSetDict.keys(), parametersToSetDict.values())
        self.bluetoothLock.release()
    #endFct on_action_Set_parameters_triggered

    @pyqtSlot("bool")
    def on_action_Save_robot_state_triggered(self, checked):
        "Perform action when demanding the robot to save its parameters"
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SendMessage(SettingsPC.BT_saveRobotState, "")
        self.bluetoothLock.release()
    #endFct on_action_Save_robot_state_triggered

    @pyqtSlot("bool")
    def on_action_Load_robot_state_triggered(self, checked):
        "Perform action when demanding the robot to save its parameters"

        # request the available robot states to load
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SendMessage(SettingsPC.BT_getAvailableRobotStates, "")
        self.bluetoothLock.release()
        # wait in the QDialog for the up-to-date loadable robot states
        dialogWindow = LoadRobotStateDialog(self.bluetoothLock, self.bluetoothSocket, self.logger, self)
        dialogWindow.show()
        res = dialogWindow.exec_()
    #endFct on_action_Load_robot_state_triggered

    @pyqtSlot("bool")
    def on_action_Shutdown_robot_triggered(self, checked):
        "Perform action when demanding the robot to shutdown"
        self.bluetoothLock.acquire()
        self.bluetoothSocket.SendMessage(SettingsPC.BT_shutdownRobot, "")
        self.bluetoothLock.release()
        self.logger.info("Shutting down robot")
        self.lblStatusBar_RobotStatus.clear()
        self.lblStatusBar_Messages.clear()
    #endFct on_action_Shutdown_robot_triggered


    @pyqtSlot("bool")
    def on_action_Tracking_Testmode_triggered(self, checked):
        "Handles clicking on menu bar -> Tracking -> Test mode"
        if checked:
            # make robot start operations impossible while in test mode:
            self.btnStartStop.setEnabled(False)
            # no changes during tracking are allowed to evaluation settings
            self.grpbx_Tracking_EvaluationSettings.setEnabled(False)

            # make some evaluation inits
            self.stopTrackingEvent.clear()
            self.trackingPropertiesLock.acquire()
            TrackingProperties.TrackingFailed = False
            TrackingProperties.ResultReferenceTimeStamp = 0
            self.trackingPropertiesLock.release()

            self.modelPCLock.acquire()
            # start new tracking thread in testmode        
            trackingObj = Tracking("Thread-Tracking-Testmode", self.stopTrackingEvent, self.logger, self.bluetoothLock, self.bluetoothSocket, 0, array([]), array([]), SettingsPC.TestModeTrackingTime, -1, -1, self.modelPC.trackingPropertiesLock, self.modelPC.startPosMinRectSize, None, True)
            # connect some signals from Tracking to GUI
            self.connect(trackingObj, trackingObj.testModeTrackingFinished, self.TestModeTrackingFinished)
            self.connect(trackingObj, trackingObj.updateEvaluationStatusLabel, self.UpdateEvaluationStatusLabel)
            # start the thread and register the started thread in modelPC
            trackingObj.start()
            
            self.modelPC.evaluationThread = trackingObj
            self.modelPCLock.release()
        else:
            # stop tracking in testmode:
            self.modelPCLock.acquire()
            evaluationThread = self.modelPC.evaluationThread
            self.modelPCLock.release()
            if evaluationThread.isFinished():
                self.logger.warning("robotControlMainWindow:on_action_Tracking_Testmode_triggered: Tracking thread is not alive anymore => can't stop it")
            else:
                # stop Tracking thread:
                self.stopTrackingEvent.set()
            

            # re-enable start operations button if activated
            self.btnStartStop.setEnabled(self.state_StartbtnActivated)
            # re-enable evaluation settings after tracking:
            self.grpbx_Tracking_EvaluationSettings.setEnabled(True)
    #endFct on_action_Tracking_Testmode_triggered

    @pyqtSlot("bool")
    def TestModeTrackingFinished(self, value):
        "Set checkbox in menubar up-to-date"
        self.action_Tracking_Testmode.setChecked(False)
        # re-enable start operations button if activated
        self.btnStartStop.setEnabled(self.state_StartbtnActivated)
        # re-enable evaluation settings after tracking:
        self.grpbx_Tracking_EvaluationSettings.setEnabled(True)
    #endFct TestModeTrackingFinished

    @pyqtSlot("QString")
    def UpdateEvaluationStatusLabel(self, message):
        "Updates info in statusbar about current tracking"
        self.lblStatusBar_TrackingStatus.setText(str(message))
    #endFct UpdateEvaluationStatusLabel

    @pyqtSlot("QString")
    def UpdateRobotStatusLabel(self, message):
        "Updates info in statusbar about current tracking"
        self.lblStatusBar_RobotStatus.setText(str(message))
    #endFct UpdateRobotStatusLabel

    @pyqtSlot("QString")
    def UpdateStatusMessageLabel(self, message):
        "Updates info message in statusbar"
        self.lblStatusBar_Messages .setText(str(message))
    #endFct UpdateStatusMessageLabel

    @pyqtSlot("QString", "int", "bool")
    def UpdateMessageBox(self, message, loggingLevelNr, isPCMessage):
        "Updates the GUI its message box to display messages"
        # clear any user selections first:
        textCursorMessageBox = self.txtEditLogMessages.textCursor()
        textCursorMessageBox.clearSelection()
        self.txtEditLogMessages.setTextCursor(textCursorMessageBox)

        if isPCMessage:
        # do some formatting here:
            if loggingLevelNr >= logging.CRITICAL:
                self.txtEditLogMessages.setTextColor(QColor("red"))
                self.txtEditLogMessages.setFontWeight(QFont.Bold)
            elif loggingLevelNr >= logging.ERROR:
                self.txtEditLogMessages.setTextColor(QColor("red"))
                self.txtEditLogMessages.setFontWeight(QFont.Normal)
            elif loggingLevelNr >= logging.WARNING:
                self.txtEditLogMessages.setTextColor(QColor("black"))
                self.txtEditLogMessages.setFontWeight(QFont.Bold)
            elif loggingLevelNr >= logging.INFO:
                # info messages:
                self.txtEditLogMessages.setTextColor(QColor("black"))
                self.txtEditLogMessages.setFontWeight(QFont.Normal)
            else:
                # debug messages:
                self.txtEditLogMessages.setTextColor(QColor("gray"))
                self.txtEditLogMessages.setFontWeight(QFont.Light)
        else:
            # message from robot:
            self.txtEditLogMessages.setTextColor(QColor("blue"))
            if loggingLevelNr == logging.CRITICAL:
                self.txtEditLogMessages.setFontWeight(QFont.Bold)
            else:
                self.txtEditLogMessages.setFontWeight(QFont.Normal)

        self.txtEditLogMessages.append(str(message))
    #endFct UpdateMessageBox

    @pyqtSlot("bool")
    def Change_RobotStartable(self, startable):
        "Enable/disable start button based on received message of robot"
        settingsAllowed = False
        actionsAllowed = False
        if startable:
            self.btnStartStop.setEnabled(True)
            self.state_StartbtnActivated = True
            self.btnStartStop.setChecked(False)
            self.btnStartStop.setText("Start")
            self.btnStopRestartCycle.setEnabled(False)
            self.btnStopRestartCycle.setChecked(False)
            self.btnStopRestartCycle.setText("Stop Cycle")
            settingsAllowed = True
            actionsAllowed = True
            
        else:
            # Robot crashed => reset robot state in GUI:
            self.btnStartStop.setEnabled(False)
            self.state_StartbtnActivated = False
            self.btnStartStop.setChecked(False)
            self.btnStartStop.setText("Start")
            self.btnStopRestartCycle.setEnabled(False)
            self.btnStopRestartCycle.setChecked(False)
            self.btnStopRestartCycle.setText("Stop Cycle")
            settingsAllowed = True
            actionsAllowed = False
        #endIf robot startable

        # enable/disable some input fields according to operating robot or not:
        self.grpBxMotor1Params.setEnabled(settingsAllowed)
        self.grpBxMotor2Params.setEnabled(settingsAllowed)
        self.grpBxMotor3Params.setEnabled(settingsAllowed)
        self.grpbx_PSOSettings.setEnabled(settingsAllowed)
        self.grpbx_Tracking_EvaluationSettings.setEnabled(settingsAllowed)
        # enable/disable menu bar items:
        self.action_Download_status.setEnabled(actionsAllowed)  # state here does not depend on btnStopRestartCycle
        self.action_Set_parameters.setEnabled(actionsAllowed)
        self.action_Save_robot_state.setEnabled(actionsAllowed)  # state here does not depend on btnStopRestartCycle
        self.action_Load_robot_state.setEnabled(actionsAllowed)
        self.action_Shutdown_robot.setEnabled(actionsAllowed)
        self.action_Manual_operation.setEnabled(settingsAllowed)
        self.action_Particle_Swarm_operation.setEnabled(settingsAllowed)
        self.action_Tracking_Testmode.setEnabled(settingsAllowed)

        # disable signalling from setting fields wich are disabled:
        self.blockedRobotSettingsSignals(not settingsAllowed)
    #endFct Change_RobotStartable

    @pyqtSlot("bool")
    def Change_RobotReadyToRestart(self, restartable):
        "Enable/disable restart cycle button based on received messages from robot"
        if restartable:
            self.btnStopRestartCycle.setEnabled(True)
            # enable/disable menu bar items:
            self.action_Download_status.setEnabled(self.btnStartStop.isChecked())   # robot operations should be running here
            self.action_Save_robot_state.setEnabled(self.btnStartStop.isChecked())  # robot operations should be running here
        else:
            # disable button StopRestartCycle
            self.btnStopRestartCycle.setEnabled(False)
            self.btnStopRestartCycle.setChecked(False)
            self.btnStopRestartCycle.setText("Stop Cycle")
    #endFct Change_RobotReadyToRestart

    @pyqtSlot("double", "int", "int")
    def StartNewEvaluation(self, referenceTimestamp, optimizationIteration, loopIteration):
        "Robot asks a new evaluation cycle => start one"
        if not self.btnStartStop.isChecked():
            self.logger.error("RobotControlMainWindow:StartNewEvaluation: Robot asks a new evaluation while GUI start button is not pressed!")
            self.logger.info("RobotControlMainWindow:StartNewEvaluation: ignoring request from robot")
            return
        #endIf start button
        # enable stop cycle button:
        self.btnStopRestartCycle.setEnabled(True)
        self.btnStopRestartCycle.setChecked(False)
        self.btnStopRestartCycle.setText("Stop Cycle")

         # make some evaluation inits
        self.stopTrackingEvent.clear()
        self.trackingPropertiesLock.acquire()
        TrackingProperties.TrackingFailed = False
        TrackingProperties.ResultReferenceTimeStamp = 0
        self.trackingPropertiesLock.release()

        # start new evaluation thread:
        self.modelPCLock.acquire()
        
        # start new tracking thread in testmode     
        trackingObj = Tracking("Thread-Tracking-Operation{0}".format(datetime.datetime.fromtimestamp(referenceTimestamp).strftime("%Y-%m-%d_%H:%M:%S")), self.stopTrackingEvent, self.logger,
                               self.bluetoothLock, self.bluetoothSocket, referenceTimestamp, self.modelPC.speedPointsList, self.modelPC.intervalLengthsList, self.modelPC.operationCycleTime,
                               optimizationIteration, loopIteration, self.modelPC.trackingPropertiesLock, self.modelPC.startPosMinRectSize,self.ackAckStartNewEvaluationReceived, False)
        # connect some signals from Tracking to GUI
        self.connect(trackingObj, trackingObj.updateEvaluationStatusLabel, self.UpdateEvaluationStatusLabel)
        # start the thread and register the started thread in modelPC
        trackingObj.start()
        self.modelPC.evaluationThread = trackingObj
        self.modelPCLock.release()
    #endFct StartNewEvaluation

    @pyqtSlot()
    def StartRobotBootedDialog(self):
        "Starts a new dialog screen to let user choose which parameters to use in robot"
        dialogWindow = RobotBootedDialog(self.bluetoothLock, self.bluetoothSocket, self.modelPCLock, self.modelPC, self.logger, self)
        dialogWindow.show()
        res = dialogWindow.exec_()
        if res == 0:
            self.logger.warning("RobotControlWindow:StartRobotBootedDialog: User cancelled to choose which parameters to use => use PC parameters by default.")
            self.modelPCLock.acquire()
            modelPCDict = self.modelPC.__dict__
            self.modelPCLock.release()
            # NOTE: PSOstate is not integrated with gui
            parametersToSetDict = {variableName: modelPCDict[variableName] for variableName in SettingsPC.AllowedParametersToSet}  

            self.bluetoothLock.acquire()
            self.bluetoothSocket.SetParameters(parametersToSetDict.keys(), parametersToSetDict.values())
            self.bluetoothLock.release()
        #endIf res
        self.robotBootedWaitingOnUserChoice.clear()
        return
    #endFct StartRobotBootedDialog

    @pyqtSlot("bool")
    def on_action_Window_ClearMessages_triggered(self, checked):
        "Handle click events on menubar item ClearMessages"
        self.txtEditLogMessages.clear()

        #DEBUG
        self.grphcsViewMotor1.plotItem.clear()
    #endFct on_action_Window_ClearMessages_triggered

if __name__ == "__main__":
    import sys, linecache, traceback
    from stacktracer import trace_start, trace_stop
    trace_start("Log/threadsTrace-{0}.html".format(time.strftime("%Y-%m-%d_%H-%M-%S")), interval=5, auto=True)

    app = QApplication(sys.argv)

    mainPC = MainPC("Thread-MainPC")
    modelPC = mainPC.modelPC
    mainPC.start()

    window = RobotControlMainWindow(modelPC)
    window.show()
    try:
        app.exec_()
    except:
        exc_type, exc_obj, tb = sys.exc_info()
        frame = tb.tb_frame
        linenr = tb.tb_lineno
        filename = frame.f_code.co_filename
        linecache.checkcache(filename)
        line = linecache.getline(filename, linenr, frame.f_globals)
        modelPC.logger.critical("RobotControlMainWindow: EXCEPTION in {0} on line {1}: {2}".format(filename, linenr, line))
        modelPC.logger.critical("Unhandled Exception occurred in RobotControlMainWindow of type: {0}\n".format(sys.exc_info()[0]))
        modelPC.logger.info("Unhandled Exception value = {0}\n".format(sys.exc_info()[1] if sys.exc_info()[1] is not None else "None"))
        modelPC.logger.info("Unhandled Exception traceback = {0}\n".format(traceback.format_exc()))
        modelPC.logger.info("\RobotControlMainWindow CRASHED\n")

    modelPC.modelPCLock.acquire()
    modelPC.logger.debug("Exiting Main GUI")
    modelPC.stopTrackingEvent.set()
    modelPC.GUIfinishedEvent.set()
    modelPC.modelPCLock.release()

    trace_stop()


