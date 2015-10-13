import threading
import logging
from modelPC import ModelPC
from bluetoothSocketPC import BluetoothSocketPC
import sys, traceback
from PyQt4.QtCore import *
#from PyQt4.QtGui import *
import time
from settingsPC import SettingsPC
from loggingHandler_GUIMessageBox import LoggingHandler_GUIMessageBox
import serial
from TrackingProperties import TrackingProperties
# DEBUG
from numpy import *

class MainPC(QThread):
    """ Main class of PC controlling ball robot"""

    def __init__(self, threadName, parent = None):
        super(MainPC, self).__init__(parent)
        # inits
        self.name = threadName
        # init logger here
        self.logger = logging.getLogger("PC_Main_Logger")
        self.logger.setLevel(logging.DEBUG)
        # handler to stdOutput:
        loggingHandlerStdIO = logging.StreamHandler(sys.stdout)
        loggingHandlerStdIO.setLevel(logging.DEBUG)
        loggingHandlerStdIO.setFormatter(logging.Formatter("%(levelname)s::%(threadName)s::%(asctime)s::%(message)s"))
        self.logger.addHandler(loggingHandlerStdIO)

         # init logger GUI handler:
        loggingHandlerGUI = LoggingHandler_GUIMessageBox()
        loggingHandlerGUI.setLevel(logging.DEBUG)  # set to warning?
        self.logger.addHandler(loggingHandlerGUI)
        # init file handler here
        fileLoggingHandler = logging.FileHandler(SettingsPC.FileLogPath + "PC_logFile_{0}.log".format(time.strftime("%Y-%m-%d %H-%M-%S")))
        fileLoggingHandler.setLevel(logging.DEBUG)
        formatterFileLog = logging.Formatter("%(levelname)s::%(threadName)s::%(asctime)s::%(message)s")
        fileLoggingHandler.setFormatter(formatterFileLog)
        self.logger.addHandler(fileLoggingHandler)

        self.bluetoothLock = threading.Lock()
        self.trackingPropertiesLock = threading.Lock()
        self.GUIfinishedEvent = threading.Event()
        self.modelPCLock = threading.Lock()
        self.stopTrackingEvent = threading.Event()
        self.trackingFinishedEvent = threading.Event()
        self.guiInitedEvent = threading.Event()

        # events to signal robotstates that require handling
        self.robotBooted = threading.Event()
        self.robotIsReadyToStart = threading.Event()
        self.robotAskedEvaluation = threading.Event()
        self.robotBootedWaitingOnUserChoice = threading.Event()

        # init bluetoothSocket here:
        try:
            socket = serial.Serial(SettingsPC.BT_port, SettingsPC.BT_baudrate, timeout=SettingsPC.BT_Timeout)
            self.bluetoothSocket = BluetoothSocketPC(socket, self.modelPCLock, self.logger, self.robotBooted, self.robotIsReadyToStart, self.robotAskedEvaluation, self.robotBootedWaitingOnUserChoice)
        except:
            self.logger.critical("Exception occurred while initiating bluetoothsocket of type {0}".format(sys.exc_info()[0]))
            self.logger.info("Unhandled Exception value = {0}".format(sys.exc_info()[1] if sys.exc_info()[1] is not None else "None"))
        
        self.modelPC = ModelPC(self.logger, self.bluetoothLock, self.bluetoothSocket, self.trackingPropertiesLock, self.modelPCLock, self.GUIfinishedEvent, self.stopTrackingEvent, 
                               self.guiInitedEvent, loggingHandlerGUI.signalHolder)
        # add modelPC to bluetoothSocket
        self.bluetoothSocket.modelPC = self.modelPC

    def run(self):
        try:
            # set threadName:
            threading.currentThread().setName(self.name)

            # wait for gui to init:
            self.guiInitedEvent.wait()
            self.logger.debug("MainPC started running")

            #time.sleep(3)
            ## DEBUG:
            ## testing upload parameters:
            #self.bluetoothLock.acquire()
            #self.logger.debug("DEBUG: starting upload of parameters")
            #self.bluetoothSocket.RobotUploadedStatus({"saveRobotStateOnInterruption": True, "robotOperationType" : "Manual",
            #                                          "PSOsettings" : {"swarmsize": 99, "omega": 0.99, "phig": 0.99, "phip": 0.99, "maxiter": 999, "minstep": 9e-8, "minfunc": 9e-8},
            #                                          "PSOstate" : {"overwriteState": True, "S": 0, "D": 0, "x": array([]), "v": array([]), "p": array([]), "fp": array([]), "g": [], "fg": 0, "optimizationIteration": 0},
            #                                          "evaluationMetricToUse" : "AverageSpeed", "saveIntermediateStates" : True, "intermediateSaveRobotStateEvaluationThreshold" : 999,
            #                          "operationCycleTime" : 99, "maxMotorIntervalLength" : 101, "startPosMinRectSize" : 99, "speedPointsList": arange(0,27).reshape((3,9)), "intervalLengthsList" : arange(0,27).reshape((3,9))})
            #self.bluetoothLock.release()
            #self.logger.debug("DEBUG: uploading finished")

            # DEBUG:
            #self.bluetoothLock.acquire()
            #self.bluetoothSocket.RequestTime("")
            #self.bluetoothLock.release()

            # loop of handling interactions with robot
            while not self.GUIfinishedEvent.isSet():
                # processIncomingMessages:
                self.bluetoothLock.acquire()
                self.bluetoothSocket.ProcessIncomingMessages( allowedMessages = [SettingsPC.BT_logMessage, SettingsPC.BT_requestTime, SettingsPC.BT_readyToStart, SettingsPC.BT_startNewEvaluation, SettingsPC.BT_ackAckStartNewEvaluation,
                                                                                      SettingsPC.BT_askResultEvaluation, SettingsPC.BT_uploadStatus, SettingsPC.BT_readyToRestartCycle, SettingsPC.BT_notifyOperationCycleProgress,
                                                                                      SettingsPC.BT_respAvailableRobotStates, SettingsPC.BT_notifyRobotInError])
                self.bluetoothLock.release()

                # if requestTime received:
                # - responseTime
                # - stop any evaluations, reset PC state to init
                # - demand via dialog popup the action: set current GUI parameters, get robot parameters, load other robot parameters
                if self.robotBooted.isSet():
                    # stop any evaluations if running:
                    self.modelPCLock.acquire()
                    if self.modelPC.evaluationThread is not None and self.modelPC.evaluationThread.isRunning():
                        # NOTE: running evaluation could be a testmode tracking
                        self.stopTrackingEvent.set()
                        self.logger.warning("Robot just booted and an evaluation is still running...\n OK?\n")
                    #endIf tracking thread running
                    self.modelPCLock.release()
                    # Event handled => clear for next case
                    self.robotBooted.clear()
                #endIf robot booted
           
                # if robotReadyToStart received:
                # - SIG: make btnStart operations enabled
                # - SIG: set robotReady in statusbar
                # - stop any evaluations
                if self.robotIsReadyToStart.isSet():
                    self.modelPCLock.acquire()
                    if self.modelPC.evaluationThread is not None and self.modelPC.evaluationThread.isRunning():
                        # NOTE: running evaluation could be a testmode tracking
                        # self.stopTrackingEvent.set()
                        self.logger.warning("Robot is ready to start and an evaluation is still running... \n OK?\n")
                    self.modelPCLock.release()
                    # Event handled => clear for next case
                    self.robotIsReadyToStart.clear()
                #endIf robot is ready to start operations

                # if askResultEvaluation received:
                # set event that main starts checking trackingProperties result reference timestamp for a match before sending a response
                # - wait for current evaluationThread to finish first if running or timeout?
                # - check if evalResultReference timestamp matches with received message
                # - respond result or evt TODO: send stop Cycle and restart Cycle to restart this operation if evaluation failed and not guiStopCyclePressed.isSet() and not guiStopOperationsPressed.isSet()
                # - SIG: disable stop cycle button on finish
                # set robot status: received evaluation result
                if self.robotAskedEvaluation.isSet():
                    # compare evaluation result reference timestamp with the robot request reference timestamp
                    self.bluetoothLock.acquire()
                    requestTimestamp = self.bluetoothSocket.robotAskedEvaluationResultReferenceTimestamp
                    self.bluetoothLock.release()
                    self.trackingPropertiesLock.acquire()
                    evaluationTimestamp = TrackingProperties.ResultReferenceTimeStamp
                    trackingFailed = TrackingProperties.TrackingFailed
                    trackingResult = TrackingProperties.ResultTracking
                    self.trackingPropertiesLock.release()
                    if trackingFailed:
                        self.logger.error("MainPC: Waiting on the evaluation result but noticed that tracking failed!")
                        # an auto stop robot cycle %EVT: and immediately restart it without user intervention
                        self.modelPCLock.acquire()
                        self.modelPC.Stop_RobotOperationCycle()
                        self.modelPCLock.release()
                        self.robotAskedEvaluation.clear()
                        continue
                    if abs(requestTimestamp - evaluationTimestamp) < 1e-10:
                        # requested evaluation result is ready to be sent:
                        self.trackingPropertiesLock.acquire()
                        evaluationResult = TrackingProperties.ResultTracking
                        self.trackingPropertiesLock.release()
                        self.bluetoothLock.acquire()
                        self.bluetoothSocket.RespAskEvaluation(evaluationResult)  # bluetooth class can emit a signal to GUI to disable the stop cycle button
                        self.bluetoothLock.release()
                        # check if robot leaved its starting position:
                        if SettingsPC.AutoStopNextOperationCycle_onRobotLeavedRect and trackingResult["RobotLeavesStartRect"]:
                            # if autostop robot when he leaved its position: robot needs to be repositioned
                            self.modelPCLock.acquire()
                            self.modelPC.Stop_RobotOperationCycle()
                            self.modelPC.UpdateGUI_StatusMessage("Reposition robot...")
                            self.modelPCLock.release()
                            self.logger.info("Please REPOSITION robot and continue next cycle...")

                        self.robotAskedEvaluation.clear()
                    else:
                        # requested evaluation result is not yet ready -> check if not yet timeout:
                        self.bluetoothLock.acquire()
                        timeoutTime = self.bluetoothSocket.robotAskedEvaluationTimeoutTime
                        self.bluetoothLock.release()
                        if time.time() >= timeoutTime:
                            self.logger.critical("MainPC: Waiting on the evaluation result to respond to the robot took too long: timeout!")
                            # implement an auto stop robot cycle #EVT: and immediately restart it without user intervention
                            self.modelPCLock.acquire()
                            self.modelPC.Stop_RobotOperationCycle()
                            self.modelPCLock.release()
                            self.robotAskedEvaluation.clear()
                        #endIf timeout
                    #endIf comparing evaluation timestamps
                #endIf robotAskedEvaluation
                time.sleep(0.25)
            #endWhile main loop
            self.logger.debug("MainPC exiting")
            return
        except:
            self.logger.critical("Unhandled Exception occurred in MainPC:Run of type: {0}\n".format(sys.exc_info()[0]))
            self.logger.info("Unhandled Exception value = {0}\n".format(sys.exc_info()[1] if sys.exc_info()[1] is not None else "None"))
            self.logger.info("Unhandled Exception traceback = {0}\n".format(traceback.format_exc()))
            self.logger.info("\MainPC CRASHED\n")
