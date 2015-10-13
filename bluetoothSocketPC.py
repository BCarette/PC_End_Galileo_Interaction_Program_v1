import logging
from settingsPC import SettingsPC
from PyQt4.QtCore import *
import time
import re
import pickle
import threading

class BluetoothSocketPC(QObject):
    """description of class"""

    def __init__(self, serialSocket, modelPCLock, logger, robotBooted, robotIsReadyToStart, robotAskedEvaluation, robotBootedWaitingOnUserChoice, parent = None):
        super(BluetoothSocketPC, self).__init__(parent)
        self.serialSocket = serialSocket
        self.logger = logger
        #self.modelPC = modelPC  # NOTE: set manually via mainPC to avoid the chicken or the egg causality problem
        self.modelPCLock = modelPCLock
        self.sendingBluetoothMessagesActivated = False
        self.fletcherChecksumConstants = SettingsPC.BT_checksumConstants
        self.packageNrIn = 0
        self.packageNrOut = 0
        # evaluation signaling:
        self.ackAckStartNewEvaluationReceived = threading.Event()
        self.lastStartNewEvaluationReferenceTimestamp = 0

        # robot interaction data:
        self.robotReadyToRestartCycle_ReferenceTimestamp = 0
        self.robotBooted = robotBooted
        self.robotIsReadyToStart = robotIsReadyToStart
        self.robotAskedEvaluationResultReferenceTimestamp = 0
        self.robotAskedEvaluation = robotAskedEvaluation
        self.robotAskedEvaluationTimeoutTime = 0
        self.robotBootedWaitingOnUserChoice = robotBootedWaitingOnUserChoice

        # signal to pass available robot states to QDialog if connected
        self.availableRobotStateToLoadReceived = SIGNAL("availableRobotStateToLoadReceived")  # short circuited signal
        # signal to pass log messages
        self.updateGUIMessageBox = SIGNAL("newRobotLogMessage(QString, int, bool)")
        # signal to update robot status:
        self.updateRobotStatus = SIGNAL("updateRobotStatus(QString)")
        self.notifyRobotStartable = SIGNAL("notifyRobotStartable(bool)")
        self.notifyRobotReadyToRestartCycle = SIGNAL("notifyRobotReadyToRestartCycle(bool)")
        # signal to start a new tracking cycle:
        self.startNewTrackingCycle = SIGNAL("startNewTrackingCycle(double, int, int)")
        # update GUI progressbar:
        self.updateProgressBar = SIGNAL("updateProgressBar(int)")
        # update statusbar info message:
        self.updateStatusMessage = SIGNAL("updateStatusMessage(QString)")
        # notify main window to show pop-up screen robotBootedDialog:
        self.startRobotBootedDialog = SIGNAL("startRobotBootedDialog()")

        # construct dictionary of methods to call for each type of incoming bluetooth command:
        self.processIncomingMessageOfType = {SettingsPC.BT_logMessage: self.RobotLogMessage, SettingsPC.BT_requestTime: self.RequestTime, SettingsPC.BT_readyToStart: self.RobotReadyToStart,
                                             SettingsPC.BT_startNewEvaluation: self.StartNewEvaluation, SettingsPC.BT_ackAckStartNewEvaluation: self.AckAckStartNewEvaluation,
                                             SettingsPC.BT_askResultEvaluation: self.AskResultEvaluation, SettingsPC.BT_uploadStatus: self.RobotUploadedStatus,
                                             SettingsPC.BT_readyToRestartCycle: self.RobotReadyToRestartCycle, SettingsPC.BT_notifyOperationCycleProgress: self.NotifyOperationCycleProgress,
                                             SettingsPC.BT_respAvailableRobotStates: self.ResponseAvailableRobotStates, SettingsPC.BT_notifyRobotInError: self.NotifyRobotInError}
    #endFct __init__

    def SendMessage(self, command, data):
        "Encapsulate data and transmit via serialSocket"
        if not self.sendingBluetoothMessagesActivated:
            self.logger.debug("BluetoothSocketPC:SendMessage: Sending bluetooth messages not activated now")
            return
        self.logger.debug("BluetoothSocket sending protocol {0}, data = {1}".format(command, data))

        if data != "":
            # serialize and encapsulate the data to transmit
            dataMsg = chr(self.packageNrOut) + pickle.dumps(data, pickle.HIGHEST_PROTOCOL)
            dataMsg += self.fletcher16(dataMsg)
            # datamsg may not be longer than 65535: because only 2 bytes used in command message for number of bytes to expect
            if len(dataMsg) > 65535:
                self.logger.error("bluetoothSocketPC tries to send a message with length = {0}\n".format(len(dataMsg)))
                self.logger.info("Failed to send bluetooth command {0} with data = {1}".format(command, data))
                return

            # construct the command message
            dataMsgBytes = len(dataMsg)
            commandMsg = chr(command) + chr(self.packageNrOut) + chr(dataMsgBytes >> 8) + chr(dataMsgBytes & 0xff)
            
            # append checksum to command msg
            commandMsg += self.fletcher16(commandMsg)

            # transmit messages
            self.serialSocket.write(commandMsg)
            self.serialSocket.write(dataMsg)
        else:
            # no data message:
            commandMsg = chr(command) + chr(self.packageNrOut) + chr(0) + chr(0)
            
            # append checksum to command msg
            commandMsg += self.fletcher16(commandMsg)

            # transmit messages
            self.serialSocket.write(commandMsg)
        #endIf send message

        # update package number out for next message
        self.packageNrOut = self.packageNrOut + 1 if self.packageNrOut < 255 else 0
    #endFct SendMessage

    def ProcessIncomingMessages(self, allowedMessages):
        """
        Checks if incoming messages available and processes them.
        
        :param allowedMessages: list of uint8, allow these types of messages
        """
        if allowedMessages is None:
            return

        # check amount of bytes in input buffer:
        inputBufferBytes = self.serialSocket.inWaiting()
        if inputBufferBytes == 0:
            # nothing received
            return
        elif inputBufferBytes < SettingsPC.BT_commandMessageLength:
            # not yet enough bytes received for an entire command message
            # wait max a few ms for this command message: else wrong bytes waiting in inputbuffer => remove them
            timeoutTime = time.time() + SettingsPC.BT_maxWaitingTimeInputCommand
            while self.serialSocket.inWaiting() < SettingsPC.BT_commandMessageLength:
                if time.time() >= timeoutTime:
                    # waiting time is up: the bytes in the inputbuffer have to be wrong => flush input buffer
                    self.serialSocket.flushInput()
                    return
                #endIf timeout
                time.sleep(0.001)
            #endWhile waiting for command bytes to be complete
        #endIf enough bytes in input buffer

        # read command bytes:
        self.serialSocket.timeout = 0.001  # set timeout to read command message
        commandMsg = self.serialSocket.read(SettingsPC.BT_commandMessageLength)

        # check first if commandMsg is not corrupted via checksum bytes
        if not self.check_fletcher16(commandMsg):
            # read message is not a valid commandMsg:
            # evt log event: but risk for overflowing logfile
            # not a valid command message => expect all other bytes in input buffer to be wrong => clear input buffer
            self.serialSocket.flushInput()
            return

        # at this point a valid command message is received:
        # extract package number + number of data message bytes to expect next
        protocolCode = ord(commandMsg[0])
        self.packageNrIn = ord(commandMsg[1])
        expectedDataMsgBytes = (ord(commandMsg[2]) << 8) + ord(commandMsg[3])
        inputData = None

        if expectedDataMsgBytes > 0:
            # wait for next accompanying data message:
            self.serialSocket.timeout = float(expectedDataMsgBytes) / SettingsPC.BT_baudrate + SettingsPC.BT_Timeout  # add some default time for robustness
            dataMsg = self.serialSocket.read(expectedDataMsgBytes)
            # check received data its checksum
            if self.check_fletcher16(dataMsg):
                # process incoming data message:
                
                if ord(dataMsg[0]) != self.packageNrIn:
                    # package number does not match with received command message => neglect messages
                    self.logger.warning("BluetoothSocketPC:ProcessIncomingMessages: Received data message package number {0} does not correspond to received command message number {1}".format(ord(dataMsg[0]), self.packageNrIn))
                    return
                #endIf packageNrIn
                inputData = pickle.loads(dataMsg[1:-2])  # cut packagNrIn and checksum bytes of to extract the data
            else:
                # corrupted data message => neglect messages
                return
            #endIf processing data message
        #endIf data message follows command

        # check if this command should currently be processed, else neglect it:
        if protocolCode not in allowedMessages:
            self.logger.info("BluetoothSocketPC:ProcessIncomingMessages: ProcotolCode {0} is not in list {1} of allowed messages => neglected message with data: {2}".format(protocolCode, allowedMessages, inputData if inputData is not None else "None"))
            return
        #endIf allowedMessages

        self.logger.debug("BluetoothSocketPC:ProcessIncomingMessages: Received a command {0} with data = {1}".format(protocolCode, inputData))
        # handle command and inputData to its corresponding custom processing function:
        self.processIncomingMessageOfType[protocolCode](inputData)
        return
    #endFct ProcessIncomingMessages

    def fletcher16(self, data):
        "calculates a 16bit checksum according to the Fletcher-16 algorithm"
        sum1 = self.fletcherChecksumConstants[0]
        sum2 = self.fletcherChecksumConstants[1]

        for char in data:
            sum1 += ord(char)
            sum2 += sum1
            sum1 %= 255
            sum2 %= 255
        
        # calculate checkbytes
        CB0 = 255 - ((sum1 + sum2) % 255)
        CB1 = 255 - ((sum1 + CB0) % 255)
        return chr(CB0)+chr(CB1)
    #endFct fletcher16

    def check_fletcher16(self, msg):
        "checks if msg its 16bit checksum at the end is correct according to the Fletcher 16 algorithm"
        sum1 = self.fletcherChecksumConstants[0]
        sum2 = self.fletcherChecksumConstants[1]

        for char in msg:
            sum1 += ord(char)
            sum2 += sum1
            sum1 %= 255
            sum2 %= 255

        return sum1 == 0  and sum2 == 0
    #endFct check_fletcher16

    ########### Custom sending functions ###########
    def SetParameters(self, nameList, valueList):
        "Sends a message to robot to set all parameters contained in nameList to their corresponding value"
        if not (set(nameList) <= SettingsPC.AllowedParametersToSet):
            self.logger.error("BluetoothSocketPC:SetParameters: Trying to set a parameter which is not allowed: {0}".format(nameList))
            return
        #endIf allowed parameters to set

        # fill a dictionairy with all parameters to set
        parameterDict = {nameList[i] : valueList[i] for i in range(0, len(nameList))}
        #for i in range(0, len(nameList)):
        #    parameterDict[nameList[i]] = valueList[i]

        self.SendMessage(SettingsPC.BT_setParameters, parameterDict)
    #endFct SetParameters

    def DownloadStatus(self, nameList):
        "Sends a message to robot demanding to upload the values of the asked variables in nameList"
        if not (set(nameList) <= set(SettingsPC.AvailableRobotParametersToDownload)):
            self.logger.error("BluetoothSocketPC:DownloadStatus: Trying to get a parameter which is not allowed: {0}".format(nameList))
            return
        #endIf allowed parameters to download

        self.SendMessage(SettingsPC.BT_downloadStatus, nameList)
    #endFct DownloadStatus

    def RespAskEvaluation(self, evaluationResult):
        "Performs all necessary actions to notify robot about its evaluation result"
        # disable Stop/restartCycle button
        self.emit(self.notifyRobotReadyToRestartCycle, False)
        # respond to robot:
        self.SendMessage(SettingsPC.BT_respAskResultEvaluation, evaluationResult)
    #endFct RespAskEvaluation

    ########### Custom receiving functions ###########

    def RobotLogMessage(self, inputData):
        "Handles incoming log messages from the robot"
        if type(inputData) != str:
            self.logger.error("BluetoothSocketPC:RobotLogMessage: Expects data of type str but received: {0}".format(type(inputData)))
            return
        #endIf valid inputData
        # log the message to PC logfile:
        self.logger.debug("ROBOT-logmessage::" + inputData)
        # determine level of logmessage:
        matchDebug = re.match(r"^DEBUG::", inputData)
        matchInfo = re.match(r"^INFO::", inputData)
        matchWarning = re.match(r"^WARNING::", inputData)
        matchError = re.match(r"^ERROR::", inputData)
        matchCritical = re.match(r"^CRITICAL::", inputData)
        levelNr = 0

        if matchDebug is not None:
            levelNr = logging.DEBUG
        elif matchInfo is not None:
            levelNr = logging.INFO
        elif matchWarning is not None:
            levelNr = logging.WARNING
        elif matchError is not None:
            levelNr = logging.ERROR
        elif matchCritical is not None:
            levelNr = logging.CRITICAL

        self.emit(self.updateGUIMessageBox, inputData, levelNr, False)
    #endFct RobotLogMessage        

    def RequestTime(self, inputData):
        "Handles incoming requestTime messages"
        self.SendMessage(SettingsPC.BT_responseTime, time.time())  # NOTE: make sure that timezone settings in galileo match with PC!
        
        # only handle 1 booted request at a time:
        if not self.robotBootedWaitingOnUserChoice.isSet():
            self.robotBootedWaitingOnUserChoice.set()
            # PC knows that robot just booted now:
            self.robotBooted.set()
            # allow PC to send bluetooth messages now:
            self.sendingBluetoothMessagesActivated = True
            # signal to robotControlMainWindow to start a dialog screen requesting: set GUI, load robot or load a saved state
            self.emit(self.startRobotBootedDialog)
            # update statusbar: from "no robot connected" -> robot not ready
            self.emit(self.updateRobotStatus, "Robot not ready")
    #endFct RequestTime

    def RobotReadyToStart(self, inputData):
        "Processes incoming readyToStart messages of the robot"
        self.robotIsReadyToStart.set()
        self.emit(self.updateRobotStatus, "Robot ready")
        self.emit(self.updateProgressBar, 0) # autoset progressbar to 0
        # make btnStart operations enabled
        self.emit(self.notifyRobotStartable, True)
    #endFct RobotReadyToStart

    def StartNewEvaluation(self, inputData):
        "Processes incoming startNewEvaluation messages from robot"
        if type(inputData) != dict:
            self.logger.error("BluetoothSocketPC:StartNewEvaluations:Incoming data type {0} is not a dict".format(type(inputData)))
            return
        #endIf valid inputdata type
        # expects: {"operationTimeStamp": operationTimeStamp, "optimizationIteration": optimizationIteration, "loopIteration": loopiteration ,"speedPointsList": speedPointsList, "intervalLengthsList": intervalLengthsList, 
        #           "operationCycleTime": operationCycleTime}

        # if startNewEvaluation received:
        # - SIG: enable stop cycle button
        # - stop any running evaluations
        # - start new evaluations thread
        self.modelPCLock.acquire()
        if self.modelPC.evaluationThread is not None and self.modelPC.evaluationThread.isRunning():
            self.modelPC.stopTrackingEvent.set()
            self.logger.error("BluetoothSocketPC:StartNewEvaluations: Robot requests a new evaluation cycle, but there is still one running on the PC")
        # set new robot data
        self.modelPC.Change_MotorSpeedPointsList(inputData["speedPointsList"])
        self.modelPC.Change_MotorIntervalLengthsList(inputData["intervalLengthsList"])
        if self.modelPC.operationCycleTime != inputData["operationCycleTime"]:
            self.logger.critical("BluetoothSocketPC:StartNewEvaluation: Robot its operation cycle time ({0}) does not match with this of PC: {1}".format(inputData["operationCycleTime"], self.modelPC.operationCycleTime))
            # default: use setting of robot:
            self.modelPC.Change_OperationCycleTime(inputData["operationCycleTime"])
        self.modelPCLock.release()
        # signal to robotControlMainWindow to start a new evaluation:
        self.lastStartNewEvaluationReferenceTimestamp = inputData["operationTimeStamp"]
        self.ackAckStartNewEvaluationReceived.clear() 
        self.emit(self.startNewTrackingCycle, inputData["operationTimeStamp"], inputData["optimizationIteration"], inputData["loopIteration"])
        self.emit(self.updateRobotStatus, "Robot running")
        if inputData["optimizationIteration"] != -1:
            # update statusbar with current optimizationIteration:
            self.emit(self.updateStatusMessage, "Optimization iteration = {0}, loop iteration = {1}".format(inputData["optimizationIteration"], inputData["loopIteration"]))
        else:
            self.emit(self.updateStatusMessage, "")
    #endFct StartNewEvaluation

    def RobotReadyToRestartCycle(self, inputData):
        "Processes incoming readyToRestart messages from robot"
        if type(inputData) != float:
            self.logger.error("BluetoothSocketPC:RobotReadyToRestartCycle: Incoming data type {0} is not a float".format(type(inputData)))
            return
        #endIf valid inputData type
        # if readyToRestartCycle received:
        # - stop any running evaluations
        # - enable btnRestartCycle enabled -> some GUI parameters become enabled
        self.robotReadyToRestartCycle_ReferenceTimestamp = inputData
        # stop any running evaluations:
        self.modelPCLock.acquire()
        if self.modelPC.evaluationThread is not None and self.modelPC.evaluationThread.isRunning():
            self.modelPC.stopTrackingEvent.set()
            self.logger.warning("BluetoothSocketPC:RobotReadyToRestartCycle: A tracking thread was still running.")
        self.modelPCLock.release()
        # notify GUI about robot being ready to restart:
        self.emit(self.notifyRobotReadyToRestartCycle, True)
        self.emit(self.updateRobotStatus, "Robot ready to restart")
        self.emit(self.updateProgressBar, 0) # autoset progressbar to 0
    #endFct RobotReadyToRestartCycle

    def AskResultEvaluation(self, inputData):
        "Processes incoming askResultEvaluation messages from robot"
        if type(inputData) != float:
            self.logger.error("BluetoothSocketPC:RobotReadyToRestartCycle: Incoming data type {0} is not a float".format(type(inputData)))
            return
        #endIf valid inputData type
        self.robotAskedEvaluationResultReferenceTimestamp = inputData
        self.robotAskedEvaluationTimeoutTime = time.time() + SettingsPC.EvaluationResponseTimeout
        self.robotAskedEvaluation.set()
        self.emit(self.updateRobotStatus, "Robot asking evaluation result")
        # robot main handles the waiting on the evaluation thread to respond to the robot
        return
    #endFct AskResultEvaluation
    def ResponseAvailableRobotStates(self, inputData):
        "Process incoming response of robot its available robotstates"
        if type(inputData) != list:
            self.logger.error("BluetoothSocketPC:ResponseAvailableRobotStates:Incoming data type {0} is not a list".format(type(inputData)))
            return
        #endIf valid data type
        # emit signal that robotstates are received:
        self.emit(self.availableRobotStateToLoadReceived, inputData)
    #endFct ResponseAvailableRobotStates

    def RobotUploadedStatus(self, inputData):
        "Processes incoming uploadStatus messages from robot: update PC variables with uploaded ones"
        if type(inputData) != dict:
            self.logger.error("BluetoothSocketPC:RobotUploadedStatus:Incoming data type {0} is not a dict".format(type(inputData)))
            return
        #endIf valid data type

        # check if inputData does not contain any unallowed parameters to set
        if not set(inputData.keys()) <=  SettingsPC.AllowedParametersToSetInPC:
            self.logger.error("BluetoothSocketPC:RobotUploadedStatus: Tries to set a parameter which is not allowed! Given parameters = {0}".format(inputData.keys()))
            return
        #endIf allowed variables to set
        self.logger.debug("BluetoothSocketPC:RobotUploadedStatus: Uploading to PC variables: {0}".format(inputData.keys()))
        self.modelPCLock.acquire()
        # make sure that maxMotorIntervalLength variable is set before new motor parameters:
        if "maxMotorIntervalLength" in inputData:
            # update this variable first
            self.modelPC.setterMethodsDict["maxMotorIntervalLength"](inputData.pop("maxMotorIntervalLength", None))

        # NOTE: no checks if inputData its field are valid data are done here
        for key, value in inputData.items():
            # set each entry from inputData in modelPC
            #self.logger.debug("BT_PC:UploadedStatus:setterMethodsDict[{0}] with value = {1}".format(key, value))
            self.modelPC.setterMethodsDict[key](value)
        self.modelPCLock.release()
    #endFct RobotUploadedStatus

    def NotifyOperationCycleProgress(self, inputData):
        "Processes incoming notifyOperationCycleProgress messages from robot"
        #self.logger.debug("BluetoothSocketPC:NotifyOperationCycleProgress: Function started with data = {0}".format(inputData))
        if type(inputData) != tuple:
            self.logger.error("BluetoothSocketPC:NotifyOperationCycleProgress:Incoming data type {0} is not a tuple".format(type(inputData)))
            return
        #endIf valid data type
        operationCycleProgress = round((inputData[0] / float(inputData[1])) * 100)
        self.emit(self.updateProgressBar, int(operationCycleProgress))
        #self.logger.debug("BluetoothSocketPC:NotifyOperationCycleProgress: emitted signal updateProgressBar")
    #endFct NotifyOperationCycleProgress

    def NotifyRobotInError(self, inputData):
        "Processes incoming notifyRobotInError messages from robot"
        self.logger.critical("Robot is in ERROR state!")
        self.emit(self.updateRobotStatus, "Robot in ERROR")
        self.sendingBluetoothMessagesActivated = False
        self.emit(self.notifyRobotStartable, False)
    #endFct NotifyRobotInError

    def AckAckStartNewEvaluation(self, inputData):
        "Processes incoming ackAckStartNewEvaluation messages"
        if type(inputData) != float:
            self.logger.error("BluetoothSocketPC:AckAckStartNewEvaluationReceived: Expecting a float but received {0}".format(type(inputData)))
            return
        #endIf valid data type

        # check if ack ack reference timestamp matches last received start new evaluation timestamp:
        if abs(self.lastStartNewEvaluationReferenceTimestamp - inputData) > 1e-10:
            self.logger.warning("BluetoothSocketPC:AckAckStartNewEvaluation: Received operation timestamp does not match with last received start new evaluation timestamp!")
            return
        #endIF reference timestamp

        self.ackAckStartNewEvaluationReceived.set()
    #endFct AckAckStartNewEvaluation

