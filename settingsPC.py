from numpy import *

class SettingsPC(object):
    """
    Class containing global settings of PC
    """

    # environment settings:
    FileLogPath = "Log/"
    BT_port = "COM5"
    BT_baudrate = 115200
    BT_Timeout = 0.002  # 2ms timeout default
    
    # bluetooth settings
    BT_checksumConstants = array([41, 22], dtype= uint8)
    BT_commandMessageLength = 6  # 1 command byte, 1 packageNr byte, 2 #bytes byte, 2 checksum bytes
    BT_protocolMessageTimeout = 10  # time in seconds that PC waits in its Bluetooth protocol until aborting its operation
    BT_maxWaitingTimeInputCommand = 0.002  # in seconds, wait at most 2ms for a full command message to arrive if it is only partially arrived

    # Each bluetooth protocol should have a unique command code:
    BT_shutdownRobot = uint8(0)
    BT_logMessage = uint8(1)
    BT_requestTime = uint8(2)
    BT_responseTime = uint8(3)
    BT_readyToStart = uint8(4)
    BT_startOperations = uint8(5)
    BT_stopOperations = uint8(6)
    BT_startNewEvaluation = uint8(7)
    BT_ackStartNewEvaluation = uint8(8)
    BT_ackAckStartNewEvaluation = uint8(9)
    BT_askResultEvaluation = uint8(10)
    BT_respAskResultEvaluation = uint8(11)
    BT_setParameters = uint8(12)
    BT_downloadStatus = uint8(13)
    BT_uploadStatus = uint8(14)
    BT_stopCycle = uint8(15)
    BT_readyToRestartCycle = uint8(16)
    BT_restartCycle = uint8(17)
    BT_notifyOperationCycleProgress = uint8(18)
    BT_saveRobotState = uint8(19)
    BT_getAvailableRobotStates = uint8(20)
    BT_respAvailableRobotStates = uint8(21)
    BT_loadRobotState = uint8(22)
    BT_notifyRobotInError = uint8(23)
    

    # View frame options during tracking:
    AvailableFramesToView = ["maskColourThreshold", "res_MorphOp", "contourFilters_on_res_MorphOp", "TrackedBall"]
    AvailableEvaluationMetrics = ["TotalDistance", "AverageSpeed", "RobotLeavesStartRect", "invTotalDistance_LeavingRect"]
    
    # params of robot:
    AvailableRobotParametersToDownload = ["saveRobotStateOnInterruption", "robotOperationType", "PSOsettings", "evaluationMetricToUse", "saveIntermediateStates", 
                                          "intermediateSaveRobotStateEvaluationThreshold", "operationCycleTime", "maxMotorIntervalLength", "speedPointsList", "intervalLengthsList"]
    # ! This set is bound to the variable names in the robot project!
    # AllowedParametersToSet in robot:
    AllowedParametersToSet = set(["saveRobotStateOnInterruption", "robotOperationType", "PSOsettings", "PSOstate", "evaluationMetricToUse", "intermediateSaveRobotStateEvaluationThreshold", 
                                           "saveIntermediateStates", "operationCycleTime", "speedPointsList", "intervalLengthsList", "maxMotorIntervalLength", "startPosMinRectSize"])
    # Allowed parameters to set in PC:
    AllowedParametersToSetInPC = set(["saveRobotStateOnInterruption", "robotOperationType", "PSOsettings", "PSOstate", "evaluationMetricToUse", "saveIntermediateStates", "intermediateSaveRobotStateEvaluationThreshold",
                                      "operationCycleTime", "maxMotorIntervalLength", "startPosMinRectSize", "speedPointsList", "intervalLengthsList"])

    # default tracking in testmode time:
    TestModeTrackingTime = 300

    # minimumTracking rate for successful evaluation:
    MinimumTrackingRate = 5  # [Hz]: on average minimum number of tracked positions per second
    
    # timeout of response evaluation result:
    # if PC has no evaluation response ready after this time to answer to the robot which is waiting for it
    # => demand a restart of the cycle
    EvaluationResponseTimeout = 10  # [seconds] < galileo bluetooth protocol timeout

    # setting of optimization loop flow:
    AutoStopNextOperationCycle_onRobotLeavedRect = True # halt robot's next operationCycle to reposition it again