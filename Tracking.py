import cv2
import time
import threading
from settingsPC import SettingsPC
from TrackingProperties import TrackingProperties
import datetime
from numpy import *
import logging
import sys, linecache
import xml.etree.ElementTree as ET
from PyQt4.QtCore import *


class Tracking(QThread):
    """
    Class to perform tracking of the robotball
    """

    def __init__(self, threadName, stopTrackingEvent, logger, bluetoothLock, bluetoothSocket, referenceTimestamp, motorSpeedPointsList,
                    motorIntervalLengthsList, trackingTime, optimizationIteration, loopIteration, trackingPropertiesLock, startPosMinRectSize, ackAckStartNewEvaluationReceived, testMode, parent = None):
        #threading.Thread.__init__(self)
        super(Tracking, self).__init__(parent)
        # thread inits
        self.name = threadName
        self.stopTrackingEvent = stopTrackingEvent
        # self.trackingFinishedEvent = trackingFinishedEvent
        self.logger = logger
        self.bluetoothLock = bluetoothLock
        self.bluetoothSocket = bluetoothSocket
        self.trackingPropertiesLock = trackingPropertiesLock
        self.ackAckStartNewEvaluationReceived = ackAckStartNewEvaluationReceived

        # robot operation data
        self.referenceTimestamp = referenceTimestamp
        self.motorSpeedPointsList = motorSpeedPointsList
        self.motorIntervalLengthsList = motorIntervalLengthsList
        self.trackingTime = trackingTime
        self.optimizationIteration = optimizationIteration
        self.loopIteration = loopIteration

        # tracking settings
        self.videoOutputFilename = "Output/Robot_pso_3"
        self.screenshotOutputFilename = "Output/Scrnsht-"
        self.evaluationOutputFilename = "Output/EvaluationData-"
        self.recordingFPS = 10 #14
        self.testMode = testMode
        self.ifMultipleObjects_findClosest = True
        self.startPosMinRectSize = startPosMinRectSize  # width and height of the rectangle around the starting position that the robot should leave

        # tracking properties shared with other threads
        #self.trackingProperties = trackingProperties

        # tracking data
        self.trackedPoints = [[0],[0],[0.]]  # trackedPoints[0, :] = x-coordinates, trackedPoints[1, :] = y-coordinates, trackedPoints[2, :] = frameTimeStamp
        self.firstFrame = None
        self.totalDistance = 0
        self.averageSpeed = 0
        self.speedBtwnPoints = []  # speedBtwnPoints[i] = the speed between trackedPoints[i] and [i+1]
        self.boundingRectOfPath = None # the minimum bounding rect around the tracked path
        self.startPosMinRect = None # the rectangle around the starting position that the center of the robot should leave for larger reward
        self.robotLeavesStartRect = False
        self.invTotalDistance_LeavingRect = trackingTime * 1280  # init value
        self.resultTracking = {"operationTimeStamp": referenceTimestamp}

        # custom signals to GUI
        self.testModeTrackingFinished = SIGNAL("testModeTrackingFinished(bool)")  # signals in test mode to GUI that tracking finished
        self.updateEvaluationStatusLabel = SIGNAL("updateEvaluationStatusLabel(QString)")

        #self.vc # videostream

        #DEBUG
        self.DEBUGState = False #True
        #self.debugInputVideoFilename = "Video/SingleTraject1_output9_1280x720.mpg"
        self.debugInputVideoFilename = "Video/output8_1280x720.avi"
        self.debugFrameDelay = 0.06  # 1/14.0

    def run(self):
        " Main function of class"
        # set threadName:
        threading.currentThread().setName(self.name)
        self.logger.debug("Tracking:run: Started running")

        try:
            # init videostream
            if not self.DEBUGState:
                self.vc = cv2.VideoCapture(1)
                self.vc.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            else:
                self.vc = cv2.VideoCapture(self.debugInputVideoFilename)

            if not self.vc.isOpened():
                self.logger.error("CameraStream is not open!")
                self.emit(self.updateEvaluationStatusLabel, "Evaluation crashed on opening camera stream")
                return

            # show window of stream
            cv2.namedWindow("Trackingview", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Trackingview", 960, 540)  # 3/4 of original size (1280, 720)

            dontStartTracking = False
            if not self.testMode:
                # notify robot that tracking started
                self.bluetoothLock.acquire()
                self.bluetoothSocket.SendMessage(SettingsPC.BT_ackStartNewEvaluation, self.referenceTimestamp)
                self.bluetoothLock.release()
                self.logger.info("Tracking: ackStartNewEvaluation is sent for reference {0}".format(datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S")))
            
                # wait on ackAckStartNewEvaluation:
                timeoutWaiting = time.time() + SettingsPC.BT_protocolMessageTimeout
                #self.logger.info("Tracking: timeoutTime = {0}".format(timeoutWaiting))
                while True:
                    if self.ackAckStartNewEvaluationReceived.isSet():
                        self.logger.info("Tracking: ackAckStartNewEvaluation received, tracking started for reference {0}".format(datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S")))
                        self.ackAckStartNewEvaluationReceived.clear() 
                        break

                    if time.time() > timeoutWaiting:
                        self.logger.error("Tracking: TIMEOUT of waiting on ackAckStartNewEvaluation for reference {0}".format(datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S")))
                        dontStartTracking = True
                        break
                    #endIf timeout
                    #self.logger.debug("Tracking: time remaining of waiting = {0}".format(time.time() - timeoutWaiting))
                    time.sleep(0.001)
                #endWhile waiting on ackAck

            else:
                self.logger.info("Tracking: Tracking started in test mode")

            if not dontStartTracking:
                # set statusbar info
                self.emit(self.updateEvaluationStatusLabel, "Tracking: running")

                # start tracking
                trackingSucceeded = self.Track()

                if trackingSucceeded:
                    # set statusbar info
                    self.emit(self.updateEvaluationStatusLabel, "Tracking: evaluating")
                    evalSuccess = self.Evaluate()
                    if evalSuccess:
                        self.SaveTrackingEvaluation()
                else:
                    # let main thread know that tracking failed
                    self.logger.warning("Tracking:Track() failed")
                    self.trackingPropertiesLock.acquire()
                    TrackingProperties.TrackingFailed = True
                    TrackingProperties.ResultTracking = {}
                    TrackingProperties.ResultReferenceTimeStamp = 0
                    self.trackingPropertiesLock.release()
                #endIf trackingSucceeded
            #endIf not dontStartTracking

            #if self.DEBUGState:
            #    # wait in debug state for key press before exit
            #    print "Waiting for keypress to finish"
            #    cv2.waitKey()

            # when closing thread: destroy window
            cv2.destroyWindow("Trackingview")
            self.vc.release()

            # set statusbar info
            self.emit(self.updateEvaluationStatusLabel, "Not tracking")
            if self.testMode:
                self.emit(self.testModeTrackingFinished, True)

            self.logger.debug("Tracking:run: ENDED")
        except:
            exc_type, exc_obj, tb = sys.exc_info()
            frame = tb.tb_frame
            linenr = tb.tb_lineno
            filename = frame.f_code.co_filename
            linecache.checkcache(filename)
            line = linecache.getline(filename, linenr, frame.f_globals)
            self.logger.critical("Tracking: EXCEPTION in {0} on line {1}: {2}".format(filename, linenr, line))
            self.logger.critical("Unhandled Exception occurred in Tracking of type: {0}\n".format(sys.exc_info()[0]))
            self.logger.info("Unhandled Exception value = {0}\n".format(sys.exc_info()[1] if sys.exc_info()[1] is not None else "None"))
    #endFct run

    def Track(self):
        "Function to track the ball and log it"
        hsvThresholds = zeros((2,3), dtype= uint8)
        endTime = time.time() + self.trackingTime

        # read properties for tracking first
        self.trackingPropertiesLock.acquire()
        hsvThresholds[:,0] = TrackingProperties.Hue
        hsvThresholds[:,1] = TrackingProperties.Saturation
        hsvThresholds[:,2] = TrackingProperties.Value
        morphologicalElementDiam = TrackingProperties.MorphologicalElementDiam
        contourRadius = TrackingProperties.ContourRadius
        deltaRadius = TrackingProperties.DeltaRadius
        frameToShow = TrackingProperties.FrameToShow
        TrackingProperties.NewSettingsPushed = False
        self.trackingPropertiesLock.release()

        # perform settings:
        structureElement = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morphologicalElementDiam, morphologicalElementDiam))

        # open recording video buffer
        fourcc_divx = cv2.VideoWriter_fourcc(*'XVID')
        self.videoOutputFilename = "{0}-Ref_{1}--Rec_{2}{3}.avi".format(self.videoOutputFilename, datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S"), 
                                                                        time.strftime("%Y-%m-%d %H-%M-%S"), "-TESTMODE" if self.testMode else "")
        outputVideo = cv2.VideoWriter(self.videoOutputFilename, fourcc_divx, self.recordingFPS, (1280, 720), True)
        # grab the first frame
        rval, frame = self.vc.read()
        frameTimeStamp = time.time()

        if rval:
            self.firstFrame = frame.copy()

        while time.time() < endTime:
            # clear necessary loop variables:
            filtered_res_MorphOp_MaskParams = array([[], [], []])  # filtered_res_MorphOp_MaskParams[:, i] = [xcoordinate_center[i], ycoordinate_center[i], radius[i]]
            found_objects_in_frame = array([[], [], []])  # found_objects_in_frame[:, i] = [xcoordinate_center[i], ycoordinate_center[i], radius[i]]
            closestObjectIndex = 0  # int, if multiple object found, this signifies the closest object found when self.ifMultipleObjects_findClosest == True

            if rval == False:
                self.logger.warning("Tracking:Track: Could not grab a new frame from videostream for operation Reference: {0}".format(datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S")))
                return False
            #endIf no new frame

            # check for if new tracking properties set:
            self.trackingPropertiesLock.acquire()
            # look at flag if new properties:
            if TrackingProperties.NewSettingsPushed:
                # set new trackingProperties
                hsvThresholds[:,0] = TrackingProperties.Hue
                hsvThresholds[:,1] = TrackingProperties.Saturation
                hsvThresholds[:,2] = TrackingProperties.Value
                morphologicalElementDiam = TrackingProperties.MorphologicalElementDiam
                contourRadius = TrackingProperties.ContourRadius
                deltaRadius = TrackingProperties.DeltaRadius
                frameToShow = TrackingProperties.FrameToShow
                TrackingProperties.NewSettingsPushed = False
                # perform settings:
                if structureElement.shape != (morphologicalElementDiam, morphologicalElementDiam):
                    structureElement = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morphologicalElementDiam, morphologicalElementDiam))
            self.trackingPropertiesLock.release()

            # imageOperation 1: convert BGR to HSV
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # imageOperation 2: threshold HSV image:
            maskColourThreshold = cv2.inRange(frame_hsv, hsvThresholds[0,:], hsvThresholds[1, :])
            # imageOperation 3: perform morphological filter
            res_MorphOp = cv2.morphologyEx(maskColourThreshold, cv2.MORPH_OPEN, structureElement)

            #DEBUG
            #cv2.imwrite("Output/DBoutput_res_MorphOp - {0}.png".format(time.strftime("%Y-%m-%d %H-%M-%S")), res_MorphOp)

            # imageOperation 3b: extract all outer contours from res_MorphOp
            imageContours, contours, contourHierarchy = cv2.findContours(res_MorphOp.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # filter contours:
            for i in range(0, len(contours)):
                # fit a circle around a given contour:
                fittedCenter, fittedRadius = cv2.minEnclosingCircle(contours[i])
                # determine if contour is desired based on shape of enclosing circle:
                if(fittedRadius >= contourRadius[0] and fittedRadius <= contourRadius[1]):
                    # use this circle around the accepted contour as a mask on res_MorphOp to determine the best enclosing circle
                    filtered_res_MorphOp = cv2.bitwise_and(res_MorphOp, res_MorphOp, mask= cv2.circle(zeros(res_MorphOp.shape, uint8), (int(round(fittedCenter[0])), int(round(fittedCenter[1]))),
                                                            int(round(fittedRadius)) + deltaRadius, 1, -1))
                    filtered_res_MorphOp_MaskParams = append(filtered_res_MorphOp_MaskParams, array([[int(fittedCenter[0])], [int(fittedCenter[1])], [int(fittedRadius) + deltaRadius]]), axis=1)
                    # fit a new circle around the filtered_res_MorphOp:
                    fittedCenter, fittedRadius = cv2.minEnclosingCircle(transpose(asarray(where(filtered_res_MorphOp > 0))))

                    # save newly found circle in temp array: # fittedCenter[1] = x-coordinate, fittedCenter[0] = Y-coordinate
                    found_objects_in_frame = append(found_objects_in_frame, array([[int(fittedCenter[1])], [int(fittedCenter[0])], [int(fittedRadius)]]), axis=1)
                #endIf valid contour found
            #endFor all found contours

            # perform logic on found objects before adding info to tracking records
            if found_objects_in_frame.shape[1] == 0:
                self.logger.warning("Tracking:Track: Could not find an object in frame")
            elif found_objects_in_frame.shape[1] == 1:
                # successfully found 1 ball:
                self.trackedPoints[0].append(found_objects_in_frame[0, 0])
                self.trackedPoints[1].append(found_objects_in_frame[1, 0])
                self.trackedPoints[2].append(frameTimeStamp)
            else:
                # found multiple objects in frame
                self.logger.warning("Tracking:Track: Multiple objects found in frame")
                if self.ifMultipleObjects_findClosest:
                    # record the object closest to previous location
                    closestOjectDistance = 1500
                    for i in range(0, found_objects_in_frame.shape[1]):
                        distance = linalg.norm(found_objects_in_frame[[0,1], [i, i]] - array(self.trackedPoints)[[0,1], [-1, -1]])
                        if distance < closestOjectDistance:
                            closestOjectDistance = distance
                            closestObjectIndex = i
                    #endFor searching closest object
                    # append closest distance
                    self.trackedPoints[0].append(found_objects_in_frame[0, closestObjectIndex])
                    self.trackedPoints[1].append(found_objects_in_frame[1, closestObjectIndex])
                    self.trackedPoints[2].append(frameTimeStamp)
                #else:
                #    # ignore all objects if multiple
                #    pass
                #endIf Search closest object from multiple found
            #endIf adding new object location to tracking records

            # determine which frame to show
            if frameToShow == "maskColourThreshold":
                cv2.imshow("Trackingview", maskColourThreshold)
            elif frameToShow == "res_MorphOp":
                cv2.imshow("Trackingview", res_MorphOp)
            elif frameToShow == "contourFilters_on_res_MorphOp":
                # plot the contour filter masks as circles on res_MorphOp:
                # convert res_MorphOp to BGR image
                outputIm = zeros((res_MorphOp.shape[0], res_MorphOp.shape[1], 3), dtype = uint8)
                outputIm[res_MorphOp > 0, :] = 255
                # draw the filter areas as circles
                for i in range(0, filtered_res_MorphOp_MaskParams.shape[1]):
                    outputIm = cv2.circle(outputIm, (int(round(filtered_res_MorphOp_MaskParams[0, i])), int(round(filtered_res_MorphOp_MaskParams[1, i]))), int(round(filtered_res_MorphOp_MaskParams[2, i])), (0,255,0) if i == closestObjectIndex else (0,0,255), 3)

                # show the image
                cv2.imshow("Trackingview", outputIm)
                #cv2.imwrite("Output/DBoutput_ContourFilter - {0}.png".format(time.strftime("%Y-%m-%d %H-%M-%S")), outputIm)
            elif frameToShow == "TrackedBall":
                outputIm = frame.copy()

                for i in range(0, found_objects_in_frame.shape[1]):
                    outputIm = cv2.circle(outputIm, (int(round(found_objects_in_frame[0, i])), int(round(found_objects_in_frame[1, i]))), int(round(found_objects_in_frame[2, i])), (0,255,0) if i == closestObjectIndex else (0,0,255), 3)

                # show the image
                cv2.imshow("Trackingview", outputIm)
                #cv2.imwrite("Output/DBoutput_TrackedBall - {0}.png".format(time.strftime("%Y-%m-%d %H-%M-%S")), outputIm)
            #endIf show frame

            # record input frame:
            outputVideo.write(frame)

            # check if tracking should be stopped:
            if self.stopTrackingEvent.isSet():
                self.logger.info("Tracking:Track: Tracking interrupted by stopTrackingEvent")
                outputVideo.release()  # stop recording
                return False

            if self.DEBUGState:
                time.sleep(self.debugFrameDelay)
            cv2.waitKey(1)  # wait 1ms to let opencv window refresh

            # read new frame
            rval, frame = self.vc.read()
            frameTimeStamp = time.time()

        #endWhile capturing input video

        # write recording video
        outputVideo.release()
        return True

    def Evaluate(self):
        "Function to evaluate the tracking log"
        # check if enough points tracked for valid evaluation:
        self.logger.info("Tracking:Evaluate: Evaluation of tracking cycle started")
        validEvaluation = True if len(self.trackedPoints[0]) > self.trackingTime * SettingsPC.MinimumTrackingRate else False

        if not validEvaluation:
            self.logger.warning("Tracking:Evaluate: Not enough tracked points {0} w.r.t. minimum {1}, with trackingTime {2}s and tracking rate {3}".format(
                len(self.trackedPoints[0]), self.trackingTime * SettingsPC.MinimumTrackingRate, self.trackingTime, SettingsPC.MinimumTrackingRate))

        # tracking needs to have captured at least 1 point before being able to calculate something
        if len(self.trackedPoints[0]) < 2 or len(self.trackedPoints[1]) < 2 or len(self.trackedPoints[2]) < 2:
            self.logger.warning("Tracking:Evaluate: Not a single position captured!")
            self.trackingPropertiesLock.acquire()
            TrackingProperties.TrackingFailed = True
            self.trackingPropertiesLock.release()
            return False

        # draw resulting path
        pts = transpose(array(self.trackedPoints, int32)[:, 1:])
        pathIm = cv2.polylines(self.firstFrame, [(pts[:, 0:2]).reshape((-1, 1, 2))], False, (255, 150, 0))
        cv2.imshow("Trackingview", pathIm)
        # save screenshot
        self.screenshotOutputFilename = "{0}-TrackPath-Ref_{1}--Rec_{2}{3}{4}.png".format(self.screenshotOutputFilename, datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S"),
                                                                                    time.strftime("%Y-%m-%d %H-%M-%S"), "-TESTMODE" if self.testMode else "", "-Eval_FAILED" if  not validEvaluation else "")
        cv2.imwrite(self.screenshotOutputFilename, pathIm)

        # determine minimum rectangle around start position
        minRectHalfSize = self.startPosMinRectSize / 2.
        self.startPosMinRect = ((pts[0, 0], pts[0, 1]), (self.startPosMinRectSize, self.startPosMinRectSize), 90)
        #startPosMinRectBox = array([[pts[0, 0] - minRectHalfSize, pts[1, 0] - minRectHalfSize], [pts[0, 0] - minRectHalfSize, pts[1, 0] + minRectHalfSize], [pts[0, 0] + minRectHalfSize, pts[1, 0] + minRectHalfSize],
                                      #[pts[0, 0] + minRectHalfSize, pts[1, 0] - minRectHalfSize]], int32)
        self.boundingRectOfPath = cv2.minAreaRect(pts[:, 0:2])
        #resIntersection, intersectionReg = cv2.rotatedRectangleIntersection(self.startPosMinRect, self.boundingRectOfPath)
        resIntersection, intersectionReg = cv2.rotatedRectangleIntersection(self.startPosMinRect, self.boundingRectOfPath)

        #print "resIntersection = {0}".format(resIntersection)
        self.robotLeavesStartRect = resIntersection == 1  # both rectangles contain the starting position => rectangles can only intersect or are fully contained
        # write rectangles screenshot
        rectsIm = cv2.drawContours(self.firstFrame, [int0(cv2.boxPoints(self.startPosMinRect))], 0, (0,0,255))
        rectsIm = cv2.drawContours(rectsIm, [int0(cv2.boxPoints(self.boundingRectOfPath))], 0, (0,255,0))
        cv2.imwrite(self.screenshotOutputFilename[:-4] + "--Rects.png", rectsIm) 
        
        # work with float arrays:
        ptsFlt = transpose(array(self.trackedPoints)[:, 1:])
        # perform evaluation on recorded path
        for i in range(0, ptsFlt.shape[0] - 1):
            distance = linalg.norm(ptsFlt[i + 1, 0:2] - ptsFlt[i, 0:2])
            deltaTime = ptsFlt[i + 1, 2] - ptsFlt[i, 2]
            if distance == 0. or deltaTime == 0.:
                self.speedBtwnPoints.append(0.)
            else:
                self.speedBtwnPoints.append(distance / deltaTime)
            
            self.totalDistance += distance
        #endFor loop over tracked path
        self.averageSpeed = self.totalDistance / (ptsFlt[-1, 2] - ptsFlt[0,2])

        # calculate inverse reward here
        self.invTotalDistance_LeavingRect -= self.totalDistance
        if not self.robotLeavesStartRect:
            self.invTotalDistance_LeavingRect += 100 * self.trackingTime  # punishment

        # return also evaluation results in trackingProperties
        self.resultTracking["TotalDistance"] = self.totalDistance
        self.resultTracking["AverageSpeed"] = self.averageSpeed
        self.resultTracking["RobotLeavesStartRect"] = self.robotLeavesStartRect
        self.resultTracking["invTotalDistance_LeavingRect"] = self.invTotalDistance_LeavingRect

        self.trackingPropertiesLock.acquire()
        TrackingProperties.ResultTracking = self.resultTracking
        TrackingProperties.ResultReferenceTimeStamp = self.referenceTimestamp
        TrackingProperties.TrackingFailed = not validEvaluation
        self.trackingPropertiesLock.release()
        return validEvaluation
    #endFct Evaluate

    def SaveTrackingEvaluation(self):
        "Function to save the evaluation result of this tracking period"
        timeOfSaving = time.time()
        xmlRoot = ET.Element("Save_Evaluation_Result", {"OperationReferenceTime": datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d_%H:%M:%S"), 
                                                        "TimeOfSaving": datetime.datetime.fromtimestamp(timeOfSaving).strftime("%Y-%m-%d_%H:%M:%S")})

        trackingDataNode = ET.Element("TrackingData")
        robotInfoNode = ET.Element("RobotInfo")
        trackingSettingsNode = ET.Element("TrackingSettings")
        trackingPropertiesNode = ET.Element("TrackingProperties")
        
        # add fields to trackingDataNode
        tempEl = ET.Element("TotalDistance")
        tempEl.text = str(self.totalDistance)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("AverageSpeed")
        tempEl.text = str(self.averageSpeed)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("RobotLeavesStartRect")
        tempEl.text = str(self.robotLeavesStartRect)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("invTotalDistance_LeavingRect")
        tempEl.text = str(self.invTotalDistance_LeavingRect)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("BoundingRectOfPath")
        tempEl.text = str(self.boundingRectOfPath)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("StartPosMinRect")
        tempEl.text = str(self.startPosMinRect)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("TrackedPoints")
        tempEl.text = str(self.trackedPoints)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("speedBtwnPoints")
        tempEl.text = str(self.speedBtwnPoints)
        trackingDataNode.append(tempEl)
        tempEl = ET.Element("ResultTracking")
        tempEl.text = str(self.resultTracking)
        trackingDataNode.append(tempEl)

        # add fields robotInfoNode
        tempEl = ET.Element("ReferenceTimestamp")
        tempEl.text = str(self.referenceTimestamp)
        robotInfoNode.append(tempEl)
        tempEl = ET.Element("trackingTime")
        tempEl.text = str(self.trackingTime)
        robotInfoNode.append(tempEl)
        tempEl = ET.Element("OptimizationIteration")
        tempEl.text = str(self.optimizationIteration)
        robotInfoNode.append(tempEl)
        tempEl = ET.Element("LoopIteration")
        tempEl.text = str(self.loopIteration)
        robotInfoNode.append(tempEl)
        tempEl = ET.Element("motorSpeedPointsList")
        tempEl.text = str(self.motorSpeedPointsList.tolist())
        robotInfoNode.append(tempEl)
        tempEl = ET.Element("motorIntervalLengthsList")
        tempEl.text = str(self.motorIntervalLengthsList.tolist())
        robotInfoNode.append(tempEl)

        # add fields trackingSettingsNode
        tempEl = ET.Element("TestMode")
        tempEl.text = str(self.testMode)
        trackingSettingsNode.append(tempEl)
        tempEl = ET.Element("ifMultipleObjects_findClosest")
        tempEl.text = str(self.ifMultipleObjects_findClosest)
        trackingSettingsNode.append(tempEl)
        tempEl = ET.Element("startPosMinRectSize")
        tempEl.text = str(self.startPosMinRectSize)
        trackingSettingsNode.append(tempEl)
        tempEl = ET.Element("recordingFPS")
        tempEl.text = str(self.recordingFPS)
        trackingSettingsNode.append(tempEl)
        tempEl = ET.Element("VideoOutputFilename")
        tempEl.text = str(self.videoOutputFilename)
        trackingSettingsNode.append(tempEl)
        tempEl = ET.Element("screenshotOutputFilename")
        tempEl.text = str(self.screenshotOutputFilename)
        trackingSettingsNode.append(tempEl)

        # add fields tracking properties
        self.trackingPropertiesLock.acquire()
        tempEl = ET.Element("ColourFilter")
        tempEl2 = ET.Element("Hue")
        tempEl2.text = str(TrackingProperties.Hue)
        tempEl.append(tempEl2)
        tempEl2 = ET.Element("Saturation")
        tempEl2.text = str(TrackingProperties.Saturation)
        tempEl.append(tempEl2)
        tempEl2 = ET.Element("Value")
        tempEl2.text = str(TrackingProperties.Value)
        tempEl.append(tempEl2)
        trackingPropertiesNode.append(tempEl)
        tempEl = ET.Element("MorphologicalElementDiam")
        tempEl.text = str(TrackingProperties.MorphologicalElementDiam)
        trackingPropertiesNode.append(tempEl)
        tempEl = ET.Element("ContourRadius")
        tempEl.text = str(TrackingProperties.ContourRadius)
        trackingPropertiesNode.append(tempEl)
        tempEl = ET.Element("DeltaRadius")
        tempEl.text = str(TrackingProperties.DeltaRadius)
        trackingPropertiesNode.append(tempEl)
        self.trackingPropertiesLock.release()

        # add nodes to root node
        xmlRoot.append(trackingDataNode)
        xmlRoot.append(robotInfoNode)
        xmlRoot.append(trackingSettingsNode)
        xmlRoot.append(trackingPropertiesNode)

        xmlTree = ET.ElementTree(xmlRoot)
        xmlTree.write("{0}Ref_{1}--Rec_{2}{3}.xml".format(self.evaluationOutputFilename, datetime.datetime.fromtimestamp(self.referenceTimestamp).strftime("%Y-%m-%d %H-%M-%S"),
                                                       datetime.datetime.fromtimestamp(timeOfSaving).strftime("%Y-%m-%d %H-%M-%S"), "-TESTMODE" if self.testMode else ""))
    #endFct SaveTrackingEvaluation
