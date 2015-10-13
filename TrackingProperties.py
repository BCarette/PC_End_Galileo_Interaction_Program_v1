from numpy import *


class TrackingProperties(object):
    """
    Class containing properties used by the Tracking Class and set at runtime by user interaction
    """

    # colour filter 1 settings:
    Hue = array([0, 255], dtype=uint8)
    Saturation = array([0, 255], dtype=uint8)
    Value = array([0, 70], dtype=uint8)

    # image filter settings:
    MorphologicalElementDiam = 7
    ContourRadius = array([100, 150], dtype=uint16)
    DeltaRadius = 10

    # output settings
    FrameToShow = "contourFilters_on_res_MorphOp"  # 
    #frameToShow = "TrackedBall"

    # flag to indicate Tracking class that new settings are given
    NewSettingsPushed = False

    # Tracking results
    ResultTracking = {}
    ResultReferenceTimeStamp = 0
    TrackingFailed = False
        
