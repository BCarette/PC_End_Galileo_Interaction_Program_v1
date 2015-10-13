import sys, linecache, traceback
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from GUI.robotControlMainWindow import RobotControlMainWindow
from mainPC import MainPC

def main():
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


if __name__ == "__main__":
    # start the application
    main()