#!/usr/bin/env python3
#
#  Make sure the Tomm-I/Tomm-I-sim/python_modules directory is
# in your PYTHONPATH.
#
#  e.g.
#
#     setenv PYTHONPATH $HOME/builds/Tomm-I/Tomm-I-sim/python_modules
#
#
#  This example shows how to reset the simulation and start it from scratch.
# This is needed when the robot has fallen over or gotten so far out of a
# reasonable orientation that we need to restart the simulation.
#

import TommIsim
from math import *

#------------------------------------------------------------
# Define our callback method
def MyPythonCallback():

    # Get status of robot
    status = TommIsim.GetStatus()

    # Check if robot has left the fiducial operational range.
    # Below we turn on the pre-programmed set of actions that
    # end with the robot rolling over. Here, we check when this
    # happens by the "roll" exceeding 55 degrees in either
    # direction. For an actual AI training script, one should
    # probably reset if any of yaw, pitch, or roll go out of
    # range.
    if fabs(status['roll']) > 55.0:
        TommIsim.Reset()


#======================================================================

# These initializations only need to be done once.
TommIsim.SetRunRealTime( False )
TommIsim.RegisterCallback(MyPythonCallback)
TommIsim.SetUsePreprogrammedActions( True ) # comment this line to turn off the pre-programmed sequence used for the initial video
TommIsim.Setup()

# n.b. this will block forever
TommIsim.Run()


