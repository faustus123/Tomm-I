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

import TommIsim

# This procedure gets registered below as a callback so that it
# is called at every iteration of the simulation loop. The time
# steps may be smaller than would be realistic for a physical
# robot so one may wish to only pay attention to every Nth call
# to this.
def MyPythonCallback():

    # Get the current status of the robot in the form of a dictionary.
    # Entries like "BL_hip" indicate the angle of the Back Left leg
    # hip joint.
    status = TommIsim.GetStatus()
    # print('yaw: %f  pitch: %f  roll: %f' % (status['yaw'], status['pitch'], status['roll']))
    # print('x: %f  y: %f  z: %f' % (status['x'], status['y'], status['z']))

    # One can set the desired angles of each of the leg joints using
    # a dictionary with the same names as are in the status (e.g. "FR_foot").
    # Note that the dictionary should only include the joint settings.
    # Do not pass in other values you may find in the status dictionary.
    # Note also that the simulation will set a torque on the motor based
    # on the desired setting so it may take several steps for the joint to
    # actually get there.
    #
    # The joints themselves have hard limits on where they can go which
    # are based on the physical limitations of the robot. Setting to values
    # outside of these ranges will cause the joint to go only to the
    # appropriate limit. These are:
    #
    #  hip:  45 - 140 degrees
    # foot:  50 - 180 degrees
    #
    # Finally, the simulation will remember the last setting for each joint
    # and will keep trying to set it there if a new value is not entered
    # in this dictionary. (i.e. you don't have to set every motor with
    # every call to this).
    #
    # motors = {}
    # motors['BL_foot'] = 47.2;
    # motors['BL_hip' ] = 55.7
    # TommIsim.SetMotors(motors)


#======================================================================
TommIsim.SetRunRealTime( False )
TommIsim.SetUsePreprogrammedActions( True ) # comment this line to turn off the pre-programmed sequence used for the initial video
TommIsim.Setup()
TommIsim.RegisterCallback(MyPythonCallback)

# n.b. this will block until the simulation is complete
TommIsim.Run()

print("Done.")


