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
import time

TommIsim.SetRunRealTime( False )
TommIsim.SetupAndRun()

time.sleep(4)
print("Done.")


