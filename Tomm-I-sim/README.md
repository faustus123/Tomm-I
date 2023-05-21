# Tomm-I

Here are instructions for building the Tomm-I simulation. The
instructions below were developed under Mac OS X 13.2 using
XCode 14.2.

This relies on the Open Dynamics Engine (ODE) library. This is
freely available and designed to model physics for some basic
shapes and constraints for the purposes of simulation.  

Note that these instructions violate the ODE author recommendation
to not use the drawStuff library

The drawStuff library comes with ODE but is intended to be
rudimentary for the purposes of their demo programs. I believe
they don't recommend it because it is not fully featured and
they don't want to have to support it.

The way I have this set up on my system is that the ODE library
is built inside of the "extern" subdirectory of the Tomm-I
source. Thus, the first thing to do is clone the Tomm-I source:
```
git clone https://github.com/faustus123/Tomm-I
cd Tomm-I
```

Second, download the ODE source and build it. Note that the 
libdrawstuff library needs to be copied to a more convenient
place at the end.
```
cd extern
wget https://bitbucket.org/odedevs/ode/downloads/ode-0.16.2.tar.gz
tar xzf ode-0.16.3.tar.gz
setenv ODE_HOME "$PWD/ode-0.16.3"
cd "${ODE_HOME}/build"
cmake -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=0 ..
make -j4 install
cp libdrawstuff.* ../lib
cd ../../../  # move back to Tomm-I directory
```

Here are instructions for building the Tomm-I simulation code. 
n.b. make sure your ODE_HOME environment variable is set to point
to your ODE library build. These assume you are currently in the
parent directory of the Tomm-I source code directory.
```
cmake -S Tomm-I/Tomm-I-sim -B build-Tomm-I-sim
cmake --build build-Tomm-I-sim -- -j8
```

Finally, to run the simulation, do the following. 
```
./build-Tomm-I-sim/Tomm-I-sim
(hit ctl-C in terminal window to quit)
```
If you have some issue with it finding the textures, you can add an
extra argument to tell it where they are:
```
./build-Tomm-I-sim/Tomm-I-sim -texturepath $ODE_HOME/drawstuff/textures
```

## Python
To run the simulation from python you need to add the directory where
the python module (TommIsim.so) gets installed to your PYTHONPATH.
e.g. (assuming you are in the Tomm-I/Tomm-I-sim directory)

```
setenv PYTHONPATH ${PWD}/build-Tomm-I-sim/
python3 example.py
```

The example.py file has some detailed comments so check there for how
to use it. The file example2.py expands on this a bit by showing how
to reset the simulation when it starts to go off the rails something
like AI training can keep trying over and over.
