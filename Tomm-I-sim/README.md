# Tomm-I

Here are instructions for building the Tomm-I simulation. The
instructions below were developed under Mac OS X 10.15.7 using
XCode 12.3.

This relies on the Open Dynamics Engine (ODE) library. This is
freely available and designed to model physics for some basic
shapes and constraints for the purposes of simulation.  

Note that these instructions immediately violate two of the ODE
author reccommendations:

1. Do out-of-source builds
2. Do not use the drawStuff library

The drawStuff library comes with ODE but is intended to be
rudimentary for the purposes of their demo programs. I believe
they don't recommend it because it is not fully featured and
they don't want to have to support it.

Having said that, here are step-by-step instructions for building
the ODE library.
```
mkdir ODE
cd ODE
wget https://bitbucket.org/odedevs/ode/downloads/ode-0.16.2.tar.gz
tar xzf ode-0.16.2.tar.gz
setenv ODE_HOME $PWD/ode-0.16.2
cd ${ODE_HOME}/build
cmake -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=0 ..
make -j4 install
cp libdrawstuff.* ../lib
```

Here are instructions for building the Tomm-I simulation code. 
n.b. make sure your ODE_HOME environment variable is set to point
to your ODE library build. 
```
git clone --recurse-submodules https://github.com/faustus123/Tomm-I
cd Tomm-I/Tomm-I-sim
mkdir build
cd build
cmake ..
make -j4
```

Finally, to run the simulation, do the following. Note this assumes
you are still in the build directory.
```
./Tomm-I-sim
(hit ctl-C in terminal window to quit)
```
If you have some issue with it finding the textures, you can add an
extra argument to tell it where they are:
```
./Tomm-I-sim -texturepath $ODE_HOME/drawstuff/textures
```

## Python
To run the simulation from python you need to add the directory where
the python module (TommIsim.so) gets installed to your PYTHONPATH.
e.g. (assuming you are in the Tomm-I/Tomm-I-sim directory)

```
setenv PYTHONPATH ${PWD}/python_modules
python3 example.py
```

The example.py file has some detailed comments so check there for how
to use it.