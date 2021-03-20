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
git clone faustus123/Tomm-I
cd Tomm-I/Tomm-I-sim
mkdir build
cd build
cmake ..
make -j4
```

Finally, to run the simulation, do the following. Note this assumes
you are still in the build directory.
```
./Tomm-I-sim -texturepath $ODE_HOME/drawstuff/textures
(hit command-Q to quit)
```
If you don't want to have to type in the "-texturepath ..." arguments
every time then edit the texturepath.h file and recompile with the
full path to the directory in your ODE build.
