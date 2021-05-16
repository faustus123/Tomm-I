//
// Tomm-I-sim
//
// This is a simulation of the Tomm-I quadraped robot. It is
// used for training model to make the robot walk.
//
// This used the Open Dynamics Engine (ODE) library for modeling
// the physics.
//
// Full source and instructions for building this (and ODE) can
// be found on github here:
//
//   https://github.com/faustus123/Tomm-I/tree/main/Tomm-I-sim
//

#include "Tomm-I-sim.h"

//-----------------------------------------------------
// main
//
int main(int argc, char **argv)
{
    // This is deliberately kept minimal so most of the simulation
    // code can be accessed from both the stand-alone executable
    // and the python module.

    USE_PREPROGRAMED_ACTIONS = true; // comment this out to disable the set of pre-programmed robot actions

    TommI_SimulationSetup();
    TommI_SimulationRun(argc, argv);
    TommI_SimulationCleanup();

    return 0;
}
