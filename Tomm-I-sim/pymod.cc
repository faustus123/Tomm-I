//
// This file provides the python interface to the simulation.
// The routines ending in "PY" here are intended to be called from python.
// For the most part routines here named "TommI_SimulationXXXPY()" will be
// mapped so that they may be called with just XXX() from python.
//
// Note that this is set up using a callback mechanism so that the "main"
// thread can be used to draw all of the graphics. This is a requirement
// on macos. The users Python script needs to provide an
// object of a class that has a "step()" method that can be called at each
// iteration of the simulation loop.
//


#include <thread>
#include <chrono>
#include <pybind11/pybind11.h>

#include "Tomm-I-sim.h"
#include "RobotGeom.h"
#include "Tomm-I-sim.h"
#include "RobotGeom.h"

pybind11::list PYCALLBACKS;

//-------------------------------------------
// Global flag wrapper and trivial wrapper functions
void TommI_SimulationSetRunRealTimePY(pybind11::object &pyobj) { RUN_REAL_TIME = pybind11::cast<bool>(pyobj); }
void TommI_SimulationSetUsePreprogrammedActionsPY(pybind11::object &pyobj) { USE_PREPROGRAMED_ACTIONS = pybind11::cast<bool>(pyobj); }
void TommI_SimulationSetUseGraphicsPY(pybind11::object &pyobj) { USE_GRAPHICS = pybind11::cast<bool>(pyobj); }
void TommI_SimulationSetupPY(void){ TommI_SimulationSetup(); }


//-------------------------------------------
// TommI_SimulationRunPYCallbacks
//
/// Run all python user supplied callbacks. This gets called
/// for every iteration of the simulation loop when it runs
/// the USERCALLBACKS.
///
/// See notes for TommI_SimulationRegisterCallbackPY for more.
void TommI_SimulationRunPYCallbacks(void) {
    for(auto fun : PYCALLBACKS){
        fun();
    }
}

//-------------------------------------------
// TommI_SimulationRegisterCallbackPY
//
/// Register a python routine to be called back for every iteration
/// of the simulation loop. On the first call to this, the
/// TommI_SimulationRunPYCallbacks routine is registered in the
/// user callbacks. Thus, the way this works is that all python
/// callbacks are handled by a single C++ callback in USERCALLBACKS.
void TommI_SimulationRegisterCallbackPY(pybind11::function &pyfun){

    PYCALLBACKS.append(pyfun);
    if( PYCALLBACKS.size() ==1 ){
        USERCALLBACKS.push_back( TommI_SimulationRunPYCallbacks );
    }
}

//-------------------------------------------
// TommI_SimulationRunPY
//
/// Call the TommI_SimulationRun() subroutine
void TommI_SimulationRunPY(void){
    std::vector<char *> vargv;
    TommI_SimulationRun(vargv.size(), vargv.data());
}

//-------------------------------------------
// TommI_SimulationCleanupPY
//
void TommI_SimulationCleanupPY(void){
    TommI_SimulationCleanup();
}

//-------------------------------------------
// TommI_SimulationGetStatusPY
//
pybind11::object TommI_SimulationGetStatusPY(void){

    // Get status of robot as map
    std::map<std::string, dReal> vals;
    robotgeom->GetStatus(vals);

    // Convert map into python dictionary
    auto pydict = pybind11::dict();
    for( auto p : vals ){
        pydict[pybind11::str(p.first)] = p.second;
    }

    return std::move(pydict);
}

//-------------------------------------------
// TommI_SimulationGetMotorNamesPY
//
pybind11::dict TommI_SimulationGetMotorNamesPY(pybind11::dict &pydict){

    auto pymotor_names = pybind11::list();

    // If we are called before the RobotGeom object is created then we can't
    // get the names of the joints. Tell the user.
    if( robotgeom == nullptr ) {
        std::cout << "WARNING: GetMotorNames() called before SetupAndRun(). The motor joints have not yet been defined!" << std::endl;
    }else {
        // Copy names of motors from the joints map in robotgeom
        for (auto p : robotgeom->joints) {
            pymotor_names.append(pybind11::str(p.first));
        }
    }

    return std::move( pymotor_names );
}

//-------------------------------------------
// TommI_SimulationSetMotorsPY
//
pybind11::object TommI_SimulationSetMotorsPY(pybind11::dict &pydict){

    // pydict is a python dictionary with keys being the motor
    // name and value being the angle (in degrees) to set the
    // motor to.

    // Loop over motors
    for( auto p : pydict ){
        auto name = p.first.cast<std::string>();
        auto angle = p.second.cast<dReal>();
        robotgeom->SetServoAngle( name, angle );
    }

    return std::move(pydict);
}

//=============================================================================================================



//  Define the TommIsim python module
PYBIND11_MODULE(TommIsim, m) {

	m.doc() = "Tomm-I-sim simulation of quadraped robot";

    m.def("SetRunRealTime", &TommI_SimulationSetRunRealTimePY, "Set this flag to add delay (if possible) to simulation loop to try and make it run in close to real time.");
    m.def("SetUsePreprogrammedActions", &TommI_SimulationSetUsePreprogrammedActionsPY, "Set this flag to run a set of pre-programmed actions (for debugging).");
    m.def("SetUseGraphics", &TommI_SimulationSetUseGraphicsPY, "Set this flag to draw the graphics during the simulation (this will slow it down significantly).");

    m.def("RegisterCallback", &TommI_SimulationRegisterCallbackPY, "Register a python routine to be called during each iteration of the simulation loop.");
    m.def("Setup", &TommI_SimulationSetupPY, "Setup the simulation and start it.");
    m.def("Run", &TommI_SimulationRunPY, "Setup the simulation and start it.");
    m.def("Cleanup", &TommI_SimulationCleanupPY, "Setup the simulation and start it.");
    m.def("GetStatus", &TommI_SimulationGetStatusPY, "Get current robot status.");
    m.def("GetMotorNames", &TommI_SimulationGetMotorNamesPY, "Get list of motor names.");
    m.def("SetMotors", &TommI_SimulationSetMotorsPY, "Set motor angles. Pass values as dictionary with keys being motor names and values being target angle in degrees.");

}


