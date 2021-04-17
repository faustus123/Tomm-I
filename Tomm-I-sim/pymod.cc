
#include <thread>
#include <pybind11/pybind11.h>

#include "Tomm-I-sim.h"
#include "RobotGeom.h"
extern RobotGeom *robotgeom;

extern bool RUN_REAL_TIME;
extern bool USE_PREPROGRAMED_ACTIONS;
extern bool USE_GRAPHICS;

//-------------------------------------------
// Global flag wrapper functions
void TommI_SimulationSetRunRealTime(pybind11::object &pyobj) { RUN_REAL_TIME = pybind11::cast<bool>(pyobj); }
void TommI_SimulationSetUsePreprogrammedActions(pybind11::object &pyobj) { USE_PREPROGRAMED_ACTIONS = pybind11::cast<bool>(pyobj); }
void TommI_SimulationSetUseGraphics(pybind11::object &pyobj) { USE_GRAPHICS = pybind11::cast<bool>(pyobj); }

//-------------------------------------------
// TommI_SimulationSetupAndRunPYWrapper
//
// Call the TommI_SimulationSetupAndRun() subroutine
// using a separate, detached thread.
void TommI_SimulationSetupAndRunPYWrapper(void){

    std::thread thr( [](){
            std::vector<char *> vargv;
            TommI_SimulationSetupAndRun(vargv.size(), vargv.data());
        }
    );

    thr.detach();
}

//-------------------------------------------
// TommI_SimulationCleanupPYWrapper
//
void TommI_SimulationCleanupPYWrapper(pybind11::object &pyobj){
    TommI_SimulationCleanup();
}

//-------------------------------------------
// TommI_SimulationGetStatus
//
pybind11::object TommI_SimulationGetStatus(void){

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
// TommI_SimulationGetMotorNames
//
pybind11::dict TommI_SimulationGetMotorNames(pybind11::dict &pydict){

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
// TommI_SimulationSetMotors
//
pybind11::object TommI_SimulationSetMotors(pybind11::dict &pydict){

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

//-------------------------------------------
// TommI_SimulationStep
//
// Execute one step of the simulation and return
// the robot status after the step.
pybind11::object TommI_SimulationStep(pybind11::object &pyobj){

    // Take one simulation step (blocks until step is complete)
    simStep();

    return TommI_SimulationGetStatus();
}

//=============================================================================================================

//  Define the TommIsim python module
PYBIND11_MODULE(TommIsim, m) {

	m.doc() = "Tomm-I-sim simulation of quadraped robot";

    m.def("SetRunRealTime", &TommI_SimulationSetRunRealTime, "Set this flag to add delay (if possible) to simulation loop to try and make it run in close to real time.");
    m.def("SetUsePreprogrammedActions", &TommI_SimulationSetUsePreprogrammedActions, "Set this flag to run a set of pre-programmed actions (for debugging).");
    m.def("SetUseGraphics", &TommI_SimulationSetUseGraphics, "Set this flag to draw the graphics during the simulation (this will slow it down significantly).");


    m.def("SetupAndRun", &TommI_SimulationSetupAndRunPYWrapper, "Setup the simulation and start it.");
    m.def("Cleanup", &TommI_SimulationCleanupPYWrapper, "Setup the simulation and start it.");
    m.def("GetStatus", &TommI_SimulationGetStatus, "Get current robot status.");
    m.def("GetMotorNames", &TommI_SimulationGetMotorNames, "Get list of motor names.");
    m.def("SetMotors", &TommI_SimulationSetMotors, "Set motor angles. Pass values as dictionary with keys being motor names and values being target angle in degrees.");
    m.def("Step", &TommI_SimulationStep, "Take one simulation step.");


//	py::class_<JEventProcessorPY>(m, "JEventProcessor")\
//	.def(py::init<py::object&>())\
//	.def("Init",       &JEventProcessorPY::Init)\
//	.def("Process",    &JEventProcessorPY::Process)\
//	.def("Finish",     &JEventProcessorPY::Finish)\
//	.def("Prefetch",   &JEventProcessorPY::Prefetch, py::arg("fac_name"), py::arg("tag")="")\
//	.def("Get",        &JEventProcessorPY::Get, py::arg("fac_name"), py::arg("tag")="");\
//	\
//	/* C-wrapper routines */ \
//	m.def("Start",                       &janapy_Start,                       "Allow JANA system to start processing data. (Not needed for short scripts.)"); \
//	// (see src/python/common/janapy.h)


}


