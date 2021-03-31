

#include "Tomm-I-sim.h"

#include <pybind11/pybind11.h>

//-------------------------------------------
// TommI_SimulationSetupAndRunPYWrapper
//
// A wrapper function to convert the given python object into
// an argument list format that can be passed to the
// TommI_SimulationSetupAndRun() subroutine.
void TommI_SimulationSetupAndRunPYWrapper(pybind11::object &pyobj){

    std::vector<char*> vargv;
    TommI_SimulationSetupAndRun(vargv.size(), vargv.data());
}

void TommI_SimulationCleanupPYWrapper(pybind11::object &pyobj){
    TommI_SimulationCleanup();
}

//  Define the TommIsim python module
PYBIND11_MODULE(TommIsim, m) {

	m.doc() = "Tomm-I-sim simulation of quadraped robot";

    m.def("SetupAndRun", &TommI_SimulationSetupAndRunPYWrapper, "Setup the simulation and start it.");
    m.def("Cleanup", &TommI_SimulationCleanupPYWrapper, "Setup the simulation and start it.");


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


