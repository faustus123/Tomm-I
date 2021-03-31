

#include <pybind11/pybind11.h>

PYBIND11_MODULE(TommIsim, m) {

	m.doc() = "Tomm-I-sim simulation of quadraped robot";

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


