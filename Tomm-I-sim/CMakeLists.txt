cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_VERBOSE_MAKEFILE ON)

project(Tomm-I-sim)

# OpenGL and GLUT libraries for drawStuff
find_package(OpenGL)
find_package(GLUT)
if(APPLE AND OPENGL_FOUND)
    set(HAVE_APPLE_OPENGL_FRAMEWORK ON)
endif()

# Open Dynamics Engine
# n.b. the value of ODE_HOME will be compiled into the executable as part of
# the path to the texture files.
if(DEFINED ENV{ODE_HOME})
	set(ODE_HOME $ENV{ODE_HOME})
else()
	set(ODE_HOME ${CMAKE_SOURCE_DIR}/extern/ode-0.16.2)
endif()
message(STATUS "ODE_HOME: ${ODE_HOME}")

# Pybind11
#
# Developer's note:
#   To get a new version of pybind11 to bundle do something
#   like this:
#      cd Tomm-I-sim/extern
#      wget https://github.com/pybind/pybind11/archive/refs/tags/v2.10.4.tar.gz
#      tar xzf v2.10.4.tar.gz
#      git rm -r pybind11-2.10.3
#      git add pybind11-2.10.4
find_package(Python 3.6 COMPONENTS Interpreter Development REQUIRED)
set(bundled_pybind11 extern/pybind11-2.10.4)

# If the user has defined USE_BUNDLED_PYBIND11 then
# exclusively try using what they indicated.
#
# If USE_BUNDLED_PYBIND11 is not set, then first
# look in the environment with find_package() and if
# not found there fall back to the bundled version.
if(DEFINED USE_BUNDLED_PYBIND11)
	if( ${USE_BUNDLED_PYBIND11} )
		add_subdirectory(${bundled_pybind11})
		message(STATUS "Found pybind11: ${pybind11_INCLUDE_DIR}")
	else()
		find_package(pybind11 REQUIRED)
	endif()
else()
	find_package(pybind11 QUIET)
	if(NOT ${pybind11_FOUND})
		message(STATUS "External pybind11 not found (set pybind11_ROOT envar. to use external). Using bundled version.")
		add_subdirectory(${bundled_pybind11})
		message(STATUS "Found pybind11: ${pybind11_INCLUDE_DIR}")
	endif()
endif()


# Build source common to executable and python mod into library
# so both can use it.
set(LIBSOURCE
        Tomm-I-sim.cc
		action.hpp
		action.cc
        )
include_directories(.)
include_directories(${ODE_HOME}/include)
include_directories(${ODE_HOME}/drawstuff/src)
include_directories(${OPENGL_INCLUDE_DIR})
add_compile_options(-Wno-deprecated-declarations -Wall -DDRAWSTUFF_TEXTURE_PATH="${ODE_HOME}/drawstuff/textures")

add_library(Tomm-I-simlib ${LIBSOURCE})

# Tomm-I-sim stand-alone executable
add_executable(Tomm-I-sim main.cc)
target_link_directories(Tomm-I-sim BEFORE PUBLIC ${ODE_HOME}/lib)
target_link_libraries (Tomm-I-sim LINK_PUBLIC Tomm-I-simlib ode drawstuff ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

# Tomm-I-sim python module
pybind11_add_module(Tomm-I-sim_pymod pymod.cc)
target_include_directories(Tomm-I-sim_pymod BEFORE PUBLIC ${PYTHON_INCLUDE_DIRS} ${PYBIND11_INCLUDE_DIR})
target_link_directories(Tomm-I-sim_pymod BEFORE PUBLIC ${ODE_HOME}/lib)
target_link_libraries(Tomm-I-sim_pymod PUBLIC Tomm-I-simlib ode drawstuff ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${PYTHON_LIBRARIES} pybind11::embed)
set_target_properties(Tomm-I-sim_pymod PROPERTIES PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_NAME "TommIsim")

# Install
install(TARGETS Tomm-I-sim DESTINATION ${CMAKE_SOURCE_DIR}/bin)
install(TARGETS Tomm-I-sim_pymod DESTINATION ${CMAKE_SOURCE_DIR}/python_modules)

