cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 14)

project(Tomm-I-sim)

find_package(OpenGL)
find_package(GLUT)
if(APPLE AND OPENGL_FOUND)
    set(HAVE_APPLE_OPENGL_FRAMEWORK ON)
endif()

if(DEFINED ENV{ODE_HOME})
	set(ODE_HOME $ENV{ODE_HOME})
else()
	set(ODE_HOME /Users/davidl/David/Tomm-I/Software/ODE/ode-0.16.2)
endif()
message(STATUS "ODE_HOME: ${ODE_HOME}")


set(SOURCE
        Tomm-I-sim.cc
		action.hpp
		action.cc
        )

add_executable(Tomm-I-sim ${SOURCE})
add_compile_options(-Wno-deprecated-declarations)

include_directories(.)
include_directories(${ODE_HOME}/include)
include_directories(${ODE_HOME}/drawstuff/src)
include_directories(${OPENGL_INCLUDE_DIR})

target_link_directories(Tomm-I-sim BEFORE PUBLIC ${ODE_HOME}/lib)
target_link_libraries (Tomm-I-sim LINK_PUBLIC ode drawstuff ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
