

#ifndef __TOMM_I_SIM_H__

#include <mutex>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

// Global flags used to control simulation.
// (see full descriptions in Tomm-I-sim.cc)
extern bool RUN_REAL_TIME;
extern bool USE_PREPROGRAMED_ACTIONS;
extern bool USE_GRAPHICS;

//typedef void (*UserCallbackFunc_t)(void);
extern std::vector<  void (*)(void) > USERCALLBACKS;


void TommI_SimulationSetup(void);
void TommI_SimulationRun(int argc, char **argv);
void TommI_SimulationReset(void);
void TommI_SimulationCleanup(void);

void start(void);
void stop(void);
void drawGeom(dGeomID g);
void simStep(void);
void simLoop(int pause);

#endif // __TOMM_I_SIM_H__



