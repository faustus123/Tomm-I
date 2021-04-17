

#ifndef __TOMM_I_SIM_H__

#include <mutex>
#include <condition_variable>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

extern bool SIM_EXTERNAL_CONTROL;
extern std::mutex SIM_MUTEX;
extern std::condition_variable SIM_CV;

void TommI_SimulationSetupAndRun(int argc, char **argv);
void TommI_SimulationCleanup(void);

void start(void);
void stop(void);
void drawGeom(dGeomID g);
void simStep(void);
void simLoop(int pause);

#endif // __TOMM_I_SIM_H__



