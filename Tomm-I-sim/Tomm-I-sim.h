
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

void TommI_SimulationSetupAndRun(int argc, char **argv);
void TommI_SimulationCleanup(void);

void start(void);
void stop(void);
void drawGeom(dGeomID g);
static void nearCallback(void *data, dGeomID o1, dGeomID o2);
void simLoop(int pause);





