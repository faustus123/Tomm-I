//
//

#include <assert.h>
#include <chrono>
#include <thread>

//#ifdef HAVE_APPLE_OPENGL_FRAMEWORK
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
//#endif

#include "Tomm-I-sim.h"
#include "action.hpp"
#include "RobotGeom.h"
RobotGeom *robotgeom = nullptr;

// This flag is used to tell start() whether it is being called
// from dsSimulationLoop or not.
bool SIMLOOP_CALLED = false;

// These are used to control the sim loop externally (e.g. from python)
//bool SIM_DONE = false;
//bool SIM_GO = true;
//bool SIM_EXTERNAL_CONTROL = false;
//std::mutex SIM_MUTEX;
//std::condition_variable SIM_CV;

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#define dsDrawCapsule dsDrawCapsuleD
#endif

dWorldID world=0;
dSpaceID space;
dGeomID groundID;
dJointGroupID contactgroup;

// The RUN_REAL_TIME flag tells the simulation to add some delay to loop
// (if needed) to try and match the frame rate so that the simulation
// runs more or less in real time. This can make it a little easier to visualize
// if it is doing the right thing.
// Note that this only works if the time spent simulating every step is less
// than the frame rate. (e.g. step variable in simLoop needs to be large enough)
bool RUN_REAL_TIME = true;

// This flag tells the simulation to run the sequence of pre-programmed motions
// defined in action.cc::SetupActions(). Set this to true to check that the
// physics simulation is working by watching that the robot moves in a reasonable
// way.
bool USE_PREPROGRAMED_ACTIONS = false;

// Set this to false to skip drawing frames during the simulation
bool USE_GRAPHICS = true;

// Used to hold functions that are to be called during each iteration of the
// main simulation loop.
std::vector< void (*)(void) > USERCALLBACKS;

//// The USE_SINGLE_STEP and SINGLE_STEP_HOLD flags are used to stall
//// stepping through the simulation loop so it can be controlled
//// externally from python.
//bool USE_SINGLE_STEP = false;
//bool SINGLE_STEP_HOLD = true;

//-----------------------------------------------------
// TommI_SimulationSetup
//
/// This is not strictly required!
/// This will initialize the ODE engine and setup the robot
/// geometry by calling "start()". The way the ODE authors set
/// this up was to have start() called as a callback from
/// dsSimulationLoop(). However, for the python inferface we
/// want to have the option of doing something after the geometry
/// has been set up but before the simulation proper starts().
/// The start() function has been tooled to only setup the
/// geometry on the first call and to do nothing for subsequent
/// calls. Thus, if you do not call this, start() will still get run
/// when dsSimulationLoop is called from TommI_SimulationRun()
/// later.
void TommI_SimulationSetup(void) {

    // Initialize ODE engine
    dInitODE();

    // Call start() to initialize geometry.
    // n.b. this will set the "world" global which is used to
    // check whether start() has already been called.
    start();
}

//-----------------------------------------------------
// TommI_SimulationRun
//
void TommI_SimulationRun(int argc, char **argv) {
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = 0;
    fn.stop = &stop;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    // Initialize ODE engine
    if( world==0 ) dInitODE();

    // run demo
    SIMLOOP_CALLED = true; // let start() know to setup the viewpoint
    dsSimulationLoop(argc, argv, 800, 600, &fn);
}

//-----------------------------------------------------
// TommI_SimulationReset
//
// Called during the simulation to reset the simulation
// to its starting configuration.
void TommI_SimulationReset(void) {

    // The OS X implementation of the drawstuff library from ODE uses
    // GLUT and calls glutMainLoop() at the end of dsSimulationLoop().
    // That routine never returns so resetting is just the simulation
    // part itself.
    std::cout << "Simulation resetting ...." << std::endl;

    // Destroy everything and then recreate it from scratch
    TommI_SimulationCleanup();
    SIMLOOP_CALLED = false;
    TommI_SimulationSetup();
//    start();
}

//-----------------------------------------------------
// TommI_SimulationCleanup
//
void TommI_SimulationCleanup(void)
{
    dJointGroupEmpty (contactgroup);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    world=0;

    delete robotgeom;
    robotgeom = nullptr;
    ClearAllActions();

    dCloseODE();
}

//-----------------------------------------------------
// start
//
void start(void)
{
    // (see note above for TommI_SimulationSetup() )
    if( world == 0 ) {

        world = dWorldCreate();
        dWorldSetGravity(world, 0, 0, -9.81);

        dWorldSetDamping(world, 1e-4, 8e-3);
        dWorldSetERP(world, 0.5);

        contactgroup = dJointGroupCreate(0);

        space = dSimpleSpaceCreate(0);
        groundID = dCreatePlane(space, 0, 0, 1, 0);

        robotgeom = new RobotGeom(world, space);
        robotgeom->SetPark(); // This is so the servos have an initial setting

        SetupActions();
        ScaleActions(10.0 / 17.0 * 23.0 / 17.5);
    }

    // initial camera position
    // n.b. dsSetViewpoint can only be called IF we are being called
    // from dsSimulationLoop().
    if( SIMLOOP_CALLED ) {
        static float xyz[3] = {-0.05, +0.9, 0.10};
        static float hpr[3] = {-86.0, +15.0, 0};
        if (USE_GRAPHICS)dsSetViewpoint(xyz, hpr);
    }
}

//-----------------------------------------------------
// stop
//
// this is a callback invoked by dsSimulationLoop
void stop(void)
{

}

//-----------------------------------------------------
// drawGeom
//
void drawGeom(dGeomID g)
{
    int gclass = dGeomGetClass(g);
    const dReal *pos = dGeomGetPosition(g);
    const dReal *rot = dGeomGetRotation(g);

    switch (gclass) {
        case dBoxClass:
        {
            dVector3 lengths;
            dsSetColor(1, 1, 0);
            dsSetTexture (DS_WOOD);
            dGeomBoxGetLengths(g, lengths);
            dsDrawBox(pos, rot, lengths);
            break;
        }
        case dCapsuleClass:
        {
            dReal radius, length;
            dsSetColor(1, 1, 0);
            dsSetColor(1, 0.65, 0);
            dGeomCapsuleGetParams(g, &radius, &length);
            dsDrawCapsule(pos, rot, length, radius);
            break;
        }
        default:
        {}
    }
}

//-----------------------------------------------------
// nearCallback
//
// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    assert(o1);
    assert(o2);

    // Geometry objects may be grouped together into a "space".
    // In case one or both of these objects are actually spaces,
    // call the special dSpaceCollide2.
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    {
        // colliding a space with something
        dSpaceCollide2(o1,o2,data,&nearCallback);
        // Note we do not want to test intersections within a space,
        // only between spaces.
        return;
    }

    // Check for contact points between the two objects.
    const int max_contacts = 2;  // maximum number of contact points allowed between these two objects
    dContact contact[max_contacts];
    int n = dCollide (o1,o2,max_contacts,&(contact[0].geom),sizeof(dContact));
    if (n > 0)
    {
        // Looks like there was a collision. Loop over contact points found.
        for (int i=0; i<n; i++)
        {
            // Set surface contact parameters
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
            contact[i].surface.mu = 0.07;
            contact[i].surface.soft_erp = 0.50;
            contact[i].surface.soft_cfm = 0.03;
            contact[i].surface.bounce = 0.1;
            contact[i].surface.bounce_vel = 0.001;
            dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
            dJointAttach
                    (
                            c,
                            dGeomGetBody(contact[i].geom.g1),
                            dGeomGetBody(contact[i].geom.g2)
                    );
        }
    }
}

////-----------------------------------------------------
//// simStep
////
//// NOTE: This is only used when the simLoop is being
//// controlled externally (e.g. by python).
////
//// Tell simLoop to perform one step. This will block until
//// the step is complete.
//void simStep(void)
//{
//    // Notify simLoop to take a step
//    {
//        SIM_EXTERNAL_CONTROL = true; // ensure simLoop is allowing us to control the steps
//        std::unique_lock<std::mutex> lk(SIM_MUTEX);
//        SIM_GO = true;
//        SIM_CV.notify_all();
//    }
//
//    // Wait for step to complete
//    while( SIM_GO && ! SIM_DONE ) std::this_thread::sleep_for(std::chrono::milliseconds(200));
//}

//-----------------------------------------------------
// simLoop
//
void simLoop(int pause)
{
    static dReal T = 0.0;
    const dReal step = 0.017; // seconds/step
    const unsigned nsteps = 1; // number of steps to take for each frame drawn

    if (!pause) {

        for (unsigned i=0; i<nsteps; ++i) {
            dSpaceCollide (space,0,&nearCallback);
            dWorldStep(world, step);  // n.b. dWorldQuickStep seems quite unstable so don't use it
            dJointGroupEmpty (contactgroup);
        }
    }

    // Run all of the actions
    if( USE_PREPROGRAMED_ACTIONS ) RunActions(robotgeom);

    //-------------------------------------------------
    // here is where we should place code to interact
    // with the AI model. Inputs on the robot state
    // can be obtained from the robotgeom object. Settings
    // will also be provided to the robot via
    // robotgeom->SetServoAngle(theta)
    // where theta is in degrees.
    //-------------------------------------------------
    for( auto f : USERCALLBACKS ) f();

    // Refresh all servos so ones that have not had their settings
    // explicitly updated this step will still try and honor the
    // last setting.
    robotgeom->RefreshServos();

    // now we draw everything
    if( USE_GRAPHICS ) {
        unsigned ngeoms = dSpaceGetNumGeoms(space);
        for (unsigned i = 0; i < ngeoms; ++i) {
            dGeomID g = dSpaceGetGeom(space, i);

            drawGeom(g);
        }

        // Draw lines at hinge joints (for debugging)
        robotgeom->DrawHingeLines();
    }

    // Optionally add time delays so the simulation runs in real time
    // so it looks realistic by eye.
    if( RUN_REAL_TIME ) {
        static auto next = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (next > now) {
            _DBG_ << "sleeping ..." << std::endl;
            std::this_thread::sleep_until(next);
        } else {
            auto tdiff = now - next;
            _DBG_ << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(tdiff).count() << " ms too long!"
                  << std::endl;
            next = now;
        }
        uint32_t t_ms = step * nsteps * 1000.0;
        next += std::chrono::milliseconds(t_ms);

        T += t_ms / 1000.0;
    }
}


