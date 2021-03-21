//
// Tomm-I-sim
//
// This is a simulation of the Tomm-I quadraped robot. It is
// used for training model to make the robot walk.
//
// This used the Open Dynamics Engine (ODE) library for modeling
// the physics.
//
// Full source and instructions for building this (and ODE) can
// be found on github here:
//
//   https://github.com/faustus123/Tomm-I/tree/main/Tomm-I-sim
//

#include <assert.h>
#include <chrono>
#include <thread>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#include "action.hpp"
#include "RobotGeom.h"
RobotGeom *robotgeom = nullptr;

dWorldID world;
dSpaceID space;
dGeomID groundID;
dJointGroupID contactgroup;

bool applyForce = false;

void start()
{
    world = dWorldCreate();
    dWorldSetGravity (world,0,0,-9.81);

    dWorldSetDamping(world, 1e-4, 8e-3);
    dWorldSetERP(world, 0.5);

    contactgroup = dJointGroupCreate (0);

    space = dSimpleSpaceCreate (0);
    groundID = dCreatePlane (space,0,0,1,0);

    robotgeom = new RobotGeom(world, space);
    robotgeom->SetPark(); // This is so the servos have an initial setting

    SetupActions();
    ScaleActions(10.0/17.0*23.0/17.5);

    // initial camera position
    static float xyz[3] = {-0.05, +0.9, 0.10};
    static float hpr[3] = {-86.0, +15.0, 0};
    dsSetViewpoint (xyz,hpr);
}

void stop()
{
    dSpaceDestroy(space);
    dWorldDestroy(world);
}


void drawGeom(dGeomID g)
{
    int gclass = dGeomGetClass(g);
    const dReal *pos = dGeomGetPosition(g);
    const dReal *rot = dGeomGetRotation(g);

    switch (gclass) {
        case dBoxClass:
        {
            dVector3 lengths;
            if (applyForce)
                dsSetColor(1, .5, 0);
            else
                dsSetColor(1, 1, 0);
            dsSetTexture (DS_WOOD);
            dGeomBoxGetLengths(g, lengths);
            dsDrawBox(pos, rot, lengths);
            break;
        }
        case dCapsuleClass:
        {
            dReal radius, length;
            if (applyForce)
                dsSetColor(1, .5, 0);
            else
                dsSetColor(1, 1, 0);
//            dsSetTexture (DS_WOOD);
            dsSetColor(1, 0.65, 0);
            dGeomCapsuleGetParams(g, &radius, &length);
            dsDrawCapsule(pos, rot, length, radius);
            break;
        }
        default:
        {}
    }
}

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
            // make all contact have no slippage (infinite friction)
//            contact[i].surface.mode = 0;
//            contact[i].surface.mu = dInfinity;
//            contact[i].surface.slip1 = 0.7;
//            contact[i].surface.slip2 = 0.7;
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

void simLoop(int pause)
{
    static dReal T = 0.0;
    const dReal step = 0.017; // seconds/step
    const unsigned nsteps = 1; // number of steps to take for each frame drawn

    if (!pause) {

        for (unsigned i=0; i<nsteps; ++i) {
            dSpaceCollide (space,0,&nearCallback);
//            dWorldQuickStep(world, step);
            dWorldStep(world, step);
            dJointGroupEmpty (contactgroup);
        }
    }
    
    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        drawGeom(g);
    }

    // Draw lines at hinge joints (for debugging)
    robotgeom->DrawHingeLines();

    // Set all servo motors to the "standing" position

    // Run all of the actions
    // n.b. this assumes it is called about every 10ms
    RunActions(robotgeom);

//    if(T < 2.0)
//        robotgeom->SetPark();
//    else if( T < 4.0 )
//        robotgeom->SetStand();
//    else if( T < 8.0 )
//        robotgeom->SetSit();
//    else if( T < 15.0 )
//        robotgeom->Shake(step);
//    else if( T < 18.0 )
//        robotgeom->SetLaydown();
//    else
//        robotgeom->Relax();
////    std::cout << "T="<<T << std::endl;

    // Run simulation in real time so it looks realistic
    static auto next = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if( next > now) {
        _DBG_ << "sleeping ..." << std::endl;
        std::this_thread::sleep_until(next);
    }else {
        auto tdiff = now-next;
        _DBG_ << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(tdiff).count() << " ms too long!" << std::endl;
        next = now;
    }
    uint32_t t_ms = step*nsteps*1000.0;
    next += std::chrono::milliseconds(t_ms);

    T += t_ms/1000.0;
}

int main(int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = 0;
    fn.stop = stop;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    // create world
    dInitODE();

    // run demo
    dsSimulationLoop (argc, argv, 800, 600, &fn);

    dJointGroupEmpty (contactgroup);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
    return 0;
}
