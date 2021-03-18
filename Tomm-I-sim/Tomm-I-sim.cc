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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#define dsDrawCapsule dsDrawCapsuleD
#endif

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
    dWorldSetGravity (world,0,0,-9.8);

    dWorldSetDamping(world, 1e-4, 1e-5);
    dWorldSetERP(world, 0.2);

    contactgroup = dJointGroupCreate (0);

    space = dSimpleSpaceCreate (0);
    groundID = dCreatePlane (space,0,0,1,0);

    robotgeom = new RobotGeom(world, space);

    // initial camera position
    static float xyz[3] = {0.0, -0.7, 0.5};
    static float hpr[3] = {90.0, -15.0, 0};
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
            contact[i].surface.slip1 = 0.7;
            contact[i].surface.slip2 = 0.7;
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
            contact[i].surface.mu = 500.0;
            contact[i].surface.soft_erp = 0.50;
            contact[i].surface.soft_cfm = 0.03;
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
    if (!pause) {

        const dReal step = 0.005; // seconds/step
        const unsigned nsteps = 2; // number of steps to take for each frame drawn

        for (unsigned i=0; i<nsteps; ++i) {

//            applyForce = fmodf(t, 3.) > 2.;

//            if (applyForce) {
//                dReal f = 0.3 * sin(t*1.2);
//                dBodyAddForceAtRelPos(body1,
//                                      f, 0, 0,
//                                      0, 0, -0.5); // at the lower end
//
//                dReal g = 0.3 * sin(t*0.7);
//                dBodyAddForceAtRelPos(body2,
//                                      0, g, 0,
//                                      0, 0, -0.5); // at the lower end
//            }


//            t += step;
//            if (t > 20.) t = 0.;

            dSpaceCollide (space,0,&nearCallback);
            dWorldQuickStep(world, step);
//            dWorldStep(world, step);
            dJointGroupEmpty (contactgroup);
        }
    }
    
    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        drawGeom(g);
    }

    robotgeom->DrawHingeLines();
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
