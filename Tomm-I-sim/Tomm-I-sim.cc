/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

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

#include "Leg.h"

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
    dWorldSetERP(world, 0.001);

    contactgroup = dJointGroupCreate (0);

    space = dSimpleSpaceCreate (0);
    groundID = dCreatePlane (space,0,0,1,0);

    Leg FL(world, space);
    Leg BL(world, space);
    Leg FR(world, space);
    Leg BR(world, space);

    FL.MoveRelative(0.02,0.13, 0.0);
    BL.Mirror(0);
    BL.MoveRelative(-0.02,0.13, 0.0);
    FR.MoveRelative(0.02,-0.13, 0.0);
    BR.Mirror(0);
    BR.MoveRelative(-0.02,-0.13, 0.0);


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
            dsSetTexture (DS_WOOD);
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
            contact[i].surface.mode = 0;
            contact[i].surface.mu = dInfinity;
//            contact[i].surface.slip1 = 0.7;
//            contact[i].surface.slip2 = 0.7;
//            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
//            contact[i].surface.mu = dInfinity; // was: 500.0
//            contact[i].surface.soft_erp = 0.50;
//            contact[i].surface.soft_cfm = 0.03;
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

//        static dReal t = 0;

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
            //dWorldStep(world, step);
            dJointGroupEmpty (contactgroup);
        }
    }
    
    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        drawGeom(g);
    }
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
