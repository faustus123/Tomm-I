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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif


dWorldID world;
dSpaceID space;
dBodyID body1;
dBodyID body2;
dJointID joint1, joint2;
bool applyForce = false;

void start()
{
    world = dWorldCreate();
    dWorldSetGravity (world,0,0,-9.8);

    dWorldSetDamping(world, 1e-4, 1e-5);
//    dWorldSetERP(world, 1);

    space = dSimpleSpaceCreate (0);

    dGeomID g;
    dMass mass;

    // Set angle of body1
    dReal theta1 = 40.0; // angle about y-axis in degrees
    dReal C1 = (dReal)cos(theta1/57.3/2.0);
    dReal S1 = (dReal)sin(theta1/57.3/2.0);
    dQuaternion Q1 = {C1, 1.0f*S1, 0.0f*S1, 0.0f*S1};

    // Create body1
    g = dCreateCapsule(space, 0.1, 0.8);
    body1 = dBodyCreate(world);
    dBodySetPosition(body1, 0, 0, 3);
    dGeomSetQuaternion(g, Q1);
    dMassSetBox(&mass, 1, 0.2, 0.2, 1);
    dBodySetMass(body1, &mass);
    dGeomSetBody(g, body1);

    // Set angle of body2
    dReal theta2 = -30.0; // angle about y-axis in degrees
    dReal C2 = (dReal)cos(theta2/57.3/2.0);
    dReal S2 = (dReal)sin(theta2/57.3/2.0);
    dQuaternion Q2 = {C2, 1.0f*S2, 0.0f*S2, 0.0f*S2};

    dReal Y = -0.5*sin(theta1/57.3) + 0.5*sin(theta2/57.3);
    dReal Z = 3 - 0.5*cos(theta1/57.3) + 0.5*cos(theta2/57.3);

    // Create body2
    g = dCreateCapsule(space, 0.1, 0.8);
    body2 = dBodyCreate(world);
    dBodySetPosition(body2, 0, Y, Z);
    dGeomSetQuaternion(g, Q2);
    dMassSetCapsule(&mass, 1, 3, 0.1, 0.8);
    dBodySetMass(body2, &mass);
    dGeomSetBody(g, body2);

#if 1
    joint1 = dJointCreateDHinge(world, 0);
    dJointAttach(joint1, body1, 0);
    dJointSetDHingeAxis(joint1, 1, 0, 0);
    dJointSetDHingeAnchor1(joint1, 0, 0, 3.5);
    dJointSetDHingeAnchor2(joint1, 0, 0, 4.5);
#endif

#if 1
    joint2 = dJointCreateDHinge(world, 0);
    dJointAttach(joint2, body1, body2);
    dJointSetDHingeAxis(joint2, 1, 0, 0);
    dJointSetDHingeAnchor1(joint2, 0, 0, 2.5);
    dJointSetDHingeAnchor2(joint2, 0, 0, 2.5);
#else
    joint2 = dJointCreateDBall(world, 0);
    dJointAttach(joint2, body1, body2);
    dJointSetDBallAnchor1(joint2, 0, 0, 2.5);
    dJointSetDBallAnchor2(joint2, 0, 0, 1.5);
#endif

    //dBodyAddForce(body1, 20, 0, 0);


    // initial camera position
    static float xyz[3] = {3.8966, -2.0614, 4.0300};
    static float hpr[3] = {153.5, -16.5, 0};
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
//            dGeomCGetLengths(g, lengths);
            dsDrawCapsuleD(pos, rot, length, radius);
//            dsDrawBox(pos, rot, lengths);
            break;
        }
        default:
        {}
    }
}


void simLoop(int pause)
{
    if (!pause) {

        static dReal t = 0;

        const dReal step = 0.005;
        const unsigned nsteps = 2;

        for (unsigned i=0; i<nsteps; ++i) {

            applyForce = fmodf(t, 3.) > 2.;

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
            
            t += step;
            if (t > 20.)
                t = 0.;

            dWorldQuickStep(world, step);
        }
    }
    
    // now we draw everything
    unsigned ngeoms = dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        drawGeom(g);
    }

#if 1
    dVector3 a11, a12;
    dJointGetDHingeAnchor1(joint1, a11);
    dJointGetDHingeAnchor2(joint1, a12);
    dsSetColor(1, 0, 0);
    dsDrawLine(a11, a12);
    //printf("Error 1: %f\n", fabs(dJointGetDHingeDistance(joint1) - dCalcPointsDistance3(a11, a12)));
#endif

#if 1
    dVector3 a21, a22;
    dJointGetDHingeAnchor1(joint2, a21);
    dJointGetDHingeAnchor2(joint2, a22);
    dsSetColor(0, 1, 0);
    dsDrawLine(a21, a22);

    //printf("Error 2: %f\n", fabs(dJointGetDHingeDistance(joint2) - dCalcPointsDistance3(a21, a22)));
#endif
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

    dCloseODE();
    return 0;
}
