
#include <ode/ode.h>
#include <vector>
#include <iostream>

class Leg {
public:
    Leg(dWorldID worldID, dSpaceID spaceID):worldID(worldID),spaceID(spaceID){

        // Create body objects for 3 pieces of leg
        hipID   = dBodyCreate(worldID);
        thighID = dBodyCreate(worldID);
        footID  = dBodyCreate(worldID);
        all_bodies.push_back( hipID );
        all_bodies.push_back( thighID );
        all_bodies.push_back( footID );

        // Create shape objects for each of the bodies
        hipGeomID   = dCreateCapsule( spaceID, radius, len_hip   );
        thighGeomID = dCreateCapsule( spaceID, radius, len_thigh );
        footGeomID  = dCreateCapsule( spaceID, radius, len_foot  );

        // Assign shapes to bodies
        dGeomSetBody( hipGeomID,   hipID   );
        dGeomSetBody( thighGeomID, thighID );
        dGeomSetBody( footGeomID,  footID  );

        // Set the masses of each of the bodies
        dReal density_PLA = 1.25 * 1E-3 * 1E6; // 1.25 g/cm^3 converted to kg/m^3
        dMassSetCapsule( &massHip,   density_PLA, 3, radius, len_hip   );
        dMassSetCapsule( &massThigh, density_PLA, 3, radius, len_thigh );
        dMassSetCapsule( &massFoot,  density_PLA, 3, radius, len_foot  );
        dBodySetMass( hipID,   &massHip   );
        dBodySetMass( thighID, &massThigh );
        dBodySetMass( footID,  &massFoot );

        // Set positions of each bone
        // Calculate position and angle based on theta_hip and theta_angle.
        // Make it such that hip is perfectly horizontal and at a height
        // that makes the foot justr touch the ground.
        dReal len_tot_hip   = len_hip   + 2.0*radius;
        dReal len_tot_thigh = len_thigh + 2.0*radius;
        dReal len_tot_foot  = len_foot  + 2.0*radius;

        dReal deg2rad = M_PI/180.0;
        dReal angleZ_thigh = 270 - theta_ankle ;                // angle wrt +z axis
        dReal angleZ_foot  = 270.0 - (theta_ankle + theta_hip); // angle wrt +z axis

        dReal height_thigh = fabs(len_tot_thigh*cos(angleZ_thigh*deg2rad));  // Total extent in Z
        dReal height_foot  = fabs(len_tot_foot*cos(angleZ_foot*deg2rad));    // Total extent in Z
        dReal height_hip = height_foot + height_thigh;

        // Set hip to be horizontal with one end at 0,0,height_hip
        // Calculate x-positions of center of each bone
        dReal Xhip   = len_tot_hip/2.0;
        dReal Xthigh = Xhip - (len_hip-dist_hip_thigh_axis) + len_tot_hip/2.0 + len_tot_thigh*sin(angleZ_thigh*deg2rad)/2.0;
        dReal Xfoot  = Xthigh + len_tot_thigh*sin(angleZ_thigh*deg2rad)/2.0 - len_tot_foot*sin(angleZ_foot*deg2rad)/2.0;
        dBodySetPosition(hipID,   Xhip,   0, height_hip);
        dBodySetPosition(thighID, Xthigh, 0, height_hip - height_thigh/2.0);
        dBodySetPosition(footID,  Xfoot,  0, height_foot/2.0);

        // Set angles of each bone (these are all rotations about y-axis)
        dReal C = (dReal)cos(90.0*deg2rad/2.0);
        dReal S = (dReal)sin(90.0*deg2rad/2.0);
        dQuaternion Qhip = {C, 0.0f*S, 1.0f*S, 0.0f*S};
        C = (dReal)cos(angleZ_thigh*deg2rad/2.0);
        S = (dReal)sin(angleZ_thigh*deg2rad/2.0);
        dQuaternion Qthigh = {C, 0.0f*S, 1.0f*S, 0.0f*S};
        C = (dReal)cos(angleZ_foot*deg2rad/2.0);
        S = (dReal)sin(angleZ_foot*deg2rad/2.0);
        dQuaternion Qfoot = {C, 0.0f*S, 1.0f*S, 0.0f*S};
        dGeomSetQuaternion(hipGeomID,   Qhip);
        dGeomSetQuaternion(thighGeomID, Qthigh);
        dGeomSetQuaternion(footGeomID,  Qfoot);

        // Attach bones
        dReal Xjoint = Xhip + len_tot_hip/2.0 - (len_hip-dist_hip_thigh_axis);
        joint_hip_thigh = dJointCreateDHinge(worldID, 0);
        dJointAttach(joint_hip_thigh, hipID, thighID);
        dJointSetDHingeAxis(joint_hip_thigh, 0, 1, 0);
        dJointSetDHingeAnchor1(joint_hip_thigh, Xjoint, 0, height_hip);
        dJointSetDHingeAnchor2(joint_hip_thigh, Xjoint, 0, height_hip);

        Xjoint = Xthigh + len_tot_hip/2.0 + len_tot_thigh*sin(angleZ_thigh*deg2rad)/2.0;;
        joint_thigh_foot = dJointCreateDHinge(worldID, 0);
        dJointAttach(joint_thigh_foot, thighID, footID);
        dJointSetDHingeAxis(joint_thigh_foot, 0, 1, 0);
        dJointSetDHingeAnchor1(joint_thigh_foot, Xjoint, 0, height_foot);
        dJointSetDHingeAnchor2(joint_thigh_foot, Xjoint, 0, height_foot);
    }

    //-------------------------------------------------
    // MoveRelative
    //
    // Move the entire leg relative to its current position
    void MoveRelative(dReal dX, dReal dY, dReal dZ){
        for( auto bID : all_bodies ){
            auto *pos = dBodyGetPosition( bID );
            dBodySetPosition( bID, pos[0]+dX, pos[1]+dY, pos[2]+dZ);
        }
    }

    //-------------------------------------------------
    // Mirror
    //
    // Mirror all bones about the specified axis.
    // 0=x-axis, 1=y-axis, 2=z-axis
    void Mirror(int axis){

        for( auto bID : all_bodies ) {
            auto pos = dBodyGetPosition(bID);
            dReal new_pos[3];
            for (int i = 0; i < 3; i++) new_pos[i] = pos[i];
            new_pos[axis] = -new_pos[axis];
            dBodySetPosition(bID, new_pos[0], new_pos[1], new_pos[2]);

            // Rotate body 180 degrees about z-axis.
            // This turns out to be equivalent to reversing the sign
            // of the top two rows of the rotation matrix
            auto q = dBodyGetQuaternion(bID);
            dMatrix3 rot;
            dRfromQ( rot, q );
            for(int i=0; i<3; i++){
                for(int j=0; j<2; j++){
                    rot[j*4+i] = - rot[j*4+i];
                }
            }
            dBodySetRotation(bID, rot);
        }
    }

    //-------------------------------------------------
    // RotateRelative
    //
    // Rotate the entire leg relative to its current position
    // about the far end of the hip bone.
    void RotateRelative(dReal theta_deg, dReal X, dReal Y, dReal Z){

        // TODO:  Write this
    }


    virtual ~Leg(){}

    dWorldID worldID;
    dSpaceID spaceID;
    dGeomID hipGeomID;
    dGeomID thighGeomID;
    dGeomID footGeomID;

    dBodyID hipID;
    dBodyID thighID;
    dBodyID footID;

    dMass massHip;
    dMass massThigh;
    dMass massFoot;

    dJointID joint_hip_thigh;
    dJointID joint_thigh_foot;

    // All values in meters (default for ODE)
    dReal radius = 0.01; // radius of bones
    dReal len_hip = 0.22;
    dReal len_thigh = 0.28;
    dReal len_foot = 0.17;
    dReal dist_hip_thigh_axis = 0.17; // distance from far end of hip to axis point where thigh is attached

    // Angles of joints in degrees
    dReal theta_hip   = 120.0;
    dReal theta_ankle = 130.0;

    std::vector<dBodyID> all_bodies;
};

