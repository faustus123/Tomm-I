//
// The RobotGeom class is used to define the robot geometry including joints.
//
// The constructor takes the dWorldID and dSpaceID and then defines all of the
// geometry, storing many of the elements in the class for later access.
//
// The geometry is made up of a frame and 4 legs. The frame is a single
// rigid body (dBody) with multiple parts (dGeom's). The legs are each made of
// two bodies ("thigh" and "foot"). The thigh is connected to both the frame
// and foot using hinge joints on either end.
//
//
//

#ifndef _ROBOTGEOM_H_
#define _ROBOTGEOM_H_


#include <ode/ode.h>
#include <vector>
#include <map>
#include <iostream>

void CopydVector3(dVector3 &dest, const dVector3 &src){for(int i=0;i<4;i++) dest[i] = src[i];}

class RobotGeom{
public:

    dWorldID worldID;
    dSpaceID spaceID;
    std::map<std::string, dBodyID > bodies;
    std::map<std::string, dGeomID > geoms;
    std::map<std::string, dMass>    masses;
    std::map<std::string, dVector3> jointpos;
    std::map<std::string, dJointID> joints;

    dReal deg2rad                = M_PI/180.0;
    dReal initial_height         = 0.40;
    dReal backbone_len           = 0.16;
    dReal backbone_radius        = 0.02; // radius of backbone
    dReal leg_radius             = 0.01; // radius of bones
    dReal hip_len                = 0.22;
    dReal thigh_len              = 0.28;
    dReal foot_len               = 0.17;
    dReal dist_hip_thigh_axis    = 0.17; // distance from far end of hip to axis point where thigh is attached
    dReal hip_theta              = 120.0; // degrees
    dReal ankle_theta            = 130.0; // degrees

    dReal density_PLA = 1.25 * 1E-3 * 1E6; // 1.25 g/cm^3 converted to kg/m^3

    RobotGeom(dWorldID worldID, dSpaceID spaceID):worldID(worldID),spaceID(spaceID){

        // The frame is the main rigid body of the robot.
        // It will have several geoms attached to it.
        bodies["frame"] = dBodyCreate(worldID);
        masses["frame"] = dMass();
        dBodySetPosition(bodies["frame"], 0, 0, initial_height);

        // Create main backbone
        // n.b. this runs horizontal across the body of the robot
        // not actually along the body like a real backbone
        geoms["backbone"] = dCreateCapsule( spaceID, backbone_radius, backbone_len );
        dGeomSetBody( geoms["backbone"],   bodies["frame"]   );
        dReal C = (dReal)cos(90.0*deg2rad/2.0);
        dReal S = (dReal)sin(90.0*deg2rad/2.0);
        dQuaternion Q = {C, 1.0f*S, 0.0f*S, 0.0f*S};
        dGeomSetOffsetQuaternion(geoms["backbone"], Q);
        dMass M;
        dMassSetCapsule( &M,   density_PLA, 2, backbone_radius, backbone_len );
        dMassAdd( &masses["frame"], &M );

        // Create legs
        // hips will be rigidly attached to frame while other parts will be
        // independent bodies connected by hinges
        dReal leg_shift_parallel_to_body = 0.02;
        dReal leg_shift_perpendicular_to_body = 0.13;
        CreateLeg("FL", leg_shift_parallel_to_body,  leg_shift_perpendicular_to_body, 0.0, false);
        CreateLeg("FR", leg_shift_parallel_to_body, -leg_shift_perpendicular_to_body, 0.0, false);
        CreateLeg("BL", leg_shift_parallel_to_body,  leg_shift_perpendicular_to_body, 0.0, true);
        CreateLeg("BR", leg_shift_parallel_to_body, -leg_shift_perpendicular_to_body, 0.0, true);
    }

    //-----------------------------------------
    // CreateLeg
    void CreateLeg(std::string name, dReal shiftX, dReal shiftY, dReal shiftZ, bool rotateZ){

        auto hip_name = name + "_hip";
        auto thigh_name = name + "_thigh";
        auto foot_name = name + "_foot";

        // Total length of capsules
        dReal tot_hip_len   = hip_len   + 2.0*leg_radius;
        dReal tot_thigh_len = thigh_len + 2.0*leg_radius;
        dReal tot_foot_len  = foot_len  + 2.0*leg_radius;

        //---------- Create hip bone
        geoms[hip_name] = dCreateCapsule( spaceID, leg_radius, hip_len );
        dGeomSetBody( geoms[hip_name],   bodies["frame"]   );
        dReal C = (dReal)cos(90.0*deg2rad/2.0);
        dReal S = (dReal)sin(90.0*deg2rad/2.0);
        dQuaternion Q = {C, 0.0f*S, 1.0f*S, 0.0f*S};
        dVector3 offset = {shiftX + 0.5*tot_hip_len, shiftY, shiftZ};
        dVector3 pos_hip_joint = {offset[0]+0.5*tot_hip_len, offset[1], offset[2]+initial_height};
        if(rotateZ){
            RotateZ180(Q);
            offset[0] = -offset[0];
            offset[1] = -offset[1];
        }
        dGeomSetOffsetQuaternion(geoms[hip_name], Q);
        dGeomSetOffsetPosition(geoms[hip_name], offset[0], offset[1], offset[2]);
        dMass M;
        dMassSetCapsule( &M,   density_PLA, 1, backbone_radius, backbone_len );
        dMassTranslate (&M, offset[0], offset[1], offset[2]);
        dMassAdd( &masses["frame"], &M );

        // Initial angle of joints given by height of frame and length of bones.
        // Use same angle for both foot and thigh.
        C = initial_height/(tot_thigh_len + tot_foot_len);  // cos(thetaZ)
        if(C>1.0) C=1.0;
        S = sqrt(1-C*C);                                    // sin(thetaZ)
        dReal C_2 = cos(0.5*acos(C));
        dReal S_2 = -sin(0.5*acos(C));

        //---------- Create thigh bone
        bodies[thigh_name] = dBodyCreate(worldID);
        geoms[thigh_name] = dCreateCapsule( spaceID, leg_radius, thigh_len );
        dGeomSetBody( geoms[thigh_name],   bodies[thigh_name]   );
        Q[0]=C_2; Q[1]=0.0f*S_2; Q[2]=1.0f*S_2; Q[3]=0.0f*S_2;
        offset[0] = pos_hip_joint[0] + 0.5*S*tot_thigh_len;
        offset[1] = pos_hip_joint[1];
        offset[2] = pos_hip_joint[2] - 0.5*C*tot_thigh_len;
        dVector3 pos_foot_joint = {offset[0]+0.5*S*tot_thigh_len, offset[1], offset[2]- 0.5*C*tot_thigh_len};
        if(rotateZ){
            RotateZ180(Q);
            offset[0] = -offset[0];
            offset[1] = -offset[1];
            pos_hip_joint[0] = -pos_hip_joint[0];
            pos_hip_joint[1] = -pos_hip_joint[1];
        }
        dBodySetPosition(bodies[thigh_name], offset[0], offset[1], offset[2]);
        dBodySetQuaternion(bodies[thigh_name], Q);
        masses[thigh_name] = dMass();
        dMassSetCapsule( &masses[thigh_name],   density_PLA, 2, leg_radius, thigh_len );

        //---------- Create foot bone
        bodies[foot_name] = dBodyCreate(worldID);
        geoms[foot_name] = dCreateCapsule( spaceID, leg_radius, foot_len );
        dGeomSetBody( geoms[foot_name],   bodies[foot_name]   );
        S_2 = -S_2; // foot is rotated in opposite direction
        Q[0]=C_2; Q[1]=0.0f*S_2; Q[2]=1.0f*S_2; Q[3]=0.0f*S_2;
        offset[0] = pos_foot_joint[0] - 0.5*S*tot_foot_len;
        offset[1] = pos_foot_joint[1];
        offset[2] = pos_foot_joint[2] - 0.5*C*tot_foot_len;
        if(rotateZ){
            RotateZ180(Q);
            offset[0] = -offset[0];
            offset[1] = -offset[1];
            pos_foot_joint[0] = -pos_foot_joint[0];
            pos_foot_joint[1] = -pos_foot_joint[1];
        }
        dBodySetPosition(bodies[foot_name], offset[0], offset[1], offset[2]);
        dBodySetQuaternion(bodies[foot_name], Q);
        masses[foot_name] = dMass();
        dMassSetCapsule( &masses[foot_name],   density_PLA, 2, leg_radius, foot_len );

        // Remember positions of joints in world coordinates
        CopydVector3(jointpos[name+"_hip"], pos_hip_joint);
        CopydVector3(jointpos[name+"_foot"], pos_foot_joint);

        // Attach bones with hinge joints
        dJointGroupID jointgroupID = dJointGroupCreate (0);
        auto joint_hip = dJointCreateHinge(worldID, jointgroupID);
        dJointAttach(joint_hip, bodies["frame"], bodies[thigh_name]);
        dJointSetHingeAxis(joint_hip, 0, 1, 0);
        dJointSetHingeAnchor(joint_hip, pos_hip_joint[0], pos_hip_joint[1], pos_hip_joint[2]);
        joints[hip_name] = joint_hip;

        auto joint_foot = dJointCreateHinge(worldID, jointgroupID);
        dJointAttach(joint_foot, bodies[thigh_name], bodies[foot_name]);
        dJointSetHingeAxis(joint_foot, 0, 1, 0);
        dJointSetHingeAnchor(joint_foot, pos_foot_joint[0], pos_foot_joint[1], pos_foot_joint[2]);
        joints[foot_name] = joint_foot;
    }

    //-----------------------------------------
    // RotateZ
    //
    /// Rotate given quaternion 180 degrees about z-axis
    void RotateZ180(dQuaternion &q){
        // This turns out to be equivalent to reversing the sign
        // of the top two rows of the rotation matrix
        dMatrix3 rot;
        dRfromQ(rot, q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                rot[j * 4 + i] = -rot[j * 4 + i];
            }
        }
        dQfromR(q, rot);
    }

    //-------------------------------------------------
    // DrawHingeLines
    //
    // Draw lines at the to hinge positions
    void DrawHingeLines(bool drawOriginalPosition=false, bool drawCurrentPosition=true){

        // Draw based on stored position
        if(drawOriginalPosition) {
            for (auto p: jointpos) {
                dsSetColor(0, 0, 0);
                if (p.first.find("_hip") != std::string::npos) dsSetColor(1, 0, 0);
                if (p.first.find("_foot") != std::string::npos) dsSetColor(0, 0, 1);
                dVector3 pos1, pos2;
                CopydVector3(pos1, p.second);
                CopydVector3(pos2, p.second);
                pos1[1] -= 0.05;
                pos2[1] += 0.05;
                dsDrawLine(pos1, pos2);
            }
        }

        // Draw based on current hinge location
        if(drawCurrentPosition) {
            for (auto p: joints) {
                dsSetColor(0, 0, 0);
                if (p.first.find("_hip") != std::string::npos) dsSetColor(0, 1, 0);
                if (p.first.find("_foot") != std::string::npos) dsSetColor(1, 0, 1);
                dVector3 a11, a12;
                dJointGetDHingeAnchor1(p.second, a11);
                dJointGetDHingeAnchor2(p.second, a12);
                a11[1] -= 0.05;
                a12[1] += 0.05;
                dsDrawLine(a11, a12);
            }
        }
    }

};


#endif // _ROBOTGEOM_H_


