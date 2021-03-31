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
#include <drawstuff/drawstuff.h>
#include <vector>
#include <map>
#include <iostream>

#ifndef _DBG_
#define _DBG_ std::cout<<__FILE__<<":"<<__LINE__<<" "
#define _DBG__ std::cout<<__FILE__<<":"<<__LINE__<<std::endl
#endif

// This is used to make it easier to write rotation matrix elements
#define r(i,j) rot[i-1 + (j-1)*4]

inline void CopydVector3(dVector3 &dest, const dVector3 &src){for(int i=0;i<4;i++) dest[i] = src[i];}

class RobotGeom{
public:

    dWorldID worldID;
    dSpaceID spaceID;
    std::map<std::string, dBodyID > bodies;
    std::map<std::string, dGeomID > geoms;
    std::map<std::string, dMass>    masses;
    std::map<std::string, dVector3> jointpos;
    std::map<std::string, dJointID> joints;

    std::map<std::string, dReal> last_servo_setting;

    dReal deg2rad                = M_PI/180.0;
    dReal rad2deg                = 180.0/M_PI;
    dReal initial_height         = 0.60; // set this so robot is completely off of the ground and legs are hanging straight (see GetJointAngle)
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
    dReal maxServoTorque = 12.0*9.81/100.0*2.0; // nominal stall torque is 12kg-cm or (12kg)(9.81m/s^2)(1cm) ~= 0.12 N-m however both motors are geared up to double the torque
    dReal max_servo_velocity = M_PI/0.5; // max servo velocity in radians per second
    dReal max_servo_velocity_scale = 1.0;

    //-----------------------------------------
    // RobotGeom
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
        CreateLeg("BR", leg_shift_parallel_to_body,  leg_shift_perpendicular_to_body, 0.0, true);
        CreateLeg("BL", leg_shift_parallel_to_body, -leg_shift_perpendicular_to_body, 0.0, true);

        // Add mass objects to all bodies
        for(auto p : bodies){
            if( masses.count(p.first) ){
                std::cout << "Setting mass of " << p.first << " to " << masses[p.first].mass << " kg" << std::endl;
                dBodySetMass(p.second, &masses[p.first]);
            }else{
                std::cout << "ERROR: no mass object for " << p.first << "!" << std::endl;
             }
        }
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

        //---------- Create hip bone (attach directly to frame body)
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

        // Put stops on the angles for the joints
        dReal hip_lo_stop = -30.0*deg2rad;
        dReal hip_hi_stop = +50.0*deg2rad;
        dReal foot_lo_stop = -160.0*deg2rad;
        dReal foot_hi_stop =   +0.0*deg2rad;
        if( rotateZ ){
            dReal tmp = hip_lo_stop;
            hip_lo_stop = -hip_hi_stop;
            hip_hi_stop = -tmp;
            tmp = foot_lo_stop;
            foot_lo_stop = -foot_hi_stop;
            foot_hi_stop = -tmp;
        }
        dJointSetHingeParam (joint_hip, dParamLoStop, hip_lo_stop); // stop angle (relative to initial position)
        dJointSetHingeParam (joint_hip, dParamHiStop, hip_hi_stop); // stop angle (relative to initial position)
        dJointSetHingeParam (joint_foot, dParamLoStop,  foot_lo_stop); // stop angle (relative to initial position)
        dJointSetHingeParam (joint_foot, dParamHiStop,  foot_hi_stop); // stop angle (relative to initial position)

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
    // Draw lines at the to hinge positions (for debugging)
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
                dsDrawLineD(pos1, pos2);
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
                dsDrawLineD(a11, a12);
            }
        }
    }

    //-------------------------------------------------
    // GetJointAngle
    //
    // Get the current angle of the specified joint in degrees.
    dReal GetJointAngle(const std::string &joint_name){

        // Assume here that the robot geometry was initially defined such
        // that the legs are hanging straight down. This is so we easily
        // know what the "zero" angle is for the joint. The zero angle
        // is recorded by the joint when the bodies are attached to it
        // so that dJointGetHingeAngle measures relative to that.
        //
        // This means for the convention used in the physical robot arduino
        // programming, the zero angle corresponds to 90 degrees for the
        // hip and 180 degrees for the foot.

        if( joints.count(joint_name) == 0){
            _DBG_<< "ERROR: Requested joint \"" << joint_name << "\" does not exist!" << std::endl;
            return 0.0;
        }

        auto theta = dJointGetHingeAngle(joints[joint_name]);
        if( joint_name[0] =='B'  ) theta = -theta;
        if( joint_name.find("_hip") != std::string::npos  ) theta = theta + M_PI_2;
        if( joint_name.find("_foot") != std::string::npos ) theta = M_PI + theta;

        return theta*rad2deg;
    }

    //-------------------------------------------------
    // SetServoAngle
    //
    // Adjust the motor corresponding to the specified joint
    // to try and hold this angle
    void SetServoAngle(const std::string &joint_name, dReal theta){

        if( joints.count(joint_name) == 0){
            _DBG_<< "ERROR: Requested joint \"" << joint_name << "\" does not exist!" << std::endl;
            return;
        }

        dReal theta_curr = GetJointAngle(joint_name);
        dReal delta_theta = theta - theta_curr;

        // tanh transitions between +/-1. The "scale" factor tells it how
        // smoothly to transition from max velocity to zero and in some
        // sense what the dead band is. It is best to think of it as roughly
        // +/- the angular range in degrees the servos will start to settle.
        // n.b. this should be angle of leg, not servo since gears/pulleys
        // make those differ by factor of 2.
        //
        // The both the max_servo_velocity and max_servo_velocity_scale
        // factors are globally set, but max_servo_velocity should stay
        // constant. The max_servo_velocity_scale can be set to a number
        // 0-1 to limit the servo velocity to something slower.
        // Note that in the arduino program this speed governance was
        // achieved by setting the servo value in steps.
        dReal scale = 5.0; // This should be roughly
        dReal w = tanh(delta_theta/scale)*max_servo_velocity*max_servo_velocity_scale; // max_servo_angular_velocity = pi radians per 0.5 sec
        if( joint_name[0] =='B'  ) w = -w;
        dJointSetHingeParam(joints[joint_name], dParamFMax, maxServoTorque);
        dJointSetHingeParam (joints[joint_name], dParamVel, w);

        last_servo_setting[joint_name] = theta;
    }

    //-------------------------------------------------
    // SetServoIdle
    //
    // Set specified servo to idle
    void SetServoIdle( const std::string &name ) {
        if( joints.count(name) == 0){
            _DBG_<< "ERROR: Requested joint \"" << name << "\" does not exist!" << std::endl;
            return;
        }

        dJointSetHingeParam (joints[name], dParamFMax, 0.0);
    }

    //-------------------------------------------------
    // RefreshServos
    //
    // Set all servos based on last call to SetServoAngle.
    // This is called at the end of each step in the simulation
    // to mimic how the PWM + servos themselves keep track of
    // their angle settings.
    void RefreshServos(void) {
        for( auto p : joints ) {
            SetServoAngle(p.first, last_servo_setting[p.first]);
        }
    }

    //-------------------------------------------------
    // SetStand
    //
    // Set all servos to the standing positons
    void SetStand(void) {

        max_servo_velocity_scale = 0.5;

        dReal theta_hip = 120.0;
        dReal theta_foot = 130.0;
        SetServoAngle("FL_hip", theta_hip);
        SetServoAngle("FR_hip", theta_hip);
        SetServoAngle("BL_hip", theta_hip);
        SetServoAngle("BR_hip", theta_hip);
        SetServoAngle("FL_foot", theta_foot);
        SetServoAngle("FR_foot", theta_foot);
        SetServoAngle("BL_foot", theta_foot);
        SetServoAngle("BR_foot", theta_foot);
    }

    //-------------------------------------------------
    // SetPark
    //
    // Set all servos to the park positons
    void SetPark(void) {

        max_servo_velocity_scale = 0.5;

        dReal theta_hip = 90.0;
        dReal theta_foot = 90.0;
        SetServoAngle("FL_hip", theta_hip);
        SetServoAngle("FR_hip", theta_hip);
        SetServoAngle("BL_hip", theta_hip);
        SetServoAngle("BR_hip", theta_hip);
        SetServoAngle("FL_foot", theta_foot);
        SetServoAngle("FR_foot", theta_foot);
        SetServoAngle("BL_foot", theta_foot);
        SetServoAngle("BR_foot", theta_foot);
    }

    //-------------------------------------------------
    // SetSit
    //
    // Set all servos to the sit positons
    void SetSit(void) {

        max_servo_velocity_scale = 0.5;

        dReal SIT_FRONT_HIP_ANGLE = 90; // degrees
        dReal SIT_FRONT_KNEE_ANGLE= 170; // degrees
        dReal SIT_BACK_HIP_ANGLE  = 110; // degrees
        dReal SIT_BACK_KNEE_ANGLE = 90; // degrees

        SetServoAngle("FL_hip", SIT_FRONT_HIP_ANGLE);
        SetServoAngle("FR_hip", SIT_FRONT_HIP_ANGLE);
        SetServoAngle("BL_hip", SIT_BACK_HIP_ANGLE);
        SetServoAngle("BR_hip", SIT_BACK_HIP_ANGLE);
        SetServoAngle("FL_foot", SIT_FRONT_KNEE_ANGLE);
        SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE);
        SetServoAngle("BL_foot", SIT_BACK_KNEE_ANGLE);
        SetServoAngle("BR_foot", SIT_BACK_KNEE_ANGLE);
    }

    //-------------------------------------------------
    // SetLaydown
    //
    // Set all servos to the park positons
    void SetLaydown(void) {

        max_servo_velocity_scale = 0.5;

        dReal theta_hip = 160.0;
        dReal theta_foot = 65.0;
        SetServoAngle("FL_hip", theta_hip);
        SetServoAngle("FR_hip", theta_hip);
        SetServoAngle("BL_hip", theta_hip);
        SetServoAngle("BR_hip", theta_hip);
        SetServoAngle("FL_foot", theta_foot);
        SetServoAngle("FR_foot", theta_foot);
        SetServoAngle("BL_foot", theta_foot);
        SetServoAngle("BR_foot", theta_foot);
    }

    //-------------------------------------------------
    // Shake
    //
    // Start the shake sequence. Subsequent calls will advance
    // the sequence until it is done.
    void Shake(dReal sec_per_call) {

        max_servo_velocity_scale = 0.5;

        dReal SIT_FRONT_HIP_ANGLE = 90; // degrees
        dReal SIT_FRONT_KNEE_ANGLE= 170; // degrees
        dReal SIT_BACK_HIP_ANGLE  = 110; // degrees
        dReal SIT_BACK_KNEE_ANGLE = 90; // degrees

        static dReal icounter = 0.0;
        dReal T = icounter*sec_per_call;
        icounter += 1.0;
//        if(T>4.0) icounter=0;

        // Convert T to units consistent with numbers from arduino code. Those
        // numbers are in tics which should represent about 10ms each.
        T*=1000.0;
//        std::cout << "T=" << T << std::endl;

        if( T < 170 ) {
            SetServoAngle("BR_foot", 125);
            SetServoAngle("BL_hip", SIT_BACK_HIP_ANGLE+20);
        }else if( T< (170+100)){
            SetServoAngle("FR_hip", SIT_FRONT_HIP_ANGLE+40);
        }else if( T< (170+100+40*1)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE+10);
        }else if( T< (170+100+40*2)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE-10);
        }else if( T< (170+100+40*3)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE+10);
        }else if( T< (170+100+40*4)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE-10);
        }else if( T< (170+100+40*5)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE+10);
        }else if( T< (170+100+40*6)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE-10);
        }else if( T< (170+100+40*7)){
            SetServoAngle("FR_foot", SIT_FRONT_KNEE_ANGLE+10);
        }else{
            SetStand();
        }
    }

    //-------------------------------------------------
    // GetStatus
    //
    // Get robot status in form of std::map.
    void GetStatus(std::map<std::string, dReal> &vals){

        // Get orientation of body wrt lab x,y, and z axes.
        // The cos_* parameters form a unit vector of the "frame"
        // direction. The original frame direction should be
        // pointing straight up along the z-axis.
        auto rot = dBodyGetRotation(bodies["frame"]);
        dReal theta_x = atan2(r(3,2), r(3,3) );
        dReal theta_y = atan2(-r(3,1), sqrt( pow(r(3,2), 2.0) + pow(r(3,3),2.0)) );
        dReal theta_z = atan2(r(2,1), r(1,1) );

        vals["theta_x"] = theta_x;
        vals["theta_y"] = theta_y;
        vals["theta_z"] = theta_z;

        // Get angles of all servos
        for(auto p : joints ){
            dReal theta_deg = rad2deg*dJointGetHingeAngle( p.second );
            vals[p.first] = theta_deg;
        }
    }

    //-------------------------------------------------
    // Relax
    //
    // Set all servos off
    void Relax(void) {
        for(auto p: joints){
            dJointSetHingeParam (p.second, dParamFMax, 0.0);
        }
    }

};


#endif // _ROBOTGEOM_H_


