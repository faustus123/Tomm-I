
#ifndef __ACTION__
#define __ACTION__

// Set PWM board channel numbers
#define FL1 0
#define FL2 1
#define FR1 2
#define FR2 3
#define BL1 4
#define BL2 5
#define BR1 6
#define BR2 7

#include <map>
#include <string>
extern std::map<int, std::string> chanid2name;


#include "RobotGeom.h"

//#include <Adafruit_PWMServoDriver.h>
//extern Adafruit_PWMServoDriver pwm; // decalred in main.cpp

//#include <servo_calib.hpp>
#include <stdint.h>

#define MAX_ACTIONS 50
#define CURRENT_ANGLE -1.0
#define NOT_RUNNING 0
#define START_OF_ACTION 1
#define END_OF_ACTION 65535
#define NO_PREDECESSOR MAX_ACTIONS

// Defined in action.cc
void SetupActions(void);
void ClearAllActions(void);
void ScaleActions(dReal factor);
void RunActions(RobotGeom *robotgeom);

class action{

public:
    uint8_t predecessor;
    uint16_t offset_tics;
    uint8_t chanid;
    float start_angle;         // May be set to CURRENT_ANGLE
    float end_angle;
    float active_start_angle;  // either copy of start_angle, or the last angle set for this servo before the action started
    uint16_t duration_tics;

    uint16_t tic; // current tic in cycle 0-duration_tics (NOT_RUNNING if not currently active)
};


class ActionManager{

public:

    action actions[MAX_ACTIONS];
    uint8_t Nactions = 0;
    uint8_t Niterations = 1;

    ActionManager(){

        // Setup map from servo chanid to joint name in RobotGeom
        chanid2name[FL1] ="FL_hip";
        chanid2name[FL2] ="FL_foot";
        chanid2name[FR1] ="FR_hip";
        chanid2name[FR2] ="FR_foot";
        chanid2name[BL1] ="BL_hip";
        chanid2name[BL2] ="BL_foot";
        chanid2name[BR1] ="BR_hip";
        chanid2name[BR2] ="BR_foot";
    }

    //-----------------------------------
    // ClearAllActions
    //-----------------------------------
    void ClearAllActions(void) {
        /// This simply resets the Nactions counter and Niterations counter
        /// to their defaults.
        Nactions = 0;
        Niterations = 1;
    }

    //-----------------------------------
    // AddAction
    //-----------------------------------
    uint8_t AddAction(uint8_t chanid, float end_angle, uint16_t duration_tics, uint8_t predecessor=NO_PREDECESSOR, uint16_t offset_tics=END_OF_ACTION){
        //uint8_t SetNewAction(uint8_t predecessor, uint16_t offset, uint8_t chanid, float start_angle, float end_angle, uint16_t duration_tics){

        if( Nactions >= MAX_ACTIONS ){
            // Serial.println("ERROR: MAX_ACTIONS exceeded!");
            return MAX_ACTIONS;
        }

        if( (predecessor >= Nactions) && (predecessor != NO_PREDECESSOR) ){
            // Serial.println("ERROR: specified predecessor has not been defined!");
            return MAX_ACTIONS;
        }

        action *a = &actions[Nactions];

        a->chanid             = chanid;
        a->end_angle          = end_angle;
        a->duration_tics      = duration_tics;
        a->predecessor        = predecessor;
        a->offset_tics        = offset_tics;    // may be overwritten below if predecssor is specified
        a->start_angle        = CURRENT_ANGLE;
        a->active_start_angle = 0;
        a->tic                = NOT_RUNNING;

        if( (predecessor != NO_PREDECESSOR) && (a->offset_tics==END_OF_ACTION) ){
            a->offset_tics = actions[predecessor].duration_tics;
        }
        Nactions++;
        return Nactions-1;
    }

    //-----------------------------------
    // AddHipAction
    //
    // Add the same action to all hip joints
    //-----------------------------------
    uint8_t AddHipAction(float end_angle, uint16_t duration_tics, uint8_t predecessor=NO_PREDECESSOR, uint16_t offset_tics=END_OF_ACTION){
        AddAction(FL1, end_angle, duration_tics, predecessor, offset_tics);
        AddAction(FR1, end_angle, duration_tics, predecessor, offset_tics);
        AddAction(BL1, end_angle, duration_tics, predecessor, offset_tics);
        return AddAction(BR1, end_angle, duration_tics, predecessor, offset_tics);
    }

    //-----------------------------------
    // AddKneeAction
    //
    // Add the same action to all knee joints
    //-----------------------------------
    uint8_t AddKneeAction(float end_angle, uint16_t duration_tics, uint8_t predecessor=NO_PREDECESSOR, uint16_t offset_tics=END_OF_ACTION){
        AddAction(FL2, end_angle, duration_tics, predecessor, offset_tics);
        AddAction(FR2, end_angle, duration_tics, predecessor, offset_tics);
        AddAction(BL2, end_angle, duration_tics, predecessor, offset_tics);
        return AddAction(BR2, end_angle, duration_tics, predecessor, offset_tics);
    }

    //-----------------------------------
    // AddDelay
    //
    // Add a delay action after the last defined action
    //-----------------------------------
    uint8_t AddDelay(uint16_t duration_tics){
        uint8_t predecessor = Nactions>0 ? Nactions-1:NO_PREDECESSOR;
        return AddAction(15, 0, duration_tics, predecessor, END_OF_ACTION);
    }

    //-----------------------------------
    // SetStartAngle
    //-----------------------------------
    uint8_t SetStartAngle(uint8_t idx, float start_angle){
        if( idx >= Nactions ){
            // Serial.println("ERROR: specified action has not been defined!");
            return MAX_ACTIONS;
        }
        action *a = &actions[idx];

        a->start_angle = start_angle;

        return 0;
    }
    //-----------------------------------
    // RunActions
    //-----------------------------------
    void RunActions(RobotGeom *robotgeom){

        robotgeom->max_servo_velocity_scale = 1.0; // Make sure this is reset from anything done before we started this

        // Update tic counter for all active actions.
        uint8_t Ninactive=0;
        for( uint8_t i=0; i < Nactions; i++ ){
            action *a = &actions[i];
            if( a->tic == NOT_RUNNING ){
                Ninactive++;
            }else{
                // Serial.println("Advancing...");
                a->tic++;
            }
        }

        // If no actions are active then activate all actions marked as having no predecessor.
        if( Ninactive == Nactions ){
            if(Niterations > 0){
                // Serial.println("Starting all actions...");
                Niterations--;
                for( uint8_t i=0; i < Nactions; i++ ){
                    action *a = &actions[i];
                    if( a->predecessor == NO_PREDECESSOR ) {
//                        _DBG_<<" starting action " << (int)i << std::endl;
                        a->tic = 1;  // start action
                        a->active_start_angle = a->start_angle;
                        if( chanid2name.count(a->chanid) == 0) continue;
                        if(a->active_start_angle == CURRENT_ANGLE) a->active_start_angle = robotgeom->last_servo_setting[chanid2name[a->chanid]];
                    }
                }
            }else{
                // No more iterations. Turn off all servos
                for(int chanid=0; chanid<16; chanid++)
                    if( chanid2name.count(chanid) ) robotgeom->SetServoIdle(chanid2name[chanid]);
            }
        }

        // Check all actions to see if predecessor state indicates starting an action
        for( uint8_t i=0; i < Nactions; i++ ){
            action *a = &actions[i];
            if( a->predecessor == NO_PREDECESSOR ) continue;
            action *a_pred = &actions[a->predecessor];
            // Serial.print("a_pred->tic: "); Serial.print(a_pred->tic); Serial.print("  a->tic: "); Serial.print(a->tic); ; Serial.print("  a->offset: "); Serial.println(a->offset_tics);
            // if( a_pred->tic == NOT_RUNNING ) continue;
            if( a_pred->tic == a->offset_tics ){
                // Serial.print("Starting secondary action: "); Serial.println(i);
//                _DBG_<<" starting action " << (int)i << std::endl;
                a->tic = 1;
                a->active_start_angle = a->start_angle;
//                if(a->active_start_angle == CURRENT_ANGLE) a->active_start_angle = servos[a->chanid].last_setting;
                if(a->active_start_angle == CURRENT_ANGLE)
                    if(chanid2name.count(a->chanid)) a->active_start_angle = robotgeom->last_servo_setting[chanid2name[a->chanid]];
            }
        }

        // Check all actions to see if they have reached their end
        for( uint8_t i=0; i < Nactions; i++ ){
            action *a = &actions[i];
            if( a->tic > a->duration_tics ) {
                // Serial.print("Ending action..."); Serial.print(i); Serial.print("  chan:"); Serial.println(a->chanid);
                a->tic = NOT_RUNNING;
            }
        }
        // Update all servos for active actions
        for( uint8_t i=0; i < Nactions; i++ ){

            action *a = &actions[i];
            // if( a->chanid==0 ) {Serial.print("Channel 0 tic: "); Serial.println(a->tic);}
            if( a->tic == NOT_RUNNING ) continue;
            if( chanid2name.count(a->chanid) == 0 ) continue;  // delay actions use a non-used servo channel

            if( chanid2name[a->chanid] == "" ){
                _DBG_<<"ERROR: Action " << i << " is trying to use chanid " << (int)a->chanid << " which has no name!" << std::endl;
            }

            float angle = a->active_start_angle + (a->end_angle - a->active_start_angle)*(float)a->tic/(float)a->duration_tics;
            robotgeom->SetServoAngle(chanid2name[a->chanid], angle);

            // if( a->chanid==0 ) {Serial.print("Setting channel 0 to: "); Serial.println(angle);}
        }
    }
};

#endif // __ACTION__

