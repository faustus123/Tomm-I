

// This implements the action configuration done in the Arduino code
// as part of the setup() routine there.
//
// Note that all of the methods of the ActionManager class are defined
// in the action.hpp header file.

#include "action.hpp"

#define TIME_DELAY          10 // milliseconds (this sets the time delay between tics of the actions)

#define PARK_HIP_ANGLE    90 // degrees
#define PARK_KNEE_ANGLE   90 // degrees
#define STAND_HIP_ANGLE  120 // degrees
#define STAND_KNEE_ANGLE 130 // degrees

#define SIT_FRONT_HIP_ANGLE  90 // degrees
#define SIT_FRONT_KNEE_ANGLE 170 // degrees
#define SIT_BACK_HIP_ANGLE  110 // degrees
#define SIT_BACK_KNEE_ANGLE 90 // degrees

#define LAY_HIP_ANGLE  160
#define LAY_KNEE_ANGLE 65

std::map<int, std::string> chanid2name;
ActionManager action_manager;

//------------------------------------------------------------
// SetupActions
//
void SetupActions(void){

    uint8_t last_action = 0;

    // Delay 2 seconds to allow robot to settle in park position
    last_action = action_manager.AddDelay(2000/TIME_DELAY);

    // Stand
    uint16_t duration = 100;
    action_manager.AddHipAction(STAND_HIP_ANGLE, duration, last_action);
    // action_manager.AddKneeAction(STAND_KNEE_ANGLE+20, duration);
    // last_action = action_manager.AddDelay(20);
    action_manager.AddKneeAction(STAND_KNEE_ANGLE, 50, last_action);

    // Delay 3 seconds
    last_action = action_manager.AddDelay(3000/TIME_DELAY);

    // Sit
    duration = 120;
    action_manager.AddAction(BL1, SIT_BACK_HIP_ANGLE,   duration, last_action);
    action_manager.AddAction(BR1, SIT_BACK_HIP_ANGLE,   duration, last_action);
    action_manager.AddAction(BL2, SIT_BACK_KNEE_ANGLE,  duration, last_action);
    action_manager.AddAction(BR2, SIT_BACK_KNEE_ANGLE,  duration, last_action);
    action_manager.AddAction(FL1, SIT_FRONT_HIP_ANGLE,  duration, last_action);
    action_manager.AddAction(FR1, SIT_FRONT_HIP_ANGLE,  duration, last_action);
    action_manager.AddAction(FL2, SIT_FRONT_KNEE_ANGLE, duration, last_action);
    action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE, duration, last_action);

    // Delay 2 seconds
    last_action = action_manager.AddDelay(2000/TIME_DELAY);

    // Shake
    duration = 170;
    last_action = action_manager.AddAction(BR2, 125,   duration, last_action);                      // push back right foot down
    last_action = action_manager.AddAction(BL1, SIT_BACK_HIP_ANGLE+20,   duration, last_action, START_OF_ACTION); // push back left hip out
    last_action = action_manager.AddAction(FR1, SIT_FRONT_HIP_ANGLE+40,   100, last_action, 70);    // left front right hip
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE+10,   40, last_action, 20);    // extend front right foot
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE-10,  40, last_action);         // shake front right foot down
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE+10,  40, last_action);         // shake front right foot up
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE-10,  40, last_action);         // shake front right foot down
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE+10,  40, last_action);         // shake front right foot up
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE-10,  40, last_action);         // shake front right foot down
    last_action = action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE+10,  40, last_action);         // shake front right foot up


    // Restore Sitting after shake
    duration = 120;
    action_manager.AddAction(BL1, SIT_BACK_HIP_ANGLE,   duration, last_action);
    // action_manager.AddAction(BR1, SIT_BACK_HIP_ANGLE,   duration, last_action);
    // action_manager.AddAction(BL2, SIT_BACK_KNEE_ANGLE,  duration, last_action);
    action_manager.AddAction(BR2, SIT_BACK_KNEE_ANGLE,  duration, last_action);
    // action_manager.AddAction(FL1, SIT_FRONT_HIP_ANGLE,  duration, last_action);
    action_manager.AddAction(FR1, SIT_FRONT_HIP_ANGLE,  duration, last_action);
    // action_manager.AddAction(FL2, SIT_FRONT_KNEE_ANGLE, duration, last_action);
    action_manager.AddAction(FR2, SIT_FRONT_KNEE_ANGLE, duration, last_action);

    // Delay 2 seconds
    last_action = action_manager.AddDelay(2000/TIME_DELAY);

    // Laydown
    duration = 200;
    action_manager.AddHipAction(LAY_HIP_ANGLE, duration, last_action);
    action_manager.AddKneeAction(LAY_KNEE_ANGLE, duration, last_action);

    // Delay 2 seconds
    last_action = action_manager.AddDelay(2000/TIME_DELAY);

    // Roll over
    duration = 100;
    last_action =
    action_manager.AddAction(FL1,  90, duration, last_action);
    action_manager.AddAction(FL2, 180, duration, last_action, START_OF_ACTION);
    action_manager.AddAction(BL1,  90, duration, last_action, START_OF_ACTION);
    action_manager.AddAction(BL2, 180, duration, last_action, START_OF_ACTION);

    duration = 50;
    action_manager.AddAction(FL1, LAY_HIP_ANGLE, duration, last_action);
    action_manager.AddAction(FL2, LAY_KNEE_ANGLE, duration, last_action);
    action_manager.AddAction(BL1, LAY_HIP_ANGLE, duration, last_action);
    action_manager.AddAction(BL2, LAY_KNEE_ANGLE, duration, last_action);

    // Park
    // action_manager.AddHipAction(PARK_HIP_ANGLE, duration, last_action);
    // action_manager.AddKneeAction(PARK_KNEE_ANGLE, duration, last_action);

    // Delay 2 seconds after first action is complete
    last_action = action_manager.AddDelay(20000/TIME_DELAY);

    std::cout << "Nactions defined: " << (int)action_manager.Nactions << " (last_action="<< (int)last_action << ")" << std::endl;
}

//------------------------------------------------------------
// ClearAllActions
//
void ClearAllActions(void) {
    action_manager.ClearAllActions();
}

//------------------------------------------------------------
// ScaleActions
//
// This will scale all of the currently registered action durations
// by the given amount. This is done since the simulation time steps
// may be different than the standard amount of time taken for a tic
// on the arduino
void ScaleActions(dReal factor)
{
    for(int i=0; i<action_manager.Nactions; i++){
        action &a = action_manager.actions[i];
        a.duration_tics = ceil(((dReal)a.duration_tics)*factor);
        a.offset_tics = ceil(((dReal)a.offset_tics)*factor);
    }
}

//------------------------------------------------------------
// RunActions
//
// This is called from the simLoop routine once every time it
// is called. It just bounces the call to the ActionManager
// which updates one step.
void RunActions(RobotGeom *robotgeom)
{
    action_manager.RunActions(robotgeom);
}