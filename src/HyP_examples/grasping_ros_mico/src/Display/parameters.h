#ifndef PARAMETERS_H
#define PARAMETERS_H

/**
model parameters
*/
#define GRIPPER_LINK1_LEN 0.06
#define GRIPPER_LINK2_LEN 0.03

//#define OFFSET_MICO_TIP 0.16
#define OFFSET_MICO_TIP 0.06
#define LEN_GRIPPER_BOTTOM 0.055
#define LEN_GRIPPER_TOP 0.17
// #define LEN_GRIPPER 0.0922
#define LEN_GRIPPER 0.08
#define HIGH_GRIPPER 0.0546

#define CUP_RADIUS_BOTTOM 0.035
#define CUP_RADIUS_TOP 0.045
// #define CUP_RADIUS_MIDDLE 0.04
#define CUP_RADIUS_MIDDLE 0.045


#define TOUCH_THRESHOLD 0.20
#define NUM_POSITIVE_TOUCH 5
// the resolution of travel distribution
#define RESOLUTION_TRAVEL 0.01
#define STEP_SIZE 0.01
#define MULTI_STEPS 1

#define OBJECT_MOVED_THRESHOLD 0.01

#define E_LOG 2.71828182845904524
#define INIT_CUP_HEIGHT 0.25701

// reward settings
// #define REWARD_MOVE_REGION0 -1
// #define REWARD_MOVE_REGION1 -2
// #define REWARD_MOVE_REGION2 -5
#define REWARD_MOVE -1
#define REWARD_CLOSE -1
#define REWARD_OPEN -1
#define REWARD_PICK_SUCCESS 100
#define REWARD_PICK_FAIL -100

#define REWARD_PUSH_OBJECT -10
#define REWARD_GRIPPER_OUT 0
#define REWARD_CLOSE_NSTABLE 0

#define TOUCH_OBS_NOISE 0.2
#define CLOSE_STABLE_DENOMINATE 2

// close action lim of delta_x and delta_y
#define CLOSE_STABLE_DELTAX_THRE 0.245
#define CLOSE_STABLE_DELTAY_THRE 0.036

// close unstable cup push forward distance
#define CLOSE_UNSTABLE_PUSH_FORWARD 0.04

// joint stable threshold
#define JOINT_STABLE_THRES 0.93


enum { // status of the gripper
	G_OPEN = 0,
	G_CLOSE_NSTABLE = 1,
	G_CLOSE_STABLE = 2
};

/**===============================================
information about the workspace
===================================================*/
// Gripper Worksapce
//#define MIN_X 0.16914
//#define MAX_X 0.44686
//#define MIN_Y -0.2876
//#define MAX_Y -0.1152

#define MIN_X 0.3379 //range for gripper movement
#define MAX_X 0.5279  // range for gripper movement
#define MIN_Y 0.0816 // range for gripper movement
#define MAX_Y 0.2316 // range for gripper movement 

// object valid locations
// #define MIN_X_O 0.4586
//#define MIN_X_O 0.51455
//#define MAX_X_O 0.70455
//#define MIN_Y_O -0.3789
//#define MAX_Y_O -0.10782

#define MIN_X_O 0.4586 //range for object location
#define MAX_X_O 0.5517  // range for object location
#define MIN_Y_O  0.0829 // range for object location
#define MAX_Y_O  0.2295 // range for object location


// criteria of the object fall down
#define MIN_Z_O 1.7000

// PICK action
#define PICK_Z_DIFF 0.06
#define PICK_X_VAL 0.3079
#define PICK_Y_VAL 0.1516

/**===============================================
noisy of the vision sensor
===================================================*/
// standard deviation of the Gaussian Distribution
/*#define INITIAL_GAUSSIAN_STD_X 0.03
#define INITIAL_GAUSSIAN_STD_Y 0.03
#define RESAMPLE_GAUSSIAN_STD_X 0.005
#define RESAMPLE_GAUSSIAN_STD_Y 0.005*/
#define INITIAL_GAUSSIAN_STD_X 0.03
#define INITIAL_GAUSSIAN_STD_Y 0.03
#define RESAMPLE_GAUSSIAN_STD_X 0.008
#define RESAMPLE_GAUSSIAN_STD_Y 0.008

/**========================================
Noisy of the observation model
=========================================*/
#define NOISY_TOUCH 0.0

/**========================================
Particle info
=========================================*/
#define NUM_PARTICLE 20
#define NUM_PARTICLE_DISPLAY 50

/**========================================
Observation info
=========================================*/
enum { // status of the gripper
	OBS_NTOUCH = 0,
	OBS_RTOUCH = 1,
	OBS_LTOUCH = 2,
	OBS_BTOUCH = 3,
	OBS_STABLE = 4,
	OBS_NSTABLE = 5
};

/**========================================
Confidence to close the gripper
=========================================*/
#define CONFIDENCE_CLOSE 0.95

#endif
