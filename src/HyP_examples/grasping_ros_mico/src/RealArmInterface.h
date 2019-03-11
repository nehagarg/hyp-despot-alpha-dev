/*
 * File:   RealArmInterface.h
 * Author: neha
 *
 * Created on May 6, 2015, 2:06 PM
 */

#ifndef REALARMINTERFACE_H
#define	REALARMINTERFACE_H

#include "VrepDataInterface.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "hyp_despot/MicoActionFeedback.h"

class RealArmInterface : public VrepDataInterface {
public:
    RealArmInterface(int start_state_index_ = -1);
    RealArmInterface(const RealArmInterface& orig);
    virtual ~RealArmInterface();


    bool StepActual(GraspingStateRealArm& state, double random_num, int action, double& reward, GraspingObservation& obs) const;

    void CreateStartState(GraspingStateRealArm& initial_state, std::string type) const;
    std::pair<std::map<int,double>,std::vector<double> > GetBeliefObjectProbability(std::vector<int> belief_object_ids) const;

    //bool IsValidState(GraspingStateRealArm grasping_state) const;

    //double ObsProb(GraspingObservation grasping_obs, const GraspingStateRealArm& grasping_state, int action) const;

        //For real scenario
    //double real_min_x_i = 0.3379; //range for gripper movement
    //double real_max_x_i = 0.5279;  // range for gripper movement
    //double real_min_y_i = 0.0816; // range for gripper movement
    //double real_max_y_i = 0.2316; // range for gripper movement
    //double real_gripper_in_x_i = 0.3779;


    //double real_min_x_o = 0.4586; //range for object location
   // double real_max_x_o = 0.5517;  // range for object location
   // double real_min_y_o = 0.0829; // range for object location
    //double real_max_y_o = 0.2295; // range for object location

    double vrep_finger_joint_min = 0;
    double vrep_finger_joint_max = 1.07;
    double vrep_finger_joint_for_dummy_joint_value_change = 1.5;
    double vrep_dummy_finger_joint_min = 0.0;
    double vrep_dummy_finger_joint_max = 0.4;
    double real_finger_joint_min = 0.0;
    double real_finger_joint_max = 1.4; //Actual is 1.5  but mico gripper sometimes does not close fully
    double vrep_touch_value_max = 2.5; //2.6;
    double vrep_touch_value_min = 0.0;

    double tip_wrt_hand_link_x = 0.127; //(Mico link 7 in vrep)

    mutable double real_gripper_offset_x;
    mutable double real_gripper_offset_y;
    mutable double real_gripper_offset_z;
    mutable double real_touch_threshold; //Fetched from mico action feedback node = 75.0;
    mutable double real_touch_value_min = 0.0;
    mutable double real_touch_value_max = 1000.0;

private:

    mutable ros::NodeHandle grasping_n;
    mutable ros::ServiceClient micoActionFeedbackClient;


    //void CheckAndUpdateGripperBounds(GraspingStateRealArm& grasping_state, int action) const;
    //bool CheckTouch(double current_sensor_values[], int on_bits[], int size = 2) const;
    //void GetDefaultPickState(GraspingStateRealArm& grasping_state) const;
    //void GetRewardBasedOnGraspStability(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs, double& reward) const;
    //bool IsValidPick(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs) const;

    void AdjustRealGripperPoseToSimulatedPose(geometry_msgs::PoseStamped& gripper_pose) const;
    void AdjustRealObjectPoseToSimulatedPose(geometry_msgs::PoseStamped& object_pose) const;
    void AdjustRealFingerJointsToSimulatedJoints(GraspingStateRealArm& state, double gripper_obs_values[]) const;
    void AdjustTouchSensorToSimulatedTouchSensor(double gripper_joint_values[]) const;
    PyObject* get_belief_module;

};

#endif	/* REALARMINTERFACE_H */
