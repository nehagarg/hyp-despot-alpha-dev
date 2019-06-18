/*
 * File:   GraspingObservation.h
 * Author: neha
 *
 * Created on May 5, 2015, 11:20 AM
 */

#ifndef GRASPINGOBSERVATION_H
#define	GRASPINGOBSERVATION_H

#include "geometry_msgs/PoseStamped.h"
#include "observation.h"
#include "GraspingStateRealArm.h"

class GraspingObservation : public ObservationClass {
 public :

    geometry_msgs::PoseStamped gripper_pose;
    geometry_msgs::PoseStamped mico_target_pose; //difference in mico tip and target pose
    double finger_joint_state[4]; //For finger configuration
    //geometry_msgs::Vector3 force_values[48];
    //geometry_msgs::Vector3 torque_values[48];
    double touch_sensor_reading[2];
    int vision_movement;
    //double x_change = 0.0; //change in x coordinate after taking an action
    //double y_change = 0.0; // change in y coordinate after taking an action
    //double z_change = 0.0; //change in z coordinate after taking an action
    //TODO : add a struct for tactile sensor observation, implement gethash function
    //uint64_t GetHash() const {return obs;     };

    std::vector <double> top_view_image;
    std::vector <double> depth_view_image_size;
    std::string rgb_image_name;
    std::string depth_view_image_name;
    std::vector<float> keras_observation;

   void getObsFromString(std::string obs_string, int version_no)
    {
      //  0.3379 0.1516 1.73337 -0.694327 -0.0171483 -0.719 -0.0255881|0.4586 0.0829 1.7066 -0.0327037 0.0315227 -0.712671 0.700027|-2.95639e-05 0.00142145 -1.19209e-
//06 -0.00118446
        std::istringstream inputString(obs_string);
        getObsFromStream(inputString, version_no);

    }

    void getObsFromStream(std::istream& inputString, int version_no)
    {

        char c;
        inputString >> gripper_pose.pose.position.x;
        inputString >> gripper_pose.pose.position.y;
        inputString >> gripper_pose.pose.position.z;
        inputString >> gripper_pose.pose.orientation.x;
        inputString >> gripper_pose.pose.orientation.y;
        inputString >> gripper_pose.pose.orientation.z;
        inputString >> gripper_pose.pose.orientation.w;
        inputString >> c;

        inputString >> mico_target_pose.pose.position.x;
        inputString >> mico_target_pose.pose.position.y;
        inputString >> mico_target_pose.pose.position.z;
        inputString >> mico_target_pose.pose.orientation.x;
        inputString >> mico_target_pose.pose.orientation.y;
        inputString >> mico_target_pose.pose.orientation.z;
        inputString >> mico_target_pose.pose.orientation.w;
        inputString >> c;

        for(int i = 0; i < 4; i++)
        {
            inputString >> finger_joint_state[i];
        }
        inputString >> c;
        for(int i = 0; i < 2; i++)
        {
            inputString >> touch_sensor_reading[i];
        }

        if(version_no == 7)
        {
            inputString >> c;
           inputString >> vision_movement;
        }
    }

    void getObsFromState(GraspingStateRealArm initial_state)
    {
        gripper_pose.pose.position.x  = initial_state.gripper_pose.pose.position.x ;
        gripper_pose.pose.position.y  = initial_state.gripper_pose.pose.position.y ;
        gripper_pose.pose.position.z  = initial_state.gripper_pose.pose.position.z ;
        gripper_pose.pose.orientation.x = initial_state.gripper_pose.pose.orientation.x ;
        gripper_pose.pose.orientation.y = initial_state.gripper_pose.pose.orientation.y ;
        gripper_pose.pose.orientation.z = initial_state.gripper_pose.pose.orientation.z  ;
        gripper_pose.pose.orientation.w = initial_state.gripper_pose.pose.orientation.w ;

        mico_target_pose.pose.position.x  = initial_state.gripper_pose.pose.position.x ;
        mico_target_pose.pose.position.y  = initial_state.gripper_pose.pose.position.y ;
        mico_target_pose.pose.position.z  = initial_state.gripper_pose.pose.position.z ;
        mico_target_pose.pose.orientation.x = initial_state.gripper_pose.pose.orientation.x ;
        mico_target_pose.pose.orientation.y = initial_state.gripper_pose.pose.orientation.y ;
        mico_target_pose.pose.orientation.z = initial_state.gripper_pose.pose.orientation.z  ;
        mico_target_pose.pose.orientation.w = initial_state.gripper_pose.pose.orientation.w ;




        finger_joint_state[0] = initial_state.finger_joint_state[0]  ;
        finger_joint_state[1] = initial_state.finger_joint_state[1] ;
        finger_joint_state[2]= initial_state.finger_joint_state[2] ;
        finger_joint_state[3]= initial_state.finger_joint_state[3] ;

        touch_sensor_reading[0] = initial_state.touch_value[0];
        touch_sensor_reading[1] = initial_state.touch_value[1];

        vision_movement = initial_state.vision_movement;

    }
};


#endif	/* GRASPINGOBSERVATION_H */
