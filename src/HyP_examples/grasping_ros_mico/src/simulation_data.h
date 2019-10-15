/*
 * simulation_data.h
 *
 *  Created on: Oct 13, 2019
 *      Author: neha
 */

#ifndef SIMULATION_DATA_H_
#define SIMULATION_DATA_H_
#include "geometry_msgs/PoseStamped.h"
class SimulationData
{
public:
    geometry_msgs::PoseStamped current_gripper_pose;
    geometry_msgs::PoseStamped current_object_pose;
    double current_finger_joint_state[4];
    geometry_msgs::PoseStamped next_gripper_pose;
    geometry_msgs::PoseStamped next_object_pose;
    double next_finger_joint_state[4];
    geometry_msgs::PoseStamped mico_target_pose;
    //double touch_sensor_reading[48];
    double touch_sensor_reading[2];
    int vision_movement;
    std::string image_file_name;
    std::vector<double>image_pca_components;
    double theta_z_degree_current_object_pose;
    double theta_z_degree_next_object_pose;
    int pick_success; //Used when reading from python generated txt files

    void PrintSimulationData(std::ostream& out = std::cout);
};



#endif /* SIMULATION_DATA_H_ */
