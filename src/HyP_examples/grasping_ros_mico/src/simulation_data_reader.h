/* 
 * File:   simulation_data_reader.h
 * Author: neha
 *
 * Created on April 30, 2015, 4:41 PM
 */

#ifndef SIMULATION_DATA_READER_H
#define	SIMULATION_DATA_READER_H

#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include "GraspObject.h"
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
    double image_pca_components[150];
    double theta_z_degree_current_object_pose;
    double theta_z_degree_next_object_pose;
    int pick_success; //Used when reading from python generated txt files
    
    void PrintSimulationData(std::ostream& out = std::cout);
};


class SimulationDataReader {
public:
    SimulationDataReader(int version_no = 6);
    SimulationDataReader(const SimulationDataReader& orig);
    virtual ~SimulationDataReader();
    
    void parseSimulationDataLineTableData(std::istream& simulationDataFile, SimulationData& simData, int& action, double& reward);

    void parseSimulationDataLine(std::istream& simulationDataFile, SimulationData& simData, int& action, double& reward);
    void parsePythonGeneratedTxtFile(std::string txt_file_name, int action, GraspObject* graspObject);

private:
    int version_no;
};

#endif	/* SIMULATION_DATA_READER_H */

