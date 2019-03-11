/* 
 * File:   simulation_data_reader.cpp
 * Author: neha
 * 
 * Created on April 30, 2015, 4:41 PM
 */

#include "simulation_data_reader.h"


void SimulationData::PrintSimulationData(std::ostream& out){
    out <<  current_gripper_pose.pose.position.x << " ";
    out <<  current_gripper_pose.pose.position.y << " ";
    out <<  current_gripper_pose.pose.position.z << " ";
    out <<  current_gripper_pose.pose.orientation.x << " ";
    out <<  current_gripper_pose.pose.orientation.y << " ";
    out <<  current_gripper_pose.pose.orientation.z << " ";
    out <<  current_gripper_pose.pose.orientation.w << " ";
    out << "|";

    out << current_object_pose.pose.position.x << " ";
    out <<  current_object_pose.pose.position.y << " ";
    out <<  current_object_pose.pose.position.z << " ";
    out <<  current_object_pose.pose.orientation.x << " ";
    out <<  current_object_pose.pose.orientation.y << " ";
    out <<  current_object_pose.pose.orientation.z << " ";
    out <<  current_object_pose.pose.orientation.w << " ";
    out << "|";

    for(int i = 0; i < 4; i++)
    {
       out << current_finger_joint_state[i] << " "; 
    }
    out << "|";

    out << next_gripper_pose.pose.position.x << " ";
    out << next_gripper_pose.pose.position.y << " ";
    out << next_gripper_pose.pose.position.z << " ";
    out << next_gripper_pose.pose.orientation.x << " ";
    out << next_gripper_pose.pose.orientation.y << " ";
    out << next_gripper_pose.pose.orientation.z << " ";
    out << next_gripper_pose.pose.orientation.w << " ";
    out << "|";

    out << next_object_pose.pose.position.x << " ";
    out << next_object_pose.pose.position.y << " ";
    out << next_object_pose.pose.position.z << " ";
    out << next_object_pose.pose.orientation.x << " ";
    out << next_object_pose.pose.orientation.y << " ";
    out << next_object_pose.pose.orientation.z << " ";
    out << next_object_pose.pose.orientation.w << " ";
    out << "|";

    for(int i = 0; i < 4; i++)
    {
       out << next_finger_joint_state[i] << " "; 
    }
    out << "|";

  

    out << mico_target_pose.pose.position.x << " ";
    out << mico_target_pose.pose.position.y << " ";
    out << mico_target_pose.pose.position.z << " ";
    out << mico_target_pose.pose.orientation.x << " ";
    out << mico_target_pose.pose.orientation.y << " ";
    out << mico_target_pose.pose.orientation.z << " ";
    out << mico_target_pose.pose.orientation.w << " ";
    out << "|";

    for(int i = 0; i < 2; i++)
    {
        out <<  touch_sensor_reading[i]; 
        out << " ";
    }
    out << std::endl;
}

SimulationDataReader::SimulationDataReader(int v_no) {
    version_no = v_no;
}

SimulationDataReader::SimulationDataReader(const SimulationDataReader& orig) {
}

SimulationDataReader::~SimulationDataReader() {
    
}

void SimulationDataReader::parseSimulationDataLineTableData(std::istream& simulationDataFile, SimulationData& simData, int& action, double& reward) {
    int i;
    simulationDataFile >> i;
    //std::cout << i << " ";
    simulationDataFile >> i;
    //std::cout << i << "|" ;
    parseSimulationDataLine(simulationDataFile, simData, action, reward);
}

void SimulationDataReader::parseSimulationDataLine(std::istream& simulationDataFile, SimulationData& simData, int& action, double& reward)
{
    char c; double temp_read;
    simulationDataFile >> simData.current_gripper_pose.pose.position.x;
    simulationDataFile >> simData.current_gripper_pose.pose.position.y;
    simulationDataFile >> simData.current_gripper_pose.pose.position.z;
    simulationDataFile >> simData.current_gripper_pose.pose.orientation.x;
    simulationDataFile >> simData.current_gripper_pose.pose.orientation.y;
    simulationDataFile >> simData.current_gripper_pose.pose.orientation.z;
    simulationDataFile >> simData.current_gripper_pose.pose.orientation.w;
    simulationDataFile >> c;

    simulationDataFile >> simData.current_object_pose.pose.position.x;
    simulationDataFile >> simData.current_object_pose.pose.position.y;
    simulationDataFile >> simData.current_object_pose.pose.position.z;
    simulationDataFile >> simData.current_object_pose.pose.orientation.x;
    simulationDataFile >> simData.current_object_pose.pose.orientation.y;
    simulationDataFile >> simData.current_object_pose.pose.orientation.z;
    simulationDataFile >> simData.current_object_pose.pose.orientation.w;
    simulationDataFile >> c;

    for(int i = 0; i < 4; i++)
    {
       simulationDataFile >> simData.current_finger_joint_state[i]; 
    }
    simulationDataFile >> c;

    simulationDataFile >> action;
    simulationDataFile >> c;

    simulationDataFile >> simData.next_gripper_pose.pose.position.x;
    simulationDataFile >> simData.next_gripper_pose.pose.position.y;
    simulationDataFile >> simData.next_gripper_pose.pose.position.z;
    simulationDataFile >> simData.next_gripper_pose.pose.orientation.x;
    simulationDataFile >> simData.next_gripper_pose.pose.orientation.y;
    simulationDataFile >> simData.next_gripper_pose.pose.orientation.z;
    simulationDataFile >> simData.next_gripper_pose.pose.orientation.w;
    simulationDataFile >> c;

    simulationDataFile >> simData.next_object_pose.pose.position.x;
    simulationDataFile >> simData.next_object_pose.pose.position.y;
    simulationDataFile >> simData.next_object_pose.pose.position.z;
    simulationDataFile >> simData.next_object_pose.pose.orientation.x;
    simulationDataFile >> simData.next_object_pose.pose.orientation.y;
    simulationDataFile >> simData.next_object_pose.pose.orientation.z;
    simulationDataFile >> simData.next_object_pose.pose.orientation.w;
    simulationDataFile >> c;

    for(int i = 0; i < 4; i++)
    {
       simulationDataFile >> simData.next_finger_joint_state[i]; 
    }
    simulationDataFile >> c;

    for(int i = 0; i < 7; i++)
    {
        simulationDataFile >> temp_read;
    }
    simulationDataFile >> c;

    simulationDataFile >> simData.mico_target_pose.pose.position.x;
    simulationDataFile >> simData.mico_target_pose.pose.position.y;
    simulationDataFile >> simData.mico_target_pose.pose.position.z;
    simulationDataFile >> simData.mico_target_pose.pose.orientation.x;
    simulationDataFile >> simData.mico_target_pose.pose.orientation.y;
    simulationDataFile >> simData.mico_target_pose.pose.orientation.z;
    simulationDataFile >> simData.mico_target_pose.pose.orientation.w;
    simulationDataFile >> c;

    for(int i = 0; i < 4; i++)
    {
       simulationDataFile >> temp_read; 
    }
    simulationDataFile >> c;

    //for(int i = 0; i < 48; i++)
    for(int i = 0; i < 2; i++)
    {
        simulationDataFile >> simData.touch_sensor_reading[i]; 
    }
    simulationDataFile >> c;
    if(version_no >= 7)
    {
        simulationDataFile >> simData.vision_movement;
        simulationDataFile >> c;
    }



    simulationDataFile >> reward;
}
