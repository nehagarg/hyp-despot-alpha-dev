/* 
 * File:   simulation_data_reader.cpp
 * Author: neha
 * 
 * Created on April 30, 2015, 4:41 PM
 */

#include "simulation_data_reader.h"
#include "ActionSpecification.h"


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
    out << "|";
    //if(RobotInterface::version8)
    {
    	out << theta_z_degree_current_object_pose << " ";
    	out << theta_z_degree_next_object_pose << " ";
    }
    out << "|";
    out << pick_success;
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

    if(version_no == 8)
        {
            simulationDataFile >> simData.image_file_name;
            simulationDataFile >> c;
        }

    if(version_no == 8)
    {
    	//Reading from PythonGeneratedTxtFile
    	//Image pca
    	//theta_z_degree current state
    	//theta_z_degree next state
    }

    simulationDataFile >> reward;
}


void SimulationDataReader::parsePythonGeneratedTxtFile(std::string txt_file_name, int action, GraspObject* graspObject)
{
	std::ifstream simulationDataFile;
	simulationDataFile.open(txt_file_name);
	if(!simulationDataFile.good())
	    {
	        std::cout << txt_file_name << "is not good" << std::endl;
	        return ;
	    }
	std::string line;
	std::getline(simulationDataFile, line);
	std::istringstream iss(line);
	int num_samples;
	iss >> num_samples;
	int temp;
	for(int j = 0; j < num_samples; j++)
	{
		std::getline(simulationDataFile, line);
		iss.str(line);
		iss.clear();
		SimulationData simData;
		iss >> simData.current_gripper_pose.pose.position.x;
		iss >> simData.current_gripper_pose.pose.position.y;
		iss >> simData.current_object_pose.pose.position.x;
		iss >> simData.current_object_pose.pose.position.y;
		for(int i = 0; i < 4; i++)
		    {
		       iss >> simData.current_finger_joint_state[i];
		    }
		iss >> temp;
		if(action != A_PICK)
		{
			iss >> simData.next_gripper_pose.pose.position.x;
			iss >> simData.next_gripper_pose.pose.position.y;
			iss >> simData.next_object_pose.pose.position.x;
			iss >> simData.next_object_pose.pose.position.y;
			for(int i = 0; i < 4; i++)
				{
				   iss >> simData.next_finger_joint_state[i];
				}

			for(int i = 0; i < 2; i++)
			    {
			        iss >> simData.touch_sensor_reading[i];
			    }

			double vision_movement_double;
			iss >> vision_movement_double;
			simData.vision_movement = int(vision_movement_double);
			iss >> simData.theta_z_degree_current_object_pose;
			iss >> simData.theta_z_degree_next_object_pose;
			iss >> simData.image_file_name;
			double image_pca_components;
			for(int i = 0; i < 150; i++)
			{
				iss >> image_pca_components;
				simData.image_pca_components.push_back(image_pca_components);
			}

		}
		else
		{
			iss >> simData.theta_z_degree_current_object_pose;
			iss >> simData.pick_success;
		}
		graspObject->simulationDataCollectionWithObject[action].push_back(simData);
		/*if(j==1584)
		{
			simData.PrintSimulationData();
		}*/
	}

	std::getline(simulationDataFile, line);
	iss.str(line);
	iss.clear();
	iss >> num_samples;
	for(int j = 0; j < num_samples; j++)
	{
		std::getline(simulationDataFile, line);
		iss.str(line);
		iss.clear();
		int rel_x, rel_y, theta_z;
		iss >> rel_x;
		iss >> rel_y;
		iss >> theta_z;
		int num_entries;
		iss >> num_entries;
		//std::cout << "Line is " << line << std::endl;
		//std::cout << "Iss str is " << iss.str() << std::endl;
		//std::cout << "Index " << rel_x << " " << rel_y << " " << theta_z << " " << num_entries << std::endl;
		for(int i = 0; i < num_entries; i++)
		{
			int entry;
			iss >> entry;
			graspObject->discretizedSimulationDataInitState[action][std::make_tuple(rel_x,rel_y,theta_z)].push_back(entry);
		}
	}

	if(action !=A_PICK)
	{
		std::getline(simulationDataFile, line);
		iss.str(line);
		iss.clear();
		iss >> num_samples;
		for(int j = 0; j < num_samples; j++)
		{
			std::getline(simulationDataFile, line);
			iss.str(line);
			iss.clear();
			int rel_x, rel_y, theta_z;
			iss >> rel_x;
			iss >> rel_y;
			iss >> theta_z;
			int num_entries;
			iss >> num_entries;
			for(int i = 0; i < num_entries; i++)
			{
				int entry;
				iss >> entry;
				graspObject->discretizedSimulationDataNextState[action][std::make_tuple(rel_x,rel_y,theta_z)].push_back(entry);
			}
		}
	}


}
