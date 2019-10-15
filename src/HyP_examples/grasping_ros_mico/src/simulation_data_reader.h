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
#include "simulation_data.h"
#include "GraspObject.h"



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

