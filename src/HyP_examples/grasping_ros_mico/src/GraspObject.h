/* 
 * File:   GraspObject.h
 * Author: neha
 *
 * Created on November 30, 2017, 2:38 PM
 */

#ifndef GRASPOBJECT_H
#define	GRASPOBJECT_H

#include "ActionSpecification.h"
#include "simulation_data_reader.h"
#include "ScikitModels.h"
#include "Python.h"
#include "GraspingStateRealArm.h"
class GraspObject {
public:
    GraspObject(std::string object_name, std::string data_dir_name_, bool low_friction, bool load_in_vrep=false);
    GraspObject(const GraspObject& orig);
    virtual ~GraspObject();
    void SetObject_name(std::string object_name);
    std::string GetObject_name() const;
    std::string getRegressionModelDir();
    std::vector<std::string> getSASOFilenames(bool use_pruned_data, bool dicretize_data = false);
    void loadObject(bool load_in_vrep = false);
    void rotateZ(double rot_z); //rotation value in degrees
    std::pair<int, int> getDiscretizationIndex(double x1, double y1);
    std::tuple<int, int,int> getDiscretizationIndex(double x1, double y1,double theta_z_degrees);
    std::vector<SimulationData> getSimulationData(geometry_msgs::PoseStamped object_pose, geometry_msgs::PoseStamped gripper_pose, int action, bool use_next = false);
    void getSimulationData(geometry_msgs::PoseStamped object_pose, geometry_msgs::PoseStamped gripper_pose, double theta_z_degrees, std::vector<SimulationData> & tempDataVector,int action, bool use_next = false);
    geometry_msgs::PoseStamped getInitialObjectPose();
    static std::string getObjectPropertyDir(std::string object_name);
    
    static std::string g3db_object_property_dir ;
    static std::string pure_object_property_dir ;
    static std::string object_pointcloud_dir ;
    static std::string object_pointcloud_for_classification_dir ;
    static std::string raw_vision_feedback_dir;
    static std::string with_pca_txt_file_dir;
    
    std::vector<SimulationData> simulationDataCollectionWithObject[A_PICK+1];
    //std::vector<int> simulationDataIndexWithObject[A_PICK+1];
    PyObject* dynamicModels[A_PICK+1];
    MultiScikitModels* dynamicModelsC[A_PICK+1];
    std::map< std::pair<int,int>, std::vector<int> > discretizedSimulationDataWithoutAngleInitState[A_PICK+1];
    std::map< std::pair<int,int>, std::vector<int> > discretizedSimulationDataWithoutAngleNextState[A_PICK+1];
    
    std::map< std::tuple<int,int,int>, std::vector<int> > discretizedSimulationDataInitState[A_PICK+1];
	std::map< std::tuple<int,int,int>, std::vector<int> > discretizedSimulationDataNextState[A_PICK+1];

    
    double min_x_o ;
    double max_x_o ;  // range for object location
    double min_y_o ; // range for object location
    double max_y_o ; // range for object location
    double min_z_o; //= 1.1200 ; //for objects on table
    
    double initial_object_x ;
    double initial_object_y ; //0.148582;
    double initial_object_pose_z; // = 1.1248; //1.7066; //for amazon shelf
    double initial_object_pose_xx;
    double initial_object_pose_yy;
    double initial_object_pose_zz;
    double initial_object_pose_w;
    
    double default_initial_object_pose_z;
    double default_initial_object_pose_x;
    double default_initial_object_pose_y;
    double default_min_z_o;
    
    static double pick_point_x_diff ;
    static double pick_point_y_diff ;
    
    double discretization_step;
    std::string data_dir_name;
    std::string regression_data_dir_name;
    
private:
    
    std::string object_name;
    
    
    
    double min_x_o_low_friction_table ;
    double min_x_o_high_friction_table ;
     //for table with high friction//range for object location
    
    double default_min_z_o_low_friction_table ;
    double default_min_z_o_high_friction_table ; //for objects on high friction table
   
    
    double initial_object_x_low_friction_table ;
    double initial_object_x_high_friction_table ; 
    
    double default_initial_object_pose_z_high_friction_table ; //for object on high friction table
    double default_initial_object_pose_z_low_friction_table ;
    
    double initial_object_y_version5;
    double initial_object_y_version6 ;
    
    std::vector<double> pick_point; //Only for low friction table
    
    std::string getOldSasoFilename();
    PyObject* callPythonFunction(std::string function_name, std::string arg1, std::string arg2, double arg3 = -1);
    
    
};

#endif	/* GRASPOBJECT_H */

