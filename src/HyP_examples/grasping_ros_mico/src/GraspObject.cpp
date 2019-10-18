/* 
 * File:   GraspObject.cpp
 * Author: neha
 * 
 * Created on November 30, 2017, 2:38 PM
 */

#include <vector>
#include <iosfwd>
#include "math.h"
#include "GraspObject.h"
#include "RobotInterface.h"

std::string GraspObject::g3db_object_property_dir = "g3db_object_labels/object_instances/object_instances_updated";
std::string GraspObject::pure_object_property_dir = "pure_shape_labels";
std::string GraspObject::object_pointcloud_dir = "point_clouds";
std::string GraspObject::object_pointcloud_for_classification_dir = "point_clouds_for_classification";
std::string GraspObject::raw_vision_feedback_dir = "raw_vision_feedback_dir";
std::string GraspObject::with_pca_txt_file_dir = "scripts/structure_data_model";
std::vector<double> GraspObject::default_image_pca[A_PICK];
double GraspObject::pick_point_x_diff = -0.03;
double GraspObject::pick_point_y_diff = 0.0;

GraspObject::GraspObject(std::string object_name_, std::string data_dir_name_, bool low_friction, bool load_in_vrep) {
    
    max_x_o = 0.5517;  // range for object location
    min_y_o = 0.0829; // range for object location
    max_y_o = 0.2295; // range for object location
    
    min_x_o_low_friction_table = 0.4319;
    min_x_o_high_friction_table = 0.4586;
     //for table with high friction//range for object location
    
    default_min_z_o_low_friction_table = 1.0950;
    default_min_z_o_high_friction_table = 1.1200 ; //for objects on high friction table
   
    
    initial_object_x_low_friction_table = 0.4919;
    initial_object_x_high_friction_table = 0.498689; 
    
    default_initial_object_pose_z_high_friction_table = 1.1248; //for object on high friction table
    default_initial_object_pose_z_low_friction_table = 1.0998;
    
    initial_object_y_version5 = 0.148582;
    initial_object_y_version6 = 0.1562;
    
    initial_object_pose_xx = 0;
    initial_object_pose_yy = 0;
    initial_object_pose_zz = 0;
    initial_object_pose_w = 1;
    
    
    
    
    
    
    object_name = object_name_;
    data_dir_name = data_dir_name_;
    regression_data_dir_name = data_dir_name_ + "/data_for_regression";
    discretization_step = 0.01;
    if(low_friction)
    {
        min_x_o = min_x_o_low_friction_table;
        default_min_z_o = default_min_z_o_low_friction_table;
        default_initial_object_pose_z = default_initial_object_pose_z_low_friction_table;
        initial_object_x = initial_object_x_low_friction_table;
    }
    else
    {
        min_x_o = min_x_o_high_friction_table;
        default_min_z_o = default_min_z_o_high_friction_table;
        default_initial_object_pose_z = default_initial_object_pose_z_high_friction_table;
        initial_object_x = initial_object_x_high_friction_table;
    }
    min_z_o = default_min_z_o;
    initial_object_pose_z = default_initial_object_pose_z;
    
    if(data_dir_name_.find("ver6") != std::string::npos || data_dir_name_.find("ver7") != std::string::npos || data_dir_name_.find("ver8") != std::string::npos)
    {
        initial_object_y = initial_object_y_version6;
    }
    else
    {
        initial_object_y = initial_object_y_version5;
    }
    
    default_initial_object_pose_x = initial_object_x;
    default_initial_object_pose_y = initial_object_y;
    //Load object properties file if it exists
    //Initialize python script for loading object
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('python_scripts')");
    PyRun_SimpleString("sys.path.append('scripts')");
    char ** argv;
    //std::cout << "Initialized python 1" << std::endl;
    PySys_SetArgvEx(0, argv, 0); //Required when python script import rospy
    loadObject(load_in_vrep);
    
    if(pick_point.size() > 0)
    {
        double expected_pick_x = initial_object_x + pick_point_x_diff;
        double x_diff = expected_pick_x - pick_point[0];
        double expected_pick_y = initial_object_y + pick_point_y_diff;
        double y_diff = expected_pick_y - pick_point[1];
        initial_object_x = initial_object_x + x_diff;
        min_x_o = min_x_o + x_diff;
        max_x_o = max_x_o + x_diff;
        
        initial_object_y = initial_object_y + y_diff;
        min_y_o = min_y_o + y_diff;
        max_y_o = max_y_o + y_diff;
        std::cout << "X_diff:" << x_diff << " Y_diff:" << y_diff << std::endl;
    }
    
    if(RobotInterface::use_wider_object_workspace)
    {
        min_y_o = min_y_o - 0.05;
        max_y_o = max_y_o + 0.05;
        max_x_o = max_x_o + 0.05;
    }
    std::cout << "Min,max x_0:" << min_x_o << "," << max_x_o 
            << " min,max y_o" << min_y_o << "," << max_y_o << std::endl;
}

geometry_msgs::PoseStamped GraspObject::getInitialObjectPose() {
    geometry_msgs::PoseStamped object_pose;
    object_pose.pose.position.x = initial_object_x ;
    object_pose.pose.position.y = initial_object_y ;
    object_pose.pose.position.z = initial_object_pose_z ;
    object_pose.pose.orientation.x = 0  ;
    object_pose.pose.orientation.y = 0;
    object_pose.pose.orientation.z = 0 ; 
    object_pose.pose.orientation.w = 1;
    return object_pose;
}

std::string GraspObject::getObjectPropertyDir(std::string object_name) {
    std::string object_property_dir;
    std::string pure_shape_prefix = "Cylinder";
    if(object_name.compare(0,pure_shape_prefix.size(), pure_shape_prefix) == 0)
    {
        object_property_dir = GraspObject::pure_object_property_dir;
    }
    else
    {
        object_property_dir = GraspObject::g3db_object_property_dir;
    }
    return object_property_dir;
}

void GraspObject::loadObject(bool load_in_vrep) {
    
    std::string function_name = "get_object_properties";
    if(load_in_vrep)
    {
        function_name = "add_object_in_scene";
    }
    std::string object_property_dir = GraspObject::getObjectPropertyDir(object_name);
    PyObject* object_properties = callPythonFunction(function_name, object_name, object_property_dir);
    
    std::vector<std::string> property_keys;
    property_keys.push_back("object_min_z");
    property_keys.push_back("object_initial_pose_z");
    property_keys.push_back("pick_point");
    
    PyObject* value1;
    for(int i = 0; i < property_keys.size();i++)
    {
        //std::cout << "Checking " << property_keys[i] << std::endl;
        PyObject* key = PyString_FromString(property_keys[i].c_str());
        if(PyDict_Contains(object_properties,key) == 1)
        {
            value1 = PyDict_GetItem(object_properties, key);

            if(i==0)
            {
                min_z_o = PyFloat_AsDouble(value1);
            }
            if(i==1)
            {
                initial_object_pose_z = PyFloat_AsDouble(value1);
            }
            if(i==2)
            {
                pick_point.push_back(PyFloat_AsDouble(PyList_GetItem(value1,0)));
                pick_point.push_back(PyFloat_AsDouble(PyList_GetItem(value1,1)));
                pick_point.push_back(PyFloat_AsDouble(PyList_GetItem(value1,2)));
            }
        }
        Py_DECREF(key);
    }
    Py_DECREF(object_properties);

}

void GraspObject::rotateZ(double rot_z) {
    //update_object_from_action_value(action, value):
    if(rot_z > -1)
    {
        PyObject* object_properties = callPythonFunction("update_object_from_action_value", "rotate_z", "", rot_z);
        Py_DECREF(object_properties);
    }
}

GraspObject::GraspObject(const GraspObject& orig) {
}

GraspObject::~GraspObject() {
}

void GraspObject::SetObject_name(std::string object_name) {
    this->object_name = object_name;
}

std::string GraspObject::GetObject_name() const {
    return object_name;
}

std::string GraspObject::getRegressionModelDir() {
    return data_dir_name + "/regression_models/" + object_name;
}

std::vector<std::string> GraspObject::getSASOFilenames(bool use_pruned_data, bool discretize_data) {
    std::vector<std::string> ans;
    if(!use_pruned_data && !discretize_data)
    {
        
        
        ans.push_back(data_dir_name + "/SASOData_Cylinder_" + getOldSasoFilename() + "cm_");
        std::cout << "Reading normal files" << std::endl;
                
    }
    else
    {
        std::string data_dir;
        if(discretize_data)
        {
            data_dir = regression_data_dir_name + "/" + object_name;
            std::cout << "Reading discretized files ";
        }
        else
        {
            data_dir = data_dir_name + "/pruned_data_files/" + object_name;
            std::cout << "Reading pruned files ";
        }
        
        PyObject* object_file_list = callPythonFunction("get_pruned_saso_files", object_name, data_dir);
        std::cout << PyList_Size(object_file_list) << std::endl;
        for(int i = 0; i < PyList_Size(object_file_list); i++)
        {
            ans.push_back(PyString_AsString(PyList_GetItem(object_file_list, i)));
        }
        Py_DECREF(object_file_list);
        
    }
    return ans;
}

std::string GraspObject::getOldSasoFilename() {
        
    std::string ans = "";
    std::string prefix  = "Cylinder_";
    std::string values[5] = {"9", "8", "7", "75", "85"};
    for(int i = 0; i < 5; i++)
    {
        if(object_name == prefix + values[i])
        {
            ans = values[i];
            return ans;
        }
        
    }
    //TODO add for coffee and yoghurt cup once their names are finalized
    
    
    return ans;
}

PyObject* GraspObject::callPythonFunction(std::string function_name, std::string arg1, std::string arg2, double arg3) {
    PyObject *pName;
    pName = PyString_FromString("load_objects_in_vrep");
    PyObject *pModule = PyImport_Import(pName);
    if(pModule == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"load objects in vrep\"\n");
        assert(0==1);
    }
    Py_DECREF(pName);

    
    PyObject *load_function = PyObject_GetAttrString(pModule, function_name.c_str());
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"add_object_in_scene\"\n");
    }

    PyObject *pArgs, *pValue;
    pArgs = PyTuple_New(2);
    pValue = PyString_FromString(arg1.c_str());
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 0, pValue);
    if(arg3 == -1)
    {
        pValue = PyString_FromString(arg2.c_str());
    }
    else
    {
        pValue = PyFloat_FromDouble(arg3);
    }
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 1, pValue);

    PyObject* object_properties = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(load_function);
    Py_DECREF(pModule);
    
    return object_properties;
}

std::pair<int, int> GraspObject::getDiscretizationIndex(double x1, double y1) {
    int x1_index = (int)(round(x1/discretization_step));
    int y1_index = (int)(round(y1/discretization_step));
    return std::make_pair(x1_index, y1_index);
}

std::tuple<int, int,int> GraspObject::getDiscretizationIndex(double x1, double y1,double theta_z_degree) {
    int x1_index = (int)(round(x1/discretization_step));
    int y1_index = (int)(round(y1/discretization_step));
    //std::cout << "theta_z_degree " << theta_z_degree << std::endl;
    while (theta_z_degree < -180)
    {
		theta_z_degree = theta_z_degree + 360;
    }
	while (theta_z_degree > 180)
	{
		theta_z_degree = theta_z_degree - 360;
	}
    int theta_z_index = (int)(round(theta_z_degree/10.0));
    if (theta_z_index == -18)
    {
       theta_z_index = 18;
    }
    return std::make_tuple(x1_index, y1_index,theta_z_index);

}

std::vector<SimulationData> GraspObject::getSimulationData(geometry_msgs::PoseStamped object_pose, geometry_msgs::PoseStamped gripper_pose, int action, bool use_next) {
    std::vector<SimulationData> tempDataVector;
    double x1 = object_pose.pose.position.x - gripper_pose.pose.position.x;
    double y1 = object_pose.pose.position.y - gripper_pose.pose.position.y;
    std::pair<int, int> d_index = getDiscretizationIndex(x1,y1);
    std::vector<int> simulationDataIndices;
    if(use_next)
    {
        simulationDataIndices = discretizedSimulationDataWithoutAngleNextState[action][d_index];
    }
    else
    {
        simulationDataIndices = discretizedSimulationDataWithoutAngleInitState[action][d_index];
    }
    for(int i = 0; i < simulationDataIndices.size(); i++)
    {
        tempDataVector.push_back(simulationDataCollectionWithObject[action][simulationDataIndices[i]]);
    }
    return tempDataVector;
}



void GraspObject::getSimulationData(geometry_msgs::PoseStamped object_pose, geometry_msgs::PoseStamped gripper_pose, double theta_z_degrees, std::vector<SimulationData> & tempDataVector, int action, bool use_next) {
    //std::vector<SimulationData> tempDataVector;
    double x1 = object_pose.pose.position.x - gripper_pose.pose.position.x;
    double y1 = object_pose.pose.position.y - gripper_pose.pose.position.y;
    std::tuple<int, int, int> d_index = getDiscretizationIndex(x1,y1, theta_z_degrees);
    //std::cout << "Query " << action << " " << use_next << " " << std::get<0>(d_index) << " " << std::get<1>(d_index) << " " << std::get<2>(d_index) << std::endl;
    std::vector<int> simulationDataIndices;
    if(use_next)
    {
    	simulationDataIndices = discretizedSimulationDataNextState[action][d_index];
    }
    else
    {
    	simulationDataIndices = discretizedSimulationDataInitState[action][d_index];
    }
    //std::cout << "Num entries retreived " << simulationDataIndices.size() << std::endl;
    for(int i = 0; i < simulationDataIndices.size(); i++)
    {
        tempDataVector.push_back(simulationDataCollectionWithObject[action][simulationDataIndices[i]]);
    }
    return ;
}







