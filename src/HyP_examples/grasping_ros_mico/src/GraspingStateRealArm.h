/* 
 * File:   GraspingStateRealArm.h
 * Author: neha
 *
 * Created on May 5, 2015, 11:18 AM
 */

#ifndef GRASPINGSTATEREALARM_H
#define	GRASPINGSTATEREALARM_H

#include <despot/interface/pomdp.h>
#include "geometry_msgs/PoseStamped.h"
#include "Quaternion.h"

class GraspingStateRealArm : public despot::State {
    public:
        geometry_msgs::PoseStamped gripper_pose;
        geometry_msgs::PoseStamped object_pose;
        double finger_joint_state[4]; //For finger configuration radians
        int object_id ;
        //double x_change = 0.0; //difference in x coordinate after taking an action
        //double y_change = 0.0; // change in y coordinate after taking an action
        //double z_change = 0.0; //change in z coordinate after taking an action
        int touch[2]; //Store if the touch is being observed at 2 fingers , useful for stop until touch actions
        int pre_action; //Store previous action, useful for putting restrictions on next action. eg. only open/pick after close
        int gripper_status; //1 for not stable or object not inside it 2 for stable grasp with object inside gripper //Based on reward achieved in last state and finger configuration. Useful to determine if the gripper did a stable grasp or unstable grasp and previous action was close
        double touch_value[2]; //Store the touch value observed for a given state. Useful to get observation for a No-OP
        bool closeCalled; //Can be used to evaluate gripper status
        int vision_movement; //0 for no movement //1 for movement
        std::vector<float> keras_input_vector;
        double theta_z_degree;
        GraspingStateRealArm() {
            object_id = -1;
            touch[0] = 0;
            touch[1] = 0;
            pre_action = -1;
            gripper_status = 0;
            touch_value[0] = 0.0;
            touch_value[1] = 0.0;
            closeCalled = false; //We always start with open gripper
            vision_movement = 0;
            keras_input_vector.resize(0);
            theta_z_degree = 36000.1;
        }
    
    GraspingStateRealArm(const GraspingStateRealArm& initial_state) : State()
    {
        gripper_pose.pose.position.x  = initial_state.gripper_pose.pose.position.x ;
        gripper_pose.pose.position.y  = initial_state.gripper_pose.pose.position.y ;
        gripper_pose.pose.position.z  = initial_state.gripper_pose.pose.position.z ;
        gripper_pose.pose.orientation.x = initial_state.gripper_pose.pose.orientation.x ;
        gripper_pose.pose.orientation.y = initial_state.gripper_pose.pose.orientation.y ;
        gripper_pose.pose.orientation.z = initial_state.gripper_pose.pose.orientation.z  ;
        gripper_pose.pose.orientation.w = initial_state.gripper_pose.pose.orientation.w ;
        object_pose.pose.position.x = initial_state.object_pose.pose.position.x ;
        object_pose.pose.position.y = initial_state.object_pose.pose.position.y ;
        object_pose.pose.position.z = initial_state.object_pose.pose.position.z ;
        object_pose.pose.orientation.x = initial_state.object_pose.pose.orientation.x  ;
        object_pose.pose.orientation.y = initial_state.object_pose.pose.orientation.y ;
        object_pose.pose.orientation.z = initial_state.object_pose.pose.orientation.z  ; 
        object_pose.pose.orientation.w = initial_state.object_pose.pose.orientation.w ;
        finger_joint_state[0] = initial_state.finger_joint_state[0]  ;
        finger_joint_state[1] = initial_state.finger_joint_state[1] ;
        finger_joint_state[2]= initial_state.finger_joint_state[2] ;
        finger_joint_state[3]= initial_state.finger_joint_state[3] ;
        object_id = initial_state.object_id;
        touch[0] = initial_state.touch[0];
        touch[1] = initial_state.touch[1];
        pre_action = initial_state.pre_action;
        gripper_status = initial_state.gripper_status;
        touch_value[0] = initial_state.touch_value[0];
        touch_value[1] = initial_state.touch_value[1];
        closeCalled = initial_state.closeCalled;
        vision_movement = initial_state.vision_movement;
    }

    ~GraspingStateRealArm() {
    }
    
    void getStateFromStream(std::istream& inputString) {
            char c;
    inputString >> gripper_pose.pose.position.x;
    inputString >> gripper_pose.pose.position.y;
    inputString >> gripper_pose.pose.position.z;
    inputString >> gripper_pose.pose.orientation.x;
    inputString >> gripper_pose.pose.orientation.y;
    inputString >> gripper_pose.pose.orientation.z;
    inputString >> gripper_pose.pose.orientation.w;
    inputString >> c;

    inputString >> object_pose.pose.position.x;
    inputString >> object_pose.pose.position.y;
    inputString >> object_pose.pose.position.z;
    inputString >> object_pose.pose.orientation.x;
    inputString >> object_pose.pose.orientation.y;
    inputString >> object_pose.pose.orientation.z;
    inputString >> object_pose.pose.orientation.w;
    inputString >> c;

    for (int i = 0; i < 4; i++) {
       inputString >> finger_joint_state[i];
    }

    }
    
    double get_theta_z_degree()
    {
    	if(theta_z_degree > 36000)
    	{
    	Quaternion q(object_pose.pose.orientation.x,
				object_pose.pose.orientation.y,
				object_pose.pose.orientation.z,
				object_pose.pose.orientation.w);
		double roll, pitch,yaw;
		Quaternion::toEulerAngle(q,roll, pitch,yaw);
		theta_z_degree =  yaw*180/3.14;
    	}
    	return theta_z_degree;
    }
    void get_keras_input(std::vector<float>& keras_input, int particle_index = 0)
    {
    	//Assuming keras_input size is already initialized
    	if(keras_input_vector.size() == 0)
    	{
    		keras_input_vector.push_back((float)gripper_pose.pose.position.x);
    		keras_input_vector.push_back((float)gripper_pose.pose.position.y);
    		keras_input_vector.push_back((float)object_pose.pose.position.x);
			keras_input_vector.push_back((float)object_pose.pose.position.y);
			Quaternion q(object_pose.pose.orientation.x,
			            object_pose.pose.orientation.y,
			            object_pose.pose.orientation.z,
			            object_pose.pose.orientation.w);
			    double roll, pitch,yaw;
			    Quaternion::toEulerAngle(q,roll, pitch,yaw);
			keras_input_vector.push_back((float)yaw);
			keras_input_vector.push_back((float)finger_joint_state[0]);
			keras_input_vector.push_back((float)finger_joint_state[2]);
			keras_input_vector.push_back((float)object_id);
			if(closeCalled)
			{
				keras_input_vector.push_back((float)1);
			}
			else
			{
				keras_input_vector.push_back((float)0);
			}
			keras_input_vector.push_back((float)touch_value[0]);
			keras_input_vector.push_back((float)touch_value[1]);
			keras_input_vector.push_back((float)vision_movement);
    	}
    	std::copy(keras_input_vector.begin(),keras_input_vector.end(),&keras_input[particle_index]);

    }
    void copy_keras_input(std::vector<float>& keras_input, int particle_index = 0) const
    {
    	std::copy(keras_input_vector.begin(),keras_input_vector.end(),&keras_input[particle_index]);
    }

    void getStateFromString(std::string state_string) {
            //  0.3379 0.1516 1.73337 -0.694327 -0.0171483 -0.719 -0.0255881|0.4586 0.0829 1.7066 -0.0327037 0.0315227 -0.712671 0.700027|-2.95639e-05 0.00142145 -1.19209e-
    //06 -0.00118446
    std::istringstream inputString(state_string);
    getStateFromStream(inputString);
    }
    
    //Used to get input for dynamic models
    std::vector<double> getStateAsVector()
    {
        std::vector<double> ans;
        ans.push_back(finger_joint_state[0]);
        ans.push_back(finger_joint_state[2]);
        ans.push_back(gripper_pose.pose.position.x);
        ans.push_back(gripper_pose.pose.position.y);
        ans.push_back(gripper_pose.pose.position.z);
        //ans.push_back(gripper_pose.pose.orientation.x);
        //ans.push_back(gripper_pose.pose.orientation.y);
        //ans.push_back(gripper_pose.pose.orientation.z);
        //ans.push_back(gripper_pose.pose.orientation.w);
        ans.push_back(object_pose.pose.position.x);
        ans.push_back(object_pose.pose.position.y);
        ans.push_back(object_pose.pose.position.z);
        //ans.push_back(object_pose.pose.orientation.x);
        //ans.push_back(object_pose.pose.orientation.y);
        //ans.push_back(object_pose.pose.orientation.z);
        //ans.push_back(object_pose.pose.orientation.w);
        ans.push_back(object_pose.pose.position.x - gripper_pose.pose.position.x);
        ans.push_back(object_pose.pose.position.y - gripper_pose.pose.position.y);
        return ans;
        
    }
    
    void getStateFromVector(std::vector<double> ans, bool rel=true)
    {
        if(rel)
        {
            gripper_pose.pose.position.x  = gripper_pose.pose.position.x + ans[0];
            gripper_pose.pose.position.y  = gripper_pose.pose.position.y + ans[1] ;
            gripper_pose.pose.position.z  = gripper_pose.pose.position.z + ans[2] ;
            gripper_pose.pose.orientation.x = gripper_pose.pose.orientation.x + ans[3] ;
            gripper_pose.pose.orientation.y = gripper_pose.pose.orientation.y + ans[4] ;
            gripper_pose.pose.orientation.z = gripper_pose.pose.orientation.z + ans[5]  ;
            gripper_pose.pose.orientation.w = gripper_pose.pose.orientation.w + ans[6] ;
            object_pose.pose.position.x = object_pose.pose.position.x + ans[7] ;
            object_pose.pose.position.y = object_pose.pose.position.y + ans[8] ;
            object_pose.pose.position.z = object_pose.pose.position.z + ans[9] ;
            object_pose.pose.orientation.x = object_pose.pose.orientation.x + ans[10]  ;
            object_pose.pose.orientation.y = object_pose.pose.orientation.y + ans[11] ;
            object_pose.pose.orientation.z = object_pose.pose.orientation.z + ans[12]  ; 
            object_pose.pose.orientation.w = object_pose.pose.orientation.w + ans[13] ;
            finger_joint_state[0] = finger_joint_state[0] + ans[14];
            finger_joint_state[2] = finger_joint_state[2] + ans[15] ;
            touch_value[0] = ans[16];
            touch_value[1] = ans[17];
        }
        else
        {
        gripper_pose.pose.position.x  = ans[0];
        gripper_pose.pose.position.y  = ans[1] ;
        gripper_pose.pose.position.z  = ans[2] ;
        gripper_pose.pose.orientation.x = ans[3] ;
        gripper_pose.pose.orientation.y = ans[4] ;
        gripper_pose.pose.orientation.z = ans[5]  ;
        gripper_pose.pose.orientation.w = ans[6] ;
        object_pose.pose.position.x = ans[7] ;
        object_pose.pose.position.y = ans[8] ;
        object_pose.pose.position.z = ans[9] ;
        object_pose.pose.orientation.x = ans[10]  ;
        object_pose.pose.orientation.y = ans[11] ;
        object_pose.pose.orientation.z = ans[12]  ; 
        object_pose.pose.orientation.w = ans[13] ;
        finger_joint_state[0] = ans[14];
        finger_joint_state[2]= ans[15] ;
        touch_value[0] = ans[16];
        touch_value[1] = ans[17];
        }
        
    }
    
    bool touchEqual(GraspingStateRealArm gs)
    {
        bool ans = true;
        for(int i = 0; i < 2; i++)
        {
            if(touch[i]!=gs.touch[i])
            {
                ans = false;
            }
            if(!approx_equal(touch_value[i], gs.touch_value[i], 0.5))
            {
                ans = false;
            }
        }
        return ans;
    }
    
    bool jointEqual(GraspingStateRealArm gs)
    {
        bool ans = true;
        if(gripper_status != gs.gripper_status)
            {
                std::cout << "gripper status do not match" 
                            << gripper_status << "," << gs.gripper_status << std::endl;
                ans = false;
            }
        for(int i = 0; i < 4; i=i+2)
        {

            
            if(!approx_equal(finger_joint_state[i], gs.finger_joint_state[i], 2*3.14/180.0))
            {
                std::cout << "joint " << i << std::endl;
                ans = false;
            }
        }
        return ans;
    }
    
    bool gripperPoseEqual(GraspingStateRealArm gs)
    {
        return poseEqual(gripper_pose, gs.gripper_pose);
    }
    
    bool objectPoseEqual(GraspingStateRealArm gs)
    {
        return poseEqual(object_pose, gs.object_pose);
    }
    
    bool poseEqual(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
    {
        bool ans = true;
        if(!approx_equal(pose1.pose.position.x, pose2.pose.position.x, 0.005))
        {
            ans = false;
        }
        if(!approx_equal(pose1.pose.position.y, pose2.pose.position.y, 0.005))
        {
            ans = false;
        }
        if(!approx_equal(pose1.pose.position.z, pose2.pose.position.z, 0.005))
        {
            ans = false;
        }
        double gripper_quaternion_distance = 1 - pow(((pose1.pose.orientation.x*pose2.pose.orientation.x)+
                                                  (pose1.pose.orientation.y*pose2.pose.orientation.y)+
                                                  (pose1.pose.orientation.z*pose2.pose.orientation.z)+
                                                  (pose1.pose.orientation.w*pose2.pose.orientation.w)), 2);
        if(gripper_quaternion_distance > 0.01)
        {
            ans = false;
        }
        
        return ans;
    }
    
    bool stateEqual(GraspingStateRealArm gs)
    {
        bool a1 = touchEqual(gs);
        if(!a1)
        {
            std::cout << "Touch not equal" << std::endl;
        }
        bool a2 = jointEqual(gs);
        if(!a2)
        {
            std::cout << "Joint not equal" << std::endl;
        }
        bool a3 = objectPoseEqual(gs);
        if(!a3)
        {
            std::cout << "Object pose equal" << std::endl;
        }
        bool a4 = gripperPoseEqual(gs);
        if(!a4)
        {
            std::cout << "Gripper pose equal" << std::endl;
        }
        return a1 && a2 && a3 && a4;
    }
    static bool approx_equal(double x,double y,double e)
    {
        return ((x-y < e) && (x-y > -e));
    }
    
    
};

#endif	/* GRASPINGSTATEREALARM_H */

