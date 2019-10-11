#ifndef CUP_DISPLAY_H
#define CUP_DISPLAY_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hyp_despot/Belief.h>
#include <hyp_despot/State.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "parameters.h"

using namespace std;

class CUPDISPLAY
{
public:
	CUPDISPLAY(ros::NodeHandle& nh, int state_index = 0);
        CUPDISPLAY(ros::NodeHandle& nh, double g_x, double g_y, double o_x, double o_y);
	~CUPDISPLAY();

	void ObjectPoseCallback(const hyp_despot::Belief::ConstPtr& msg);
	void GripperPoseCallback(const hyp_despot::State::ConstPtr& msg);

	void Init(int state_index = 0);
        void Init(double g_x, double g_y, double o_x, double o_y);
	void DrawRviz();
private:
	ros::Subscriber sub_object;
	ros::Subscriber sub_gripper;
	ros::Publisher pub_belief;

	std::vector< std::pair<double, double> > object_pose_bel;
        std::vector< double> object_pose_bel_weight;
        std::vector< int > object_pose_id;
	std::pair<double, double> gripper_pose;
        std::pair<double, double> object_pose;
        std::string header_frame_id = "/vrep_world";


    int obs;
};

#endif
