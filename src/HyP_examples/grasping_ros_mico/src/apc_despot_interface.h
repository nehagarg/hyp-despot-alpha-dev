#ifndef APC_DESPOT_INTERFACE_H_
#define APC_DESPOT_INTERFACE_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <time.h>

//#include "apc_msgs/Touch.h"
#include "belief.h"
#include "despot.h"
#include "grasping_real_arm.h"
#include "checkmotionfinished/Ismotionfinished.h"

using namespace std;

class APCInterface
{
public:
	APCInterface();
	~APCInterface();

	void GetGripperPoseCallback(const geometry_msgs::PoseStamped::ConstPtr pose);
	void GetTouchCallback(const std_msgs::Float64MultiArray::ConstPtr touch);
	void GetFingerJointCallback(const sensor_msgs::JointState::ConstPtr joint);
	void initSimulator();
	void initBelief();
	void publishAction(int);
	void copy_pose_stamped(GraspingObservation&, geometry_msgs::PoseStamped);
	ros::Publisher action_pub;
	ros::Subscriber obj_pose_sub,touch_sub,joint_sub;
	ros::ServiceClient action_executed;
	checkmotionfinished::Ismotionfinished action_executed_srv;


	//tf::TransformListener tf_;

	GraspingRealArm* despot;
	DESPOT* solver;
	GraspingObservation obs;

	int start_state_index_;
	bool first_obs;
	//observations
	geometry_msgs::PoseStamped gripper_pose;
	double finger_joint_state[4];
	double touch_sensor_reading[48];


private:
	double control_freq;
	int action;
	

	ros::Timer timer_;
	void controlLoop(const ros::TimerEvent &e);
};

#endif /*APC_DESPOT_INTERFACE_H_*/