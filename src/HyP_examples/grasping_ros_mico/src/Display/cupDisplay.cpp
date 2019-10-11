#include "cupDisplay.h"

CUPDISPLAY::CUPDISPLAY(ros::NodeHandle& nh, int state_index)
{
	sub_object = nh.subscribe("object_pose", 100, &CUPDISPLAY::ObjectPoseCallback, this);
	sub_gripper = nh.subscribe("gripper_pose", 100, &CUPDISPLAY::GripperPoseCallback, this);

	pub_belief = nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	Init(state_index);
}

CUPDISPLAY::CUPDISPLAY(ros::NodeHandle& nh, double g_x, double g_y, double o_x, double o_y)
{
	sub_object = nh.subscribe("object_pose", 10, &CUPDISPLAY::ObjectPoseCallback, this);
	sub_gripper = nh.subscribe("gripper_pose", 10, &CUPDISPLAY::GripperPoseCallback, this);

	pub_belief = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	Init( g_x, g_y, o_x, o_y);
}


CUPDISPLAY::~CUPDISPLAY()
{

}


void CUPDISPLAY::Init(double g_x, double g_y, double o_x, double o_y)
{
	//std::pair<double, double> pos(o_x, o_y);
        obs = 0;
	//object_pose_bel.push_back(pos);
        //object_pose_bel_weight.push_back(1.0)
	gripper_pose.first = g_x;
	gripper_pose.second = g_y;
        object_pose.first  = o_x;
        object_pose.second = o_y;
}
void CUPDISPLAY::Init(int state_index)
{

        //Initialize rviz display
        int i = state_index /10;
        int j = state_index % 10;
        //double o_y = MIN_Y_O + (j*(MAX_Y_O - MIN_Y_O)/9.0);
        //double o_x = MIN_X_O + (i*(MAX_X_O - MIN_X_O)/9.0);
        double o_y =0.1485;
        double o_x = 0.5019;
        double g_x = MIN_X;
        double g_y = MIN_Y + 0.07;
        Init(g_x, g_y, o_x, o_y);
	//std::pair<double, double> pos(0.5646, -0.14389);
        //obs = 0;
	//object_pose_bel.push_back(pos);
	//gripper_pose.first = 0.1797;
	//gripper_pose.second = -0.1887;


}

void CUPDISPLAY::ObjectPoseCallback(const hyp_despot::Belief::ConstPtr& msg)
{
	object_pose_bel.clear();

	for(int i = 0; i < msg->numPars * 4; i += 4)
	{
		object_pose_bel.push_back(std::pair<double, double> (msg->belief[i], msg->belief[i + 1]));
                object_pose_bel_weight.push_back(msg->belief[i+2]);
                object_pose_id.push_back(msg->belief[i+3]);
	}
}

void CUPDISPLAY::GripperPoseCallback(const hyp_despot::State::ConstPtr& msg)
{
	gripper_pose.first = msg->gripper_pose.pose.position.x;
	gripper_pose.second = msg->gripper_pose.pose.position.y;
        object_pose.first = msg->object_pose.pose.position.x;
	object_pose.second = msg->object_pose.pose.position.y;
    obs = msg->observation;
}

void CUPDISPLAY::DrawRviz()
{
	ROS_INFO("current belief!");
	// ros::Duration one_second(1.0);
	int id_ = 0;
	ros::Rate loop_rate(10);
        double y_offset = -0.30;
	while(ros::ok())
	{

		// cout << "current belif size:  " << object_pose_bel.size();
		// publish the object pose
		// int id_= 0;
		visualization_msgs::Marker marker;
		marker.header.frame_id = header_frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "grasp_display";

		int id0 = NUM_PARTICLE_DISPLAY;

		// for(int i = 0; i <= id_; i++)
		// {
		// 	marker.id = id_;
		// 	marker.action = visualization_msgs::Marker::DELETE;
		// 	pub_belief.publish(marker);
		// }
		id_= 0;

		// marker.action = visualization_msgs::Marker::deleteall;
		// pub_belief.publish(marker);

		marker.action = visualization_msgs::Marker::ADD;
		for(int i = 0; i < object_pose_bel.size(); i++)
		{
			marker.id = id_;
			id_++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.pose.position.x = object_pose_bel[i].first;
			marker.pose.position.y = object_pose_bel[i].second + y_offset;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = 0.02;
			marker.scale.y = 0.02;
			marker.scale.z = 0.02;
			// Set the color -- be sure to set alpha to something non-zero!
                        if(object_pose_id[i] == 0)
                        {
                            marker.color.r = 1.0f;
                        }
                        else
                        {
                            marker.color.r = 0.0f;
                        }
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 0.2 + 0.8*(object_pose_bel_weight[i] );
			marker.lifetime = ros::Duration();
			// marker.lifetime = one_second;
			pub_belief.publish(marker);
		}


		/*if(object_pose_bel.size() < id0)
		{
			// cout << "insufficient particles: " << id0 - object_pose_bel.size() << endl;
			for(int i = object_pose_bel.size(); i < id0; i++)
			{
				marker.id = id_;
				id_++;
				marker.type = visualization_msgs::Marker::SPHERE;
				marker.pose.position.x = 0;
				marker.pose.position.y = 0;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.color.r = 1.0f;
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
				marker.color.a = 0;
				marker.lifetime = ros::Duration();
				pub_belief.publish(marker);
			}
		}
                 */

        //publish the object position
                        marker.id = id_;
			id_++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.pose.position.x = object_pose.first;
			marker.pose.position.y = object_pose.second + y_offset;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = 0.02;
			marker.scale.y = 0.02;
			marker.scale.z = 0.02;
			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 1.0f;
			marker.color.a = 1.0;
			marker.lifetime = ros::Duration();
			// marker.lifetime = one_second;
			pub_belief.publish(marker);


		// publish the gripper position
		std::vector<std::pair<double, double> > corner_points;
        corner_points.clear();
        if(obs >= OBS_STABLE)
        {
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP + LEN_GRIPPER, y_offset+ gripper_pose.second + LEN_GRIPPER_BOTTOM / 2));
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP, y_offset+ gripper_pose.second + LEN_GRIPPER_BOTTOM / 2));
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP, y_offset+ gripper_pose.second - LEN_GRIPPER_BOTTOM / 2));
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP + LEN_GRIPPER, y_offset+gripper_pose.second - LEN_GRIPPER_BOTTOM / 2));
        }
        else
        {
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP + HIGH_GRIPPER, y_offset+gripper_pose.second + LEN_GRIPPER_TOP / 2));
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP, y_offset+gripper_pose.second + LEN_GRIPPER_BOTTOM / 2));
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP, y_offset+gripper_pose.second - LEN_GRIPPER_BOTTOM / 2));
            corner_points.push_back(std::pair<double, double>(gripper_pose.first - OFFSET_MICO_TIP + HIGH_GRIPPER, y_offset+gripper_pose.second - LEN_GRIPPER_TOP / 2));
        }

		marker.action = visualization_msgs::Marker::ADD;
		marker.id = NUM_PARTICLE_DISPLAY + 1;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.points.clear();
		for(int i = 0; i < corner_points.size(); i++)
		{
			geometry_msgs::Point point;
			point.x = corner_points[i].first;
			point.y = corner_points[i].second;
			point.z = 0;
			marker.points.push_back(point);
			// cout << "point: x-" << point.x << ", y-" << point.y << ", z-" << point.z << endl;
		}
		marker.pose.position.x = 0;
		marker.pose.position.y =  0;
		marker.scale.x = 0.01;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		// marker.lifetime = one_second;
		pub_belief.publish(marker);

        // display the observation get
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 2;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "No touch and Gripper Open";
		marker.pose.position.x = 0.2;
		marker.pose.position.y = 0.08;
		marker.scale.x = 0.1;
		marker.scale.y = 0.04;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		pub_belief.publish(marker);

        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 3;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "Right Finger in touch";
		marker.pose.position.x = 0.2;
		marker.pose.position.y = 0.06;
		marker.scale.x = 0.1;
		marker.scale.y = 0.04;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		pub_belief.publish(marker);

        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 4;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "Left Finger in touch";
		marker.pose.position.x = 0.2;
		marker.pose.position.y = 0.04;
		marker.scale.x = 0.05;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		pub_belief.publish(marker);

        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 5;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "Both Finger in touch";
		marker.pose.position.x = 0.2;
		marker.pose.position.y = 0.02;
		marker.scale.x = 0.05;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		pub_belief.publish(marker);

        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 6;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "Gripper closed and stable";
		marker.pose.position.x = 0.2;
		marker.pose.position.y = 0;
		marker.scale.x = 0.05;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		pub_belief.publish(marker);

        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 7;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = "Gripper closed but not stable";
		marker.pose.position.x = 0.2;
		marker.pose.position.y = -0.02;
		marker.scale.x = 0.05;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		pub_belief.publish(marker);

        marker.action = visualization_msgs::Marker::ADD;
        marker.id = NUM_PARTICLE_DISPLAY + 8;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = 0.12;
        marker.pose.position.y = 0.08 - 0.02 * obs;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        // marker.lifetime = one_second;
        pub_belief.publish(marker);

		ros::spinOnce();
		loop_rate.sleep();
	}
}
