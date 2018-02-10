#ifndef HC_NODE_H
#define HC_NODE_H

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

using namespace message_filters;
using namespace std_msgs;
using namespace std;

class HC_node
{
	public:
		// constructor
		HC_node();

		// callback1
		void chatterCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

		// callback2
		void chatterCallback2(const std_msgs::Float64::ConstPtr& msg);

		// send the command
		void send_cmd(double uT, double uR);
		
		//private:
		// vector containing the outputs
		vector<double> y;
		ros::Subscriber sub_camera;
		ros::Subscriber sub_imu;
		ros::Publisher u_sender;	

};

#endif

//int main(int argc, char **argv)
//{
//
//	ros::init(argc, argv, "listenerMultipleTopics");
//	HC_node myNode;
//	ros::Rate r(10);
//
//	while(ros::ok)
//	{
//		ros::spinOnce();
//		cout << myNode.y[0] << endl << myNode.y[1] << endl << myNode.y[2] << endl << myNode.y[3] << endl;
//		r.sleep();
//	}
//	return 0;
//}
