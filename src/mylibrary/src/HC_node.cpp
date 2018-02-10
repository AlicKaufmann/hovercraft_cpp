#include <HC_node.hpp>

// constructor
HC_node::HC_node()
	: y(4)
{
	ros::NodeHandle n;

	// subscribe to get the output y=[x,y,theta,omega] of the hc
	sub_camera = n.subscribe("chatter", 1, &HC_node::chatterCallback, this);
	sub_imu = n.subscribe("chatter2", 1, &HC_node::chatterCallback2, this);

	// create a publisher to send the commands u=[uT,uR] to the hc
	u_sender = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);			
}


// callback1
void HC_node::chatterCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//ROS_INFO(msg->x);
	y[0] = msg->x;
	y[1] = msg->y;
	y[2] = msg->theta;
}
// callback2
void HC_node::chatterCallback2(const std_msgs::Float64::ConstPtr& msg)
{
	//ROS_INFO("I heard from the second chatter: ", msg->data);
	y[3] = msg->data;
}
// send the command
void HC_node::send_cmd(double uT, double uR)
{
	geometry_msgs::Twist msg;

	msg.linear.z = 0;
	msg.angular.z = 0;

	if(uT>=0)
	{
		msg.linear.x = uT;
		msg.linear.y = 0;
		msg.angular.x= 0;
		msg.angular.y= uT;
	}
	else
	{
		msg.linear.x = 0;
		msg.linear.y = -uT;
		msg.angular.x = -uT;
		msg.angular.y = 0;
	}
	if(uR>=0)
	{
		msg.linear.y += uR;
		msg.angular.y += uR;
	}
	else
	{
		msg.linear.x -= uR;
		msg.angular.x -= uR;
	}
	u_sender.publish(msg);
}


