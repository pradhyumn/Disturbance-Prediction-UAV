
#include<ros/ros.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>

geometry_msgs::PoseStamped ekf2_pose;

void callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	ekf2_pose.pose.position = msg->pose.pose.position;
	ekf2_pose.pose.orientation = msg->pose.pose.orientation;
	ROS_INFO("Pose for ekf2");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_pass");
	ros::NodeHandle nh;

	ekf2_pose.header.frame_id = "map";
	
	ros::Subscriber pose_sub = nh.subscribe("/odometry/imu", 100, callback);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);


	ros::Rate rate(30);
	while(ros::ok())
	{
		ekf2_pose.header.stamp = ros::Time::now();
		pose_pub.publish(ekf2_pose);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
