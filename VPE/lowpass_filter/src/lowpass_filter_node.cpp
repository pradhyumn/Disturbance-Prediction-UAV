#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

double T = 0.3; //sec
double dt = 0.1; //ms

double x_ = 0;
double y_ = 0;
double z_ = 0;
double epsilon = 0.0001;

ros::Publisher odom_pub_;
nav_msgs::Odometry odom;

double lowPassFilter(double x, double y0, double dt, double T)          // Taken from http://en.wikipedia.org/wiki/Low-pass_filter
{
   double res = y0 + (x - y0) * (dt/(dt+T));
   
   if ((res*res) <= epsilon)
	res = 0;
   return res;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    	double x = msg->pose.pose.position.x;
    	double y = msg->pose.pose.position.y;
	double z = msg->pose.pose.position.z;

	x_ = lowPassFilter(x, x_, dt, T);
	y_ = lowPassFilter(y, y_, dt, T);
	z_ = lowPassFilter(z, z_, dt, T);


	
	odom.pose.pose.position.x = x_;
	odom.pose.pose.position.y = y_;
	odom.pose.pose.position.z = z_;

	odom_pub_.publish(odom);

}

int main(int argc, char **argv)
{

    /// Receives Twist messages for the base.
	ros::Subscriber baseCommandSubscriber;
	/// Publishes Odometry messages
	ros::Publisher baseOdometryPublisher;

	ros::init(argc, argv, "lowpass_filter");
	ros::NodeHandle n;
    /* setup input/output communication */



	ros::Subscriber odomSub = n.subscribe("/aft_mapped_to_init", 1000, &odomCallback);
	odom_pub_ = n.advertise<nav_msgs::Odometry>("/odometry_bw", 100);

	/* coordination */
	ros::Rate rate(100); //Input and output at the same time... (in Hz)
	while (n.ok()){
		ros::spinOnce();

		rate.sleep();
	}

  return 0;
}
