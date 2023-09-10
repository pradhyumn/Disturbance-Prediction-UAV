#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imu_pub_;
sensor_msgs::Imu imu;

double x_[3];
double y_[3];
double a_[2];
double gain;
bool initalized;


double reset(double sample){
    x_[0] = sample;
    x_[1] = sample;
    x_[2] = sample;
    y_[0] = sample;
    y_[1] = sample;
    y_[2] = sample;
    return sample;
}

double apply(double sample) {

    if(!initalized){
      initalized = true;
      return reset(sample);
    }
    x_[0] = x_[1];
    x_[1] = x_[2];
    x_[2] = sample / gain;
    y_[0] = y_[1];
    y_[1] = y_[2];
    y_[2] = (x_[0] + x_[2]) + 2 * x_[1] + (a_[0] * y_[0]) + (a_[1] * y_[1]);
    return y_[2];

}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    
    imu = *msg;
    imu.linear_acceleration.x = apply(msg->linear_acceleration.x);
    imu_pub_.publish(imu);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "filter_node");

    ros::NodeHandle nh;
    gain = 1.024264069e+01;
    a_[0] = -0.3333333333;
    a_[1] = 0.9428090416;

    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, &imuCallback);

    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/filter", 100);

    ros::spin();
    return 0;
}