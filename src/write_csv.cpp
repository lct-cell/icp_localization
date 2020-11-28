#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <fstream>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

int data_cnt = 201;
int id = 1;

const int SIZE = 400;
std_msgs::Float64MultiArray x;
double y[SIZE];
double z[SIZE];
double yaw[SIZE];
double pitch[SIZE];
double roll[SIZE];

void callback(const nav_msgs::Odometry::ConstPtr msg)
{
	//x[id] = msg->pose.pose.position.x;
	x.data.push_back(msg->pose.pose.position.x);
	y[id] = msg->pose.pose.position.y;
	z[id] = msg->pose.pose.position.z;
	
	tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double r_, p_, y_;
    m.getRPY(r_, p_, y_);

	yaw[id] = y_;
	pitch[id] = p_;
	roll[id] = r_;

	std::cout << id << std::endl;
	if(id == data_cnt)
	{
		std::ofstream myfile;
		myfile.open("submit_1.csv");
		myfile << "id,x,y,z,yaw,pitch,roll\n";
		for(int i=1; i<=id; i++)
		{
			myfile << i << "," << x.data[i-1] << "," << y[i] << "," << z[i] << "," << yaw[i] << "," << pitch[i] << "," << roll[i] << "\n";
		}
		myfile.close();
		std::cout << "stored data in csv\n";
	}
std::cout << msg->pose.pose.position.x << std::endl << x.data[id-1] << std::endl;
	id += 1;
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "write_csv_node");
	ros::NodeHandle nh;
	ros::Subscriber sub =  nh.subscribe("result_localization", 1000, callback);
	ros::spin();

	return 0;
}
