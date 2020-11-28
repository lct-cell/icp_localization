#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>//
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/transforms.h>
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

/*
geometry_msgs::PoseStamped gps;

void gpsCallback(geometry_msgs::PoseStamped poseStamped)
{
	std::cout << "in callback\n";
	gps = poseStamped;
	std::cerr << "GPS from topic: " << gps << "\n"; 
}*/
/*
std::ofstream myfile;
myfile.open("submit_1.csv");
myfile << "id,x,y,z,yaw,pitch,roll\n";
*/

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PCLPointCloud2::Ptr lidar_points_PCL(new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr lidar_cloud_filtered(new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr lidar_Final_PCLpc2(new pcl::PCLPointCloud2());
pcl::VoxelGrid<pcl::PCLPointCloud2> sor_;

tf::StampedTransform transform_base_velodyne;

Eigen::Matrix4d initial_guess;
int init = 1;

ros::Publisher result_lidar_pub;
ros::Publisher result_localization_pub;

int id = 1;
/*
const int SIZE = 400;
double x[SIZE];
double y[SIZE];
double z[SIZE];
double yaw[SIZE];
double pitch[SIZE];
double roll[SIZE];
*/

void icpCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_points)
{
	if(init)
	{
		
	    initial_guess << -0.7689708, -0.6391353,  0.0137796, -284.8985597,
				  	 	 0.6382825, -0.7687940, -0.0393861, 226.1278839,
				   		 0.0357667, -0.0214914,  0.9991291, -12.5875616,
						 0,			0,			 0,			1;		 //best
		
		init = 0;
	}
	pcl_conversions::toPCL(*lidar_points, *lidar_points_PCL);

	std::cerr << "Lidar pointCloud before filtering: " << lidar_points_PCL->width * lidar_points_PCL->height
		<< " data points " << std::endl;
	//downsample
	sor_.setInputCloud(lidar_points_PCL);
	sor_.setLeafSize(0.5f, 0.5f, 0.5f);//0.5*0.5*0.5
	sor_.filter(*lidar_cloud_filtered);

	std::cerr << "Lidar pointCloud after filtering: " << lidar_cloud_filtered->width * lidar_cloud_filtered->height
		<< " data points " << std::endl;

	//transform from "velodyne" frame to "base_link" frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points_point_cloud_tf(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*lidar_cloud_filtered, *lidar_points_point_cloud);
	pcl_ros::transformPointCloud(*lidar_points_point_cloud, *lidar_points_point_cloud_tf, transform_base_velodyne);

	//icp
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(lidar_points_point_cloud_tf);
	icp.setInputTarget(cloud_map);

	icp.setMaxCorrespondenceDistance (1);//1
	icp.setMaximumIterations (500); //500
	icp.setTransformationEpsilon (1e-10);//1e-10
	icp.setEuclideanFitnessEpsilon (1e-8);//1e-8

	pcl::PointCloud<pcl::PointXYZ> Final;
	//ROS_WARN_STREAM_NAMED("test", "initial_guess is " << std::endl
	//		<< initial_guess);
	icp.align(Final, initial_guess.cast<float>());

	ROS_INFO_STREAM_NAMED("RESULT", "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl);
	ROS_INFO_STREAM_NAMED("RESULT", "\n" << icp.getFinalTransformation() << std::endl);
	initial_guess = icp.getFinalTransformation().cast<double>();

	//pcl::pointcloud => pcl::pointcloud2 => sensor_msgs::pointcloud2
	sensor_msgs::PointCloud2 lidar_scan_final_pc2;
	pcl::toPCLPointCloud2(Final, *lidar_Final_PCLpc2);
	pcl_conversions::fromPCL(*lidar_Final_PCLpc2, lidar_scan_final_pc2);

	lidar_scan_final_pc2.header.frame_id = "world";
	result_lidar_pub.publish(lidar_scan_final_pc2);

	nav_msgs::Odometry odom;
	odom.header.frame_id = "world";
	odom.child_frame_id = "base_link";
	//transform matrix to translation and quaternion
	//initial_guess is our final transform matrix now
	Eigen::Quaterniond eigen_quat(initial_guess.block<3,3>(0,0).cast<double>());
	Eigen::Vector3d eigen_trans(initial_guess.block<3,1>(0,3).cast<double>());
	odom.pose.pose.position.x = eigen_trans(0);
	odom.pose.pose.position.y = eigen_trans(1);
	odom.pose.pose.position.z = eigen_trans(2);
	odom.pose.pose.orientation.x = eigen_quat.x();
	odom.pose.pose.orientation.y = eigen_quat.y();
	odom.pose.pose.orientation.z = eigen_quat.z();
	odom.pose.pose.orientation.w = eigen_quat.w();
	result_localization_pub.publish(odom);

/*	
	Eigen::Matrix3d m_rot = initial_guess.block<3,3>(0,0).cast<double>();
	Eigen::Vector3d ea = m_rot.eulerAngles(2, 1, 0);
	
	x[id] = initial_guess(0,3);
	y[id] = initial_guess(1,3);
	z[id] = initial_guess(2,3);

	yaw[id] = ea(0);
	pitch[id] = ea(1);
	roll[id] = ea(2);
*/
	//store data in csv
	/*
	if(id == 201) //201
	{
		std::ofstream myfile;
		myfile.open("test_1.csv");
		myfile << "id,x,y,z,yaw,pitch,roll\n";
		for(int i=1; i<=id; i++)
		{
			myfile << i << "," << std::setprecision(15) << x[i] << "," << y[i] << "," << z[i] << "," << yaw[i] << "," << pitch[i] << "," << roll[i] << "\n";
		}
		myfile.close();
		std::cout << "stored data in csv\n";
	}
	*/
	//std::cout << std::setprecision(15) << x[id] << " " << y[id] << std::endl;	
	std::cout<< id << std::endl; 
	id +=1;
}


int main (int argc, char** argv)
{
	//set tranformation between base_link and velodyne
	transform_base_velodyne.setOrigin( tf::Vector3(0.46, 0.0, 3.46) );
	transform_base_velodyne.setRotation( tf::Quaternion(-0.0051505, 0.018102, -0.019207, 0.99964) );
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// get map pointcloud from pcd file and filter the pointcloud
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("/home/lct/nctu_sdc/localization_ws/src/icp_localization/map/itri_map.pcd", *cloud);

	std::cerr << "Map pointCloud before filtering: " << cloud->width * cloud->height
		<< " data points " << std::endl;

	// Create the filtering object
	// change leaf size if need
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.5f, 0.5f, 0.5f);//0.5*0.5*0.5
	sor.filter(*cloud_filtered);

	std::cerr << "Map pointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points " << std::endl;

	pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_map);//from PCLpointcloud2 to pointcloud
	std::cout << "Saved " << cloud_map->size () << " data points to cloud_map:" << std::endl;
	//---
	//for (auto& point : *cloud_in)
	//	std::cout << point << std::endl;
	//*cloud_out = *cloud_in;

	//init gps in world frame (change to subscriber later)
	//double gps_x, gps_y, gps_z;
	//gps_x = -263.89384941;
	//gps_y = -67.7809286182;
	//gps_z = -8.08355457512;


/*
	std::ofstream myfile;
	myfile.open("submit_1.csv");
	myfile << "id,x,y,z,yaw,pitch,roll\n";
	myfile.close();
*/
	ros::init (argc, argv, "icp_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("lidar_points", 1000, icpCallback);
	result_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("result_lidar", 1000);
	result_localization_pub = nh.advertise<nav_msgs::Odometry>("result_localization", 1000);
	ros::spin();


	return 0;
}
