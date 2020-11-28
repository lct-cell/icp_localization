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
#include "tf_conversions/tf_eigen.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PCLPointCloud2::Ptr lidar_points_PCL(new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr lidar_cloud_filtered(new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr lidar_Final_PCLpc2(new pcl::PCLPointCloud2());
pcl::VoxelGrid<pcl::PCLPointCloud2> sor_;

tf::StampedTransform transform_base_velodyne;

Eigen::Matrix4d initial_guess;

Eigen::Matrix3d odom_rot;
Eigen::Matrix4d odom_transform = Eigen::Matrix4d::Identity();
Eigen::Matrix4d odom_transform_pre = Eigen::Matrix4d::Zero();
Eigen::Matrix4d odom_diff = Eigen::Matrix4d::Zero();
Eigen::Matrix4d odom_origin = Eigen::Matrix4d::Zero();
int init = 1;

ros::Publisher result_lidar_pub;
ros::Publisher result_localization_pub;

int id = 1;

int odom_init = 1;
void odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
	if(init)
		return;
	tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
	tf::matrixTFToEigen(m, odom_rot);
	odom_transform.block(0,0,3,3) = odom_rot;
	odom_transform(0,3) = msg->pose.pose.position.x;
	odom_transform(1,3) = msg->pose.pose.position.y;
	odom_transform(2,3) = msg->pose.pose.position.z;

	odom_transform.block(0,0,3,3) = Eigen::Matrix3d::Zero();/////
	initial_guess(0,3) = 0;
	//initial_guess(1,3) = 0;
	initial_guess(2,3) = 0;
	odom_origin(1,3)=0;
	odom_transform(1,3)=0;
	initial_guess = odom_origin + odom_transform + initial_guess;
	initial_guess(3,3) = 1; //ori->from icp, translation->from odom
/*	
	odom_diff = odom_transform - odom_transform_pre;
	odom_diff.block(0,0,3,3) = Eigen::Matrix3d::Zero();
	initial_guess = initial_guess + odom_diff;
	odom_transform_pre = odom_transform;
*/
//	std::cout << "diff\n"<< odom_diff<<std::endl;
	/*
	std::cout << std::endl <<
        msg->pose.pose.position.x <<std::endl <<
        msg->pose.pose.position.y <<std::endl <<
        msg->pose.pose.position.z <<std::endl;
	std::cout << std::endl << odom_rot << std::endl;
	std::cout << std::endl << odom_transform <<std::endl;	
	*/
}

void icpCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_points)
{
	if(init)
	{
		/*
		initial_guess << -0.770228, -0.637455, 0.0125646, -284.889,
					  0.637473, -0.76969, 0.0307848, 226.113,
					  -0.00995363, 0.0317199, 0.999464, -12.6214,
					  0.0, 0.0, 0.0, 1.0;//from /gps first message best
		*/
		/*
		initial_guess << 1, 0,  0, 1774.8362265,
					  	 0, 1, 0, 866.367052112,
					  	 0, 0,  1, 0.0153020292018,
					  0.0, 0.0, 0.0, 1.0;//from /gps first message
		*/

		initial_guess << 0.999706,  -0.0242715, -0.00184481,     1774.22,
  						 0.0243079,    0.999435,    0.023271,     866.285,
 						 0.00127891,   -0.023309,     0.99973,    -0.29595,
          				 0,           0,           0,           1;

		//init = 0; ///////////////////////////////!!!!!!!!!
	}
	pcl_conversions::toPCL(*lidar_points, *lidar_points_PCL);

	std::cerr << "Lidar pointCloud before filtering: " << lidar_points_PCL->width * lidar_points_PCL->height
		<< " data points " << std::endl;
	//downsample
	sor_.setInputCloud(lidar_points_PCL);
	sor_.setLeafSize(0.8f, 0.8f, 0.8f);//70cm*70cm*70cm
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

	icp.setMaxCorrespondenceDistance (1);//3
	icp.setMaximumIterations (200); //20
	icp.setTransformationEpsilon (1e-10);//converge condition
	icp.setEuclideanFitnessEpsilon (1e-8);//converge condition

	pcl::PointCloud<pcl::PointXYZ> Final;
	//ROS_WARN_STREAM_NAMED("test", "initial_guess is " << std::endl
	//		<< initial_guess);
	icp.align(Final, initial_guess.cast<float>());

	ROS_INFO_STREAM_NAMED("RESULT", "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl);
	ROS_INFO_STREAM_NAMED("RESULT", "\n" << icp.getFinalTransformation() << std::endl);
	initial_guess = icp.getFinalTransformation().cast<double>();

	//odom_transform_pre = initial_guess;//////////
	if(init)
	{
		odom_origin = initial_guess;//////////
		odom_origin.block(0,0,3,3) = Eigen::Matrix3d::Zero(); // only keep translation
		init = 0;
	}

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
	//std::cout << ea << std::endl;

	x[id] = initial_guess(0,3);
	y[id] = initial_guess(1,3);
	z[id] = initial_guess(2,3);

	yaw[id] = ea(0);
	pitch[id] = ea(1);
	roll[id] = ea(2);

	//store data in csv
	if(id == 201) //201
	{
		std::ofstream myfile;
		myfile.open("submit_1.csv");
		myfile << "id,x,y,z,yaw,pitch,roll\n";
		for(int i=1; i<=id; i++)
		{
			myfile << i << "," << x[i] << "," << y[i] << "," << z[i] << "," << yaw[i] << "," << pitch[i] << "," << roll[i] << "\n";
		}
		myfile.close();
		std::cout << "stored data in csv\n";
	}
*/
	std::cout<< id << std::endl;
	id +=1;
}


int main (int argc, char** argv)
{
	//set tranformation between base_link and velodyne
	transform_base_velodyne.setOrigin( tf::Vector3(0.985792994499, 0.0, 1.84019005299) );
	transform_base_velodyne.setRotation( tf::Quaternion(-0.0153009936601, 0.0173974519781, -0.707084648946, 0.706749253613) );
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_n(new pcl::PCLPointCloud2());//
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	// get map pointcloud from pcd file and filter the pointcloud
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1700_800.pcd", *cloud);

	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1800_800.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1900_800.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1700_900.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1800_900.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1900_900.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);



	std::cerr << "Map pointCloud before filtering: " << cloud->width * cloud->height
		<< " data points " << std::endl;

	// Create the filtering object
	// change leaf size if need
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.8f, 0.8f, 0.8f);//70cm*70cm*70cm
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



	std::ofstream myfile;
	myfile.open("submit_1.csv");
	myfile << "id,x,y,z,yaw,pitch,roll\n";
	myfile.close();

	ros::init (argc, argv, "icp_node");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("lidar_points", 1000, icpCallback);
	ros::Subscriber sub_odom = nh.subscribe("wheel_odometry", 1000, odomCallback);
	result_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("result_lidar", 1000);
	result_localization_pub = nh.advertise<nav_msgs::Odometry>("result_localization", 1000);
	ros::spin();


	return 0;
}
