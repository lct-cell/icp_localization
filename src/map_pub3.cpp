#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/transforms.h>

//reference: https://pointclouds.org/documentation/tutorials/voxel_grid.html

int main(int argc, char **argv)
{
    ros::init (argc, argv, "map_pub_node");
    ros::NodeHandle nh;
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2> ("map", 1);

    // new add 
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_n(new pcl::PCLPointCloud2());//
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    //reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_map.pcd", *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1600_900.pcd", *cloud);

	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1700_900.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1600_1000.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1700_1000.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1600_800.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);
	reader.read("/home/lct/nctu_sdc/localization_ws/src/localization_309512009/map/nuscenes_maps/map_1700_800.pcd", *cloud_n);
	pcl::concatenatePointCloud(*cloud, *cloud_n, *cloud);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points " << std::endl;

    // Create the filtering object
    // change leaf size if need
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(1.0f, 1.0f, 1.0f);//80cm*80cm*80cm
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
              << " data points " << std::endl;

    sensor_msgs::PointCloud2 output;

    pcl_conversions::fromPCL(*cloud_filtered, output);//to sensor_msgs/pointcloud2
    
    output.header.frame_id = "map";
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcd_pub.publish(output);
        //ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
