#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

sensor_msgs::PointCloud2 cloud_curvature;


void Callback(const sensor_msgs::PointCloud2ConstPtr& input) {
	pcl::fromROSMsg (*input, *cloud);
}

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "sq_curvature");

	cloud =  pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud/lcl", 10, Callback);

	ros::Publisher curvature_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/curvature",10);


	ros::Rate r(100);
	while(ros::ok())
	{
		 //... read, pass in or create a point cloud ...

 		 // Create the normal estimation class, and pass the input dataset to it
  		// pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  		pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  		ne.setInputCloud (cloud);

 		 // Create an empty kdtree representation, and pass it to the normal estimation object.
  		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  		ne.setSearchMethod (tree);

 		 // Output datasets
  		// pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);

 		 // Use all neighbors in a sphere of radius 3cm
  		ne.setRadiusSearch (0.03);

  		// Compute the features
		ne.compute (*cloud_normals);

        // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
        // std::cout << cloud_normals->points.size() << ", " << cloud->points.size() << std::endl;

		pcl::toROSMsg(*cloud_normals, cloud_curvature);
		curvature_pub.publish(cloud_curvature);

		r.sleep();
		ros::spinOnce();
	}
	return (0);
}
