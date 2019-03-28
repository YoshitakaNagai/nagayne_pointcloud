#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

int c = 0;

class PCNormals{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		ros::Publisher curvature_pub;
		sensor_msgs::PointCloud2 cloud_curvature;
	public:
		PCNormals();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void NormalEstimation(void);
};

PCNormals::PCNormals()
{
	sub = nh.subscribe("/cloud/lcl", 1, &PCNormals::Callback, this);
	curvature_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/curvature",10);
}

void PCNormals::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK" << std::endl;
	// pcl::fromROSMsg(*msg, *cloud);
	pcl::fromROSMsg(*msg, *normals);
	NormalEstimation();
	pcl::toROSMsg(*normals, cloud_curvature);
	curvature_pub.publish(cloud_curvature);
	c++;
	std::cout << "count:" << c << std::endl;

}

void PCNormals::NormalEstimation(void)
{
	std::cout << "NORMAL ESTIMATION" << std::endl;
	pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
	ne.setInputCloud (normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.3);
	ne.compute (*normals);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_normals");
	std::cout << "PC NORMALS" << std::endl;
	
	PCNormals pcnormals;
	ros::spin();
}
