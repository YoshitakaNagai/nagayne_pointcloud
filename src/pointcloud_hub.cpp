/*
 *	pointcloud_hub
 *
 * 	author : Yoshitaka Nagai
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLHeader.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#ifdef _OPENMP
#include <omp.h>
#endif



typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
CloudIPtr pc_unflatness_ (new CloudI);
CloudIPtr pc_intensity_ (new CloudI);

typedef pcl::PointXYZHSV PointIUT;
typedef pcl::PointCloud<PointIUT> CloudIUT;
typedef pcl::PointCloud<PointIUT>::Ptr CloudIUTPtr;
CloudIUTPtr pc_iut_ (new CloudIUT);



class FusionPC
{
public:
	FusionPC();

	void intensity_callback(const sensor_msgs::PointCloud2ConstPtr&);
	void unflatness_callback(const sensor_msgs::PointCloud2ConstPtr&);
	void fusion(void);

private:
	ros::NodeHandle nh;
	ros::Subscriber intensity_sub_pc;
	ros::Subscriber unflatness_sub_pc;
	ros::Publisher fusioned_cloud_pub;

	sensor_msgs::PointCloud2 fusioned_pc;

	float ros_time;

	bool intensity_flag;
	bool unflatness_flag;
	bool fusion_flag;
};


FusionPC::FusionPC(void)
{
	intensity_flag = false;
	unflatness_flag = false;

	intensity_sub_pc = nh.subscribe("/cloud", 10, &FusionPC::intensity_callback, this);
	unflatness_sub_pc = nh.subscribe("/nagayne_PointCloud2/unflatness", 10, &FusionPC::unflatness_callback, this);
	//fusioned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/fusioned", 1);
	fusioned_cloud_pub = nh.advertise<CloudIUT>("/nagayne_PointCloud2/fusioned", 10);
}


void FusionPC::intensity_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pc_intensity_);
	pcl::fromROSMsg(*msg, *pc_iut_);
	intensity_flag = true;
}


void FusionPC::unflatness_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *pc_unflatness_);
	unflatness_flag = true;
}


void FusionPC::fusion(void)
{
	ros::Rate r(100);
	while(ros::ok())
	{
		if(intensity_flag && unflatness_flag){
			ros_time = (float)ros::Time::now().toSec();
			size_t pc_size = pc_iut_->points.size();
			//#pragma omp parallel for
			for(size_t i=0;i<pc_size;i++){
				//h = intensity, s = unflatness, v = time
				pc_iut_->points[i].h = pc_intensity_->points[i].intensity;
				pc_iut_->points[i].s = pc_unflatness_->points[i].intensity;
				pc_iut_->points[i].v = ros_time;
			}
			//pcl::toROSMsg(*pc_iut_, fusioned_pc);
			//fusioned_pc.header.stamp = ros::Time::now();
			//fusioned_cloud_pub.publish(fusioned_pc);
			fusioned_cloud_pub.publish(*pc_iut_);
			intensity_flag = false;
			unflatness_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_fusion");
	FusionPC fusion_pointcloud;
	fusion_pointcloud.fusion();

	return 0;
}
