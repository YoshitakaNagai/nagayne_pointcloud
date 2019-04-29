
/* nagayne_pcl_class.h */
/*  */
/* auther : Yoshitaka Nagai */
/*  */

#ifndef NAGAYNE_PCL_CLASS_H
#define NAGAYNE_PCL_CLASS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLHeader.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <vector>
#include <array>
#include <algorithm>
#include <map>



class PointXYZIUTS : public pcl::PointXYZI
{
public:
	float unflatness;
	float time;
}

#endif
