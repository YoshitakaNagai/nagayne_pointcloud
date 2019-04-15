/*
 * Copyright(c) 2017, SEQSENSE, Inc.
 * All rights reserved.
 */

/**
  \author Atsushi Watanabe (SEQSENSE, Inc.)
  \brief A cached LaserScan to PointCloud2 projector class.
 **/

#ifndef SCAN_TRANSFORM_CACHED_H
#define SCAN_TRANSFORM_CACHED_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <map>
#include <algorithm>

template <typename T>
class ScanTransformCached
{
protected:
  class ScanSpec
  {
  public:
    T angle_min;
    T angle_max;
    T angle_increment;
    T time_increment;
    T scan_time;
    T range_min;
    T range_max;

    explicit ScanSpec(const sensor_msgs::LaserScan &scan)
    {
      angle_min = scan.angle_min;
      angle_max = scan.angle_max;
      angle_increment = scan.angle_increment;
      time_increment = scan.time_increment;
      scan_time = scan.scan_time;
      range_min = scan.range_min;
      range_max = scan.range_max;
    }
    bool operator==(const ScanSpec &a)
    {
      if (a.angle_min != angle_min)
        return false;
      if (a.angle_max != angle_max)
        return false;
      if (a.angle_increment != angle_increment)
        return false;
      if (a.time_increment != time_increment)
        return false;
      if (a.scan_time != scan_time)
        return false;
      if (a.range_min != range_min)
        return false;
      if (a.range_max != range_max)
        return false;
      return true;
    }
    bool operator!=(const ScanSpec &a)
    {
      return !(*this == a);
    }
  };
  class Sincos
  {
  public:
    T sin;
    T cos;
  };

  std::unique_ptr<ScanSpec> cached_scan_spec_;
  std::unique_ptr<Sincos[]> sincos_cache_;
  size_t cache_size_;

public:
  bool operator==(const ScanTransformCached &a)
  {
    if (!cached_scan_spec_)
      return false;
    return *(a.cached_scan_spec_) == *cached_scan_spec_;
  }

  explicit ScanTransformCached(const sensor_msgs::LaserScan &scan)
  {
    reset(scan);
  }
  ScanTransformCached()
  {
  }
  void reset(const sensor_msgs::LaserScan &scan)
  {
    cache_size_ = size_t(lroundf((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1);

    cached_scan_spec_.reset(new ScanSpec(scan));
    sincos_cache_.reset(new Sincos[cache_size_]);

    for (size_t i = 0; i < cache_size_; i++)
    {
      long double s, c;
      sincosl(scan.angle_min + i * scan.angle_increment, &s, &c);
      sincos_cache_[i].sin = T(s);
      sincos_cache_[i].cos = T(c);
    }
  }
  T sin(const size_t i)
  {
    return sincos_cache_[i].sin;
  }
  T cos(const size_t i)
  {
    return sincos_cache_[i].cos;
  }

  void transform(const sensor_msgs::LaserScan &scan, sensor_msgs::PointCloud2 &output,
                 const tf::StampedTransform &trans0, const tf::StampedTransform &trans1)
  {
    if (ScanSpec(scan) != *cached_scan_spec_)
      reset(scan);

    output.header = scan.header;
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = false;
    sensor_msgs::PointCloud2Modifier pc2_modifier(output);
    pc2_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::PointField::FLOAT32);
    pc2_modifier.reserve(scan.ranges.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(output, "intensity");

    const tf::Quaternion rotation_diff(trans1.getRotation() * trans0.getRotation().inverse());
    const tf::Quaternion rotation_increment(rotation_diff.getAxis(), rotation_diff.getAngle() / (cache_size_ - 1));
    tf::Quaternion rotation(trans0.getRotation());

    const tf::Vector3 origin_increment((trans1.getOrigin() - trans0.getOrigin()) / (cache_size_ - 1));
    tf::Vector3 origin(trans0.getOrigin());
    int i = 0;

    auto intensity = scan.intensities.begin();
    for (auto &r : scan.ranges)
    {
      if (scan.range_min <= r && r <= scan.range_max)
      {
        pc2_modifier.resize(pc2_modifier.size() + 1);

        const tf::Point p(tf::Transform(rotation, origin) *
                          tf::Point(r * sincos_cache_[i].cos, r * sincos_cache_[i].sin, 0.0));
        *iter_x = p.x();
        *iter_y = p.y();
        *iter_z = p.z();
        *iter_i = *(intensity++);
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_i;
      }

      rotation *= rotation_increment;
      origin += origin_increment;
      ++i;
    }
  }
};

#endif  // SCAN_TRANSFORM_CACHED_H
