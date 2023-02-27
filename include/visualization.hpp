#pragma once

#include <vector>
#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Geometry>
#include <cmath>
#include <Eigen/Dense>
#include "solver/nonminimal_solver.hpp"

const int NUM_LARGE_ENOUGH            = 5000;
const int NUM_LARGE_ENOUGH_PTS_IN_BIN = 100;

const bool CORR_TRUE  = 1;
const bool CORR_FALSE = 0;

class CorrespondenceMarker {
public:
    CorrespondenceMarker(pcl::PointCloud<PointType> src_matched, pcl::PointCloud<PointType> tgt_matched,
                         pcl::PointCloud<PointType> src_tf, double z_scale_for_viz)
            : src_matched_(src_matched), tgt_matched_(tgt_matched), src_tf_(src_tf),
              z_scale_for_viz_(z_scale_for_viz) {}

    ~CorrespondenceMarker() {}

    void
    setCorrespondenceMarker(ros::Publisher &pub_corr_true, ros::Publisher &pub_corr_false, const float corr_inlier_dis);

    void setCorrespondenceMarker(ros::Publisher &pub_corr, visualization_msgs::Marker marker, bool is_true);
//    void setLaserMarker(ros::Publisher& pub_tgt, ros::Publisher& pub_src);
//    void setLaserMarker(ros::Publisher& pub_laser, visualization_msgs::Marker marker, bool is_tgt);


private:
    pcl::PointCloud<PointType> src_matched_;
    pcl::PointCloud<PointType> src_tf_;
    pcl::PointCloud<PointType> tgt_matched_;
    std::vector<int>           corr_true, corr_false;
    visualization_msgs::Marker corr_true_marker, corr_false_marker, source_marker, target_marker;
    float                      corr_inlier_dis, corr_dis;
    int                        num_visuals = 0;

    double z_scale_for_viz_ = 0;
};
