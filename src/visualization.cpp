#include "visualization.hpp"


void CorrespondenceMarker::setCorrespondenceMarker(ros::Publisher &pub_corr_true, ros::Publisher &pub_corr_false,
                                                   const float corr_inlier_dis) {
    for (int idx = 0; idx < src_tf_.size(); ++idx) {
        const auto &sp = src_tf_[idx];
        const auto &tp = tgt_matched_[idx];
        corr_dis = sqrt(pow(sp.x - tp.x, 2) + pow(sp.y - tp.y, 2));
        if (corr_dis < corr_inlier_dis) corr_true.push_back(idx);
        else corr_false.push_back(idx);
    }
    std::cout << src_tf_.size() << " ===> " << corr_true.size() << " vs " << corr_false.size() << std::endl;
    setCorrespondenceMarker(pub_corr_true, corr_true_marker, CORR_TRUE);
//    if (!corr_false.empty()) {
    setCorrespondenceMarker(pub_corr_false, corr_false_marker, CORR_FALSE);
//    }
}

void CorrespondenceMarker::setCorrespondenceMarker(ros::Publisher &pub_corr, visualization_msgs::Marker marker,
                                                   bool is_true) {
    if (!marker.points.empty()) marker.points.clear();
    marker.header.frame_id = "/map";
    marker.header.stamp    = ros::Time();
    marker.ns              = "visualize correspondence";
    marker.id              = num_visuals;
    marker.type            = visualization_msgs::Marker::LINE_LIST;
    marker.action          = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.75;
    marker.color.a = 1.0; // Don't forget to set the alpha!

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point srcP;
    geometry_msgs::Point tgtP;
    srcP.z = 0;
    tgtP.z = 0;

    if (is_true) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (int idx = 0; idx < corr_true.size(); ++idx) {
            int pair = corr_true[idx];
            srcP.x = src_matched_.points[pair].x;
            srcP.y = src_matched_.points[pair].y;
            srcP.z = z_scale_for_viz_ + 0.5;
            tgtP.x = tgt_matched_.points[pair].x;
            tgtP.y = tgt_matched_.points[pair].y;
            tgtP.z = z_scale_for_viz_ + 0.5;

            marker.points.emplace_back(srcP);
            marker.points.emplace_back(tgtP);
        }
    } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        for (int idx = 0; idx < corr_false.size(); ++idx) {
            int pair = corr_false[idx];
            srcP.x = src_matched_.points[pair].x;
            srcP.y = src_matched_.points[pair].y;
            srcP.z -= (z_scale_for_viz_ - 0.5);
            tgtP.x = tgt_matched_.points[pair].x;
            tgtP.y = tgt_matched_.points[pair].y;
            tgtP.z -= (z_scale_for_viz_ - 0.5);

            marker.points.emplace_back(srcP);
            marker.points.emplace_back(tgtP);
        }
    }
    pub_corr.publish(marker);
    num_visuals++;
}