#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <execution>

template<typename T>
inline sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

template<typename T>
inline void eigen2pcl(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src, pcl::PointCloud<T> &cloud) {
    int num_pc = src.cols();
    T   pt_tmp;
    if (!cloud.empty()) cloud.clear();
    for (int i = 0; i < num_pc; ++i) {
        pt_tmp.x = src(0, i);
        pt_tmp.y = src(1, i);
        pt_tmp.z = src(2, i);
        cloud.points.emplace_back(pt_tmp);
    }
}

inline pcl::PointCloud<pcl::PointXYZ> eigenxy2pcl(const Eigen::MatrixXd &src) {
    int                            num_pc = src.cols();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(num_pc);

    std::vector<size_t> index(num_pc);
    for (size_t         i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        cloud.points[i].x = src(0, i);
        cloud.points[i].y = src(1, i);
        cloud.points[i].z = 0;
    });

    return cloud;
}

template<typename T>
inline void eigen2d2pcl(const Eigen::Matrix<double, 2, Eigen::Dynamic> &src, pcl::PointCloud<T> &cloud) {
    int num_pc = src.cols();
    T   pt_tmp;
    if (!cloud.empty()) cloud.clear();
    cloud.points.reserve(num_pc);

    for (int i = 0; i < num_pc; ++i) {
        pt_tmp.x = src(0, i);
        pt_tmp.y = src(1, i);
        pt_tmp.z = 0.0;
        cloud.points.emplace_back(pt_tmp);
    }
}

template<typename T>
inline void eigen2pcl(const Eigen::MatrixXd &src, pcl::PointCloud<T> &cloud, bool set_zero = false) {
    int num_pc = src.cols();
    if (!cloud.empty()) cloud.clear();
    cloud.points.resize(num_pc);

    std::vector<size_t> index(num_pc);
    for (size_t         i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        cloud.points[i].x = src(0, i);
        cloud.points[i].y = src(1, i);
        cloud.points[i].z = [&]() -> double {
            if (set_zero) return 0;
            else return src(2, i);
        }();
    });
}


#endif // CONVERSION_HPP
