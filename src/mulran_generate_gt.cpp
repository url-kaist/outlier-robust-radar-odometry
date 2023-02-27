//
// Created by shapelim on 22. 6. 10.
// To sync. the ground truth pose
//
#include <boost/format.hpp>
#include <vector>
#include <string>
#include <experimental/filesystem>
#include <sstream>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "radar_utils.hpp"
#include <tf/tf.h>
//#include "matplotlibcpp.h"  // NOLINT

using namespace std;

void vec2tf4x4(vector<float> &pose, Eigen::Matrix4f &tf4x4) {
    for (int idx = 0; idx < 12; ++idx) {
        int i = (idx) / 4;
        int j = (idx) % 4;
        tf4x4(i, j) = pose[idx];
    }
}

void loadGTPoses(string txt, vector<Eigen::Matrix4f> &poses, vector<int64> &timestamps) {
    // This function is only for mulran dataset
    // It's w.r.t. the initial base!
    // https://sites.google.com/view/mulran-pr/download
    Eigen::Matrix4f BASE2RADAR = Eigen::Matrix4f::Identity();

    BASE2RADAR(0, 0) = cos(DEG2RAD(0.9));
    BASE2RADAR(0, 1) = -sin(DEG2RAD(0.9));
    BASE2RADAR(1, 0) = sin(DEG2RAD(0.9));
    BASE2RADAR(1, 1) = cos(DEG2RAD(0.9));

    BASE2RADAR(0, 3) = 1.50;
    BASE2RADAR(1, 3) = -0.04;
    BASE2RADAR(2, 3) = 1.97;

    poses.clear();
    poses.reserve(4000);
    timestamps.clear();
    timestamps.reserve(4000);
    ifstream in(txt);
    string   line;

    int             count  = 0;
    Eigen::Matrix4f origin = Eigen::Matrix4f::Identity();
    while (getline(in, line)) {
        // Checking delimiter is important!!!
        int64         timestamp;
        // Timestamp should be parsed separately!
        vector<float> parsed_data = splitLine(line, ',', timestamp);
        timestamps.push_back(timestamp);

        Eigen::Matrix4f world2base = Eigen::Matrix4f::Identity(); // Crucial!
        vec2tf4x4(parsed_data, world2base);
        if (count == 0) {
            origin = world2base;
        }

        Eigen::Matrix4f tf4x4_wrt_init = origin.inverse() * world2base * BASE2RADAR;
        cout << "\r" << count++ << " th poses are loaded..." << flush;

        poses.emplace_back(tf4x4_wrt_init);

        count++;
    }
    in.close();
    cout << flush;
    cout << "Total " << count << " poses are loaded" << endl;
}

// Just check the pose by Cloud Compare
void savePosAsPcd(string pcd_path, vector<Eigen::Matrix4f> &poses) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(poses.size());
    for (const auto &pose: poses) {
        pcl::PointXYZ p(pose(0, 3), pose(1, 3), 0);
        cloud.points.emplace_back(p);
    }
    pcl::io::savePCDFileBinary(pcd_path, cloud);

}

int main(int argc, char *argv[]) {
    vector<string> target_dirs;
    string         abs_dir = argv[1];
    for (int       i       = 2; i < argc; ++i) {
        target_dirs.push_back(abs_dir + "/" + argv[i]);
    }
    if (argc < 3) {
        throw invalid_argument(
                "\033[1;31mWrong argument is given! Please follow \n`$ rosrun orora mulran_generate_sync_gt ${ABS_DIR} ${TARGET_SEQ1} ${TARGET_SEQ2} ...\033[0m");
    }

    cout << "\033[1;32mCheck your abs. dir: " << abs_dir << "\033[0m" << endl;

    /**
     * [NOTE] The criteria of the timestamp of poses between the Oxford and MulRan dataset is different!
     * i.e. the t-th radar scan of the Oxford -> Data are captured during t ~ t + 1
     *      the t-th radar scan of the MulRan -> Data are captured during t - 1 ~ t
    */
    std::string gt_style = "mulran"; // "mulran" or "oxford"

    for (const auto common_dir: target_dirs) {
        string datadir = common_dir + "/polar_oxford_form/"; // "/" at the end is required
        string gtpath  = common_dir + "/global_pose.csv";
        // NOTE: "/gt" folder is required.
        string gt_dir  = common_dir + "/gt";
        cout << "\033[1;32mSave destination: " << gt_dir << "\033[0m" << endl;
        // create directory and remove old files;
        int unused = system((std::string("exec rm -r ") + gt_dir).c_str());
        unused = system((std::string("mkdir -p ") + gt_dir).c_str());

        /**
         * Output of five files
         * `.pcd` files are just for visualization.
         */
        string gt_wrt_base_pcd     = gt_dir + "/ground_truth_base.pcd";
        string gt_sync_path        = gt_dir + "/ground_truth_sync.csv";
        string gt_sync_pcd         = gt_dir + "/ground_truth_sync.pcd";
        string rel_xyyaw_sync_path = gt_dir + "/radar_odometry.csv";
        string rel_xyyaw_sync_pcd  = gt_dir + "/radar_odometry.pcd";

        /**
         * 1. Load all the file names
         */
        std::vector<std::string> radar_files;
        std::cout << datadir << std::endl;
        get_file_names(datadir, radar_files);
        std::vector<int64> timestamps_of_radar;
        int                count = 0;
        for (const auto    &radar_file: radar_files) {
            std::vector<std::string> parts_dummy;
            boost::split(parts_dummy, radar_file, boost::is_any_of("."));
            int64 time_check = std::stoll(parts_dummy[0]);
            timestamps_of_radar.push_back(time_check);
        }
        std::sort(timestamps_of_radar.begin(), timestamps_of_radar.end());

        /**
         * 2. Load global poses of radar frame
         * IMPORTANT: Note that poses are w.r.t. the initial base
         */
        vector<Eigen::Matrix4f> poses;
        vector<int64>           timestamps_of_poses;
        loadGTPoses(gtpath, poses, timestamps_of_poses);
        savePosAsPcd(gt_wrt_base_pcd, poses);

        /**
         * 3. Interpolate poses
         */
        int                     idx = 0;
        vector<Eigen::Matrix4f> poses_sync;
        for (const auto         &timestamp: timestamps_of_radar) {
            // It's the chracteristics of MulRan dataset
            if (timestamps_of_poses[0] > timestamp) {
                poses_sync.emplace_back(poses[0]);
            } else {
                while (timestamps_of_poses[idx + 1] < timestamp) {
                    idx++;
                }
                float m = (timestamp - timestamps_of_poses[idx]) /
                          (timestamps_of_poses[idx + 1] - timestamps_of_poses[idx]);
                float n = 1 - m;

                Eigen::Matrix4f pose0 = poses[idx];
                Eigen::Matrix4f pose1 = poses[idx + 1];

                Eigen::Quaternionf q0(pose0.block<3, 3>(0, 0));
                Eigen::Quaternionf q1(pose1.block<3, 3>(0, 0));

                /**
                 * if n -> 0, it returns q0
                 * if n -> 1, it returns q1
                 */
                q0.slerp(n, q1);
                Eigen::Quaternionf q2;
                q2 = q0.slerp(n, q1);

                Eigen::Matrix4f pose_sync = Eigen::Matrix4f::Identity();
                pose_sync(0, 3)             = m * pose1(0, 3) + n * pose0(0, 3);
                pose_sync(1, 3)             = m * pose1(1, 3) + n * pose0(1, 3);
                pose_sync(2, 3)             = m * pose1(2, 3) + n * pose0(2, 3);
                pose_sync.block<3, 3>(0, 0) = q2.toRotationMatrix();
//                cout << "\r Interpolating...\n" << pose_sync << flush;
                poses_sync.emplace_back(pose_sync);
            }
//            cout << "\n";
        }
        std::cout << poses_sync.size() << " vs " << timestamps_of_radar.size() << std::endl;
        if (poses_sync.size() != timestamps_of_radar.size()) throw invalid_argument("Size is not same!");
        savePosAsPcd(gt_sync_pcd, poses_sync);

        // 4. Final stage
        Eigen::Matrix4f init_radar_wrt_init_base;
        for (int        i = 0; i < poses_sync.size(); ++i) {
            if (i == 0) {
                init_radar_wrt_init_base = poses_sync[0];
                poses_sync[0] = Eigen::Matrix4f::Identity();
            } else {
                Eigen::Matrix4f radar_wrt_init_base = poses_sync[i];
                Eigen::Matrix4f pose_wrt_init_radar = init_radar_wrt_init_base.inverse() * radar_wrt_init_base;
                poses_sync[i] = pose_wrt_init_radar;
            }
        }
        savePosAsPcd(rel_xyyaw_sync_pcd, poses_sync);

        std::ofstream GTSyncCSV, RelGTXYYawCSV;
        GTSyncCSV.open(gt_sync_path);
        for (int i = 0; i < poses_sync.size(); ++i) {
            const auto         &p = poses_sync[i];
            Eigen::Quaternionf q(p.block<3, 3>(0, 0));
            GTSyncCSV << timestamps_of_radar[i] << "," << p(0, 3) << "," << p(1, 3) << "," << p(2, 3)
                      << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
        }
        GTSyncCSV.close();

        RelGTXYYawCSV.open(rel_xyyaw_sync_path);
        if (gt_style == "mulran") {
            // NOTE: Timestamps in MulRan are measured at the end of the scans
            for (int i = 0; i < poses_sync.size() - 1; ++i) {
                const auto         &p      = poses_sync[i];
                const auto         &p_next = poses_sync[i + 1];
                Eigen::Matrix4f    p_rel   = p.inverse() * p_next;
                Eigen::Quaternionf q(p_rel.block<3, 3>(0, 0));

                tf::Quaternion q_rel(q.x(), q.y(), q.z(), q.w()); // x, y, z, w in order
                tf::Matrix3x3  m(q_rel);
                double         roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                RelGTXYYawCSV << timestamps_of_radar[i + 1] << "," << p_rel(0, 3) << ","
                              << p_rel(1, 3) << "," << yaw << "\n";
            }
        } else if (gt_style == "oxford") {
            // NOTE: Timestamps in Oxford are measured at the start of the scans
            // NOT RECOMMENDED!
            RelGTXYYawCSV
                    << "source_timestamp,destination_timestamp,x,y,z,roll,pitch,yaw,source_radar_timestamp,destination_radar_timestamp\n";
            for (int i = 0; i < poses_sync.size() - 1; ++i) {
                const auto         &p      = poses_sync[i];
                const auto         &p_next = poses_sync[i + 1];
                Eigen::Matrix4f    p_rel   = p.inverse() * p_next;
                Eigen::Quaternionf q(p_rel.block<3, 3>(0, 0));

                tf::Quaternion q_rel(q.x(), q.y(), q.z(), q.w()); // x, y, z, w in order
                tf::Matrix3x3  m(q_rel);
                double         roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // Generate Ground truth by Oxford Format
                int64 s_radar_ts = timestamps_of_radar[i + 1];
                int64 d_radar_ts = timestamps_of_radar[i];
                int64 d_ts       = (s_radar_ts + d_radar_ts) / 2;
                int64 s_ts       = i + 2 < poses_sync.size() ? (s_radar_ts + timestamps_of_radar[i + 2]) / 2 : d_ts +
                                                                                                               (s_radar_ts -
                                                                                                                d_radar_ts);
                RelGTXYYawCSV << s_ts << "," << d_ts << "," << p_rel(0, 3) << "," << p_rel(1, 3) << ",0,0,0,"
                              << yaw << "," << s_radar_ts << "," << d_radar_ts << "\n";
            }
        }
        RelGTXYYawCSV.close();
    }
    return 0;
}
