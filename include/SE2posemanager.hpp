
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <string>
#include <experimental/filesystem>
#include <boost/format.hpp>

#ifndef RADAR_DATALOADER_H
#define RADAR_DATALOADER_H

#define EPSILON_ROT 0.00000000001
#define EPSILON_TRANS 0.001

using namespace std;
namespace fs = std::experimental::filesystem;

void loadGTPoses(const std::string &gt_path, std::unordered_map<int64, std::vector<float> > &gt_rel) {
    // Only for (Time, x, y, yaw) file
    gt_rel.clear();
    std::ifstream ifs(gt_path);
    std::string   line;
    // If the first line consists of text, below line should be uncommented
    // std::getline(ifs, line);

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        float x,y,yaw;
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of(","));

        if (words.size() < 4) {
            std::cerr << "Skipping invalid line: " << line << std::endl;
            continue;
        }
        int64              timestamp    = stoll(words[0]);

        std::vector<float> parsed_xyyaw = {stod(words[1]), stod(words[2]), stod(words[3])};
        gt_rel[timestamp] = parsed_xyyaw;
    }

    std::cout << "Total " << gt_rel.size() << " GT rel. poses are loaded" << std::endl;
    if (gt_rel.size() == 0) {
        throw invalid_argument("Some parameters seem to be wrong! Load GT failed!");
    }
}

class SE2PoseManager {
public:
    SE2PoseManager(const std::string &seq_dir, const std::string &dataset_name,
                   const std::string &algorithm_name, const std::string keypoint_extraction, double voxel_size,
                   double noise_bound) :
            seq_dir_(seq_dir),
            dataset_name_(dataset_name),
            algorithm_name_(algorithm_name),
            keypoint_extraction_(keypoint_extraction) {
        std::cout << "Data loading...: " << seq_dir << std::endl;
        std::string datadir = seq_dir + "/radar/";
        gt_path_ = seq_dir + "/gt/radar_odometry.csv";
        if (dataset_name == "mulran") loadGTPoses(gt_path_, gt_rel_);
        // File for storing the results of estimation on each frame (and the accuracy)
        string output_dir = (boost::format("%s/outputs") % seq_dir).str();
        if (!fs::is_directory(output_dir) || !fs::exists(output_dir)) { // Check if src folder exists
            fs::create_directory(output_dir); // create src folder
        }
        std::string common_str = (boost::format("%s/outputs/%s_%s_%s_") % seq_dir
                                  % dataset_name % algorithm_name % keypoint_extraction).str() + to_string(voxel_size);
        common_str.erase(common_str.find_last_not_of('0') + 1, std::string::npos);
        common_str += "_" + to_string(noise_bound);
        common_str.erase(common_str.find_last_not_of('0') + 1, std::string::npos);
        setFilenames(common_str);

        std::string checked_filename = output_pose_error_path_;
        int         num_trials       = 1;
        while (fs::exists(checked_filename)) {
            std::cout << "\033[1;33[Warning] File already exists\033[0m" << std::endl;
            setFilenames(common_str + std::to_string(num_trials++));
            checked_filename = output_pose_error_path_;
        }
    }

    SE2PoseManager(const std::string &seq_dir, const std::string &dataset_name,
                   const std::string &algorithm_name, const std::string keypoint_extraction, double voxel_size,
                   double noise_bound,
                   double noise_bound_radial, double noise_bound_tangential) :
            seq_dir_(seq_dir),
            dataset_name_(dataset_name),
            algorithm_name_(algorithm_name),
            keypoint_extraction_(keypoint_extraction) {
        std::cout << "Data loading...: " << seq_dir << std::endl;
        std::string datadir = seq_dir + "/radar/";
        gt_path_ = seq_dir + "/gt/radar_odometry.csv";
        if (dataset_name == "mulran") loadGTPoses(gt_path_, gt_rel_);
        // File for storing the results of estimation on each frame (and the accuracy)
        std::string common_str;

        string output_dir = (boost::format("%s/outputs") % seq_dir).str();
        if (!fs::is_directory(output_dir) || !fs::exists(output_dir)) { // Check if src folder exists
            fs::create_directory(output_dir); // create src folder
        }
        if (algorithm_name_ == "ORORA") {
            bool ANALYZE_PARAM = false;
            if (ANALYZE_PARAM) {
                common_str = (boost::format("%s/outputs/%s_%s_%s_") % seq_dir
                              % dataset_name % algorithm_name % keypoint_extraction).str() +
                             to_string(voxel_size);
                common_str.erase(common_str.find_last_not_of('0') + 1, std::string::npos);
                common_str += "_" + to_string(noise_bound);
                common_str.erase(common_str.find_last_not_of('0') + 1, std::string::npos);
                common_str += "_" + to_string(noise_bound_radial);
                common_str.erase(common_str.find_last_not_of('0') + 1, std::string::npos);
                common_str += "_" + to_string(noise_bound_tangential);
                common_str.erase(common_str.find_last_not_of('0') + 1, std::string::npos);
            } else {
                common_str = (boost::format("%s/outputs/%s_%s_%s_") % seq_dir
                          % dataset_name % algorithm_name % keypoint_extraction).str();
            }

        } else {
            common_str = (boost::format("%s/outputs/%s_%s_%s_") % seq_dir
                          % dataset_name % algorithm_name % keypoint_extraction).str();
        }

        setFilenames(common_str);
        std::cout << "\033[1;34mSave dir: " << common_str << "\033[0m" << std::endl;

        std::string checked_filename       = output_pose_error_path_;
        static bool is_overwrite_available = true;
        if (is_overwrite_available) {
            std::cout << "\033[1;33m[Warning] Files are overwritten!!\033[0m" << std::endl;
            clearOutputFile(output_pose_error_path_);
            clearOutputFile(output_pose_path_);
            clearOutputFile(output_time_path_);
            clearOutputFile(est_traj_path_);
            clearOutputFile(gt_traj_path_);
        } else {
            int num_trials = 1;
            while (std::experimental::filesystem::exists(checked_filename)) {
                std::cout << "\033[1;33m[Warning] File already exists\033[0m" << std::endl;
                setFilenames(common_str + std::to_string(num_trials++));
                checked_filename = output_pose_error_path_;
            }
        }
    }

    SE2PoseManager() {}

    ~SE2PoseManager() {}

//    size_t size() const { return num_frames_; }

    // This is for MulRan dataset
    std::vector<float> getXYYawPose(const int64 t) {
        if (gt_rel_.find(t) == gt_rel_.end()) {
            throw invalid_argument("The corresponding time is not inserted!... exiting.");
        }
        return gt_rel_[t];
    }

    void setFilenames(std::string common_dir) {
        output_pose_error_path_ = common_dir + "rel_error_odom.txt";
        output_pose_path_       = common_dir + "eval_odom.txt";
        output_time_path_       = common_dir + "time_taken.txt";
        est_traj_path_          = common_dir + "est_traj.txt";
        gt_traj_path_           = common_dir + "gt_traj.txt";
    }

    std::vector<float> getXYYawPose(const int64 time1, const int64 time2) {
        std::vector<float> gtvec;
        if (!get_groundtruth_odometry(gt_path_, time1, time2, gtvec)) {
            std::cout << "ground truth odometry for " << time1 << " " << time2 << " not found... exiting." << std::endl;
            throw invalid_argument("Stop!!");
        }
        return {gtvec[0], gtvec[1], gtvec[5]};
    }

    Eigen::Matrix4d getSE2Pose(const int64 t) {
        std::vector<float> xyyaw_rel = getXYYawPose(t);
        Eigen::Matrix4d    Tgt       = Eigen::Matrix4d::Identity();
        XYYaw2SE2(xyyaw_rel, Tgt);
        return Tgt;
    }

    void XYYaw2SE2(const std::vector<float> &xyyaw_rel, Eigen::Matrix4d &Tgt) {
        Tgt(0, 3) = xyyaw_rel[0];
        Tgt(1, 3) = xyyaw_rel[1];
        tf::Quaternion q_tf;
        if (dataset_name_ == "oxford") {
            q_tf = tf::createQuaternionFromYaw(xyyaw_rel[2]);
        } else if (dataset_name_ == "mulran") {
            // ToDo. Why minus yaw is required?
            q_tf = tf::createQuaternionFromYaw(-xyyaw_rel[2]);
        }
        Eigen::Quaternionf q(q_tf.w(), q_tf.x(), q_tf.y(), q_tf.z());
        Tgt.block<3, 3>(0, 0) = q.toRotationMatrix().cast<double>();
    }

    void clearOutputFile(const std::string &path) {
        // The file is written from scratch
        ofstream FileObj(path);
        FileObj.close();
    }

    vector<double> writePose(const Eigen::Matrix4d &Tgt, const Eigen::Matrix4d &Test) {
        Eigen::Matrix4d Trel      = Tgt.inverse() * Test;
        double          ts_error  = sqrt(Trel(0, 3) * Trel(0, 3) + Trel(1, 3) * Trel(1, 3)); // Unit: m
        Eigen::Matrix3d rel_rot   = Trel.block<3, 3>(0, 0);
        double          rot_error = acos((rel_rot.trace() - 1) / 2 + EPSILON_ROT) * 180.0 / M_PI; // Unit: degree
        if (isnan(rot_error)) {
            // It means that relative rotation is too close to identity!
            rot_error = 0.0;
        }
        double ts_abs              = sqrt(Tgt(0, 3) * Tgt(0, 3) + Tgt(1, 3) * Tgt(1, 3)); // Unit: m
        double ts_perc_error       = ts_abs < EPSILON_TRANS ? 0 : ts_error / ts_abs * 100.0;
        double rot_deg_per_m_error = ts_abs < EPSILON_TRANS ? 0 : rot_error / ts_abs;

        std::ofstream FileObj;
        FileObj.open(output_pose_error_path_, std::ios::app);
        FileObj << ts_error << "," << rot_error << "," << ts_perc_error << "," << rot_deg_per_m_error << "," << ts_abs
                << "\n";
        FileObj.close();
        return {ts_error, rot_error};
    }

    void writePose(const Eigen::Matrix4d &Tgt, const Eigen::Matrix4d &Test, int64 time1, int64 time2,
                   const double time_unit_denominator) {
        double        tsx_gt   = Tgt(0, 3);
        double        tsy_gt   = Tgt(1, 3);
        double        yaw_gt   = -1 * asin(Tgt(0, 1));
        double        tsx_test = Test(0, 3);
        double        tsy_test = Test(1, 3);
        double        yaw_test = -1 * asin(Test(0, 1));
        std::ofstream FileObj;
        FileObj.open(output_pose_path_, std::ios::app);
        FileObj << tsx_gt << "," << tsy_gt << "," << yaw_gt << "," << tsx_test << "," << tsy_test << "," << yaw_test
                << "," << time1 / time_unit_denominator << "," << time2 / time_unit_denominator << "\n";
        FileObj.close();
    }

    void writeTimeTaken(int time_taken, const Eigen::Matrix4d &Tgt) {
        double          rel_ts       = sqrt(pow(Tgt(0, 3), 2) + pow(Tgt(1, 3), 2));
        Eigen::Matrix3d rot_mat      = Tgt.block(0, 0, 3, 3);
        Eigen::Vector3d euler_angles = rot_mat.eulerAngles(0, 1, 2);
        double          rel_rot      = euler_angles(2);

        bool is_stopped = false;
        if (rel_ts < 0.003 && rel_rot < 0.001) {
            is_stopped = true;
        }
        std::ofstream FileObj;
        FileObj.open(output_time_path_, std::ios::app);
        FileObj << time_taken << "," << is_stopped << "\n";
        FileObj.close();
    }

    void writeTraj(Eigen::Matrix4d Tf4x4, const std::string path) {
        double x = Tf4x4(0, 3);
        double y = Tf4x4(1, 3);

        Eigen::Matrix3d rot_mat      = Tf4x4.block(0, 0, 3, 3);
        Eigen::Vector3d euler_angles = rot_mat.eulerAngles(0, 1, 2);
        double          yaw          = euler_angles(2);

        std::ofstream FileObj;
        FileObj.open(path, std::ios::app);
        FileObj << x << "," << y << "," << yaw << "\n";
        FileObj.close();
    }

    string est_traj_path_;
    string gt_traj_path_;
private:
    int                                            num_frames_;
    std::string                                    seq_dir_;
    std::string                                    gt_path_;
    std::string                                    dataset_name_;
    std::string                                    algorithm_name_;
    std::string                                    keypoint_extraction_;
    // Output path names
    std::string                                    output_pose_path_, output_pose_error_path_, output_time_path_;
    std::unordered_map<int64, std::vector<float> > gt_rel_;
};

#endif  // FAST_GICP_KITTILOADER_H
