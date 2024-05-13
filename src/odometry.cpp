#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>

#include "radar_loader.hpp"
#include "radar_utils.hpp"
#include "association.hpp"
#include "visualization.hpp"
#include "solver/nonminimal_solver.hpp"
#include "SE2posemanager.hpp"
#include "ORORA.hpp"

void signalCallbackHandler(int signum) {
    std::cout << "Caught Ctrl + C!" << std::endl;
    // Terminate program
    exit(signum);
}

void pubTF(const Eigen::Matrix4d &latest_pose, const string odom_frame,
           const string child_frame) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped      alias_transform_msg;
    Eigen::Quaterniond                   q(latest_pose.block<3, 3>(0, 0));
    alias_transform_msg.header.stamp            = ros::Time::now();
    alias_transform_msg.transform.translation.x = latest_pose(0, 3);
    alias_transform_msg.transform.translation.y = latest_pose(1, 3);
    alias_transform_msg.transform.translation.z = latest_pose(2, 3);
    alias_transform_msg.transform.rotation.x    = q.x();
    alias_transform_msg.transform.rotation.y    = q.y();
    alias_transform_msg.transform.rotation.z    = q.z();
    alias_transform_msg.transform.rotation.w    = q.w();
    alias_transform_msg.header.frame_id         = odom_frame;
    alias_transform_msg.child_frame_id          = child_frame;
    br.sendTransform(alias_transform_msg);
}

using namespace orora;

Eigen::Matrix4d curr_odom;
Eigen::Matrix4d curr_gt;

float CORR_INLIER_DIST = 0.5;
int   end_idx          = -1; // radar_files.size() - 1;
// modified for MulRan dataset batch evaluation
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "orora", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    curr_odom = Eigen::Matrix4d::Identity(4, 4); // initial pose is I
    curr_gt   = Eigen::Matrix4d::Identity(4, 4); // initial pose is I

    ros::Publisher PubGT   = nh.advertise<nav_msgs::Odometry>("/orora/gt", 100);
    ros::Publisher PubOdom = nh.advertise<nav_msgs::Odometry>("/orora/odom", 100);

    ros::Publisher PubLaserCloudLocal  = nh.advertise<sensor_msgs::PointCloud2>("/orora/cloud_local", 100, true);
    ros::Publisher PubLaserCloudGlobal = nh.advertise<sensor_msgs::PointCloud2>("/orora/cloud_global", 100, true);
    ros::Publisher PubPrevFeat         = nh.advertise<sensor_msgs::PointCloud2>("/orora/prev/feat", 100, true);
    ros::Publisher PubPrevCompensated  = nh.advertise<sensor_msgs::PointCloud2>("/orora/prev/compensated", 100, true);
    ros::Publisher PubCurrFeat         = nh.advertise<sensor_msgs::PointCloud2>("/orora/curr/feat", 100, true);
    ros::Publisher PubCurrCompensated  = nh.advertise<sensor_msgs::PointCloud2>("/orora/curr/compensated", 100, true);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher      PubImg = it.advertise("orora/matched_img", 100);

    bool        viz_extraction, viz_matching, stop_each_frame;
    std::string seq_dir, algorithm, dataset;
    std::string keypoint_extraction;        // "cen2018", "cen2019", "orb"
    string      odom_frame  = "odom";
    string      child_frame = "radar_base";
    double         frame_rate  = 4.0; // 4 Hz
    nh.param<std::string>("seq_dir", seq_dir, "");
    nh.param<std::string>("algorithm", algorithm, "");
    nh.param<std::string>("dataset", dataset, "");
    nh.param<std::string>("keypoint_extraction", keypoint_extraction, "cen2019");
    nh.param<std::string>("odom_frame", odom_frame, "odom");
    nh.param<std::string>("child_frame", child_frame, "radar_base");
    nh.param<int>("end_idx", end_idx, -1);
    nh.param<bool>("viz_extraction", viz_extraction, false);
    nh.param<bool>("viz_matching", viz_matching, false);
    nh.param<bool>("stop_each_frame", stop_each_frame, false);
    nh.param<double>("frame_rate", frame_rate, 4.0);
    std::cout << "\033[1;32mTarget dataset: " << dataset << " \033[0m" << std::endl;
    std::cout << "\033[1;32mTarget algorithm: " << algorithm << " \033[0m" << std::endl;
    /*** Below options are for ORORA ***/

    // ORORA parameters
    bool   estimating_scale;
    int    num_max_iter;
    double noise_bound, noise_bound_radial, noise_bound_tangential;
    double noise_bound_coeff, gnc_factor, rot_cost_diff_thr, voxel_size;
    nh.param<bool>("/ORORA/estimating_scale", estimating_scale, false);
    nh.param<double>("/ORORA/noise_bound", noise_bound, 0.5);
    nh.param<double>("/ORORA/noise_bound_radial", noise_bound_radial, 0.3536);
    nh.param<double>("/ORORA/noise_bound_tangential", noise_bound_tangential, 9.0);
    nh.param<double>("/ORORA/noise_bound_coeff", noise_bound_coeff, 1.0);
    nh.param<double>("/ORORA/rotation/gnc_factor", gnc_factor, 1.39);
    nh.param<double>("/ORORA/rotation/rot_cost_diff_thr", rot_cost_diff_thr, 0.0001);
    nh.param<int>("/ORORA/rotation/num_max_iter", num_max_iter, 50);
    // DOPPLER
    double      beta = 0.049;
    // Implementation details: check stop motion
    bool check_stop_motion;
    int num_feat_thr_for_stop_motion;
    nh.param<bool>("/ORORA/stop_motion/check_stop_motion", check_stop_motion, true);
    nh.param<int>("/ORORA/stop_motion/num_feat_thr_for_stop_motion", num_feat_thr_for_stop_motion, 600);
    // Options for feature matching
    bool        use_voxelization, use_doppler_compensation, use_deskewing;
    std::string deskewing_target;
    nh.param<double>("/ORORA/voxel_size", voxel_size, 0.5);
    nh.param<bool>("/ORORA/use_voxelization", use_voxelization, false);
    nh.param<bool>("/ORORA/use_doppler_compensation", use_doppler_compensation, false);
    nh.param<bool>("/ORORA/use_deskewing", use_deskewing, false);
    nh.param<std::string>("/ORORA/deskewing_target", deskewing_target, "rot"); // "rot" or "tf"
    if (deskewing_target != "rot" && deskewing_target != "tf")
        throw std::invalid_argument("Wrong deskewing target is comming");
    std::string print_status = use_voxelization ? "Yes" : "No";
    std::cout << "\033[1;32m[ORORA] Voxelization on?: " << print_status << std::endl;
    std::cout << "[ORORA] Noise bound: " << noise_bound << std::endl;
    std::cout << "[ORORA] Radial noise bound: " << noise_bound_radial << std::endl;
    std::cout << "[ORORA] Tangential noise bound: " << noise_bound_tangential << " deg\033[0m" << std::endl;
    noise_bound_tangential = DEG2RAD(noise_bound_tangential);

    NonMinimalSolver::Params params;

    setParams(noise_bound, noise_bound_coeff,
              estimating_scale, num_max_iter, gnc_factor, rot_cost_diff_thr, params);

    ORORA  orora_estimator(dataset, deskewing_target, use_doppler_compensation,
                           use_deskewing, check_stop_motion, num_feat_thr_for_stop_motion,
                           beta, noise_bound_radial, noise_bound_tangential);
    // Get file names of the radar images
    std::vector<std::string> radar_files;
    std::string              datadir = seq_dir + "/polar_oxford_form/";
    get_file_names(datadir, radar_files);
    SE2PoseManager PoseManager(seq_dir, dataset, algorithm, keypoint_extraction, voxel_size, noise_bound,
                               noise_bound_radial, noise_bound_tangential);
    RadarLoader    Loader(dataset, keypoint_extraction, voxel_size, use_voxelization, viz_extraction);
    double         time_unit_denominator = Loader.getTimeDenominator();

    RadarFeature prev_feat, curr_feat;

    int start_idx = 0;
    if (end_idx < 0) end_idx = radar_files.size() - 1;
    for (uint i = start_idx; i < end_idx; ++i) {
        signal(SIGINT, signalCallbackHandler);
        std::cout << "\033[1;34m" << i << "/" << radar_files.size() << " of " + seq_dir << "\033[0m" << std::endl;
        if (stop_each_frame) {
            std::cout << "Press any button!" << std::endl;
            cin.ignore();
        }
        if (i > start_idx) {
            prev_feat = curr_feat;
        }

        curr_feat = Loader.extractFeature(datadir, radar_files[i]);

        if (i == start_idx)
            continue;
        auto [prev_matched, curr_matched] = Loader.matchFeatures(prev_feat, curr_feat);

        std::vector<std::string> parts;

        /**
         * [NOTE] The criteria of the timestamp of poses between the Oxford and MulRan dataset is different!
         * i.e. the t-th radar scan of the Oxford -> Data are captured during t ~ t + 1
         *      the t-th radar scan of the MulRan -> Data are captured during t - 1 ~ t
        */
        boost::split(parts, radar_files[i - 1], boost::is_any_of("."));
        int64 time0 = std::stoll(parts[0]);
        boost::split(parts, radar_files[i], boost::is_any_of("."));
        int64 time1 = std::stoll(parts[0]);
        boost::split(parts, radar_files[i + 1], boost::is_any_of("."));
        int64 time2 = std::stoll(parts[0]);

        /**
         * [NOTE] Because the unit of timestamp of pose between Oxford and MulRan dataset is different
         * Oxford: microsecond, whereas MulRan: nanosecond
         * Solution: Just parameterize delta t by `frame rate`
        */
        const double delta_t = 1.0 / frame_rate;

        pcl::PointCloud<PointType> src_mc, tgt_mc;
        pcl::PointCloud<PointType> src_inliers, src_inliers_tf, tgt_inliers;
        Eigen::MatrixXd            T, Tmd, Tmd2;
        Eigen::Matrix4d            Test  = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d            Tgt   = Eigen::Matrix4d::Identity();
        int                        time_taken;
        auto                       start = chrono::steady_clock::now();
        if (algorithm == "RANSAC") {
            // v1: Compute the transformation using RANSAC
            std::cout << "RANSAC" << std::endl;
            Ransac ransac(curr_matched.feat_in_cart_, prev_matched.feat_in_cart_, association::ransac_threshold,
                          association::inlier_ratio,
                          association::max_iterations);
            srand(i);
            ransac.computeModel();
            ransac.getTransform(T);
            Test.block<2, 2>(0, 0) = T.block<2, 2>(0, 0);
            Test(0, 3)             = T(0, 2);
            Test(1, 3)             = T(1, 2);
        } else if (algorithm == "mdRANSAC") {
            // v2: Compute the transformation using motion-distorted RANSAC
            MotionDistortedRansac mdransac(curr_matched.feat_in_cart_, prev_matched.feat_in_cart_,
                                           curr_matched.t_, prev_matched.t_,
                                           curr_matched.time_unit_denominator_,
                                           association::md_threshold,
                                           association::inlier_ratio, association::max_iterations);
            mdransac.setMaxGNIterations(association::max_gn_iterations);
            mdransac.correctForDoppler(false);
            srand(i);
            mdransac.computeModel();
            mdransac.getTransform(delta_t, Tmd);
            Tmd  = Tmd.inverse();
            // Eigen::VectorXd wbar;
            // mdransac.getMotion(wbar);
            Test = Tmd;
        } else if (algorithm == "mdRANSACDoppler") {
            // v3: MDRANSAC + Doppler
            MotionDistortedRansac mdransac(curr_matched.feat_in_cart_, prev_matched.feat_in_cart_,
                                           curr_matched.t_, prev_matched.t_,
                                           curr_matched.time_unit_denominator_,
                                           association::md_threshold,
                                           association::inlier_ratio,
                                           association::max_iterations);
            mdransac.setMaxGNIterations(association::max_gn_iterations);
            mdransac.correctForDoppler(true);
            mdransac.setDopplerParameter(beta);
            srand(i);
            mdransac.computeModel();
            Tmd2 = Eigen::MatrixXd::Zero(4, 4);
            mdransac.getTransform(delta_t, Tmd2);
            Tmd2 = Tmd2.inverse();
            Test = Tmd2;
        } else if (algorithm == "ORORA") {
            // ORORA: Outlier-RObust RAdar odometry
            Test = orora_estimator.calcTransform(params, delta_t, prev_matched, curr_matched);
        } else {
            throw std::invalid_argument("Algorithm mode is wrong! Please check the param. `algorithm`.");
        }
        auto end   = chrono::steady_clock::now();
        time_taken = chrono::duration_cast<chrono::microseconds>(end - start).count();
        cout << "\033[1;32mElapsed time: " << static_cast<float>(time_taken) / 1000.0 << " ms\033[0m\n";

        std::vector<float> xyyaw_rel;
        if (dataset == "oxford") xyyaw_rel = PoseManager.getXYYawPose(time1, time2);
        else if (dataset == "mulran") xyyaw_rel = PoseManager.getXYYawPose(time1);
        float           curr_gt_yaw = xyyaw_rel[2];
        Eigen::Matrix4d TgtTmp      = Eigen::Matrix4d::Identity();
        PoseManager.XYYaw2SE2(xyyaw_rel, TgtTmp);
        Tgt = TgtTmp;

        /**
         * Ego-motion is estimated!!!
         */
        // curuent state
        curr_gt   = curr_gt * Tgt;
        curr_odom = curr_odom * Test;
        Eigen::Matrix3d curr_odomRot   = curr_odom.block(0, 0, 3, 3);
        Eigen::Vector3d curr_odomEuler = curr_odomRot.eulerAngles(0, 1, 2);
        // Eigen::Vector3d currEulerVec = curr_odomRot.eulerAngles(2, 1, 0);
        // float curr_yaw = asin(-1 * curr_odom(0, 1));
        // float curr_yaw = currEulerVec(3);
        float           curr_yaw       = float(curr_odomEuler(2));
        // std::cout << "curr_odomEuler: " << curr_odomEuler(0) << ", " << curr_odomEuler(1) << ", " << curr_odomEuler(2) << std::endl;
        auto            time_now       = ros::Time::now();

        /**
         * Save relative pose error
         */
        // std::cout << "Trans.: " << Test(0, 3) << ", " << Test(1, 3) << std::endl;
        vector<double> rel_error = PoseManager.writePose(Tgt, Test);
        PoseManager.writePose(Tgt, Test, time1, time2, time_unit_denominator);
        PoseManager.writeTraj(curr_odom, PoseManager.est_traj_path_);
        PoseManager.writeTraj(curr_gt, PoseManager.gt_traj_path_);
        PoseManager.writeTimeTaken(time_taken, Tgt);

        /**
         * Below codes are just for visualization
         */
        auto set_odom_msg = [&](const Eigen::Matrix4d &pose, const double yaw) {
            nav_msgs::Odometry odom;
            odom.header.frame_id       = odom_frame;
            odom.child_frame_id        = child_frame;
            odom.header.stamp          = time_now;
            odom.pose.pose.position.x  = pose(0, 3);
            odom.pose.pose.position.y  = pose(1, 3);
            odom.pose.pose.position.z  = pose(2, 3);
            odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
            return odom;
        };

        PubOdom.publish(set_odom_msg(curr_odom, curr_yaw)); // last pose
        PubGT.publish(set_odom_msg(curr_gt, curr_gt_yaw)); // last pose
        cout << "Complete to publish pose! " << endl;

        // for scan context (in this naive case, we can say we will use binary scan context).
        const float constant_z_nonzero = 1.0;

        pcl::PointCloud<PointType>::Ptr laserCloudLocal(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCloudGlobal(new pcl::PointCloud<PointType>());
        eigen2pcl(curr_feat.feat_in_cart_, *laserCloudLocal, true);
        pcl::transformPointCloud(*laserCloudLocal, *laserCloudGlobal, curr_odom.cast<float>());

        for_each(std::execution::par_unseq, laserCloudLocal->points.begin(), laserCloudLocal->points.end(),
                 [&](PointType &pt) {
                     pt.z += constant_z_nonzero;
                 });

        PubLaserCloudLocal.publish(cloud2msg(*laserCloudLocal, child_frame));
        PubLaserCloudGlobal.publish(cloud2msg(*laserCloudGlobal, odom_frame));

        pubTF(curr_odom, odom_frame, child_frame);
        cout << "Complete to publish cloud! " << endl;

        /***********************************************/

        /*** * Feature visualization */
//        PubPrevFeat.publish(cloud2msg(eigenxy2pcl(prev_feat.feat_in_cart_), child_frame));
//        PubCurrFeat.publish(cloud2msg(eigenxy2pcl(curr_feat.feat_in_cart_), child_frame));
//        PubPrevCompensated.publish(cloud2msg(eigenxy2pcl(prev_feat.feat_compensated_), child_frame));
//        PubCurrCompensated.publish(cloud2msg(eigenxy2pcl(prev_feat.feat_compensated_), child_frame));

        if (viz_matching && algorithm == "ORORA") {

            int     margin_size = 50;
            cv::Mat white_margin(prev_feat.img_.rows, margin_size, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat img_tmp, img_concat, img_concat_bgr;
            // 1. Concatenate two images
            cv::hconcat(prev_feat.img_, white_margin, img_tmp);
            cv::hconcat(img_tmp, curr_feat.img_, img_concat);
            cv::cvtColor(img_concat, img_concat_bgr, cv::COLOR_GRAY2BGR);

            // 2. Check whether the match is an inlier or outlier
            pcl::PointCloud<PointType> curr_feat_tf;
            auto                       prev_feat_pcl = eigenxy2pcl(prev_matched.feat_compensated_);
            pcl::transformPointCloud(eigenxy2pcl(curr_matched.feat_compensated_), curr_feat_tf, Test.cast<float>());
            assert(curr_feat_tf.size() == prev_feat.feat_compens);

            int               N = curr_feat_tf.points.size();
            std::vector<bool> is_inliers;
            is_inliers.reserve(N);
            int      num_inliers = 0;
            for (int ll          = 0; ll < N; ++ll) {
                const auto &sp = curr_feat_tf[ll];
                const auto &tp = prev_feat_pcl[ll];

                float corr_dist = sqrt(pow(sp.x - tp.x, 2) + pow(sp.y - tp.y, 2));
                if (corr_dist < CORR_INLIER_DIST) {
                    is_inliers.push_back(true);
                    ++num_inliers;
                } else is_inliers.push_back(false);
            }
            draw_matches_concat(img_concat_bgr, prev_matched.kp_, curr_matched.kp_, is_inliers,
                                margin_size + prev_feat.img_.cols);
            std::string inlier_ratio_txt = std::to_string(static_cast<int>(static_cast<double>(num_inliers) * 100 / N));
            cv::Point   text_xy;
            text_xy.y = 150;
            text_xy.x = img_concat_bgr.cols - 380;
            std::string text_in_fig = std::string(2 - inlier_ratio_txt.length(), ' ') + inlier_ratio_txt + "%";
            cv::putText(img_concat_bgr, text_in_fig, text_xy, 1, 10, cv::Scalar(0, 128, 255), 8);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_concat_bgr).toImageMsg();
            PubImg.publish(msg);

            // If you want to save some images
//            std::string count_str = std::to_string(i);
//            std::string count_str_padded = std::string(4 - count_str.length(), '0') + count_str + "inlier_ratio_" + inlier_ratio_txt;
//            std::string output_filename = "/home/shapelim/match_viz/" + count_str_padded + ".png";
//            cv::imwrite(output_filename, img_concat_bgr);

            // If you want to visualize matched results by OpenCV.
//            cv::Mat resize_down;
//            resize(img_concat_bgr, resize_down,
//                   cv::Size(img_concat_bgr.cols / 2, img_concat_bgr.rows / 2), cv::INTER_LINEAR);
//            cv::imshow("Feature matching results", resize_down);
//            cv::waitKey(20);
        }

        // make sure under 10hz, to privde enough time for following PGO module
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ros::spinOnce();

    return 0;
}
