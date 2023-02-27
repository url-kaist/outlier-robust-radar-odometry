//
// Created by shapelim on 30.01.23.
//
#include "features.hpp"
#include "radar_utils.hpp"
#include "conversion.hpp"

#ifndef ORORA_RADAR_LOADER_HPP
#define ORORA_RADAR_LOADER_HPP

#define NOT_DETERMINED 0
#define START_OF_SCAN 1
#define END_OF_SCAN 1
using namespace std;

struct RadarFeature {
    cv::Mat              img_;
    vector<int64_t>      t_;
    // For unifying the unit of timestamp
    double time_unit_denominator_;

    // All the sizes of the below member variables have to be same!!!
    cv::Mat              desc_;
    vector<cv::KeyPoint> kp_;
    Eigen::MatrixXd      feat_in_cart_;
    Eigen::MatrixXd      feat_in_polar_;
    // Compensated features by deskwing and Doppler compensation
    Eigen::MatrixXd      feat_compensated_;
    int64_t              t_crietria_; // time corresponding to pose. It is different depend on the dataset!!!
    vector<double>       t_norm_; // 0.0 ~ 1.0


};

class RadarLoader {
public:
    using FeatureMatching = std::tuple<RadarFeature, RadarFeature>;

    RadarLoader(const std::string &dataset, const std::string keypoint_extraction,
                const double voxel_size, const bool use_voxelization, const bool viz_extraction) :
            dataset_(dataset), keypoint_extraction_(keypoint_extraction),
            voxel_size_(voxel_size), use_voxelization_(use_voxelization), viz_extraction_(viz_extraction) {

        /** [NOTE] Because the unit of timestamp between Oxford and MulRan dataset is different
         * Oxford: microsecond, whereas MulRan: nanosecond
         * Thus, `time_unit_demoninator` is important!
         */
        if (dataset == "oxford") {
            std::cout << "\033[1;32mParams of Oxford Dataset are set\033[0m" << std::endl;
            min_range_             = 58;                 // min range of radar points (bin)
            radar_resolution_      = 0.0432;    // resolution of radar bins in meters per bin
            cart_resolution_       = 0.2592;     // meters per pixel
            cart_pixel_width_      = 964;         // height and width of cartesian image in pixels
            time_unit_denominator_ = 1000000.0;
        } else if (dataset == "mulran") {
            std::cout << "\033[1;32mParams of MulRan Dataset are set\033[0m" << std::endl;
            min_range_             = 58;                 // min range of radar points (bin)
            radar_resolution_      = 0.05952;     // resolution of radar bins in meters per bin
            cart_resolution_       = 0.2592;       // meters per pixel
            cart_pixel_width_      = 964;         // height and width of cartesian image in pixels
            time_unit_denominator_ = 1000000000.0;
        } else { throw invalid_argument("Note implemented!"); }

        if (keypoint_extraction_ != "cen2018" && keypoint_extraction_ != "cen2019") {
            throw std::invalid_argument("Keypoint extraction method seems to be wrong!");
        }

        // Create ORB feature detector
        detector_ = cv::ORB::create();
        detector_->setPatchSize(patch_size_);
        detector_->setEdgeThreshold(patch_size_);
    }

    RadarFeature extractFeature(const string datadir, const string radar_file_path) {
        std::vector<int64_t> times;
        std::vector<double>  azimuths;
        std::vector<bool>    valid;
        cv::Mat              fft_data;

        string abs_polar_form_radar_path = datadir + "/" + radar_file_path;
        if (dataset_ == "mulran") {
            load_radar(abs_polar_form_radar_path, times, azimuths, valid, fft_data,
                       CIR204); // use CIR204 for MulRan dataset
        } else load_radar(abs_polar_form_radar_path, times, azimuths, valid, fft_data);

        if (is_initial_) {
            checkTimeCriteria(times, radar_file_path);
            is_initial_ = false;
        }

        RadarFeature radar_feat;

        radar_feat.t_crietria_ = getTimestamp(radar_file_path);

        if (keypoint_extraction_ == "cen2018") {
            cen2018features(fft_data, zq_, sigma_gauss_, min_range_, radar_feat.feat_in_polar_);
        } else if (keypoint_extraction_ == "cen2019") {
            cen2019features(fft_data, max_points_, min_range_, radar_feat.feat_in_polar_); // targets: 3XN
        }
        /**
         * radar_feat.feat_in_polar_ and radar_feat.feat_in_cart_ are 3 x N
         * where the elements of the last row are just set to ones
         */
        polar_to_cartesian_radar_img(azimuths, fft_data, radar_resolution_, cart_resolution_, cart_pixel_width_,
                                     use_interpolation_,
                                     radar_feat.img_, CV_8UC1);  // NOLINT
        polar_to_cartesian_points(azimuths, times, radar_feat.feat_in_polar_, radar_resolution_,
                                  radar_feat.feat_in_cart_,
                                  radar_feat.t_);

        // cout << radar_feat.feat_in_cart_.cols() << " --- " << radar_feat.t_.size() << endl;
        /**
         * Q. Why voxelization?
         * => It dramatically improves the performance!
         */
        if (use_voxelization_) {
            voxelize(voxel_size_, radar_feat.feat_in_cart_);
        }

        /** After voxelization, radar_feat.t_ is reset by the below `convert_to_bev` function
         *  i.e. radar_feat.feat_in_cart_.cols() == radar_feat.t_.size()
         */
        convert_to_bev(radar_feat.feat_in_cart_, cart_resolution_, cart_pixel_width_,
                       patch_size_, radar_feat.kp_, radar_feat.t_);

        detector_->compute(radar_feat.img_, radar_feat.kp_, radar_feat.desc_);

        auto max_t = *max_element(std::begin(times), std::end(times));
        auto min_t = *min_element(std::begin(times), std::end(times));
        radar_feat.t_norm_ = normalizeTimestamps(radar_feat.t_, min_t, max_t);

        if (viz_extraction_) {
            cv::Mat viz;
            draw_points(radar_feat.img_, radar_feat.kp_, viz);
            cv::imshow("Feature Extraction Viz", viz);
            cv::waitKey(20);
        }

        radar_feat.time_unit_denominator_ = getTimeDenominator();
        return radar_feat;
    }

    FeatureMatching matchFeatures(const RadarFeature &prev_feat, const RadarFeature &curr_feat) {
        RadarFeature prev_matched, curr_matched;
        prev_matched = prev_feat;
        curr_matched = curr_feat;

        std::vector<cv::DMatch> good_matches;
        good_matches.reserve(curr_feat.feat_in_cart_.cols());

        std::vector<std::vector<cv::DMatch>> knn_matches;
        knn_matches.reserve(curr_feat.feat_in_cart_.cols());
        //Match keypoint descriptors
        matcher_->knnMatch(prev_feat.desc_, curr_feat.desc_, knn_matches, 2);

//        std::cout << "\033[1;36mMatching?\033[0m" << std::endl;

        //Filter matches using nearest neighbor distance ratio (Lowe, Szeliski)
        for (uint j = 0; j < knn_matches.size(); ++j) {
            if (!knn_matches[j].size())
                continue;
            float nndr_ = 0.8;
            if (knn_matches[j][0].distance < nndr_ * knn_matches[j][1].distance) {
                auto &idx_pair = knn_matches[j][0];
                good_matches.emplace_back(idx_pair);
            }
        }

        int num_matches = good_matches.size();
        // Convert the good key point matches to Eigen matrices
        prev_matched.feat_in_cart_.resize(2, num_matches);
        curr_matched.feat_in_cart_.resize(2, num_matches);
        prev_matched.t_.resize(num_matches);
        curr_matched.t_.resize(num_matches);
        prev_matched.t_norm_.resize(num_matches);
        curr_matched.t_norm_.resize(num_matches);
        prev_matched.kp_.resize(num_matches);
        curr_matched.kp_.resize(num_matches);

        for (uint j = 0; j < num_matches; ++j) {
            prev_matched.feat_in_cart_.col(j) = prev_feat.feat_in_cart_.block<2, 1>(0, good_matches[j].queryIdx);
            curr_matched.feat_in_cart_.col(j) = curr_feat.feat_in_cart_.block<2, 1>(0, good_matches[j].trainIdx);

            prev_matched.t_[j] = prev_feat.t_[good_matches[j].queryIdx];
            curr_matched.t_[j] = curr_feat.t_[good_matches[j].trainIdx];

            prev_matched.t_norm_[j] = prev_feat.t_norm_[good_matches[j].queryIdx];
            curr_matched.t_norm_[j] = curr_feat.t_norm_[good_matches[j].trainIdx];

            // For visualization
            prev_matched.kp_[j] = prev_feat.kp_[good_matches[j].queryIdx];
            curr_matched.kp_[j] = prev_feat.kp_[good_matches[j].trainIdx];
        }

//        // When you want to check matched pairs via
//        cv::Mat match_curr = curr_feat.img_.clone();
//        cv::Mat match_prev = prev_feat.img_.clone();
//        draw_points(curr_feat.img_, curr_matched.kp_, match_curr, {255, 0, 255});
//        draw_points(prev_feat.img_, prev_matched.kp_, match_prev, {255, 255, 0});
//        cv::Mat img_concat;
//        cv::hconcat(match_prev, match_curr, img_concat);
//        cv::imshow("Matching After Viz", img_concat);
//        cv::waitKey(20);
        return {prev_matched, curr_matched};
    }

    void voxelize(const double voxelization_size, Eigen::MatrixXd &cart_targets) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cart_tmp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cart_tmp_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        eigen2pcl(cart_targets, *cart_tmp);
        static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cart_tmp);
        voxel_filter.setLeafSize(voxelization_size, voxelization_size, voxelization_size);
        voxel_filter.filter(*cart_tmp_voxel);
        int num_pts = cart_tmp_voxel->points.size();
        cart_targets = Eigen::MatrixXd::Zero(3, num_pts);
        for (int ii = 0; ii < num_pts; ++ii) {
            const auto &pt = (*cart_tmp_voxel).points[ii];
            cart_targets(0, ii) = pt.x;
            cart_targets(1, ii) = pt.y;
        }
    }

    int64_t getTimestamp(const string &fname_w_timestamp) {
        std::vector<std::string> parts;
        boost::split(parts, fname_w_timestamp, boost::is_any_of("."));
        return std::stoll(parts[0]);
    }

    void checkTimeCriteria(const vector<int64_t> &times, const string &fname_w_timestamp) {
        int64_t t_criteria = getTimestamp(fname_w_timestamp);
        auto    max_t      = *max_element(std::begin(times), std::end(times));
        auto    min_t      = *min_element(std::begin(times), std::end(times));

        if (abs(t_criteria - max_t) > abs(t_criteria - min_t)) {
            time_crieria_ = START_OF_SCAN;
            cout << "\033[1;33mNote that time criteria is the start point of a scan\033[0m\n";
        } else if (abs(t_criteria - max_t) < abs(t_criteria - min_t)) {
            time_crieria_ = END_OF_SCAN;
            cout << "\033[1;33mNote that time criteria is the end point of a scan\033[0m\n";
        }
    }

    vector<double> normalizeTimestamps(const vector<int64_t> &times, const int64_t min_t, const int64_t max_t) {
        vector<double> times_norm;
        times_norm.resize(times.size());
        auto denominator = static_cast<double>(max_t - min_t);
        for (int i = 0; i < times.size(); ++i) {
            times_norm[i] = static_cast<double>(times[i] - min_t) / denominator;
        }
        return times_norm;
    }

    double getTimeDenominator() {
        return time_unit_denominator_;
    }

    RadarLoader() {}

    ~RadarLoader() {}

//    size_t size() const { return num_frames_; }
private:
    int                                            num_frames_;
    std::string                                    dataset_;
    std::string                                    keypoint_extraction_;
    // Output path names
    std::string                                    output_pose_path_, output_pose_error_path_, output_time_path_;
    std::unordered_map<int64, std::vector<float> > gt_rel_;

    RadarFeature curr_feat_;
    RadarFeature prev_feat_;

    int64_t min_t_;
    int64_t max_t_;

    int    min_range_;
    int    cart_pixel_width_;
    float  radar_resolution_;
    float  cart_resolution_;
    double time_unit_denominator_;

    // cen2018 parameters
    float            zq_          = 3.0;
    int              sigma_gauss_ = 17;
    // cen2019 parameters
    int              max_points_  = 10000;  // WARNING! It directly affects the performance
    // ORB descriptor / matching parameters
    cv::Ptr<cv::ORB> detector_;
    int              patch_size_  = 21;     // width of patch in pixels in cartesian radar image
    float            nndr_        = 0.80;   // Nearest neighbor distance ratio

    double voxel_size_;

    bool is_initial_        = true;
    bool use_voxelization_  = true;
    bool use_interpolation_ = true;
    bool viz_extraction_    = false;

    int time_crieria_ = NOT_DETERMINED;

    /**
     * BRUTEFORCE_HAMMING for ORB descriptors FLANNBASED for cen2019 descriptors
     * Hyungtae: However, empirically, ORB descriptor shows more precise results
     */
    cv::Ptr<cv::DescriptorMatcher> matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    //    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
};

#endif //ORORA_RADAR_LOADER_HPP
