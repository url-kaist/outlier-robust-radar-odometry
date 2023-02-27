//
// Created by shapelim on 31.01.23.
//
#include <iostream>
#include <Eigen/Core>
#include "solver/nonminimal_solver.hpp"

#ifndef ORORA_ORORA_HPP
#define ORORA_ORORA_HPP

using namespace std;
namespace orora {
    class ORORA {
    public:
        ORORA(const string dataset,
              const string deskewing_target,
              const bool use_doppler_compenstation, const bool use_deskewing,
              const bool check_stop_motion, const int num_feat_thr_for_stop_motion,
              const double beta, const double noise_bound_radial, const double noise_bound_tangential) {
            dataset_                      = dataset;
            deskewing_target_             = deskewing_target;
            use_doppler_compensation_     = use_doppler_compenstation;
            use_deskewing_                = use_deskewing;
            check_stop_motion_            = check_stop_motion;
            num_feat_thr_for_stop_motion_ = num_feat_thr_for_stop_motion;
            beta_                         = beta;
            noise_bound_radial_           = noise_bound_radial;
            noise_bound_tangential_       = noise_bound_tangential;

            T_prev_ = Eigen::Matrix4d::Identity();
            T_curr_ = Eigen::Matrix4d::Identity();
        }

        ~ORORA() {
        }

        Eigen::MatrixXd calcTransform(const NonMinimalSolver::Params params,
                                      const double delta_t,
                                      RadarFeature &prev_matched, RadarFeature &curr_matched) {
            Eigen::Matrix4d output = Eigen::Matrix4d::Identity();

            prev_vel_se3_ = curr_vel_se3_;

            if (use_deskewing_) {
                std::cout << "\033[1;36m[ORORA] Deskewing on! (" << deskewing_target_ << ")\033[0m" << std::endl;
                motionDeskewing(prev_vel_se3_, prev_matched.t_, prev_matched.t_crietria_,
                                prev_matched.time_unit_denominator_,
                                deskewing_target_,
                                prev_matched.feat_in_cart_, dataset_);
                motionDeskewing(prev_vel_se3_, curr_matched.t_, curr_matched.t_crietria_,
                                curr_matched.time_unit_denominator_,
                                deskewing_target_,
                                curr_matched.feat_in_cart_, dataset_);
            }

            if (use_doppler_compensation_) {
                std::cout << "\033[1;36m[ORORA] Doppler compensation on!\033[0m" << std::endl;
                prev_matched.feat_compensated_ = compensateDopplerDistortion(prev_vel_se3_, beta_,
                                                                             prev_matched.feat_in_cart_);
                curr_matched.feat_compensated_ = compensateDopplerDistortion(prev_vel_se3_, beta_,
                                                                             curr_matched.feat_in_cart_);
            } else {
                prev_matched.feat_compensated_ = prev_matched.feat_in_cart_;
                curr_matched.feat_compensated_ = curr_matched.feat_in_cart_;
            }

//            cv::Mat viz;
//            draw_points(curr_matched.img_, curr_matched.feat_compensated_, 0.2592, 964, viz);
//            cv::imshow("After compensation?", viz);
//            cv::waitKey(0);

            std::cout << "\033[1;36m[ORORA] # of features: " << prev_matched.feat_compensated_.cols() << "\033[0m\n";

            if (check_stop_motion_ && curr_matched.feat_compensated_.cols() > num_feat_thr_for_stop_motion_) {
                std::cout << "\033[1;33m[ORORA] Stop motion detected. Pose estimation is skipped for real-time operation\n";
                std::cout << "[ORORA] This may slightly decrease the quality of rotation estimation\n";
                std::cout << "[ORORA] If you run ORORA for quantitative evaluation, set `check_stop_motion` as false\033[0m\n";
                output = Eigen::Matrix4d::Identity();

            } else {
                solver_.reset(new NonMinimalSolver());
                solver_->reset(params, noise_bound_radial_, noise_bound_tangential_);
                solver_->setInputSource(curr_matched.feat_compensated_);
                solver_->setInputTarget(prev_matched.feat_compensated_);
                solver_->computeTransformation(T_prev_, output);
            }

            T_prev_       = output; // For utilizing it as an initial guess
            curr_vel_se3_ = SE3tose3(output) / delta_t;
            T_curr_       = output;

            return T_curr_;
        }

        void motionDeskewing(const Eigen::VectorXd &wbar, const std::vector<int64_t> &ts,
                             const int64_t t_pose, const double time_unit_denominator,
                             const std::string deskewing_target,
                             Eigen::MatrixXd &p2d, std::string dataset) {
            /**
             * [NOTE]: When you using oxford dataset, i+1-th `t_pose` should be given
             * Again, the criteria of timestamps of MulRan dataset and Oxford dataset is different!!!!
             */
            double   ang_vel = wbar(5);
            double   del_x, del_y, del_ang;
            for (int i       = 0; i < p2d.cols(); ++i) {
                double delta_t = static_cast<double>(ts[i] - t_pose) / time_unit_denominator;
//                cout << "Delta t per pt: " << delta_t << "\n";
                del_ang = ang_vel * delta_t;
                if (deskewing_target == "tf") {
                    del_x = wbar(0) * delta_t;
                    del_y = wbar(1) * delta_t;
                }
                // Effect of translation is ignored
                double c = cos(del_ang);
                double s = sin(del_ang);

                Eigen::Vector2d p_deskewed;
                p_deskewed(0) = c * p2d(0, i) - s * p2d(1, i);
                p_deskewed(1) = s * p2d(0, i) + c * p2d(1, i);
                if (deskewing_target == "tf") {
                    p_deskewed(0) += del_x;
                    p_deskewed(1) += del_y;
                }
                /* Update the values */
                p2d(0, i)     = p_deskewed(0);
                p2d(1, i)     = p_deskewed(1);
            }
        }

        void motionDeskewing(const Eigen::VectorXd &wbar, const std::vector<int64_t> &ts, const int64_t t_pose,
                             const double time_unit_denominator, const std::string deskewing_target,
                             const Eigen::MatrixXd &p2d, Eigen::MatrixXd &compensated, std::string dataset) {
            /* [NOTE]: When you using oxford dataset, i+1-th `t_pose` should be given
             * Again, the criteria of timestamps of MulRan dataset and Oxford dataset is different!!!! */
            compensated = p2d;
            motionDeskewing(wbar, ts, t_pose, time_unit_denominator, deskewing_target, compensated, dataset);
        }

        Eigen::MatrixXd compensateDopplerDistortion(const Eigen::VectorXd &wbar, const double beta,
                                                    const Eigen::MatrixXd &p2d) {
            Eigen::MatrixXd compensated = Eigen::MatrixXd::Zero(2, p2d.cols());
            double          v           = fabs(wbar(0, 0));
            for (int        i           = 0; i < p2d.cols(); ++i) {
                Eigen::VectorXd p   = p2d.col(i);
                double          rsq = p(0) * p(0) + p(1) * p(1);
                p(0) += beta * v * p(0) * p(0) / rsq;
                p(1) += beta * v * p(0) * p(1) / rsq;
                compensated.col(i)  = p;
            }
            return compensated;
        }

        // Use it after calculating relative pose!
        orora::NonMinimalSolver::InlierSetPairs getMaxCliques() {
            return solver_->getMaxCliques();
        }

        // Use it after calculating relative pose!
        orora::NonMinimalSolver::InlierSetPairs getFinalInliers() {
            return solver_->getFinalInliers();
        }

    private:
        bool   use_doppler_compensation_;
        bool   use_deskewing_;
        bool   check_stop_motion_;
        double beta_; // Doppler constant
        double noise_bound_radial_;
        double noise_bound_tangential_;
        string deskewing_target_;
        string dataset_;
        int    num_feat_thr_for_stop_motion_;

        boost::shared_ptr<orora::NonMinimalSolver> solver_;

        Eigen::VectorXd prev_vel_se3_ = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd curr_vel_se3_ = Eigen::VectorXd::Zero(6);

        Eigen::MatrixXd T_prev_;
        Eigen::MatrixXd T_curr_;
    };
}

#endif //ORORA_ORORA_HPP
