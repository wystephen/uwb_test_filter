#pragma once
//Create by steve in 16-12-21 at 下午11:01
//
// Created by steve on 16-12-21.
//
/**
 *                             _ooOoo_
 *                            o8888888o
 *                            88" . "88
 *                            (| -_- |)
 *                            O\  =  /O
 *                         ____/`---'\____
 *                       .'  \\|     |//  `.
 *                      /  \\|||  :  |||//  \
 *                     /  _||||| -:- |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|  ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. /
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"".
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=---='
 *          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                     佛祖保佑        永无BUG
 *            佛曰:
 *                   写字楼里写字间，写字间里程序员；
 *                   程序人员写程序，又拿程序换酒钱。
 *                   酒醒只在网上坐，酒醉还来网下眠；
 *                   酒醉酒醒日复日，网上网下年复年。
 *                   但愿老死电脑间，不愿鞠躬老板前；
 *                   奔驰宝马贵者趣，公交自行程序员。
 *                   别人笑我忒疯癫，我笑自己命太贱；
 *                   不见满街漂亮妹，哪个归得程序员？
*/

#include <Eigen/Dense>

#include <thread>
#include <mutex>


#ifndef CVREADER_KALMANFILTER_HPP
#define CVREADER_KALMANFILTER_HPP

namespace own {
    /**
     * Assumed the acc is zero.
     * @tparam T double float (do not use int)
     * @tparam state_num
     * @tparam observe_num
     */
    template<typename T, int state_num, int observe_num>
    class KalmanFilter {
    public:
        /**
         *
         * @param eval_sigma
         * @param noise_sigma
         * @param state_sigma_vec
         */
        KalmanFilter(T eval_sigma,
                     T noise_sigma,
                     Eigen::VectorXd state_sigma_vec) {

            data_process_mutex_.lock();

            //Check
            if (state_sigma_vec.rows() != state_num) {
                std::cout << "ERROR: state num is not equal to state_sigam_vec.rows()" << std::endl;
            }

            Q_.setIdentity();
            Q_ = Q_ * noise_sigma;

            R_.setIdentity();
            R_ = R_ * eval_sigma;

            P_.setZero();
            for (int i(0); i < P_.rows(); ++i) {
                for (int j(0); j < P_.cols(); ++j) {
                    P_(i, j) = state_sigma_vec(i) * state_sigma_vec(j);
                }
            }

            X_.setZero();

            A_.setZero();

            C_.setZero();

            //ToDo:For test.
            InitialMatrix();
            data_process_mutex_.unlock();

        }

        /**
         *
         * @param state_vec
         * @return
         */
        bool InitialState(Eigen::VectorXd state_vec) {
            X_ = state_vec;
            return true;
        }

        /**
         *
         * @return
         */
        bool InitialMatrix() {
            A_.setZero();
            A_(0, 0) = 1.0;
            A_(0, 1) = 0.0;
            A_(0, 2) = 1.0;
            A_(0, 3) = 0.0;

            A_(1, 0) = 0.0;
            A_(1, 1) = 1.0;
            A_(1, 2) = 0.0;
            A_(1, 3) = 1.0;

            A_(2, 0) = 0.0;
            A_(2, 1) = 0.0;
            A_(2, 2) = 1.0;
            A_(2, 3) = 0.0;

            A_(3, 0) = 0.0;
            A_(3, 1) = 0.0;
            A_(3, 2) = 0.0;
            A_(3, 3) = 1.0;

            C_.setZero();
            C_(0, 0) = 1.0;
            C_(1, 1) = 1.0;

        }

        /**
         * DO NOT USE THIS!!!
         * @param observe_val
         * @return
         */
        Eigen::VectorXd OneStep(Eigen::VectorXd observe_val) {
            return OneStep(observe_val, 1.0);
        };

        /**
         *
         * @param observe_val
         * @param time_step
         * @return
         */
        Eigen::VectorXd OneStep(Eigen::VectorXd observe_val, T time_step) {
            data_process_mutex_.lock();

            last_X_ = X_;

            //predictor
            Eigen::MatrixXd t_A(A_);
            t_A(0, 2) = time_step;
            t_A(1, 3) = time_step;

            X_ = (A_) * X_;
            P_ = (t_A) * P_ * (t_A.transpose().eval() * time_step) + Q_;

            // Kalman gain

            K_ = P_ * C_.transpose().eval() * (C_ * P_ * C_.transpose().eval() + R_).inverse();

            //correct

            Eigen::MatrixXd I;
            I.resizeLike(K_ * C_);
            I.setIdentity();

            X_ = X_ + K_ * (observe_val - C_ * X_);

            data_process_mutex_.unlock();
            /**
             * P_ not be used in predict function.
             */

            P_ = (I - K_ * C_) * P_;
            //numerous modified
            P_ = (P_ * 0.5 + P_.transpose().eval() * 0.5);

            //Special to this situation.
//            X_(2) = (X_(0)-last_X_(0))/time_step;
//            X_(3) = (X_(1)-last_X_(1))/time_step;



            return X_;
        };


        /**
         * Use time_step predict next state.
         * @param time_step
         * @return
         */
        Eigen::VectorXd Predict(T time_step) {

            data_process_mutex_.lock();

            Eigen::MatrixXd t_A(A_);
            t_A(0, 2) = time_step;
            t_A(1, 3) = time_step;
            predict_x_ = (t_A) * X_;
//            std::cout << "x:" << X_.transpose()
//                      << "pre:" << predict_x_.transpose() << std::endl;

            data_process_mutex_.unlock();

            return predict_x_;
        }

        /**
         * Set noise sigma.
         * @param noise_sigma
         * @return
         */
        bool setNoiseSigma(double noise_sigma) {
            if (abs(noise_sigma - Q_(0, 0)) < 0.01) {
                return true;
            }
            data_process_mutex_.lock();
            Q_.setIdentity();
            Q_ = Q_ * noise_sigma;
            data_process_mutex_.unlock();
            return true;


        }

        /**
         * Set evaluation sigma.
         * @param eval_sigma
         * @return
         */
        bool setEvaluSigma(double eval_sigma) {
            if (abs(eval_sigma - R_(0, 0)) < 0.01) {
                return true;
            }

            data_process_mutex_.lock();
            R_.setIdentity();
            R_ = R_ * eval_sigma;
            data_process_mutex_.unlock();
            return (true);
        }




    protected:
        Eigen::Matrix<T, state_num, state_num> Q_;//process noise

        Eigen::Matrix<T, observe_num, observe_num> R_;//measurement noise


        Eigen::Matrix<T, state_num, state_num> P_; // covariance matrix to state

        Eigen::Matrix<T, state_num, 1> X_; // Last state(posterior)

        Eigen::Matrix<T, state_num, state_num> A_; // state transition matrix

        Eigen::Matrix<T, observe_num, state_num> C_; // observation matrix

        Eigen::Matrix<T, state_num, observe_num> K_; // Kalman gain.

    private:
/** save state before compute prior and use
         * to compute velocity after compute posterior.
         */
        Eigen::Matrix<T, state_num, 1> predict_x_;


        /**
        *  save state before compute prior and use to
        *  compute velocity after compute posterior.
        */
        Eigen::Matrix<T, state_num, 1> last_X_;

        /**
         * keep data safe between several thread.
         */
        std::mutex data_process_mutex_;


    };
}


#endif //CVREADER_KALMANFILTER_HPP
