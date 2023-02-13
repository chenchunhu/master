//
// Created by ccg on 23-2-7.
//

#ifndef ESTIMATOR_IMU_FACTOR_H
#define ESTIMATOR_IMU_FACTOR_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "utity.h"

class IMUFactor {
public:

    IMUFactor(double dt, Eigen::Vector3d &un_acc, Eigen::Vector3d &un_gyro) : un_acc_(std::move(un_acc)),
                                                                              un_gyro_(std::move(un_gyro)) {
        midValue(dt, un_acc_, un_gyro_);
    }

    void midValue(double dt, const Eigen::Vector3d &acc_unbias, const Eigen::Vector3d &gyro_unbias) {
        //mid intergration
//        Eigen::Vector3d un_gyro = 0.5 * (gyro_1 + gyro_0) - gyro_bias;
//        result_delta_q =
//                delta_q * Eigen::Quaterniond(1, un_gyro(0) * dt_ / 2, un_gyro(1) * dt_ / 2, un_gyro(2) * dt_ / 2);
//        Eigen::Vector3d un_acc_0 = delta_q * (acc_0 - acc_bias);
//        Eigen::Vector3d un_acc_1 = result_delta_q * (acc_1 - acc_bias);
//        result_delta_v = delta_v + 0.5 * (un_acc_1 + un_acc_0) * dt_;
//        result_delta_p = delta_p + delta_v * dt_ + 0.5 * (un_acc_1 + un_acc_0) * dt_ * dt_;

        Fx.setIdentity();
        Fi.setZero();
        Qi.setZero();
        Eigen::Quaterniond w_q_i(1, gyro_unbias(0) * dt / 2, gyro_unbias(1) * dt / 2, gyro_unbias(2) * dt / 2);
        Fx.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity() * dt;
        Fx.block<3, 3>(3, 6) = w_q_i.toRotationMatrix() * GetSkewMatrix(acc_unbias) * dt;
        Fx.block<3, 3>(3, 9) = w_q_i.toRotationMatrix() * dt;
        Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd((gyro_unbias * dt).norm(),
                                                 (gyro_unbias * dt).normalized()).toRotationMatrix().transpose();
        Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

        Qi.block<3, 3>(0, 0) = dt * dt * acc_noise_ * acc_noise_ * Eigen::Matrix3d::Identity();
        Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise_ * gyro_noise_ * Eigen::Matrix3d::Identity();
        Qi.block<3, 3>(6, 6) = acc_bias_noise_ * acc_bias_noise_ * dt * Eigen::Matrix3d::Identity();
        Qi.block<3, 3>(9, 9) = gyro_bias_noise_ * gyro_bias_noise_ * dt * Eigen::Matrix3d::Identity();

    }


private:
    Eigen::Vector3d un_acc_;
    Eigen::Vector3d un_gyro_;

    Eigen::Matrix<double, 15, 15> Fx;
    Eigen::Matrix<double, 12, 15> Fi;
    Eigen::Matrix<double, 12, 12> Qi;

    double acc_noise_;
    double gyro_noise_;
    double acc_bias_noise_;
    double gyro_bias_noise_;

};

#endif //ESTIMATOR_IMU_FACTOR_H
