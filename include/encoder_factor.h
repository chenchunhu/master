//
// Created by ccg on 23-2-7.
//

#ifndef ESTIMATOR_ENCODER_FACTOR_H
#define ESTIMATOR_ENCODER_FACTOR_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <utility>

class EncoderFactor {
public:
    EncoderFactor(double dt, const Eigen::Vector3d &Pwi, Eigen::Vector3d encoder_velocity) : enc_v_(
            std::move(encoder_velocity)) {

        Eigen::Vector3d Pwo;
        Pwo = Pwo + encoder_velocity * dt;
        Update(Pwo, Pwi);
    }

    void Update(const Eigen::Vector3d &Pwo, const Eigen::Vector3d &Pwi) {
        Eigen::Matrix<double, 15, 15> cov;
        Eigen::Matrix<double, 3, 15> jacobian;
        ComputeJacobian(cov, Pwi, Pwo);
    }

    void AddDeltaXtoState(const Eigen::Matrix<double, 15, 1> &delta_x) {
        Eigen::Matrix<double, 15, 1> state;
        state.block<3, 1>(0, 0) += delta_x.block<3, 1>(0, 0);
        state.block<3, 1>(3, 0) += delta_x.block<3, 1>(3, 0);
        state.block<3, 1>(6, 0) += delta_x.block<3, 1>(6, 0);
        state.block<3, 1>(9, 0) += delta_x.block<3, 1>(9, 0);
        state.block<3, 1>(12, 0) += delta_x.block<3, 1>(12, 0);

    }

    void
    ComputeJacobian(Eigen::Matrix<double, 15, 15> &cov, const Eigen::Vector3d &Pwi, const Eigen::Vector3d Pwo) {
        residuals = Pwi - Pwo;
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 15, 3> K = cov * H.transpose() * ((H * cov * H.transpose() + cov_wheel).inverse());
        Eigen::Matrix<double, 15, 1> delta_x = K * residuals;
        cov = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * cov;
        AddDeltaXtoState(delta_x);
    }

private:
    double wheel_noise;
    Eigen::Vector3d enc_v_;
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residuals;
    Eigen::Matrix<double, 3, 3> cov_wheel;

};


#endif //ESTIMATOR_ENCODER_FACTOR_H
