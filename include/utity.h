//
// Created by ccg on 23-2-8.
//

#ifndef ESTIMATOR_UTITY_H
#define ESTIMATOR_UTITY_H

#include <eigen3/Eigen/Core>

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> GetSkewMatrix(const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), -q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

#endif //ESTIMATOR_UTITY_H
