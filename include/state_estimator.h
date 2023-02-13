//
// Created by ccg on 23-2-6.
//

#ifndef ESTIMATOR_STATE_ESTIMATOR_H
#define ESTIMATOR_STATE_ESTIMATOR_H

#include "encoder.h"
#include "encoder_factor.h"
#include "imu_factor.h"
#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <queue>
#include <memory>
#include <mutex>
#include <thread>
#include <mutex>
#include <condition_variable>

class Estimator {

public:
    Estimator(ros::NodeHandle &nh) {
        imu_sub = nh.subscribe("/imu/data_raw", 10, &Estimator::imuCallBack, this);
        encoder_sub = nh.subscribe("/encoder_count", 10, &Estimator::EncoderCallBack, this);
        Process();
    };
public:
    void Predict(const sensor_msgs::ImuConstPtr &imu_msg, double timestamp);

    void imuCallBack(const sensor_msgs::ImuConstPtr &imu_msg);

    void EncoderCallBack(const irp_sen_msgs::encoderConstPtr &enc_msgs);

    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, irp_sen_msgs::encoderConstPtr>> getMeasurement();

    void
    ProcessIMUandEncoder(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity,
                         const Eigen::Vector3d &encoder_velocity);

    void Process();

private:
    //state
    Eigen::Vector3d Position;
    Eigen::Vector3d Velocity;
    Eigen::Quaterniond Rot;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyro_0;
    Eigen::Vector3d enc_v_0;
    Eigen::Vector3d gravity;
    Eigen::Vector3d RIO;//
    Eigen::Vector3d TIO;

    std::vector<Eigen::Vector3d> Positions;
    std::vector<Eigen::Vector3d> Velocitys;
    std::vector<Eigen::Quaterniond> Rots;

    std::queue<sensor_msgs::ImuConstPtr> imu_buff;
    std::queue<irp_sen_msgs::encoderConstPtr> enc_buff;
    std::queue<sensor_msgs::NavSatFixConstPtr> gps_buff;

    Eigen::Matrix<double, 15, 15> cov;
    Eigen::Matrix<double, 15, 15> Fx;
    Eigen::Matrix<double, 15, 12> Fi;
    Eigen::Matrix<double, 12, 12> Qi;

    double last_imu_t;
    double last_encoder_t;

    std::mutex m_lock;
    std::condition_variable m_con;
    EncoderFactor *encoder;
    IMUFactor *imu;
    ros::Subscriber imu_sub, encoder_sub;

};


#endif //ESTIMATOR_STATE_ESTIMATOR_H
