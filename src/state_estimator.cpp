#include "state_estimator.h"

/*
 * void Estimator::Predict(const sensor_msgs::ImuConstPtr &imu_msg, double timestamp) {
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;

    Eigen::Vector3d linear(dx, dy, dz);

    double rx = imu_msg->linear_acceleration.x;
    double ry = imu_msg->linear_acceleration.y;
    double rz = imu_msg->linear_acceleration.z;

    Eigen::Vector3d angular(rx, ry, rz);

    Eigen::Vector3d un_gyro = 0.5 * (gyro_0 + angular) - gyro_bias;
    //TODO
    Rot *= Eigen::Quaterniond(1, un_gyro(0) * 2 / timestamp, un_gyro(1) * 2 / timestamp,
                              un_gyro(2) * 2 / timestamp);

    Eigen::Vector3d un_acc_0 = Rot * (acc_0 - acc_bias);
    Eigen::Vector3d un_acc_1 = Rot * (linear - acc_bias);
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    Position += Velocity * timestamp + 0.5 * (un_acc_0 + un_acc_0 - gravity) * 0.5 * timestamp * timestamp;
    Velocity += 0.5 * (un_acc_0 + un_acc_0 - gravity) * 0.5 * timestamp * timestamp;
    double acc_noise = 0.1, gyro_noise = 0.1, acc_bias_noise = 0.01, gyro_bias_noise = 0.01;
    IMUFactor *imuFactor = new IMUFactor(acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise);
    imuFactor->midValue(timestamp, un_acc, un_gyro, Rot, Fx, Qi, Fi);
    delete imuFactor;
}
 */

void Estimator::imuCallBack(const sensor_msgs::ImuConstPtr &imu_msg) {
    double dt = imu_msg->header.stamp.toSec();
    if (dt < last_imu_t) {
        ROS_WARN("time disorder");
        return;
    }
    double delta_t = dt - last_imu_t;
    last_imu_t = dt;
    imu_buff.push(imu_msg);

}

void Estimator::EncoderCallBack(const irp_sen_msgs::encoderConstPtr &enc_msgs) {
    double cur_enc_t = enc_msgs->header.stamp.toSec();
    if (cur_enc_t < last_encoder_t) {
        ROS_WARN("time disorder");
        return;
    }
    enc_buff.push(enc_msgs);
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, irp_sen_msgs::encoderConstPtr>>
Estimator::getMeasurement() {
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, irp_sen_msgs::encoderConstPtr>> measurment;
    while (true) {
        if (imu_buff.empty() || enc_buff.empty()) {
            return measurment;
        }
        if (imu_buff.back()->header.stamp.toSec() <= enc_buff.front()->header.stamp.toSec()) {
            ROS_INFO("wait for imu ");
            return measurment;
        }

        if (imu_buff.front()->header.stamp.toSec() >= enc_buff.front()->header.stamp.toSec()) {
            ROS_WARN("throw encoder");
            enc_buff.pop();
            continue;
        }
        std::vector<sensor_msgs::ImuConstPtr> imus;
        irp_sen_msgs::encoderConstPtr encoder_msg = enc_buff.front();
        enc_buff.pop();
        std::cout << "imu_buff.front()->header.stamp.toSec() is" << imu_buff.front()->header.stamp.toSec()
                  << "encoder_msg->header.stamp.toSec()" << encoder_msg->header.stamp.toSec() << std::endl;
        while (imu_buff.front()->header.stamp.toSec() < encoder_msg->header.stamp.toSec()) {
            imus.emplace_back(imu_buff.front());
            imu_buff.pop();
        }
        imus.emplace_back(imu_buff.front());
        if (imus.empty()) {
            std::cerr << "no imu between two encoder" << std::endl;
        }
        measurment.emplace_back(imus, encoder_msg);

    }
    return measurment;
}

void Estimator::Process() {
    ROS_WARN("start main process");
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, irp_sen_msgs::encoderConstPtr>> measurment;
    ROS_WARN("lock");
//    std::unique_lock<std::mutex> lk(m_lock);
//    m_con.wait(lk, [&] {
//        return !(measurment = getMeasurement()).empty();
//    });
    m_lock.lock();
    measurment = getMeasurement();
    if (measurment.empty()) {
        return;
    }
    ROS_WARN("unlock");
    m_lock.unlock();
    bool init_enc = true;
    std::vector<std::pair<double, Eigen::Vector3d>> encoder_velocities;
    Eigen::AngleAxisd w_rot_o;
    Eigen::Vector3d enc_vel;
    for (auto &iter: measurment) {
        auto encoder_msg = iter.second;
        double last_enc_t, last_left_encoder, last_right_encoder, enc_v;

        if (init_enc) {
            last_right_encoder = encoder_msg->right_count;
            last_enc_t = encoder_msg->header.stamp.toSec();
            enc_v = 0;
            enc_vel << enc_v, 0, 0;
            encoder_velocities.emplace_back(encoder_msg->header.stamp.toSec(), enc_vel);
            init_enc = false;
            continue;
        }

        double dt = encoder_msg->header.stamp.toSec() - last_enc_t;
        //TODO
        double enc_vel_left = (encoder_msg->left_count - last_right_encoder) / dt;
        double enc_vel_right = (encoder_msg->left_count - last_right_encoder) / dt;
        //TODO
        double yaw = enc_vel_left - enc_vel_right / 1.6;
        w_rot_o = Eigen::AngleAxisd(yaw * dt, Eigen::Vector3d::UnitY());
        enc_v = 0.5 * (enc_vel_left + enc_vel_right);
        enc_vel << enc_v, 0, 0;
        enc_vel = w_rot_o.toRotationMatrix() * enc_vel;
        encoder_velocities.emplace_back(dt, enc_vel);

        for (auto &imu_msg: iter.first) {
            double t = imu_msg->header.stamp.toSec();
            double encoder_t = encoder_msg->header.stamp.toSec();
            double t_1 = 0, t_2 = 0;
            Eigen::Vector3d encoder_velocity;
            if (!encoder_velocities.empty()) {
                encoder_velocity = encoder_velocities[0].second;
            } else
                //TODO
                encoder_velocity = Velocity;

            std::pair<double, Eigen::Vector3d> enc_vel_0, enc_vel_1;

            for (const auto &encoder_vel: encoder_velocities) {
                // 寻找最近的小于imu消息时间的encoder消息
                if (encoder_vel.first <= t) {
                    t_1 = encoder_vel.first;
                    enc_vel_0 = encoder_vel;
                } else {
                    t_2 = encoder_vel.first;
                    enc_vel_1 = encoder_vel;
                    break;
                }
            }

            if (t_1 > 0 && t_2 > 0) {
                double dt_1 = t_2 - t;
                double dt_2 = t - t_1;
                ROS_ASSERT(dt_1 >= 0);
                ROS_ASSERT(dt_2 >= 0);
                ROS_ASSERT(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                encoder_velocity = w1 * enc_vel_0.second + w2 * enc_vel_1.second;
            }

            double _dt = t - last_imu_t;
            double dx = imu_msg->linear_acceleration.x;
            double dy = imu_msg->linear_acceleration.y;
            double dz = imu_msg->linear_acceleration.z;
            double rx = imu_msg->angular_velocity.x;
            double ry = imu_msg->angular_velocity.y;
            double rz = imu_msg->angular_velocity.z;
            Eigen::Vector3d linear(dx, dy, dz), angular(rx, ry, rz);
            ProcessIMUandEncoder(dt, linear, angular, encoder_velocity);
        }

    }
}

void Estimator::ProcessIMUandEncoder(double dt, const Eigen::Vector3d &linear_acceleration,
                                     const Eigen::Vector3d &angular_velocity, const Eigen::Vector3d &encoder_velocity) {
    bool first_imu = true;
    if (true) {
        first_imu = false;
        acc_0 = linear_acceleration;
        gyro_0 = angular_velocity;
        enc_v_0 = encoder_velocity;
    }
    Eigen::Vector3d un_gyro = 0.5 * (angular_velocity + gyro_0) - gyro_bias;

    Eigen::Vector3d un_acc_0 = Rot.toRotationMatrix() * (acc_0 - acc_bias) - gravity;
    Rot = Rot * Eigen::Quaterniond(1, un_gyro(0) / 2 * dt, un_gyro(1) / 2 * dt, un_gyro(2) / 2 * dt);
    Eigen::Vector3d un_acc_1 = Rot.toRotationMatrix() * (linear_acceleration - acc_bias) - gravity;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Eigen::Vector3d un_env = 0.5 * (encoder_velocity + enc_v_0);
    Rot.normalize();

    //TODO
    encoder = new EncoderFactor(dt, Position, un_env);
    imu = new IMUFactor(dt, un_acc, un_gyro);

    Position = Position + Velocity + 0.5 * un_acc * dt * dt;
    Velocity = Velocity + un_acc * dt;

    acc_0 = linear_acceleration;
    gyro_0 = angular_velocity;
    enc_v_0 = encoder_velocity;

}
