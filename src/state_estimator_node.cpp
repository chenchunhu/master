//
// Created by ccg on 23-2-9.
//
#include <ros/ros.h>
#include "state_estimator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh;
    Estimator estimator(nh);
 //   std::thread thread(&Estimator::Process, &estimator);
    ros::spin();
    return 1;

}