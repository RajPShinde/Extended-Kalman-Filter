#ifndef INCLUDE_UTILITIES_HPP_
#define INCLUDE_UTILITIES_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <extendedKalmanFilter.hpp>

class Utilities
{
    public:

        /**
        *   @brief Constructor of Utilities
        *   @param none
        *   @return none
        */
        Utilities();

        /**
        *   @brief Destructor of Utilities
        *   @param none
        *   @return none
        */
        ~Utilities();

        void getOdometryMeasurement(const nav_msgs::Odometry &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, double twoDimensionalMode);

        void getAccelerationMeasurement(const sensor_msgs::Imu &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, double twoDimensionalMode);

    private:

        double gravitationalAcceleration_ = 9.80665; // m/s^2

};

#endif  //  INCLUDE_UTILITIES_HPP_