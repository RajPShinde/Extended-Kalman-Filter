#ifndef INCLUDE_COMPUTECOVARIANCE_HPP_
#define INCLUDE_COMPUTECOVARIANCE_HPP_

#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

class ComputeCovariances
{
    public:
        ros::Publisher imuPub;
        ros::Subscriber imuSub;

        /**
        *   @brief Constructor of ComputeCovariances
        *   @param none
        *   @return none
        */
        ComputeCovariances(ros::NodeHandle& nh, bool imu);

        /**
        *   @brief Destructor of ComputeCovariances
        *   @param none
        *   @return none
        */
        ~ComputeCovariances();

        void imuCallback(const sensor_msgs::Imu msg);
        
    private:
        double imu_;
        ros::NodeHandle computeCovariances_;
        sensor_msgs::Imu imuData;
};

#endif  //  INCLUDE_COMPUTECOVARIANCE_HPP_