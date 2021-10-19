#ifndef INCLUDE_SETMEASUREMENTCOVARIANCE_HPP_
#define INCLUDE_SETMEASUREMENTCOVARIANCE_HPP_

#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class SetMeasurementCovariance
{
    public:
        ros::Publisher imuPub;
        ros::Publisher odomPub;
        ros::Subscriber imuSub;
        ros::Subscriber odomSub;

        /**
        *   @brief Constructor of SetMeasurementCovariance
        *   @param none
        *   @return none
        */
        SetMeasurementCovariance(ros::NodeHandle& nh, bool imu);

        /**
        *   @brief Destructor of SetMeasurementCovariance
        *   @param none
        *   @return none
        */
        ~SetMeasurementCovariance();

        void imuCallback(const sensor_msgs::Imu msg);

        void odomCallback(const nav_msgs::Odometry msg);
        
    private:
        double imu_;
        ros::NodeHandle SetMeasurementCovariance_;
        sensor_msgs::Imu imuData;
        nav_msgs::Odometry odomData_;
};

#endif  //  INCLUDE_SETMEASUREMENTCOVARIANCE_HPP_