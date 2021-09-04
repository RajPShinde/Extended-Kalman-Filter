#ifndef INCLUDE_INTERFACE_HPP_
#define INCLUDE_INTERFACE_HPP_

#include <extendedKalmanFilter.hpp>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization.hpp>
#include <measurement.hpp>
#include <utilities.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

class Interface
{
    public:

        /**
        *   @brief Constructor of Interface
        *   @param none
        *   @return none
        */
        Interface(ros::NodeHandle& nh, bool imu, bool odometry);

        /**
        *   @brief Destructor of Interface
        *   @param none
        *   @return none
        */
        ~Interface();

        void getMeasurement();

        void storeMeasurement(Eigen::VectorXd &measurement,  Eigen::VectorXd &measurementCovariance, std::vector<int> data);

        void fuseData();

        void publishOdometry(Eigen::VectorXd states);

        void publishTransform(Eigen::VectorXd states);

        void callbackIMU(const sensor_msgs::Imu &msg);
        
        // void callbackOdometry(nav_msgs::Odometry &msg);

    private:

        ros::NodeHandle interface_;
        ExtendedKalmanFilter ekf_;
        Visualization view_;
        Utilities data_;
        bool visualizeModel_, publishTransform_, useIMU_, useOdometry_ = true;
        int queueIMU_, queueOdometry_ = 1;
        ros::Subscriber odometrySub_;
        ros::Subscriber imuSub_;
        ros::Publisher odometryPub_;
        int stateSize_ = 15;
        std::vector<Measurement> sensorMeasurements_;
        Measurement sensor_;
        std::vector<int> dataIMU = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};  // x y z r p y vx vy vz vr vp vy ax ay az
        std::vector<int> dataOdometry = {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0}; // x y z r p y vx vy vz vr vp vy ax ay az
        bool firstMeasurement = false;
};

#endif  //  INCLUDE_INTERFACE_HPP_