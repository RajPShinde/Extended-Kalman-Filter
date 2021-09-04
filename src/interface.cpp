#include <interface.hpp>

Interface::Interface(ros::NodeHandle& nh, bool imu, bool odometry): interface_(nh), useIMU_(imu), useOdometry_(odometry) {
    ROS_WARN_STREAM("Interface");
    imuSub_ = interface_.subscribe("/imu/data", 1, &Interface::callbackIMU, this);
    // odomSub_ = interface_.subscribe("/odom", queueOdometry_, &Interface::callbackOdometry, this);
    odometryPub_ = interface_.advertise<nav_msgs::Odometry>("/odometry/fused",10);
    fuseData();
}

Interface::~Interface() {
}

void Interface::callbackIMU(const sensor_msgs::Imu &msg){
    ROS_WARN_STREAM("IMU Callback");
    Eigen::VectorXd measurement(stateSize_);
    Eigen::VectorXd measurementCovariance(stateSize_);
    measurement.setZero();
    measurementCovariance.setZero();
    data_.getAccelerationMeasurement(msg, ekf_, measurement, measurementCovariance);
    storeMeasurement(measurement, measurementCovariance, dataIMU);
    firstMeasurement = true;
}

// void Interface::callbackOdometry(nav_msgs::Odometry &msg){
//     Eigen::VectorXd measurement(stateSize_);
//     Eigen::MatrixXd measurementCovariance(stateSize_, stateSize_);
//     measurement.setZero();
//     measurementCovariance.setZero();
//     data_.getOdometryMeasurement(msg, ekf_, measurement, measurementCovariance);
//     storeMeasurement(measurement, measurementCovariance, dataOdometry);
// }

void Interface::storeMeasurement(Eigen::VectorXd &measurements, Eigen::VectorXd &measurementCovariances, std::vector<int> data){
    sensor_.measurements = measurements;
    sensor_.measurementCovariances = measurementCovariances;
    sensor_.dataToUse = data;
    sensorMeasurements_.push_back(sensor_);
}

void Interface::fuseData(){
}
