#include <setMeasurementCovariance.hpp>

SetMeasurementCovariance::SetMeasurementCovariance(ros::NodeHandle& nh, bool imu) : SetMeasurementCovariance_(nh),imu_(imu) {
  imuSub = SetMeasurementCovariance_.subscribe("/imu", 1, &SetMeasurementCovariance::imuCallback, this);
  imuPub = SetMeasurementCovariance_.advertise<sensor_msgs::Imu>("/imu/data",1);
  odomSub = SetMeasurementCovariance_.subscribe("/odom", 1, &SetMeasurementCovariance::odomCallback, this);
  odomPub = SetMeasurementCovariance_.advertise<nav_msgs::Odometry>("/odom/data",1);
}

SetMeasurementCovariance::~SetMeasurementCovariance(){
}

void SetMeasurementCovariance::odomCallback(const nav_msgs::Odometry msg){
  odomData_ = msg;
  odomData_.header.frame_id = "map";
  odomData_.pose.covariance[0] = 1e-3;     // x
  odomData_.pose.covariance[7] = 1e-3;     // y
  odomData_.pose.covariance[14] = 1e-300;    // z
  odomData_.pose.covariance[21] = 1e-3;    // roll
  odomData_.pose.covariance[28] = 1e-3;    // pitch
  odomData_.pose.covariance[35] = 1e-3;    // yaw
  odomData_.twist.covariance[0] = 1e-3;    // vx- Used
  odomData_.twist.covariance[7] = 1e-300;    // vy- Used
  odomData_.twist.covariance[14] = 1e-3;   // vz
  odomData_.twist.covariance[21] = 1e-3;   // omegax
  odomData_.twist.covariance[28] = 1e-3;   // omegay
  odomData_.twist.covariance[35] = 1e-3;   // omegaz- Used
  odomPub.publish(odomData_);
}

void SetMeasurementCovariance::imuCallback(const sensor_msgs::Imu msg){
  imuData = msg;
  imuData.orientation_covariance[0] = 1e-2;           // roll 
  imuData.orientation_covariance[4] = 1e-2;           // pitch
  imuData.orientation_covariance[8] = 1e-2;           // yaw- Used
  imuData.linear_acceleration_covariance[0] = 1e-2;   // ax- Used
  imuData.linear_acceleration_covariance[4] = 1e-2;   // ay
  imuData.linear_acceleration_covariance[8] = 1e-2;   // az
  imuData.angular_velocity_covariance[0] = 1e-2;      // omegax
  imuData.angular_velocity_covariance[4] = 1e-2;      // omegay
  imuData.angular_velocity_covariance[8] = 1e-2;      // omegaz- Used
  imuPub.publish(imuData);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "covariance");
    ros::NodeHandle nh;
    SetMeasurementCovariance c(nh, true);
    while(ros::ok()){
      ros::spinOnce();
    }
    return 0;
}

