#include <computeCovariance.hpp>

ComputeCovariances::ComputeCovariances(ros::NodeHandle& nh, bool imu) : computeCovariances_(nh),imu_(imu) {
  imuSub = computeCovariances_.subscribe("/imu", 1, &ComputeCovariances::imuCallback, this);
  imuPub = computeCovariances_.advertise<sensor_msgs::Imu>("/imu/data",1);
}

ComputeCovariances::~ComputeCovariances(){
}

void ComputeCovariances::imuCallback(const sensor_msgs::Imu msg){
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
    ComputeCovariances c(nh, true);
    while(ros::ok()){
      ros::spinOnce();
    }
    return 0;
}

