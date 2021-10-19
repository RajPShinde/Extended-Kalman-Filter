#include <interface.hpp>

Interface::Interface(ros::NodeHandle& nh, bool imu, bool odometry): interface_(nh), useIMU_(imu), useOdometry_(odometry) {
    imuSub_ = interface_.subscribe("/imu/data", 1, &Interface::callbackIMU, this);
    odometrySub_ = interface_.subscribe("/odom", 1, &Interface::callbackOdometry, this);
    odometryPub_ = interface_.advertise<nav_msgs::Odometry>("/odometry/fused",10);
    fuseData();
}

Interface::~Interface() {
}

void Interface::callbackIMU(const sensor_msgs::Imu &msg){
    Eigen::VectorXd measurement(stateSize_);
    Eigen::VectorXd measurementCovariance(stateSize_);
    measurement.setZero();
    measurementCovariance.setZero();
    data_.getAccelerationMeasurement(msg, ekf_, measurement, measurementCovariance, twoDimensionalMode);
    storeMeasurement(measurement, measurementCovariance, dataIMU);
    firstMeasurement = true;
}

void Interface::callbackOdometry(const nav_msgs::Odometry &msg){
    Eigen::VectorXd measurement(stateSize_);
    Eigen::VectorXd measurementCovariance(stateSize_);
    measurement.setZero();
    measurementCovariance.setZero();
    data_.getOdometryMeasurement(msg, ekf_, measurement, measurementCovariance, twoDimensionalMode);
    storeMeasurement(measurement, measurementCovariance, dataOdometry);
    firstMeasurement = true;
}

void Interface::storeMeasurement(Eigen::VectorXd &measurements, Eigen::VectorXd &measurementCovariances, std::vector<int> data){
    sensor_.measurements = measurements;
    sensor_.measurementCovariances = measurementCovariances;
    sensor_.dataToUse = data;
    sensorMeasurements_.push_back(sensor_);
    ekf_.correct(sensor_);
}

void Interface::fuseData(){
    double frequency = 10;
    ros::Rate loop(frequency);
    ROS_INFO_STREAM("Fusing");
    while(ros::ok()){
        if(firstMeasurement) {
            double dt = 1/frequency;
            ekf_.predict(dt, twoDimensionalMode);
            Eigen::VectorXd states = ekf_.getStates();
            publishOdometry(states);
            publishTransform(states);
            sensorMeasurements_.clear();
            loop.sleep();
        }
        ros::spinOnce();
    }
}

void Interface::publishOdometry(Eigen::VectorXd states){
    nav_msgs::Odometry odometry;
    odometry.header.frame_id="map";
    odometry.header.stamp=ros::Time::now();
    odometry.child_frame_id="base_link";
    odometry.pose.pose.position.x = states[0];
    odometry.pose.pose.position.y = states[1];
    odometry.pose.pose.position.z = states[2];

    odometry.twist.twist.linear.x = states[6];
    odometry.twist.twist.linear.y = states[7];
    odometry.twist.twist.linear.z = states[8];
    odometry.twist.twist.angular.x = states[9];
    odometry.twist.twist.angular.y = states[10];
    odometry.twist.twist.angular.z = states[11];

    tf2::Quaternion q;
    q.setRPY( states[3], states[4], states[5]);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();

    Eigen::MatrixXd estimateErrorCovariance_ = ekf_.getEstimateErrorCovariance();
    odometry.pose.covariance[0]=estimateErrorCovariance_(0, 0);
    odometry.pose.covariance[7]=estimateErrorCovariance_(1, 1);
    odometry.pose.covariance[14]=estimateErrorCovariance_(2, 2);   
    odometry.pose.covariance[21]=estimateErrorCovariance_(3, 3);
    odometry.pose.covariance[28]=estimateErrorCovariance_(4, 4);
    odometry.pose.covariance[35]=estimateErrorCovariance_(5, 5);
    odometry.twist.covariance[0]=estimateErrorCovariance_(6, 6);
    odometry.twist.covariance[7]=estimateErrorCovariance_(7, 7);
    odometry.twist.covariance[14]=estimateErrorCovariance_(8, 8);   
    odometry.twist.covariance[21]=estimateErrorCovariance_(9, 9);
    odometry.twist.covariance[28]=estimateErrorCovariance_(10, 10);
    odometry.twist.covariance[35]=estimateErrorCovariance_(11, 11); 
    odometryPub_.publish(odometry);

}
void Interface::publishTransform(Eigen::VectorXd state){
    // Publish Transform from Odom to base_link
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = state(0);
    transformStamped.transform.translation.y = state(1);
    transformStamped.transform.translation.z = state(2);
    tf2::Quaternion q;
    q.setRPY(state(3), state(4), state(5));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}