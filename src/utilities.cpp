#include <utilities.hpp>

Utilities::Utilities(){
}

Utilities::~Utilities() {
}

void Utilities::getOdometryMeasurement(const nav_msgs::Odometry &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, double twoDimensionalMode){
    
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set all Measured Variables
    measurement(0)= msg.pose.pose.position.x;
    measurement(1)= msg.pose.pose.position.y;
    measurement(2)= msg.pose.pose.position.z;  
    measurement(3)= roll;
    measurement(4)= pitch;
    measurement(5)= yaw;
    measurement(6)= msg.twist.twist.linear.x;
    measurement(7)= msg.twist.twist.linear.y;
    measurement(8)= msg.twist.twist.linear.z;
    measurement(9)= msg.twist.twist.angular.x;
    measurement(10)= msg.twist.twist.angular.y;
    measurement(11)= msg.twist.twist.angular.z;

    // Set all Covariances
    measurementCovariance(0) = msg.pose.covariance[0];
    measurementCovariance(1) = msg.pose.covariance[7];
    measurementCovariance(2) = msg.pose.covariance[14];
    measurementCovariance(3) = msg.pose.covariance[21];
    measurementCovariance(4) = msg.pose.covariance[28];
    measurementCovariance(5) = msg.pose.covariance[35];
    measurementCovariance(6) = msg.twist.covariance[0];
    measurementCovariance(7) = msg.twist.covariance[7];
    measurementCovariance(8) = msg.twist.covariance[14];
    measurementCovariance(9) = msg.twist.covariance[21];
    measurementCovariance(10) = msg.twist.covariance[28];
    measurementCovariance(11) = msg.twist.covariance[35];

    if(twoDimensionalMode){
        // Set all Measured Variables
        measurement(2)= 0;  
        measurement(3)= 0;
        measurement(4)= 0;
        measurement(7)= 0;
        measurement(8)= 0;
        measurement(9)= 0;
        measurement(10)= 0;

        // Set all Covariances
        measurementCovariance(2) = 0;
        measurementCovariance(3) = 0;
        measurementCovariance(4) = 0;
        measurementCovariance(7) = 0;
        measurementCovariance(8) = 0;
        measurementCovariance(9) = 0;
        measurementCovariance(10) = 0;
    }

}

void Utilities::getAccelerationMeasurement(const sensor_msgs::Imu &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance, double twoDimensionalMode){

	// Accelerations reported by IMU
	tf2::Vector3 acceleration(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

	// Accelerations expected by an IMU in IDLE state
	tf2::Vector3 idleAcc(0, 0, gravitationalAcceleration_);

	tf2::Transform idle;

  // If IMU does not provide Orientations, indicated by -1 in 1st element of covariance matrix
	if(msg.orientation_covariance[0] == -1)
	{
      const Eigen::VectorXd &state = ekf_.getStates();
      tf2::Matrix3x3 stateTmp;
      stateTmp.setRPY(state(3), state(4), state(5));
      idle.setBasis(stateTmp);
	}
	// If IMU Provides Orientation
	else{
		// Current Orientation of IMU in NEU frame
		tf2::Quaternion Attitude;
		// tf2::fromMsg(msg.orientation, Attitude);
        Attitude[0]=msg.orientation.x;
        Attitude[1]=msg.orientation.y;
        Attitude[2]=msg.orientation.z;
        Attitude[3]=msg.orientation.w;

		// Normalize Quaternion if not 
		if (fabs(Attitude.length() - 1.0) > 0.01) {
		    ROS_WARN_ONCE("An input was not normalized, this should NOT happen, but will normalize.");
		    Attitude.normalize();
		}
		idle.setRotation(Attitude);
	}

	// Rotate the gravitational acceleration in idel state and subtract it from actual IMU acceleraions
	tf2::Vector3 rotNorm = idle.getBasis().inverse() * idleAcc;
  acceleration.setX(acceleration.getX() - rotNorm.getX());
  acceleration.setY(acceleration.getY() - rotNorm.getY());
  acceleration.setZ(acceleration.getZ() - rotNorm.getZ());

  tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Set all Measured Variables  
  measurement(3)= roll;
  measurement(4)= pitch;
  measurement(5)= yaw;
  measurement(9)= msg.angular_velocity.x;
  measurement(10)= msg.angular_velocity.y;
  measurement(11)= msg.angular_velocity.z;
  measurement(12)= acceleration[0];
  measurement(13)= acceleration[1];
  measurement(14)= acceleration[2];

  // Set all Covariances
  measurementCovariance(3) = msg.orientation_covariance[0];
  measurementCovariance(4) = msg.orientation_covariance[4];
  measurementCovariance(5) = msg.orientation_covariance[8];
  measurementCovariance(9) = msg.angular_velocity_covariance[0];
  measurementCovariance(10) = msg.angular_velocity_covariance[4];
  measurementCovariance(11) = msg.angular_velocity_covariance[8];
  measurementCovariance(12) = msg.linear_acceleration_covariance[0];
  measurementCovariance(13) = msg.linear_acceleration_covariance[4];
  measurementCovariance(14) = msg.linear_acceleration_covariance[8];

  if(twoDimensionalMode){
    // Set all Measured Variables  
    measurement(3)= 0;
    measurement(4)= 0;
    measurement(9)= 0;
    measurement(10)= 0;
    measurement(13)= 0;
    measurement(14)= 0;

    // Set all Covariances
    measurementCovariance(3) = 0;
    measurementCovariance(4) = 0;
    measurementCovariance(9) = 0;
    measurementCovariance(10) = 0;
    measurementCovariance(13) = 0;
    measurementCovariance(14) = 0;
  }

}
