#include <utilities.hpp>

Utilities::Utilities(){
}

Utilities::~Utilities() {
}

// void Utilities::getOdometryMeasurement(){
// }

void Utilities::getAccelerationMeasurement(const sensor_msgs::Imu &msg, ExtendedKalmanFilter &ekf_, Eigen::VectorXd &measurement, Eigen::VectorXd &measurementCovariance){

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
  measurementCovariance(9) = msg.linear_acceleration_covariance[0];
  measurementCovariance(10) = msg.linear_acceleration_covariance[4];
  measurementCovariance(11) = msg.linear_acceleration_covariance[8];
  measurementCovariance(12) = msg.angular_velocity_covariance[0];
  measurementCovariance(13) = msg.angular_velocity_covariance[4];
  measurementCovariance(14) = msg.angular_velocity_covariance[8];

}
