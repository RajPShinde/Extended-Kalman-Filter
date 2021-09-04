#include <interface.hpp>

Interface::Interface(ros::NodeHandle& nh, bool imu, bool odometry): interface_(nh), useIMU_(imu), useOdometry_(odometry) {
    totalSources_ = (useIMU_ && useOdometry_) ? 2 : 1;
}

Interface::~Interface() {
}

void Interface::getMeasurement(){
}

void Interface::fuseData(){
    while(ros::ok()){
        ekf.predict();
        ekf.update();
    }
}

void Interface::removeIMUGravitationalAcceleration(){
}

void Interface::publishTransform(){
    if(visualizeModel_){
        view.visualizeModel();
    }
}