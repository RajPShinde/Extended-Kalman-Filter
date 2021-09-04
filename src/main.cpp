#include <interface.hpp>
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "Extended Kalman Filter");
    ros::NodeHandle nh;
    Interface stateEstiamtion(nh, true, false);
    return 0;
}