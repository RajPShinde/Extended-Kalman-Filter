# Extended-Kalman-Filter
Extended Kalman Filter for estimating 15-States (Pose, Twist & Acceleration)

## Dependencies
1. ROS KInetic >=
2. Eigen

## Results
Increasing Covarinace as No Absolute Position Fused (Data Fused- z, yaw, vx, vy, vz, Ax, omegaZ)       |  Converged Covariance since Absolute Position is Fused (Data Fused- x, y, z, yaw, vx, vy, vz, Ax, omegaZ)
:-------------------------:|:-------------------------:
<img src="/data/non-converging.gif"/> | <img src="/data/converged.gif"/>


1. The position covariances keeps increasing in left column result as there is no correction data for position and it is being estimated by integrating velocities and acceleration.
2. In the Second Column the Absolute Position is Fused, Hence position covariances converge as the position data from the odometry is being used for correction. 

## TODO
1. Fix segmentation Fault when data selection vector has all 0 elements.
2. Add way to set parameters through launch files.

## Note
This project is still in progress, and not recommended to be used apart from research purposes.

## Reference
[Robot Localization](https://github.com/cra-ros-pkg/robot_localization)
