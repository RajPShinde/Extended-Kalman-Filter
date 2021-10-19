# Extended-Kalman-Filter
Extended Kalman Filter for estimating 15-States (Pose, Twist & Acceleration)

## Dependencies
1. ROS KInetic >=
2. Eigen

## Results
Increasing Covarinace           |  Converged Covariance
:-------------------------:|:-------------------------:
No Absolute Position Fused, Hence position covariances keep on increasing as there is no correction data for position and it is being estimated by integrating velocities and acceleration, | Absolute Position Fused, Hence position covariances converge. 
<img src="/data/non-converging.gif"/> | <img src="/data/converged.png"/>

## TODO
1. Fix segmentation Fault when data selection vector has all 0 elements.
2. Add way to set parameters through launch files.

## Note
This project is still in progress, and not recommended to be used apart from research purposes.

## Reference
[Robot Localization](https://github.com/cra-ros-pkg/robot_localization)
