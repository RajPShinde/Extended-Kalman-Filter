#ifndef INCLUDE_VISUALIZATION_HPP_
#define INCLUDE_VISUALIZATION_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Visualization
{
    public:

        /**
        *   @brief Constructor of Visualization
        *   @param none
        *   @return none
        */
        Visualization(ros::NodeHandle& nh);

        /**
        *   @brief Destructor of Visualization
        *   @param none
        *   @return none
        */
        ~Visualization();

        void visualizeModel();

    private:
        ros::NodeHandle visualize_;
        ros::Publisher visPub_;

};

#endif  //  INCLUDE_VISUALIZATION_HPP_