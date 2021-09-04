#ifndef INCLUDE_INTERFACE_HPP_
#define INCLUDE_INTERFACE_HPP_

#include <extendedKalmanFilter.hpp>
#include <visualization.hpp>

class Interface
{
    public:

        /**
        *   @brief Constructor of Interface
        *   @param none
        *   @return none
        */
        Interface();

        /**
        *   @brief Destructor of Interface
        *   @param none
        *   @return none
        */
        ~Interface();

        void getMeasurement();

        void fuseData();

        void removeIMUGravitationalAcceleration();

        void publishTransform();

    private:

        ExtendedKalmanFilter ekf;
        Visualization view;
        bool visualizeModel_, publishTransform_, useIMU_, useOdometry_ = true;
        int totalSources_ = 2;
};

#endif  //  INCLUDE_INTERFACE_HPP_