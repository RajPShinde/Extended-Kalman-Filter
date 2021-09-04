#ifndef INCLUDE_EXTENDEDKALMANFILTER_HPP_
#define INCLUDE_EXTENDEDKALMANFILTER_HPP_

#include <cmath>

class ExtendedKalmanFilter
{
    public:

        /**
        *   @brief Constructor of ExtendedKalmanFilter
        *   @param none
        *   @return none
        */
        ExtendedKalmanFilter();

        /**
        *   @brief Destructor of ExtendedKalmanFilter
        *   @param none
        *   @return none
        */
        ~ExtendedKalmanFilter();

        void predict();

        void correct();

        void setErrorEstimateCovariance();

        void setProcessNoiseCovariance();

        void removeGravitationalAcceleration();

    private:

        Eigen::VectorXd state_;

        Eigen::VectorXd predictedState_;

        Eigen::MatrixXd processNoiseCovariance_;

        Eigen::MatrixXd estimateErrorCovariance_;

        Eigen::MatrixXd transferFunction_;

        Eigen::MatrixXd transferFunctionJacobian_;

        Eigen::MatrixXd identity_;

};

#endif  //  INCLUDE_EXTENDEDKALMANFILTER_HPP_