#include <extendedKalmanFilter.hpp>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    ROS_INFO_STREAM("Initializing EKF");
    state_.resize(15);
    transferFunction_.resize(15,15);
    transferFunctionJacobian_.resize(15,15);
    estimateErrorCovariance_.resize(15,15);
    processNoiseCovariance_.resize(15,15);
    identity_.resize(15,15);

    state_.setZero();
    transferFunction_.setIdentity();
    transferFunctionJacobian_.setZero();
    estimateErrorCovariance_.setIdentity();
    setInitialErrorEstimateCovariance();
    processNoiseCovariance_.setZero();
    setProcessNoiseCovariance();
    identity_.setIdentity();
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {
}

void ExtendedKalmanFilter::setInitialErrorEstimateCovariance(){
	estimateErrorCovariance_ *= 1e-9;
}

void ExtendedKalmanFilter::setanPitchrocessNoiseCovariance(){
    ROS_INFO_STREAM("Set Process Noise Covariance Matrix");
    processNoiseCovariance_(StateMemberX, StateMemberX) = 0.05;
    processNoiseCovariance_(StateMemberY, StateMemberY) = 0.05;
    processNoiseCovariance_(StateMemberZ, StateMemberZ) = 0.06;
    processNoiseCovariance_(StateMemberRoll, StateMemberRoll) = 0.03;
    processNoiseCovariance_(StateMemberPitch, StateMemberPitch) = 0.03;
    processNoiseCovariance_(StateMemberYaw, StateMemberYaw) = 0.06;
    processNoiseCovariance_(StateMemberVx, StateMemberVx) = 0.025;
    processNoiseCovariance_(StateMemberVy, StateMemberVy) = 0.025;
    processNoiseCovariance_(StateMemberVz, StateMemberVz) = 0.04;
    processNoiseCovariance_(StateMemberVroll, StateMemberVroll) = 0.01;
    processNoiseCovariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
    processNoiseCovariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
    processNoiseCovariance_(StateMemberAx, StateMemberAx) = 100;
    processNoiseCovariance_(StateMemberAy, StateMemberAy) = 100;
    processNoiseCovariance_(StateMemberAz, StateMemberAz) = 100;
}

void ExtendedKalmanFilter::removeGravitationalAcceleration(){

}

void ExtendedKalmanFilter::predict(){
    ROS_INFO_STREAM("Predicting");

    // copy state variables
    double x = state_(StateMemberX);
    double y = state_(StateMemberY);
    double z = state_(StateMemberZ);
    double roll = state_(StateMemberRoll);
    double pitch = state_(StateMemberPitch);
    double yaw = state_(StateMemberYaw);
    double xVel = state_(StateMemberVx);
    double yVel = state_(StateMemberVy);
    double zVel = state_(StateMemberVz);
    double pitchVel = state_(StateMemberVpitch);
    double yawVel = state_(StateMemberVyaw);
    double xAcc = state_(StateMemberAx);
    double yAcc = state_(StateMemberAy);
    double zAcc = state_(StateMemberAz);

    // Required Trigonometric Operations
    double sinPitch = ::sin(pitch);
    double cosPitch = ::cos(pitch);
    double cosPitchInverse = 1.0 / cosPitch;
    double tanPitch = sinPitch * cosPitch;

    double sinRoll = ::sin(roll);
    double cosRoll = ::cos(roll);

    double sinYaw = ::sin(yaw);
    double cosYaw = ::cos(yaw);

    transferFunction_(StateMemberX, StateMemberVx) = cosYaw * cosPitch * dt;
    transferFunction_(StateMemberX, StateMemberVy) = (cosYaw * sinPitch * sinRoll - sinYaw * cosRoll) * dt;
    transferFunction_(StateMemberX, StateMemberVz) = (cosYaw * sinPitch * cosRoll + sinYaw * sinRoll) * dt;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * dt;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * dt;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * dt;
    transferFunction_(StateMemberY, StateMemberVx) = sinYaw * cosPitch * dt;
    transferFunction_(StateMemberY, StateMemberVy) = (sinYaw * sinPitch * sinRoll + cosYaw * cosRoll) * dt;
    transferFunction_(StateMemberY, StateMemberVz) = (sinYaw * sinPitch * cosRoll - cosYaw * sinRoll) * dt;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * dt;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * dt;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * dt;
    transferFunction_(StateMemberZ, StateMemberVx) = -sinPitch * dt;
    transferFunction_(StateMemberZ, StateMemberVy) = cosPitch * sinRoll * dt;
    transferFunction_(StateMemberZ, StateMemberVz) = cosPitch * cosRoll * dt;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * dt;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * dt;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * dt;
    transferFunction_(StateMemberRoll, StateMemberVroll) = dt;
    transferFunction_(StateMemberRoll, StateMemberVpitch) = sinRoll * tanPitch * dt;
    transferFunction_(StateMemberRoll, StateMemberVyaw) = cosRoll * tanPitch * dt;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = cosRoll * dt;
    transferFunction_(StateMemberPitch, StateMemberVyaw) = -sinRoll * dt;
    transferFunction_(StateMemberYaw, StateMemberVpitch) = sinRoll * cosPitchInverse * dt;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = cosRoll * cosPitchInverse * dt;
    transferFunction_(StateMemberVx, StateMemberAx) = dt;
    transferFunction_(StateMemberVy, StateMemberAy) = dt;
    transferFunction_(StateMemberVz, StateMemberAz) = dt;

    // Predicted State, x = f(x,u) = transferFunction * previousStates
    state_ = transferFunction_ * state_;

    // ROS_WARN_STREAM(state_(3)<<" "<<state_(4)<<" "<<state_(5));

    resetAngleOverflow();

    // compute Jacobian of f(x,u) i.e F
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * dt * dt;

    yCoeff = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    zCoeff = -cosYaw * sinPitch * sinRoll + sinYaw * cosRoll;
    double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * dt +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dR = 1.0 + (cosRoll * tanPitch * pitchVel - sinRoll * tanPitch * yawVel) * dt;

    xCoeff = -cosYaw * sinPitch;
    yCoeff = cosYaw * cosPitch * sinRoll;
    zCoeff = cosYaw * cosPitch * cosRoll;
    double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * dt +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dP = (cosPitchInverse * cosPitchInverse * sinRoll * pitchVel + cosPitchInverse * cosPitchInverse * cosRoll * yawVel) * dt;

    xCoeff = -sinYaw * cosPitch;
    yCoeff = -sinYaw * sinPitch * sinRoll - cosYaw * cosRoll;
    zCoeff = -sinYaw * sinPitch * cosRoll + cosYaw * sinRoll;
    double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * dt +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
    zCoeff = -sinYaw * sinPitch * sinRoll - cosYaw * cosRoll;
    double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * dt +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFP_dR = (-sinRoll * pitchVel - cosRoll * yawVel) * dt;

    xCoeff = -sinYaw * sinPitch;
    yCoeff = sinYaw * cosPitch* sinRoll;
    zCoeff = sinYaw * cosPitch * cosRoll;
    double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * dt +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = cosYaw * cosPitch;
    yCoeff = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
    zCoeff = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * dt +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = cosPitch * cosRoll;
    zCoeff = -cosPitch * sinRoll;
    double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * dt +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dR = (cosRoll * cosPitchInverse * pitchVel - sinRoll * cosPitchInverse * yawVel) * dt;

    xCoeff = -cosPitch;
    yCoeff = -sinPitch * sinRoll;
    zCoeff = -sinPitch * cosRoll;
    double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * dt +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dP = (sinRoll * tanPitch * cosPitchInverse * pitchVel + cosRoll * tanPitch * cosPitchInverse * yawVel) * dt;

    // Update only elemnts that change after taking jacobian
    transferFunctionJacobian_ = transferFunction_;
    transferFunctionJacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
    transferFunctionJacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
    transferFunctionJacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
    transferFunctionJacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
    transferFunctionJacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
    transferFunctionJacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
    transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
    transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
    transferFunctionJacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
    transferFunctionJacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
    transferFunctionJacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
    transferFunctionJacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
    transferFunctionJacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;

    // Estimate Error Covariance Matrix, P = J * P * J' + Q
    estimateErrorCovariance_ = (transferFunctionJacobian_ *
                                estimateErrorCovariance_ *
                                transferFunctionJacobian_.transpose());
    estimateErrorCovariance_.noalias() += dt * (processNoiseCovariance_);

}

void ExtendedKalmanFilter::correct(){
    ROS_INFO_STREAM("Correcting");

    std::vector<int> updateVector;
    for(int i = 0; i<measurement.dataToUse.size(); i++){
        if(measurement.dataToUse[i] == 1)
            updateVector.push_back(i);
    }


    Eigen::VectorXd stateSubset(updateVector.size());                              // x (in most literature)
    Eigen::VectorXd measurementSubset(updateVector.size());                        // z
    Eigen::MatrixXd measurementCovarianceSubset(updateVector.size(), updateVector.size());  // R
    Eigen::MatrixXd stateToMeasurementSubset(updateVector.size(), state_.rows());  // H
    Eigen::MatrixXd kalmanGainSubset(state_.rows(), updateVector.size());          // K
    Eigen::VectorXd innovationSubset(updateVector.size());                         // z - Hx

    stateSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateToMeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();

    for(int i = 0; i<updateVector.size() ; i++){
        measurementSubset(i) = measurement.measurements(updateVector[i]);
        stateSubset(i) = state_(updateVector[i]);
        measurementCovarianceSubset(i, i) = measurement.measurementCovariances(updateVector[i]);
    }

    for (int i = 0; i < updateVector.size(); i++)
    {
      stateToMeasurementSubset(i, updateVector[i]) = 1;
    }


    // Innovation, y = z - h(x)
    innovationSubset = (measurementSubset - stateSubset);

        // Wrap angles in the innovation
    for (int i = 0; i < updateVector.size(); i++)
    {
      if (updateVector[i] == 3  ||
          updateVector[i] == 4 ||
          updateVector[i] == 5)
      {
        while (innovationSubset(i) < -3.141592653589793)
        {
          innovationSubset(i) += 6.283185307179587;
        }

        while (innovationSubset(i) > 3.141592653589793)
        {
          innovationSubset(i) -= 6.283185307179587;
        }
      }
    }

    // Innovation Covariance, S = H * P * H' + R
    Eigen::MatrixXd PHT = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
    Eigen::MatrixXd S  = (stateToMeasurementSubset * PHT + measurementCovarianceSubset).inverse();
    // Kalman Gain, K = P * H' / S
    kalmanGainSubset.noalias() = PHT * S;
    // Updated State Estimate, x = x + K * y
    state_.noalias() += kalmanGainSubset * innovationSubset;
    // Updated Estimate Error Covariance, P = (I - K * H) * P
    // estimateErrorCovariance_ = (identity_ - (kalmanGainSubset * measurementCovarianceSubset)) * estimateErrorCovariance_;
    Eigen::MatrixXd gainResidual = identity_;
    gainResidual.noalias() -= kalmanGainSubset * stateToMeasurementSubset;
    estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transpose();
    estimateErrorCovariance_.noalias() += kalmanGainSubset *
                                        measurementCovarianceSubset *
                                        kalmanGainSubset.transpose();
}

void ExtendedKalmanFilter::resetAngleOverflow()
{
}

double ExtendedKalmanFilter::clamp(double rotation)
{
return rotation;
}

Eigen::VectorXd ExtendedKalmanFilter::getStates(){
    return state_;
}