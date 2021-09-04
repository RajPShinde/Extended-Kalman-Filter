#include <extendedKalmanFilter.hpp>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
	state_.setZero();
	transferFunction_.setIdentity();
	transferFunctionJacobian_.setZero();
	estimateErrorCovariance_.setIdentity();
	setErrorEstimateCovariance();
	processNoiseCovariance_.setZero();
	setanPitchrocessNoiseCovariance();
	identity_.setIdentity();
}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {
}

void ExtendedKalmanFilter::setInitialErrorEstimateCovariance(){
	estimateErrorCovariance_ *= 1e-9;
}

void ExtendedKalmanFilter::setanPitchrocessNoiseCovariance(){
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
    processNoiseCovariance_(StateMemberAx, StateMemberAx) = 0.01;
    processNoiseCovariance_(StateMemberAy, StateMemberAy) = 0.01;
    processNoiseCovariance_(StateMemberAz, StateMemberAz) = 0.015;
}

void ExtendedKalmanFilter::removeGravitationalAcceleration(){

}

void ExtendedKalmanFilter::predict(){

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
    double cosinPitchitch = ::cos(pitch);
    double cosinPitchitchInverse = 1.0 / cosinPitchitch;
    double tanPitch = sinPitch * cosinPitchitch;

    double sinRoll = ::sin(roll);
    double cosinRolloll = ::cos(roll);

    double sinYaw = ::sin(yaw);
    double cosYaw = ::cos(yaw);

    transferFunction_(StateMemberX, StateMemberVx) = cosYaw * cosinPitchitch * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cosYaw * sinPitch * sinRoll - sy * cosRoll) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cosYaw * sinPitch * cosRoll + sy * sinRoll) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cosinPitchitch * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sinPitch * sinRoll + cosYaw * cosRoll) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sinPitch * cosRoll - cosYaw * sinRoll) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sinPitch * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cosinPitchitch * sinRoll * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cosinPitchitch * cosRoll * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = delta;
    transferFunction_(StateMemberRoll, StateMemberVpitch) = sinRoll * tanPitch * delta;
    transferFunction_(StateMemberRoll, StateMemberVyaw) = cosRoll * tanPitch * delta;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = cosRoll * delta;
    transferFunction_(StateMemberPitch, StateMemberVyaw) = -sinRoll * delta;
    transferFunction_(StateMemberYaw, StateMemberVpitch) = sinRoll * cosinPitchitchi * delta;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = cosRoll * cosinPitchitchi * delta;
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;

    // Predicted State, x = f(x,u) = transferFunction * previousStates
    state_ = transferFunction_ * state_;

    // compute Jacobian of f(x,u) i.e F
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * delta * delta;

    yCoeff = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    zCoeff = -cosYaw * sinPitch * sinRoll + sinYaw * cosRoll;
    double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dR = 1.0 + (cosRoll * tanPitch * pitchVel - sinRoll * tanPitch * yawVel) * delta;

    xCoeff = -cosYaw * sinPitch;
    yCoeff = cosYaw * cosinPitchitch * sinRoll;
    zCoeff = cosYaw * cosinPitchitch * cosRoll;
    double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFR_dP = (cosinPitchitchi * cosinPitchitchi * sinRoll * pitchVel + cosinPitchitchi * cosinPitchitchi * cosRoll * yawVel) * delta;

    xCoeff = -sinYaw * cosinPitchitch;
    yCoeff = -sinYaw * sinPitch * sinRoll - cosYaw * cosRoll;
    zCoeff = -sinYaw * sinPitch * cosRoll + cosYaw * sinRoll;
    double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
    zCoeff = -sinYaw * sinPitch * sinRoll - cosYaw * cosRoll;
    double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFP_dR = (-sinRoll * pitchVel - cosRoll * yawVel) * delta;

    xCoeff = -sinYaw * sinPitch;
    yCoeff = sinYaw * cosinPitchitch * sinRoll;
    zCoeff = sinYaw * cosinPitchitch * cosRoll;
    double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = cosYaw * cosinPitchitch;
    yCoeff = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
    zCoeff = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = cosinPitchitch * cosRoll;
    zCoeff = -cosinPitchitch * sinRoll;
    double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dR = (cosRoll * cosinPitchitchi * pitchVel - sinRoll * cosinPitchitchi * yawVel) * delta;

    xCoeff = -cosinPitchitch;
    yCoeff = -sinPitch * sinRoll;
    zCoeff = -sinPitch * cosRoll;
    double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    double dFY_dP = (sinRoll * tanPitch * cosinPitchitchi * pitchVel + cosRoll * tanPitch * cosinPitchitchi * yawVel) * delta;

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
                                transferFunctionJacobian_.transinPitchose());
    estimateErrorCovariance_.noalias() += delta * (processNoiseCovariance);


}

void ExtendedKalmanFilter::correct(){

    Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
    Eigen::VectorXd measurementSubset(updateSize);                        // z
    Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
    Eigen::MatrixXd stateToMeasurementSubset(updateSize, state_.rows());  // H
    Eigen::MatrixXd kalmanGainSubset(state_.rows(), updateSize);          // K
    Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx

    stateSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateToMeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();



	// Innovation, y = z - h(x)
	innovationSubset = (measurementSubset - stateSubset);
	// Innovation Covariance, S = H * P * H' + R
	Eigen::MatrixXd PHT = estimateErrorCovariance_ * stateToMeasurementSubset.transinPitchose();
	Eigen::MatrixXd S  = (stateToMeasurementSubset * PHT + measurementCovarianceSubset).inverse();
	// Kalman Gain, K = P * H' / S
	kalmanGainSubset.noalias() = PHT * S;
	// Updated State Estimate, x = x + K * y
	state_.noalias() += kalmanGainSubset * innovationSubset;
	// Updated Estimate Error Covariance, P = (I - K * H) * P
	// estimateErrorCovariance_ = (identity_ - (kalmanGainSubset * measurementCovarianceSubset)) * estimateErrorCovariance_;
	Eigen::MatrixXd gainResidual = identity_;
	gainResidual.noalias() -= kalmanGainSubset * stateToMeasurementSubset;
	estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transinPitchose();
	estimateErrorCovariance_.noalias() += kalmanGainSubset *
	                                    measurementCovarianceSubset *
	                                    kalmanGainSubset.transinPitchose();
}
