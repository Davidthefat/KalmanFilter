/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** KalmanFilter.cpp
** Implementation for a single variable Kalman Filter using only one input for 
** each prediction and measurement. Assumes process noise variance and
** measurement noise variance are constant to simplify algorithm. 
**
** Author: David Yoon
** -------------------------------------------------------------------------*/
KalmanFilter::KalmanFilter(float iE, float eE, float pN, float mN)
{
	preState = initEstimate = iE;
	estimateErr = eE;
	processNoise = pN;
	mesurementNoise = mN;
}
void KalmanFilter::processEstimate(float in)
{
	initEstimate = in;
	estimateErr += processNoise;
}
float KalmanFilter::processCorrection(float in)
{
	KalmanGain = estimateErr/(estimateErr + measurementNoise);
	postState = preState + KalmanGain*(in - initEstimate);
	preState = postState;
	estimateErr *= (1 - KalmanGain);
	return postState;
}
float KalmanFilter::error()
{
	return estimateErr;
}