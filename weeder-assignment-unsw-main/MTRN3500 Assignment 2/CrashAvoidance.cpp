#include "CrashAvoidance.h"
#include <cmath>
void rotateInPlace(double* x, double* y, double x_center, double y_center, double rads);

void CrashAvoidance::threadFunction() {
	error_state setup_err = setUp();
	if (setup_err != error_state::SUCCESS) return;

	while (!getShutdownFlag()) {
		error_state mem_err = processSharedMemory();
		if (mem_err != error_state::SUCCESS &&
			mem_err != error_state::ERR_NO_DATA)
		{
			return;
		}

		error_state hb_err = processHeartbeat();
		if (hb_err != error_state::SUCCESS) return;
		Thread::Sleep(100);
	}
	Console::WriteLine(this->name + "thread returning");
}


error_state CrashAvoidance::copyData() {
	Monitor::Enter(SM_Laser_->lockObject);
	Buffer::BlockCopy(SM_Laser_->x, 0, dataX, 0, dataX->Length * sizeof(double));
	Buffer::BlockCopy(SM_Laser_->y, 0, dataY, 0, dataY->Length * sizeof(double));
	Monitor::Exit(SM_Laser_->lockObject);

	Monitor::Enter(SM_VehicleControl_->lockObject);
	TargetSteer = SM_VehicleControl_->Steering;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	return error_state::SUCCESS;
}

error_state CrashAvoidance::processSharedMemory() {
	copyData();

	// checkCollisions may take a while -> compute in non-blocking way
	double  d_crash = INFINITY; // mm
	if (Math::Abs(TargetSteer) < use_rect_deg) {
		d_crash = rectCollision();
	}
	else {
		d_crash = rotoCollision();
	}
	double throttle = findThrottle(d_crash / 1000.0);
	//Console::WriteLine(name + "throttle {0} \t|dcrash {1}", throttle, d_crash);
	Monitor::Enter(SM_VehicleControl_->lockObject);
	SM_VehicleControl_->throttle = throttle;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	return error_state::SUCCESS;
}

double CrashAvoidance::rectCollision() {
	double min_dist = INFINITY;
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		if (dataX[i] * dataX[i] + dataY[i] * dataY[i] < ignoreRange) continue;
		double Bx = dataX[i] + LidarOffset;
		double By = dataY[i];
		// Console::WriteLine(name + "{2}, {0} {1}", Bx, By, i);
		if (By < -vehicleWidth *500. || vehicleWidth * 500. < By) continue;
		min_dist = Math::Min(Bx, min_dist);
	
	}
	return min_dist;
}

double CrashAvoidance::rotoCollision() {
	double min_dist = INFINITY;
	double R = vehicleLength / Math::Tan(TargetSteer * Math::PI / 180.);

	double lowerB = Math::Sqrt(vehicleLength * vehicleLength + (R - vehicleWidth / 2.0) * (R - vehicleWidth / 2.0));
	double upperB = Math::Sqrt(vehicleLength * vehicleLength + (R + vehicleWidth / 2.0) * (R + vehicleWidth / 2.0));

	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		if (dataX[i] * dataX[i] + dataY[i] * dataY[i] < ignoreRange) continue;
		double x = dataX[i] + LidarOffset;
		double y = dataY[i];
		if (TargetSteer < 0.0) y = -y; // mirror for neg steer

		double Px = x + (-vehicleLength);
		double Py = y + (R);

		double P_mag = Math::Sqrt(Px * Px + Py * Py);
		if (P_mag < lowerB || upperB < P_mag) continue;
		
		double theta = Math::Acos(vehicleLength / P_mag) + Math::Atan2(Py, Px);
		double dist = theta * P_mag;
		min_dist = Math::Min(min_dist, dist);
	}
	return min_dist;
}

double CrashAvoidance::findThrottle(double d_crash) {
	double throttle =  (d_crash - d_min) / (d_throttle - d_min);
	throttle = Math::Max(throttle, 0.0);
	throttle = Math::Min(throttle, 1.0);
	return throttle;
}

void rotateInPlace(double* x, double* y, double x_center, double y_center, double rads) {
	double netx = *x - x_center;
	double nety = *y - y_center;
	*x = netx * Math::Cos(rads) + nety * Math::Sin(rads) + x_center;
	*y = -nety * Math::Sin(rads) + netx * Math::Cos(rads) + y_center;
}