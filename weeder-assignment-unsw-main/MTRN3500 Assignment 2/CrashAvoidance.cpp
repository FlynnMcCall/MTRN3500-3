#include "CrashAvoidance.h"
#include <cmath>
double Mag(double x, double y) {
	return Math::Sqrt(x*x + y*y);
}

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
	double throttle = findThrottle(d_crash);
	
	if (d_crash == INFINTIY) Console::WriteLine(name + "No collision predicted \t| Throttling to {0:F2}%", 100.0 * throttle;
	else Console::WriteLine(name + "Collision in {0:F3}m \t| Throttling to {1:F2}%", (d_crash*1000.), 100.0 * throttle;
	
	Console::WriteLine(name + "Collision in {0:F3}m \t| Throttling to ", throttle, d_crash);
	Monitor::Enter(SM_VehicleControl_->lockObject);
	SM_VehicleControl_->throttle = throttle;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	return error_state::SUCCESS;
}

double CrashAvoidance::rectCollision() {
	double min_dist = INFINITY;
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		// ignore points super close to LiDAR
		if (Mag(dataX[i], dataY[i]) < ignoreRange) continue;
		double Bx = dataX[i] + LidarOffset;
		double By = dataY[i];
		// Console::WriteLine(name + "{2}, {0} {1}", Bx, By, i);
		if (Bx <= 0.0) continue; // skip points behind the vehicle
		if (By < -vehicleWidth / 2.0 || vehicleWidth  / 2.0 < By) continue;
		min_dist = Math::Min(Bx, min_dist);
	
	}
	return min_dist;
}

double CrashAvoidance::rotoCollision() {
	double min_dist = INFINITY;
	double R = Math::Abs(vehicleLength / Math::Tan(TargetSteer * Math::PI / 180.));

	double lowerB = Math::Sqrt(vehicleLength * vehicleLength + (R - vehicleWidth / 2.0) * (R - vehicleWidth / 2.0));
	double upperB = Math::Sqrt(vehicleLength * vehicleLength + (R + vehicleWidth / 2.0) * (R + vehicleWidth / 2.0));

	for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
		// ignore points super close to LiDAR
		if (Mag(dataX[i], dataY[i]) < ignoreRange) continue;
		
		double x = dataX[i] + LidarOffset;
		double y = dataY[i];
		if (TargetSteer < 0.0) y = -y; // mirror for neg steer

		double Px = x - (-vehicleLength);
		double Py = y - (R);

		if (Px <= vehicleLength) continue; // skip if point behind vehicle

		double P_mag = Mag(Px, Py);
		if (P_mag < lowerB || upperB < P_mag) continue; // skip if out of region
		
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
