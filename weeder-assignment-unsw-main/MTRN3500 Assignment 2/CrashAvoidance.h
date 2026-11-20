#pragma once

#include "UGVModule.h"

ref class CrashAvoidance : public UGVModule {
public:
    property uint8_t bitmask
    {
        uint8_t get() override {
            return bit_CRASHAVOIDANCE;
        }
    }
    property String^ name
    {
        String^ get() override {
            return "[CrashAvoidance] ";
        }
    }

    CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser, SM_VehicleControl^ SM_VehicleControl)
    {
        SM_TM_ = SM_TM;
        SM_Laser_ = SM_Laser;
        SM_VehicleControl_ = SM_VehicleControl;
        dataX = gcnew array<double>(STANDARD_LASER_LENGTH);
        dataY = gcnew array<double>(STANDARD_LASER_LENGTH);
        if (d_min >= d_throttle) throw gcnew Exception("d_throttle must be larger than d_min");
    }

    void threadFunction() override;

protected:
    error_state processSharedMemory() override;

    double rectCollision();
    double rotoCollision();

    double findThrottle(double d_crash);

    error_state copyData();

private:
    // Add any additional data members or helper functions here
    // SM_ThreadManagement^ SM_TM_;
    SM_Laser^ SM_Laser_;
    SM_VehicleControl^ SM_VehicleControl_;

    double use_rect_deg = 1.; // degrees at which to use rect instead of rotating detection



    // geometry (mm)
    double d_min = 300.; // Stop 30cm from target
    double d_throttle = 1000.; // full speed if no target up to 1m away

    double vehicleWidth  = 750.;
    double vehicleLength = 680.; // axel-to-axel length of vehicle
    double LidarOffset   = 200.;

    double ignoreRange = 100.;

    double TargetSteer = 0;

    array<double>^ dataX;
    array<double>^ dataY;

};
