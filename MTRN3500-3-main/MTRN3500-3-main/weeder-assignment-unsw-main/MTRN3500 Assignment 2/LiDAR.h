#pragma once

#include "NetworkedModule.h"

ref class LiDAR : public NetworkedModule {
public:
    property uint8_t bitmask
    {
        uint8_t get() override {
            return bit_LASER;
        }
    }
    property String^ name
    {
        String^ get() override {
            return "[LiDAR] ";
        }
    }
    property String^ address
    {
        String^ get() override {
            return WEEDER_ADDRESS;
        }
    }
    property int port
    {
        int get() override {
            return LIDAR_PORT;
        }
    }

    property bool authReq
    {
        bool get() override {
            return true;
        }
    }
    property int comTime
    {
        int get() override {
            return 100;
        }
    }
    property int readWait
    {
        int get() override {
            return 50;
        }
    }


    LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_Laser)
    {
        SM_TM_ = SM_TM;
        SM_Laser_ = SM_Laser;
    }

protected:
    error_state communicate() override;
    error_state processSharedMemory() override;

private:
    // Add any additional data members or helper functions here
    // SM_ThreadManagement^ SM_TM_;
    SM_Laser^ SM_Laser_;

};
