#pragma once
/*****************
UGVModule.h
ALEXANDER CUNIO 2024
*****************/

/*
This file defines the base class for ALL UGV modules (TMM, Laser, GPS, Vehicle Control, Display, Controller).
Basic classes (TMM, Controller) will be derived directly from this class, whereas those that require networking
to external devices (GPS, Laser, VC, Display) will derive from the networked module class (which itself inherits from this class).
For clarification of inheritance requirements see diagram in the assignment specification.
*/

#include <iostream>
#include "SMObjects.h"
#define SLEEP_TIME 50
#define SYNC_TIMEOUT 5000

// ERROR HANDLING. Use this as the return value in your functions
enum error_state {
	SUCCESS,
	ERR_STARTUP,
	ERR_NO_DATA,
	ERR_INVALID_DATA,
	ERR_SM,
	ERR_CONNECTION,
	ERR_CRITICAL_THREAD_FAILURE,
	ERR_TMM_FAILURE,
	ERR_AUTH
	// Define your own additional error types as needed
};

ref class UGVModule abstract
{
public:
	property uint8_t bitmask
	{
		virtual uint8_t get() = 0;
	}
	property String^ name
	{
		virtual String^ get() = 0;
	}


	virtual void UGVModule::threadFunction() {
		if (!dependanciesOn()) {
			return;
		}

		syncAndWait();

		while (!getShutdownFlag()) {
			if (processSharedMemory() != error_state::SUCCESS){
				Console::WriteLine(this->name + "shutting down...");
				break;
			}
			if (processHeartbeat() != error_state::SUCCESS)
			{
				Console::WriteLine(this->name + "shutting down...");
				break;
			}
			Thread::Sleep(SLEEP_TIME);
		}

		Console::WriteLine(this->name + "thread returning");
		if (this->bitmask & CRITICALMASK) SM_TM_->shutdown |= 0xFF;
		return;
	}

protected:
	SM_ThreadManagement^ SM_TM_;
	Stopwatch^ Stopwatch_;

	virtual bool getShutdownFlag() {
		return (SM_TM_->shutdown) & (this->bitmask);
	}

	virtual bool dependanciesOn() {
		return true;
	}

	
	virtual error_state syncAndWait() {
		Console::WriteLine(this->name + "waiting...");
		if (!SM_TM_->ThreadBarrier->SignalAndWait(SYNC_TIMEOUT)) {
			Console::WriteLine(name + "set up timeout");
			return error_state::ERR_STARTUP;
		}

		Stopwatch_ = gcnew Stopwatch;
		Stopwatch_->Start();
		return error_state::SUCCESS;
	}


	virtual error_state processHeartbeat() {
		Console::WriteLine(name + "heartbeat");
		if ((SM_TM_->heartbeat & this->bitmask) == 0) {
			SM_TM_->heartbeat |= this->bitmask;
			Stopwatch_->Restart();
		}
		else {
			if (Stopwatch_->ElapsedMilliseconds > SLEEP_TIME * 2) {
				SM_TM_->shutdown = 0xFF;
				Console::WriteLine(name + "TMM failure, shutting down");
				return error_state::ERR_TMM_FAILURE;
			}
		}
		return error_state::SUCCESS;
	}

	virtual error_state processSharedMemory() = 0;

	static void printError(error_state error)
	{
		switch (error)
		{
		case SUCCESS:
			std::cout << "Success." << std::endl;
			break;
		case ERR_NO_DATA:
			std::cout << "ERROR: No Data Available." << std::endl;
			break;
		case ERR_INVALID_DATA:
			std::cout << "ERROR: Invalid Data Received." << std::endl;
			break;
			// ADD PRINTOUTS FOR OTHER ERROR TYPES
		}
	}


};
