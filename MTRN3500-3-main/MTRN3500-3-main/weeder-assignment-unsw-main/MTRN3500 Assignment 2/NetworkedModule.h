#pragma once
/*****************
NetworkedModule.h
ALEXANDER CUNIO 2024
*****************/

/*
This class provides additional functionality for networked modules (Laser, GPS, Vehicle control, Display) beyond the
base UGV module class. These modules should therefore be derived from this rather than the base class.
For clarification of inheritance requirements see diagram in the assignment specification.
*/

#include "UGVModule.h"

// You will need to select which address to use depending on if you are working with the simulator (127.0.0.1)
// or the physical robot in the lab (192.168.1.200).
#define WEEDER_ADDRESS "127.0.0.1"
//#define WEEDER_ADDRESS "192.168.1.200"
#define GRU01_ADDRESS "192.168.1.200"

#define DISPLAY_ADDRESS "127.0.0.1" // Display is always running locally (run the MATLAB code separately)

#define AUTHENICATION "5488467\n"

#define LIDAR_PORT	 23000
#define GNSS_PORT	 24000
#define VC_PORT		 25000
#define GRU01_PORT	 26000
#define DISPLAY_PORT 28000

#define MAX_CONNECT_ATTEMPTS 5
#define CONNECT_TIMEOUT 1000

ref class NetworkedModule abstract : public UGVModule
{
public:
	property String^ address
	{
		virtual String^ get() = 0;
	}
	property int port
	{
		virtual int get() = 0;
	}
	property int comTime
	{
		virtual int get() = 0;
	}
	property int readWait
	{
		virtual int get() = 0;
	}

	property bool authReq
	{
		virtual bool get() = 0;
	}

	virtual void threadFunction() override{
		if (attemptConnect() != error_state::SUCCESS) {
			cleanup();
			return;
		}

		switch (authenticate()) {
		case error_state::ERR_AUTH:
			Console::WriteLine(name + "ignoring auth error");
		case error_state::SUCCESS:
			break;
		default:
			cleanup();
			return;
		}

		syncAndWait();

		commWatch_ = gcnew Stopwatch;
		commWatch_->Start();

		Console::WriteLine(this->name + "running main loop");
		while (!getShutdownFlag()) {

			if (0 < this->comTime && 
				this->comTime < commWatch_->ElapsedMilliseconds) 
			{
				if (communicate() != error_state::SUCCESS) {
					Console::WriteLine(this->name + "shutting down...");
					break;
				}
				Thread::Sleep(this->readWait);
				commWatch_->Restart();
			}
			
			if (processSharedMemory() != error_state::SUCCESS) {
				Console::WriteLine(this->name + "shutting down...");
				break;
			}
			if (processHeartbeat() != error_state::SUCCESS) {
				Console::WriteLine(this->name + "shutting down...");
				break;
			}
			Thread::Sleep(SLEEP_TIME);
		}

		cleanup();
		return;
	}


protected:
	TcpClient^ Client;
	NetworkStream^ Stream;
	array<unsigned char>^ SendData;
	array<unsigned char>^ ReadData;
	Stopwatch^ commWatch_;

	error_state authenticate() {
		if (!this->authReq) return error_state::SUCCESS;

		SendData = Encoding::ASCII->GetBytes(AUTHENICATION);
		Stream->Write(SendData, 0, SendData->Length);

		System::Threading::Thread::Sleep(100);
		String^ resp;
		try {
			int len = Stream->Read(ReadData, 0, ReadData->Length);
			String^ resp = Encoding::ASCII->GetString(ReadData, 0, len);
		}
		catch (Exception^ e) {
			Console::WriteLine(name + "Connection error");
			return error_state::ERR_CONNECTION;
		}

		if (!String::Equals(resp, "OK\n")) {
			Console::WriteLine(name + "Authentication error");
			return error_state::ERR_AUTH;
		}
		Console::WriteLine(name + "Auth successful");
		return error_state::SUCCESS;
	}

	error_state attemptConnect() {
		int tries = 0;
		for (; tries < MAX_CONNECT_ATTEMPTS; tries++) {
			if (connect(this->address, this->port) == error_state::SUCCESS) {
				Console::WriteLine(this->name + "connected");
				return error_state::SUCCESS;
			}
			Console::WriteLine(this->name + "connection failed, retrying...");
			Thread::Sleep(CONNECT_TIMEOUT);
		}
		return error_state::ERR_CONNECTION;

	}

	virtual error_state connect(String^ hostName, int portNumber) {
		SendData = gcnew array<unsigned char>(1024);
		ReadData = gcnew array<unsigned char>(2048);

		try {
			Client = gcnew TcpClient(hostName, portNumber);
			Stream = Client->GetStream();
			Client->NoDelay = true;
			Client->ReceiveTimeout = 200;
			Client->SendTimeout = 200;
			Client->ReceiveBufferSize = 2048;
			Client->SendBufferSize = 1024;
			return error_state::SUCCESS;
		}
		catch (Exception^ e) {
			return error_state::ERR_CONNECTION;
		}
	}

	virtual error_state communicate() = 0;
	
	virtual void cleanup() {
		Console::WriteLine(this->name + "thread returning");
		if (this->bitmask & CRITICALMASK) SM_TM_->shutdown |= 0xFF;
		if (Client != nullptr) Client->Close();
	}
};

