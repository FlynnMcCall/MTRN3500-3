#include "NetworkedModule.h"

error_state NetworkedModule::setUp() {
	connect(this->address, this->port);
	return UGVModule::setUp();
}

error_state NetworkedModule::connect(String ^ hostName, int portNumber) {
	Client = gcnew TcpClient(hostName, portNumber);
	Stream = Client->GetStream();
	Client->NoDelay = true;
	Client->ReceiveTimeout = 2000;
	Client->SendTimeout = 2000;
	Client->ReceiveBufferSize = 2048;
	Client->SendBufferSize = 1024;

	SendData = gcnew array<unsigned char>(1024);
	ReadData = gcnew array<unsigned char>(2048);

	return error_state::SUCCESS;
}

error_state NetworkedModule::authenticate() {
	SendData = Encoding::ASCII->GetBytes(AUTHENICATION);
	Stream->Write(SendData, 0, SendData->Length);

	System::Threading::Thread::Sleep(100);
	int len = Stream->Read(ReadData, 0, ReadData->Length);
	String^ resp = Encoding::ASCII->GetString(ReadData, 0, len);
	if (!String::Equals(resp, "OK\n")) {
		Console::WriteLine(name + "Authentication error");
		return error_state::ERR_CONNECTION;
	}
	Console::WriteLine(name + "Auth successful");
	return error_state::SUCCESS;
}

void NetworkedModule::CoreLoop(int sleep, bool auth) {
	try {
		setUp();
		if (auth) {
			NetworkedModule::authenticate();
		}
		while (!getShutdownFlag()) {
			processSharedMemory();
			communicate();
			processHeartbeat();
			Thread::Sleep(sleep);
		}
	}
	catch (Exception^ e) {
		Console::WriteLine("{0} error {1}", this->name, e);
	}

	Console::WriteLine(this->name + "thread returning");
}