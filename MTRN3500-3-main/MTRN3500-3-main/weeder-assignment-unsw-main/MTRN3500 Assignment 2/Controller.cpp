#include "Controller.h"

// Xbox: 0
// Keyboard : 1
#define INPUT_MODE 0

error_state Controller::controllerSetUp() {
	Console::WriteLine(this->name + "connecting...");

	ControllerInterface_ = new ControllerInterface(1, INPUT_MODE);
	for (int i = 0; !ControllerInterface_->IsConnected(); i++) {
		Thread::Sleep(100);
		if (i > 100) {
			Console::WriteLine(this->name + "failed connection; aborting");
			SM_TM_->shutdown |= 0xFF;
			return error_state::ERR_CONNECTION;
		}
	}

	String^ mode = (INPUT_MODE == 0) ? "Xbox" : "keyboard";
	Console::WriteLine(this->name + "mode {0}", INPUT_MODE);
	return error_state::SUCCESS;
}

bool Controller::dependanciesOn() {
	if (!(SM_TM_->onList & bit_VC)) {
		Console::WriteLine(name + "cannot run without VC");
		return false;
	}
	return true;
}

void Controller::threadFunction() {
	if (!dependanciesOn()) {
		return;
	}
	if (controllerSetUp() != error_state::SUCCESS) {
		return;
	}

	syncAndWait();

	while (!getShutdownFlag()) {
		if (processSharedMemory() != error_state::SUCCESS) {
			Console::WriteLine(this->name + "shutting down...");
			break;
		}
		if (processHeartbeat() != error_state::SUCCESS){
			Console::WriteLine(this->name + "shutting down...");
			break;
		}
		Thread::Sleep(SLEEP_TIME);
	}

	SM_TM_->shutdown |= 0xFF;

	// blank out commands just in case
	Monitor::Enter(SM_VehicleControl_->lockObject);
	SM_VehicleControl_->Speed = 0.0;
	SM_VehicleControl_->Steering = 0.0;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	Console::WriteLine(this->name + "returning...");
}

error_state Controller::processSharedMemory() {
	_controllerState state = ControllerInterface_->GetState();
	if (!state.isConnected) {
		return error_state::ERR_CONNECTION;
	}

	double Speed = 0;
	double Steering = 0;
	double eps = 1e-2;

	// Xbox
	// controls are weird -> check in lab
	if (INPUT_MODE == 0 && state.back) {
		SM_TM_->shutdown |= 0xFF;
		Console::WriteLine(this->name + "shutdown button pressed, shutting down...");
	}
	if (INPUT_MODE == 0) {
		Steering = -state.rightThumbX;
		Speed = state.rightTrigger - state.leftTrigger;
		if (eps < Math::Abs(state.leftTrigger) && eps < Math::Abs(state.rightTrigger)) {
			Speed = 0.0;
		}
	}

	// keyboard
	if (INPUT_MODE == 1) {
		Steering = -state.rightThumbX;
		Speed = state.rightTrigger - state.leftTrigger;
		if (eps < Math::Abs(state.leftTrigger) && eps < Math::Abs(state.rightTrigger)) {
			Speed = 0.0;
		}
	}

	Monitor::Enter(SM_VehicleControl_->lockObject);
	SM_VehicleControl_->Speed = Speed;
	SM_VehicleControl_->Steering = Steering;
	Monitor::Exit(SM_VehicleControl_->lockObject);

	//Console::WriteLine(this->name + "\nSpeed\t{0:F2}\nSteer\t{1:F2}", Speed, Steering);
	// */
	return error_state::SUCCESS;
}

// */
