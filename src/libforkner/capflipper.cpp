/**
 * Functionality for interfacing with the cap flipping mechanism
 */
#include "main.h"

#define RETRACTED_POS 0
#define DOWN_POS 190
#define UP_POS 110
#define TIP_POS 120

#define MOVE_TIMEOUT 1500

std::unique_ptr<okapi::Motor> flipperMotor;

namespace flipper {
state _state;

static state fuzzify(double pos) {
	state out;
	if (abs(RETRACTED_POS - pos) < 10) {
		out = state::RETRACTED;
	} else if (abs(DOWN_POS - pos) < 10) {
		out = state::DOWN;
	} else if (abs(UP_POS - pos) < 10) {
		out = state::UP;
	} else if (abs(TIP_POS - pos) < 10) {
		out == state::TIP;
	} else {
		out = state::INVALID;
	}
	return out;
}

void init() {
	flipperMotor =
	    std::make_unique<okapi::Motor>(CAP_FLIPPER, false, okapi::AbstractMotor::gearset::red);
	flipperMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void retract() {
	flipperMotor->moveAbsolute(RETRACTED_POS, 100);
}

void retractBlocking() {
	retract();
	uint32_t now = pros::millis();
	while (!(fuzzify(flipperMotor->getPosition()) == state::RETRACTED) &&
	       (pros::millis() - now) > MOVE_TIMEOUT)
		pros::delay(10);
}

void down() {
	flipperMotor->moveAbsolute(DOWN_POS, 100);
}

void downBlocking() {
	down();
	uint32_t now = pros::millis();
	while (!(fuzzify(flipperMotor->getPosition()) == state::DOWN) &&
	       (pros::millis() - now) > MOVE_TIMEOUT)
		pros::delay(10);
}

void up() {
	flipperMotor->moveAbsolute(UP_POS, 30);
}

void upBlocking() {
	up();
	uint32_t now = pros::millis();
	while (!(fuzzify(flipperMotor->getPosition()) == state::RETRACTED) &&
	       (pros::millis() - now) > MOVE_TIMEOUT)
		pros::delay(10);
}

void tip() {
	flipperMotor->moveAbsolute(TIP_POS, 100);
}

void tipBlocking() {
	tip();
	uint32_t now = pros::millis();
	while (!(fuzzify(flipperMotor->getPosition()) == state::TIP) &&
	       (pros::millis() - now) > MOVE_TIMEOUT)
		pros::delay(10);
}

state getState() {
	_state = fuzzify(flipperMotor->getPosition());
	return _state;
}
}  // namespace flipper
