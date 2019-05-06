/**
 * Odometry
 *
 * Credit goes to the E-Bots Pilons team for the odometry math, the paper
 * containing this can be found at: https://pos.tixo.ca/
 */
#include <math.h>
#include "main.h"

odom::state expectedState;

odom::state _state;
std::unique_ptr<pros::ADIEncoder> leftEnc;
std::unique_ptr<pros::ADIEncoder> rightEnc;

std::unique_ptr<pros::Task> odomTask;
std::unique_ptr<pros::Mutex> stateMutex;

static bool initialized = false;
static bool logEnabled = false;

using namespace okapi;

void _odomTask(void* ign) {
	int prevLeft = 0, prevRight = 0;
	uint32_t now = pros::millis();
	if (logEnabled && logger) {
		logger->log("O, sX, sY, sT, eX, eY, eT\n");
	}
	int logCounter = 0;
	while (true) {
		stateMutex->take(1);
		int leftVal = leftEnc->get_value();
		int rightVal = rightEnc->get_value();

		// REDACTED

		_state.y += QLength(yDif);
		_state.x += QLength(xDif);
		_state.theta += QAngle(thetaDif);
		stateMutex->give();

		if (logEnabled && logger && !(logCounter++ % 10)) {
			logger->log("O, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f, %08.3f\n",
			            _state.x.convert(inch), _state.y.convert(inch),
			            _state.theta.convert(degree), expectedState.x.convert(inch),
			            expectedState.y.convert(inch),
			            expectedState.theta.convert(degree));
			logCounter = 0;
		}

		prevLeft = leftVal;
		prevRight = rightVal;
		pros::Task::delay_until(&now, 5);
	}
}

namespace odom {
void start(bool ilog) {
	if (!initialized) {
		leftEnc = std::make_unique<pros::ADIEncoder>(ENC_LEFT_1, ENC_LEFT_2);
		rightEnc = std::make_unique<pros::ADIEncoder>(ENC_RIGHT_1, ENC_RIGHT_2);
		stateMutex = std::make_unique<pros::Mutex>();
		logEnabled = ilog;
		pros::delay(20);
		odomTask = std::make_unique<pros::Task>(_odomTask, nullptr, TASK_PRIORITY_DEFAULT + 2);
		initialized = true;
	} else {
		odomTask->resume();
	}
}

void stop() {
	if (odomTask) odomTask->suspend();
}

state getState() {
	return _state;
}

void setState(state istate) {
	stateMutex->take(10);
	_state = istate;
	stateMutex->give();
}

QLength getXLength(QLength ihypotenuse, QAngle itheta) {
	return QLength(ihypotenuse.convert(meter) *
	               cos(PI / 2 - itheta.convert(radian)));
}

QLength getYLength(QLength ihypotenuse, QAngle itheta) {
	return QLength(ihypotenuse.convert(meter) *
	               sin(PI / 2 - itheta.convert(radian)));
}
}  // namespace odom
