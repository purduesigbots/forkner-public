#include "main.h"

namespace test {
void flywheelvariance() {
	okapi::Motor* flyTop;
	flyTop = new okapi::Motor(FLYWHEEL_TOP);
	flyTop->setGearing(okapi::AbstractMotor::gearset::blue);
	flyTop->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	okapi::Motor* flyBottom;
	flyBottom = new okapi::Motor(-FLYWHEEL_BOTTOM);
	flyBottom->setGearing(okapi::AbstractMotor::gearset::blue);
	flyBottom->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	flyTop->moveVoltage(12000);
	flyBottom->moveVoltage(12000);
	pros::delay(5000);  // give the flywheel some time to spin up

	printf("Velocity (RPM), Velocity (rads)\n");

	uint32_t now = pros::millis();
	// currently running for 10 seconds
	while (pros::millis() - now < 10000) {
		printf("%f, %f\n", flyTop->getActualVelocity(),
		       flyTop->getActualVelocity() * 0.1047);
		pros::delay(10);
	}

	flyTop->moveVoltage(0);
	flyBottom->moveVoltage(0);
}
}  // namespace test
