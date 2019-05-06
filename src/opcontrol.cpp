#include "main.h"
// XXX: Okapi Controller currently has to be declared as global to work
okapi::Controller master(master);

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	flywheel::start();
	flywheel::setVelocitySetpoint(2);

	drive::stopTasks();
	drive::setDriveMaxVoltage(12000);

	bool intakeIn = true;

	okapi::ControllerButton buttonUp = master[okapi::ControllerDigital::up];
	okapi::ControllerButton buttonDown = master[okapi::ControllerDigital::down];
	okapi::ControllerButton buttonLeft = master[okapi::ControllerDigital::left];
	okapi::ControllerButton buttonRight = master[okapi::ControllerDigital::right];
	okapi::ControllerButton buttonL1 = master[okapi::ControllerDigital::L1];
	okapi::ControllerButton buttonL2 = master[okapi::ControllerDigital::L2];
	okapi::ControllerButton buttonL2Copy = master[okapi::ControllerDigital::L2];
	okapi::ControllerButton buttonR1 = master[okapi::ControllerDigital::R1];
	okapi::ControllerButton buttonR2 = master[okapi::ControllerDigital::R2];
	okapi::ControllerButton buttonX = master[okapi::ControllerDigital::X];
	okapi::ControllerButton buttonY = master[okapi::ControllerDigital::Y];
	okapi::ControllerButton buttonA = master[okapi::ControllerDigital::A];
	okapi::ControllerButton buttonB = master[okapi::ControllerDigital::B];

	bool prevIndexState = !flywheel::indexerCheck();
	bool prevIntakeState = !flywheel::intakeCheck();
	int printIndex = 0;

	flipper::retract();
	bool flipperDown = false;
	while (true) {
		// pros::lcd::print(7, "%f %f %f", odom::getState().x.convert(inch),
		//                  odom::getState().y.convert(inch),
		//                  odom::getState().theta.convert(degree));
		drive::cheesyDrive(master.getAnalog(okapi::ControllerAnalog::leftY),
		                   master.getAnalog(okapi::ControllerAnalog::rightX));

		if (buttonL2.isPressed() && !buttonL1.isPressed()) {
			flywheel::intake(-12000);
		} else if (intakeIn) {
			flywheel::intake(12000);
		} else {
			flywheel::intake(0);
		}

		// Indexer
		if (buttonA.changedToPressed()) {
			// fire!!
			flywheel::shoot();
		} else if (buttonY.changedToPressed()) {
			flywheel::indexOne();
		}
		if (buttonR1.changedToPressed()) {
			flywheel::setIntakeLift(HIGH);
			// flipper::tip();
			flipperDown = true;
		} else if (buttonR2.changedToPressed()) {
			flywheel::setIntakeLift(LOW);
			// flipper::retract();
			flipperDown = false;
		}

		if (buttonB.changedToPressed()) {
			flywheel::doubleShoot(true);
		} else if (buttonX.changedToPressed()) {
			flywheel::setAngleChanger(ANGLE_CHANGER_OUT);
			pros::delay(50);
			flywheel::shoot();
		}

		// Only print every 125ms
		if (!(printIndex % 25)) {
			// Print indexer status to the controller
			if (flywheel::indexerCheck() ^ prevIndexState) {
				if (flywheel::indexerCheck())
					master.setText(0, 0, "Indexed:   YES");
				else
					master.setText(0, 0, "Indexed:   NO ");
				prevIndexState = flywheel::indexerCheck();
			}

			// Print a guess at the intake state to the controller
			if (flywheel::intakeCheck() ^ prevIntakeState) {
				if (flywheel::intakeCheck())
					master.setText(1, 0, "Intaken:   YES");
				else
					master.setText(1, 0, "Intaken:   NO ");
				prevIntakeState = flywheel::intakeCheck();
			}

			printIndex = 1;
		} else {
			printIndex++;
		}

		// Cap Flipper
		if (buttonL1.isPressed() && buttonL2.isPressed()) {
			intakeIn = true;
			flipper::tip();
		} else if (!flipperDown) {
			flipper::retract();
		}

		if (buttonRight.changedToPressed()) {
			flipperDown = true;
			drive::releaseBands();
		}

		pros::delay(5);
	}
}
