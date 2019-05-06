#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::ADIDigitalOut tmp1(INTAKE_LIFT);
	tmp1.set_value(LOW);
	pros::ADIDigitalOut tmp2(ANGLE_CHANGER);
	tmp2.set_value(LOW);

	drive::init();
	flipper::init();
	auton::acConfig();
	auton::caConfig();
	auton::psConfig();
	auton::defaultConfig();

	capCam = std::make_shared<Camera>(CAP_CAMERA, true);

	lcdSelector::init();

	if (lcdSelector::getSide() == auton::color::red) {
		capCam->setColor(true);
	} else {
		capCam->setColor(false);
	}

	if (!pros::lcd::is_initialized()) {
		pros::lcd::initialize();
	}

	flywheel::start();
	odom::start();
	pros::lcd::print(0, "Initializing flywheel/odom");
	pros::delay(1000);
	pros::lcd::clear_line(0);
	flywheel::stop();
	odom::stop();
	printf("finished init\n");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	if (logger) logger->close();
	pros::ADIDigitalOut tmp1(INTAKE_LIFT);
	tmp1.set_value(LOW);
	pros::ADIDigitalOut tmp2(ANGLE_CHANGER);
	tmp2.set_value(LOW);
	odom::stop();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	// This takes a couple seconds to generate an RNG seed
	logger = new DataLogger(DataLogger::output::SD_CARD);

	lcdSelector::select();
	lcdSelector::executeInit();
	printf("finished comp init\n");
}
