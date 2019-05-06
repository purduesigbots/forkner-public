/**
 * Flywheel Functionality
 */
#ifndef _FLYWHEEL_H_
#define _FLYWHEEL_H_

/**
 * Debug info
 *
 * Pass the LCD line number [0-7] for where the info should go or 8+ to turn the
 * printing off
 */
#define FLYWHEEL_MODE_INFO_LINE 0
#define FLYWHEEL_VELOCITY_SETPOINT_LINE 1
#define FLYWHEEL_MOTOR_TELEMETRY_LINE 2
#define INDEXER_STATE_INFO 8  // just for debug really

#define FLYWHEEL_SPEED_AUTON_HIGH 0
#define FLYWHEEL_SPEED_AUTON_MIDDLE 1
#define FLYWHEEL_SPEED_OP_CONTROL 2

// used to retain previous set voltage for indexOne()
#define INDEX_PREV_INTAKE -1

namespace flywheel {

/**
 *  Starts the flywheel tasks.
 */
void start();

/**
 * Suspends the Flywheel's tasks.
 */
void stop();

/**
 * Signals to fire a ball. The firing may not happen instantaneously, it will
 * wait until the flywheel is at the desired setpoint. Regardless, this
 * function will return as soon as the signal is sent.
 *
 * This is probably best for opcontrol.
 */
void shoot();

/**
 * Signals to fire a ball and blocks program execution until the ball is shot.
 *
 * This is probably best for auton.
 *
 * Logs a message with the "F," prefix.
 */
void shootBlocking();

/**
 * Signals to fire two balls, one at the high flag and one at the middle flag.
 * This does not block program execution.
 */
void doubleShoot(bool noDelay = true);

/**
 * Signals to fire two balls, one at the high flag and one at the middle flag.
 * Blocks program execution until both balls have been shot.
 *
 * Logs a message with the "F," prefix.
 */
void doubleShootBlocking(bool noDelay = true);

/**
 * Cue up a ball in the indexer but don't shoot it.
 *
 * \param intakePower
 *        The voltage value to set to the intake after the indexing is completed
 *
 * Logs a message with the "F," prefix.
 */
void indexOne(int intakePower = INDEX_PREV_INTAKE);

/**
 * Checks the indexer photogate to see if we've indexed a ball.
 *
 * \return True if there is a ball in the indexer, false otherwise
 */
bool indexerCheck();

/**
 * Guesses at whether the intake has a ball in it based on the speed/vs. set
 * voltage of the intake. Don't use this in auton.
 */
bool intakeCheck();

/**
 * Set the intake motor voltage.
 *
 * /param power
 *        The intake motor power value [-127, 127]
 */
void intake(int power, bool inoStallCheck = false);

/**
 * Set the intake's lift state. HIGH is up, LOW is down.
 *
 * /param state
 *        HIGH is up, LOW is down.
 */
void setIntakeLift(bool state);

/**
 * Passthrough call to okapi::Motor::setCurrentLimit() for the intake.
 *
 * \param limit
 *        The new current limit for the intake motor in mA
 */
void setIntakeCurrentLimit(int limit);

/**
 * Sets the piston state for the angle changing mechanism.
 *
 * \param val
 *        HIGH will extend the angle changer, LOW will retract it.
 */
void setAngleChanger(bool val);

/**
 * Set one of the flywheel velocity setpoints to the flywheel.
 *
 * Logs a message with the "F," prefix.
 *
 * /param velIndex
 *        The array index for the desired setpoint
 *
 * /return The string describing the velocity setpoint
 */
const char* setVelocitySetpoint(int velIndex);

/**
 * Gets the string describing the velocity setpoint.
 *
 * \return The string describing the velocity setpoint
 */
const char* getVelocitySetpointString();

bool isIntakeStalled();

}  // namespace flywheel

#endif
