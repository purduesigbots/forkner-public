/**
 * Odometry
 *
 * x, y are in meters, theta is in radians.
 *
 *   ^ Positive Y
 *   |
 *   |
 *   ------> Positive X
 *
 * Theta is measured from the Y axis of the robot's starting configuration and
 * is positive in the clockwise direction.
 */
#ifndef _ODOM_HPP_
#define _ODOM_HPP_

using namespace okapi;

/**
 * Logging note: using the prefix 'O, ' to signal state/eS logs.
 */

namespace odom {
class state {
	public:
	QLength x, y;
	QAngle theta;

	state() {}
	state(const state& s2) {
		x = s2.x;
		y = s2.y;
		theta = s2.theta;
	}
};

/**
 * Similar to the flywheel, this will initialize the associated objects and
 * tasks, or resume the task if initialization has already occurred.
 *
 * \param ilog
 *        True turns on logging to the SD Card
 */
void start(bool ilog = false);

/**
 * This will suspend the odometry task.
 */
void stop();

/**
 * Returns the x,y,theta coordinates of the robot. x and y are in inches, theta
 * is in radians.
 *
 * \return The current position state of the robot.
 */
odom::state getState();

/**
 * Set the robot's position manually. Useful for aligning to the field or
 * initial setup.
 *
 * \param state
 *        The new state to set for the robot.
 */
void setState(odom::state istate);

/**
 * Returns the x and y distances for the given right triangle.
 *
 * \param hypotenuse
 *        The length of the movement
 * \param theta
 *        The angle that the movement was executed at
 *
 * \return The distance traveled in the given direction
 */
QLength getXLength(QLength ihypotenuse, QAngle itheta);
QLength getYLength(QLength ihypotenuse, QAngle itheta);
}  // namespace odom

extern odom::state expectedState;

#endif
