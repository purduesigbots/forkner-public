/**
 * Adaptive Pure Pursuit Control, sort of.
 */
#ifndef _PURE_PURSUIT_HPP_
#define _PURE_PURSUIT_HPP_

#include "main.h"
#include "ramsete.hpp"

class PurePursuitFollower : public okapi::AsyncMotionProfileController {
	public:
	PurePursuitFollower(
	    double imaxVel, double imaxAccel, double imaxJerk,
	    const std::shared_ptr<okapi::ChassisControllerIntegrated>& ichassis,
	    const okapi::TimeUtil& itimeUtil = okapi::TimeUtilFactory::create());

	/**
	 * Executes a path with the given ID. If there is no path matching the ID, the
	 * method will return. Any targets set while a path is being followed will be
	 * ignored.
	 *
	 * \param ipathId
	 *        A unique identifier for the path, previously passed to
	 *        generatePath().
	 * \param iexpected
	 *        The assumed starting spot for the movement. 0,0,0 should be passed
	 *        to all generatePath() calls, this will account for the robot's
	 *        position.
	 * \param ibackwards
	 *        True will run the path backwards
	 * \param icurvatureMode
	 *        True will use a traditional Pure Pursuit calculation, False will use
	 *        a proportional heading controller.
	 * \param iconstrainEnd
	 *        True will run the RAMSETE controller after the path is finished to
	 *        ensure that the desired end position is reached.
	 */
	void setTarget(std::string ipathId, odom::state iexpected,
	               bool ibackwards = false, bool icurvatureMode = false,
	               bool iconstrainEnd = false);

	/**
	 * Delays program execution until the robot is at its target.
	 *
	 * \param timeout
	 *        The maximum delay time for this function. Passing 0 will allow the
	 *        function to delay infinitely.
	 * \param iexitCondition
	 *        A function that will be checked every 5ms to look for a return value
	 *        of True, signaling that the movement should exit prematurely.
	 *
	 * \return 0 for normal exit, 1 for timeout, and 2 for exit condition
	 */
	int32_t waitUntilSettled(int timeout, std::function<bool()> iexitCondition);

	/**
	 * Sets the Angle Mode gains for the Pure Pursuit Controller. Passing 0 for
	 * any of the gains will not change the value.
	 */
	void setAngleModeGains(/* REDACTED */);

	/**
	 * Sets the Curvature Mode gains for the Pure Pursuit Controller. Passing 0
	 * for any of the gains will not change the value.
	 */
	void setCurvatureModeGains(/* REDACTED */);

	/**
	 * Sets the gain values for the RAMSETE controller. Setting 0 for any of the
	 * gain values will prevent changing from the previous value.
	 */
	void setRamseteGains(/* REDACTED */);

	/**
	 * Generates a path which intersects the given waypoints and saves it
	 * internally with a key of pathId. Call executePath() with the same pathId to
	 * run it.
	 *
	 * If the waypoints form a path which is impossible to achieve, an instance of
	 * std::runtime_error is thrown (and an error is logged) which describes the
	 * waypoints. If there are no waypoints, no path is generated.
	 *
	 * \param iwaypoints
	 *        The waypoints to hit on the path.
	 * \param ipathId
	 *        A unique identifier to save the path with.
	 * \param imaxVel
	 *        The maximum forward velocity that the robot will be expected to
	 *        drive at
	 * \param imaxAccel
	 *        The maximum forward acceleration that the robot will be expected to
	 *        drive at
	 * \param imaxJerk
	 *        The maximum forward jerk that the robot will be expected to drive at
	 */
	void generatePath(std::initializer_list<okapi::Point> iwaypoints,
	                  const std::string& ipathId, double imaxVel,
	                  double imaxAccel, double imaxJerk);

	/**
	 * Generates a path which intersects the given waypoints and saves it
	 * internally with a key of pathId. Call executePath() with the same pathId to
	 * run it.
	 *
	 * If the waypoints form a path which is impossible to achieve, an instance of
	 * std::runtime_error is thrown (and an error is logged) which describes the
	 * waypoints. If there are no waypoints, no path is generated.
	 *
	 * \param iwaypoints
	 *        The waypoints to hit on the path.
	 * \param ipathId
	 *        A unique identifier to save the path with.
	 */
	void generatePath(std::initializer_list<okapi::Point> iwaypoints,
	                  const std::string& ipathId);

	/**
	 * Gets the last error value passed to the SettledUtil.
	 *
	 * \return The controller's last error value.
	 */
	double getError();

	protected:
	/**
	 * Follow the supplied path. Must follow the disabled lifecycle.
	 */
	virtual void executeSinglePath(const TrajectoryPair& path,
	                               std::unique_ptr<okapi::AbstractRate> rate);

	/**
	 * Rotate the given point around the origin by the given angle.
	 *
	 * \param[in, out] x
	 *                 The X coordinate of the point to be rotated
	 * \param[in, out] y
	 *                 The Y coordinate of the point to be rotated
	 * \param angle
	 *        The angle to rotate by in radians
	 */
	void rotatePoint(double* x, double* y, double angle);

	void reset();

	std::shared_ptr<okapi::ChassisControllerIntegrated> chassis;

	int timeout = 0;

	double error = 0;

	odom::state expectedState;

	std::atomic_bool curvatureMode{false};
	std::atomic_bool constrainEnd{false};

	std::shared_ptr<RamseteController> rctrl;
};

#endif
