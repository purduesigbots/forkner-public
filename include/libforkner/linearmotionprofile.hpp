/**
 * Follower class for a 1D motion profile.
 *
 * Subclasses Okapi's AsyncLinearMotionProfileController.
 */
#ifndef _LINEAR_MOTION_PROFILE_HPP_
#define _LINEAR_MOTION_PROFILE_HPP_

#include "main.h"

class LinearMotionProfileFollower
    : public okapi::AsyncLinearMotionProfileController {
	public:
	LinearMotionProfileFollower(
	    double imaxVel, double imaxAccel, double imaxJerk, double ikF, double ikP,
	    double ikI, double ikD,
	    const std::shared_ptr<okapi::ControllerInput<double>>& iinput,
	    const std::shared_ptr<okapi::ControllerOutput<double>>& ioutput,
	    const okapi::TimeUtil& iPIDTimeUtil = okapi::TimeUtilFactory::create(),
	    const okapi::TimeUtil& iProfileTimeUtil =
	        okapi::TimeUtilFactory::create());

	/**
	 * Delays program execution until the robot is at its target.
	 *
	 * \param timeout
	 *        The maximum delay time for this function. Passing 0 will allow the
	 *        function to delay infinitely.
	 */
	void waitUntilSettled(int timeout);

	/**
	 * Executes a path with the given ID. If there is no path matching the ID, the
	 * method will return. Any targets set while a path is being followed will be
	 * ignored.
	 *
	 * \param ipathId
	 *        A unique identifier for the path, previously passed to
	 *        generatePath().
	 * \param iexpected
	 *        The assumed starting spot for the movement. 0 should be passed
	 *        to all `generatePath` calls, this will account for the robot's
	 *        position.
	 * \param ibackwards
	 *        True will run the path backwards
	 */
	void setTarget(std::string ipathId, double iexpected,
	               bool ibackwards = false);

	/**
	 * Sets the gain values for the follower's PID controller.
	 *
	 * \param ikF
	 *        A multiplier for the feedforward output from the motion profile
	 * \param ikP
	 *        The proportional constant
	 * \param ikI
	 *        The integral constant
	 * \param ikD
	 *        The derivative constant
	 * \param iintegralCap
	 *        The maximum value that the integral term can be. Default is 1.
	 */
	void setGains(double ikF, double ikP, double ikI, double ikD,
	              double iintegralCap);

	/**
	 * Creates a motion profile path that crosses through the given points.
	 *
	 * \param iwaypoints
	 *        A list of 2+ points that the path will cross through
	 * \param ipathId
	 *        The name for the path
	 * \param imaxVel
	 *        The maximum forward velocity for the movement
	 * \param imaxAccel
	 *        The maximum forward acceleration for the movement
	 * \param imaxJerk
	 *        The maximum forward jerk for the movement
	 */
	void generatePath(std::initializer_list<double> iwaypoints,
	                  const std::string& ipathId, double imaxVel,
	                  double imaxAccel, double imaxJerk);

	/**
	 * Creates a motion profile path that crosses through the given points.
	 *
	 * \param iwaypoints
	 *        A list of 2+ points that the path will cross through
	 * \param ipathId
	 *        The name for the path
	 */
	void generatePath(std::initializer_list<double> iwaypoints,
	                  const std::string& ipathId);

	/**
	 * Helper functions for the tuner
	 */

	/**
	 * Gets the value of the input.
	 */
	double controllerGet();

	/**
	 * Returns the TimeUtil for the LMPF.
	 */
	std::shared_ptr<okapi::TimeUtil> getTimeUtil();

	std::atomic_bool pidDone{true};

	protected:
	virtual void executeSinglePath(const TrajectoryPair& path,
	                               std::unique_ptr<okapi::AbstractRate> rate);

	std::shared_ptr<okapi::IterativePosPIDController> pid;
	std::shared_ptr<okapi::ControllerInput<double>> input;

	double expectedState;

	int outDirection;
	double kF, kP, kI, kD, integralCap;
	double state;
};

#endif
