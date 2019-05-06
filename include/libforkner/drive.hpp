/**
 * Drivetrain stuff
 */
#ifndef _DRIVE_HPP_
#define _DRIVE_HPP_

#include "main.h"

#define DRIVE_LINEAR_SPEED_REDUCTION \
	0.0f  // fiddle factor to reduce max linear vel
#define DRIVE_LINEAR_MAX_VEL                                    \
	((DRIVE_MOTOR_MAX_VEL / 60) * DRIVE_DRIVEN_WHEEL_DIAM * PI) - \
	    DRIVE_LINEAR_SPEED_REDUCTION

// Not calculated, just guess here
#define DRIVE_LINEAR_MAX_ACCEL 2.4f
// #define DRIVE_LINEAR_MAX_ACCEL 3.0f // uncomment for prog skills
#define DRIVE_LINEAR_MAX_JERK 6.0f
// #define DRIVE_LINEAR_MAX_JERK 15.0f // uncomment for prog skills

#define DRIVE_TURN_SPEED_REDUCTION 0.3f
#define DRIVE_TURN_MAX_VEL                                       \
	(((DRIVE_MOTOR_MAX_VEL / 60) * DRIVE_DRIVEN_WHEEL_DIAM * PI) / \
	 (DRIVE_CHASSIS_WIDTH / 2)) -                                  \
	    DRIVE_TURN_SPEED_REDUCTION  // was divided by 2 before
// not calculated, just guess here
#define DRIVE_TURN_MAX_ACCEL 10.0f
#define DRIVE_TURN_MAX_JERK 20.0f

/**
 * waitUntilSettled error codes
 */
#define DRIVE_EXIT_NORMAL 0
#define DRIVE_EXIT_TIMEOUT 1
#define DRIVE_EXIT_LINE 2

namespace drive {
extern std::shared_ptr<LinearMotionProfileFollower> turnController,
    linearController;
extern std::shared_ptr<PurePursuitFollower> driveController;

/**
 * Creates relevant objects for the drivetrain subsystem and starts their tasks.
 */
void init();

/**
 * Disables the LMPF and Pure Pursuit Controllers.
 */
void stopTasks();

/**
 * A wrapper to the Pure Pursuit Controller's generatePath method that allows
 * for configurable kinematic constraints.
 *
 * \param iwaypoints
 *        A list of 2+ points that the path will cross through
 * \param ipathId
 *        The name for the path
 * \param imaxVel
 *        The maximum forward velocity that the robot will be expected to drive
 *        at
 * \param imaxAccel
 *        The maximum forward acceleration that the robot will be expected to
 *        drive at
 * \param imaxJerk
 *        The maximum forward jerk that the robot will be expected to drive at
 */
void generateDrivePath(std::initializer_list<okapi::Point> iwaypoints,
                       const std::string& ipathId,
                       double imaxVel = DRIVE_LINEAR_MAX_VEL,
                       double imaxAccel = DRIVE_LINEAR_MAX_ACCEL,
                       double imaxJerk = DRIVE_LINEAR_MAX_JERK);

/**
 * A wrapper to the LMPF's generatePath method that allows for configurable
 * kinematic constraints.
 *
 * \param iwaypoints
 *        A list of 2+ points that the path will cross through
 * \param ipathId
 *        The name for the path
 * \param imaxVel
 *        The maximum forward velocity that the robot will be expected to turn
 *        at
 * \param imaxAccel
 *        The maximum forward acceleration that the robot will be expected to
 *        turn at
 * \param imaxJerk
 *        The maximum forward jerk that the robot will be
 *        expected to turn at
 */
void generateTurnPath(std::initializer_list<double> iwaypoints,
                      const std::string ipathId,
                      double imaxVel = DRIVE_TURN_MAX_VEL,
                      double imaxAccel = DRIVE_TURN_MAX_ACCEL,
                      double imaxJerk = DRIVE_TURN_MAX_JERK);

/**
 * A wrapper to the LMPF's generatePath method that allows
 * for configurable kinematic constraints.
 *
 * \param iwaypoints
 *        A list of 2+ points that the path will cross through
 * \param ipathId
 *        The name for the path
 * \param imaxVel
 *        The maximum forward velocity that the robot will be expected to drive
 *        at
 * \param imaxAccel
 *        The maximum forward acceleration that the robot will be expected to
 *        drive at
 * \param imaxJerk
 *        The maximum forward jerk that the robot will be
 *        expected to drive at
 */
void generateLinearDrivePath(std::initializer_list<double> iwaypoints,
                             const std::string ipathId,
                             double imaxVel = DRIVE_LINEAR_MAX_VEL,
                             double imaxAccel = DRIVE_LINEAR_MAX_ACCEL,
                             double imaxJerk = DRIVE_LINEAR_MAX_JERK);

/**
 * Starts a Pure Pursuit movement in its task.
 *
 * Logs a message starting with the "M," prefix.
 *
 * \param itarget
 *        The path name to be executed
 * \param iexpected
 *        The expected starting pose for the movement
 * \param ireversed
 *        True for driving backwards, false for driving forwards
 * \param icurvatureMode
 *        True for using a traditional Pure Pursuit calculation, false for the
 *        proportional heading control.
 * \param iconstrainEnd
 *        True will run the RAMSETE controller after the path is finished to
 *        ensure that the desired end position is reached.
 */
void setDriveTarget(std::string itarget, odom::state iexpected,
                    bool ireversed = false, bool icurvatureMode = false,
                    bool iconstrainEnd = false);

/**
 * Starts a Turn Linear Motion Profile Movement in its task.
 *
 * Logs a message starting with the "M," prefix.
 *
 * \param itarget
 *        The path name to be executed
 * \param iexpected
 *        The expected starting pose for the movement
 * \param ireversed
 *        True for turning CCW, false for turning CW
 */
void setTurnTarget(std::string itarget, odom::state iexpected,
                   bool ireversed = false);

/**
 * Starts a Forward/Backward Drive Linear Motion Profile Movement in its task.
 *
 * This will not constrain heading, but is much more reliable at reaching
 * exactly the goal distance compared to a pure pursuit movement.
 *
 * Logs a message starting with the "M," prefix.
 *
 * \param itarget
 *        The path name to be executed
 * \param iexpected
 *        The expected starting pose for the movement
 * \param ireversed
 *        True for backwards, false for forwards
 */
void setLinearDriveTarget(std::string target, odom::state iexpected,
                          bool reversed = false);

/**
 * Delays program execution until the drivetrain's active movement exits, the
 * timeout is exceeded or a line is detected.
 *
 * \param itimeout
 *        The maximum allowed duration of the movement. Passing 0 will allow
 *        for an unlimited duration.
 * \param icheckForLine
 *        True will stop the movement if a line is detected, false will not.
 *
 * \return One of the DRIVE_EXIT_* macros indicating why the movement ended.
 */
int32_t waitUntilSettled(int itimeout, bool icheckForLine = false, bool iflipCap = false);

/**
 * Sets the Angle Mode gains for the drive's Pure Pursuit Controller. Passing 0
 * for any of the gains will not change the value.
 */
void setDriveAMGains(/* REDACTED */);

/**
 * Sets the Curvature Mode gains for the drive's Pure Pursuit Controller.
 * Passing 0 for any of the gains will not change the value.
 */
void setDriveCMGains(/* REDACTED */);

/**
 * Sets the gain values for the RAMSETE controller used for end position
 * correction. Setting 0 for any of the gain values will prevent changing
 * from the previous value.
 *
 * \param ib
 *        Akin to a proportional gain. Must be > 0.
 * \param izeta
 *        A damping gain. Must be > 0 and < 1.
 * \param ilinVelkP
 *        Proportional constant for the linear velocity estimate.
 * \param iangVelkP
 *        Proportional constant for the angular velocity estimate.
 * \param isettleError
 *        The maximum admissable error value for the SettledUtil in meters.
 */
void setDriveRamseteGains(double ib, double izeta = 0, double ilinVelkP = 0,
                          double iangVelkP = 0, double isettleError = 0);

/**
 * Sets the default gains for the drivetrain's Pure Pursuit Controller
 */
void setDriveGainDefaults();

/**
 * Sets the gain values for the Linear Motion Profile Controller responsible
 * for point turning. Setting 0 for any of the values will not change the value.
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
void setTurnGains(double ikF, double ikP, double ikI, double ikD,
                  double iintegralCap);

/**
 * Sets the default gain values for the Linear Motion Profile Controller
 * responsible for point turning.
 */
void setTurnGainsDefault();

/**
 * Sets an overdamped set of gain values to the turn LMPF
 */
void setTurnGainsOverDamped();

/**
 * Sets an underdamped set of gain values to the turn LMPF
 */
void setTurnGainsUnderDamped();

/**
 * Frees the malloc'ed memory used for the specified path and removes it from
 * the list of available drive paths.
 *
 * \param itarget
 *        The name of the path to be freed
 */
void freeDrivePath(std::string itarget);

/**
 * Frees the malloc'ed memory used for the specified path and removes it from
 * the list of available turn paths.
 *
 * \param itarget
 *        The name of the path to be freed
 */
void freeTurnPath(std::string itarget);

/**
 * Frees the malloc'ed memory used for the specified path and removes it from
 * the list of available linear drive paths.
 *
 * \param itarget
 *        The name of the path to be freed
 */
void freeLinearDrivePath(std::string itarget);

/**
 * Sets the brake mode for the drivetrain's motors.
 *
 * \param imode
 *        The new brake mode for the motors
 */
void setBrakeMode(okapi::AbstractMotor::brakeMode imode);

/**
 * Sets the maximum output voltage for the drivetrain motors.
 *
 * \param ilimit
 *        The new maximum output voltage
 */
void setDriveMaxVoltage(int ilimit);

void setDriveMaxCurrent(int ilimit);

/**
 * Constant curvature drive control designed by FRC Team 254 The Cheesy Poofs.
 * This is a replacement for traditional arcade control.
 *
 * \param ithrottle
 *        The forward/backward stick input
 * \param iturn
 *        The turn stick input
 */
void cheesyDrive(double ithrottle, double iturn);

/**
 * A preconfigured routine that will drive backwards until the robot senses that
 * it has gone up two platforms.
 */
void park();

/**
 * Returns the voltage currently set to the left side motors.
 */
int32_t getLeftVoltage();

/**
 * Returns the voltage currently set to the right side motors.
 */
int32_t getRightVoltage();

/**
 * Returns the average current drawn by the drivtreain motors
 */
int32_t getCurrentDraw();

/**
 * Returns the average velocity of the drivetrain.
 */
double getVelocity();

/**
 * Turns the rubber band release mechanism to prepare for parking
 */
void releaseBands();

/**
 * Helper Classes for the Linear Motion Profile Controllers
 */
class TurnOutput : public okapi::ControllerOutput<double> {
	public:
	virtual void controllerSet(double value) override;
};

class TurnInput : public okapi::ControllerInput<double> {
	public:
	virtual double controllerGet() override;
};

class LinearOutput : public okapi::ControllerOutput<double> {
	public:
	virtual void controllerSet(double value) override;
};

class LinearInput : public okapi::ControllerInput<double> {
	public:
	virtual double controllerGet() override;
};
}  // namespace drive

#endif
