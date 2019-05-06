/**
 * Drivetrain movement for auton.
 *
 * A good guide for this algorithm can be found at:
 * https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf
 */
#include <algorithm>
#include <numeric>
#include "main.h"

using namespace okapi;

// This will print a csv file to the terminal for each movement
#define PURE_PURSUIT_DEBUG 0

PurePursuitFollower::PurePursuitFollower(
    double imaxVel, double imaxAccel, double imaxJerk,
    const std::shared_ptr<ChassisControllerIntegrated>& ichassis,
    const TimeUtil& itimeUtil)
    : AsyncMotionProfileController(
          itimeUtil, imaxVel, imaxAccel, imaxJerk, ichassis->getChassisModel(),
          ichassis->getChassisScales(), ichassis->getGearsetRatioPair()) {
	chassis = ichassis;
	rctrl = std::make_shared<RamseteController>(kB, kZeta);
	this->startThread();
}

void PurePursuitFollower::rotatePoint(double* x, double* y, double angle) {
	double xin, yin;
	xin = *x;
	yin = *y;
	*x = cos(angle) * xin + sin(angle) * yin;
	*y = -sin(angle) * xin + cos(angle) * yin;
}

void PurePursuitFollower::setTarget(std::string ipathId, odom::state iexpected,
                                    bool ibackwards, bool icurvatureMode,
                                    bool iconstrainEnd) {
	expectedState = iexpected;
	curvatureMode.store(icurvatureMode, std::memory_order_release);
	currentPath = ipathId;
	isRunning.store(true, std::memory_order_release);
	direction.store(boolToSign(!ibackwards), std::memory_order_release);
	constrainEnd.store(iconstrainEnd, std::memory_order_release);
}

int32_t PurePursuitFollower::waitUntilSettled(
    int itimeout, std::function<bool()> exitCondition) {
	// make it as large as possible, so effectively no timeout
	if (!itimeout) itimeout = ~itimeout;
	timeout = itimeout;
	auto rate = timeUtil.getRate();
	uint32_t now = pros::millis();
	bool settled = isSettled(), timeLeft = (pros::millis() - now < timeout),
	     exit = exitCondition();
	while (!settled && timeLeft && !exit) {
		rate->delayUntil(10_ms);

		settled = isSettled();
		timeLeft = (pros::millis() - now < timeout);
		exit = exitCondition();
	}
	this->flipDisable(true);
	model->setBrakeMode(AbstractMotor::brakeMode::hold);
	model->stop();
	if (settled)
		return 0;
	else if (!timeLeft)
		return 1;
	else if (exit)
		return 2;
}

void PurePursuitFollower::executeSinglePath(
    const TrajectoryPair& path, std::unique_ptr<AbstractRate> rate) {
		// REDACTED
}

void PurePursuitFollower::setAngleModeGains(/* REDACTED */) {
	// REDACTED
}

void PurePursuitFollower::setCurvatureModeGains(/* REDACTED */) {
	// REDACTED
}

void PurePursuitFollower::setRamseteGains(/* REDACTED */) {
	// REDACTED
}

void PurePursuitFollower::generatePath(std::initializer_list<Point> iwaypoints,
                                       const std::string& ipathId,
                                       double imaxVel, double imaxAccel,
                                       double imaxJerk) {
	if (iwaypoints.size() == 0) {
		// No point in generating a path
		logger->warn(
		    "AsyncMotionProfileController: Not generating a path because no "
		    "waypoints were given.");
		return;
	}

	std::vector<Waypoint> points;
	points.reserve(iwaypoints.size());
	for (auto& point : iwaypoints) {
		points.push_back(Waypoint{point.x.convert(meter), point.y.convert(meter),
		                          point.theta.convert(radian)});
	}

	TrajectoryCandidate candidate;
	logger->info("AsyncMotionProfileController: Preparing trajectory");
	pathfinder_prepare(points.data(), static_cast<int>(points.size()),
	                   FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST,
	                   0.01,  // was 0.001
	                   imaxVel, imaxAccel, imaxJerk, &candidate);

	const int length = candidate.length;

	if (length < 0) {
		auto pointToString = [](Waypoint point) {
			return "Point{x = " + std::to_string(point.x) +
			       ", y = " + std::to_string(point.y) +
			       ", theta = " + std::to_string(point.angle) + "}";
		};

		std::string message =
		    "AsyncMotionProfileController: Path is impossible with waypoints: " +
		    std::accumulate(std::next(points.begin()), points.end(),
		                    pointToString(points.at(0)),
		                    [&](std::string a, Waypoint b) {
			                    return a + ", " + pointToString(b);
		                    });

		logger->error(message);

		if (candidate.laptr) {
			free(candidate.laptr);
		}

		if (candidate.saptr) {
			free(candidate.saptr);
		}

		throw std::runtime_error(message);
	}

	auto* trajectory = static_cast<Segment*>(malloc(length * sizeof(Segment)));

	if (trajectory == nullptr) {
		std::string message =
		    "AsyncMotionProfileController: Could not allocate trajectory. The path "
		    "is probably impossible.";
		logger->error(message);

		if (candidate.laptr) {
			free(candidate.laptr);
		}

		if (candidate.saptr) {
			free(candidate.saptr);
		}

		throw std::runtime_error(message);
	}

	logger->info("AsyncMotionProfileController: Generating path");
	pathfinder_generate(&candidate, trajectory);

	auto* leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
	auto* rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

	if (leftTrajectory == nullptr || rightTrajectory == nullptr) {
		std::string message =
		    "AsyncMotionProfileController: Could not allocate left and/or right "
		    "trajectories. The path is probably impossible.";
		logger->error(message);

		if (leftTrajectory) {
			free(leftTrajectory);
		}

		if (rightTrajectory) {
			free(rightTrajectory);
		}

		if (trajectory) {
			free(trajectory);
		}

		throw std::runtime_error(message);
	}

	logger->info("AsyncMotionProfileController: Modifying for tank drive");
	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
	                       scales.wheelbaseWidth.convert(meter));

	free(trajectory);

	// Free the old path before overwriting it
	removePath(ipathId);

	paths.emplace(ipathId,
	              TrajectoryPair{leftTrajectory, rightTrajectory, length});
	logger->info("AsyncMotionProfileController: Completely done generating path");
	logger->info("AsyncMotionProfileController: " + std::to_string(length));
}

void PurePursuitFollower::reset() {
	prevTurn = 0.0;
	quickStopAccumlator = 0.0;
	negInertiaAccumlator = 0.0;
	prevThrottle = 0.0;
}

double PurePursuitFollower::getError() {
	return error;
}
