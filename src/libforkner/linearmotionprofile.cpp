/**
 * 1D Motion Profile Follower
 */
#include <numeric>
#include "main.h"

using namespace okapi;

LinearMotionProfileFollower::LinearMotionProfileFollower(
    double imaxVel, double imaxAccel, double imaxJerk, double ikF, double ikP,
    double ikI, double ikD,
    const std::shared_ptr<ControllerInput<double>>& iinput,
    const std::shared_ptr<ControllerOutput<double>>& ioutput,
    const TimeUtil& iPIDTimeUtil, const TimeUtil& iProfileTimeUtil)
    : okapi::AsyncLinearMotionProfileController(iProfileTimeUtil, imaxVel,
                                                imaxAccel, imaxJerk, ioutput),
      pid(std::make_shared<IterativePosPIDController>(
          ikP, ikI, ikD, 0, iPIDTimeUtil, std::make_unique<MedianFilter<5>>())),
      input(iinput) {
	// setting a default 0 for ikBias for now
	kF = ikF;
	kP = ikP;
	kI = ikI;
	kD = ikD;
	integralCap = 1;
	pid->setSampleTime(10_ms);
	this->startThread();
}

void LinearMotionProfileFollower::setTarget(std::string ipathId,
                                            double iexpected, bool ibackwards) {
	expectedState = iexpected;
	outDirection = boolToSign(!ibackwards);

	currentPath = ipathId;
	isRunning = true;
}

void LinearMotionProfileFollower::executeSinglePath(
    const TrajectoryPair& path, std::unique_ptr<AbstractRate> rate) {
	pidDone = false;
	pid->reset();
	double profileEndPosition =
	    outDirection * path.segment[path.length - 1].position + expectedState;
	for (int i = 0; i < path.length && !isDisabled(); ++i) {
		if (i)
			currentProfilePosition =
			    outDirection * path.segment[i - 1].position + expectedState;
		else
			currentProfilePosition =
			    outDirection * path.segment[0].position + expectedState;
		double feedforward = path.segment[i].velocity / maxVel;

		pid->setTarget(currentProfilePosition);
		state = input->controllerGet();
		if ((state > profileEndPosition) && (outDirection) == 1)
			break;
		else if ((state < profileEndPosition) && (outDirection) == -1)
			break;
		double pidOut = pid->step(state);

		output->controllerSet(outDirection * kF * feedforward + pidOut);

		rate->delayUntil(10_ms);
	}

	pid->setTarget(profileEndPosition);
	while (!pid->isSettled() && !isDisabled()) {
		// make sure we're exactly at the target before exiting
		output->controllerSet(pid->step(input->controllerGet()));
		rate->delayUntil(10_ms);
	}
	output->controllerSet(0);
	pidDone = true;
}

void LinearMotionProfileFollower::waitUntilSettled(int timeout) {
	if (!timeout)
		timeout =
		    ~timeout;  // make it as large as possible, so effectively no timeout
	auto rate = timeUtil.getRate();
	uint32_t now = pros::millis();
	while (!isSettled() && (pros::millis() - now < timeout)) {
		rate->delayUntil(10_ms);
	}
	// force a move to end if the timeout elapses
	output->controllerSet(0);
	this->flipDisable(true);
}

void LinearMotionProfileFollower::setGains(double ikF, double ikP, double ikI,
                                           double ikD, double iintegralCap) {
	if (!ikF) ikF = kF;
	if (!ikP) ikP = kP;
	if (!ikI) ikI = kI;
	if (!ikD) ikD = kD;
	if (!iintegralCap) iintegralCap = integralCap;
	pid->setGains(ikP, ikI, ikD);
	pid->setIntegralLimits(iintegralCap, -iintegralCap);
	kF = ikF;
	kP = ikP;
	kI = ikI;
	kD = ikI;
	integralCap = iintegralCap;
}

void LinearMotionProfileFollower::generatePath(
    std::initializer_list<double> iwaypoints, const std::string& ipathId,
    double imaxVel, double imaxAccel, double imaxJerk) {
	if (iwaypoints.size() == 0) {
		// No point in generating a path
		logger->warn(
		    "AsyncLinearMotionProfileController: Not generating a path because no "
		    "waypoints were given.");
		return;
	}

	std::vector<Waypoint> points;
	points.reserve(iwaypoints.size());
	for (auto& point : iwaypoints) {
		points.push_back(Waypoint{point, 0, 0});
	}

	TrajectoryCandidate candidate;
	logger->info("AsyncLinearMotionProfileController: Preparing trajectory");
	pathfinder_prepare(points.data(), static_cast<int>(points.size()),
	                   FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.01, imaxVel,
	                   imaxAccel, imaxJerk, &candidate);

	const int length = candidate.length;

	if (length < 0) {
		auto pointToString = [](Waypoint point) {
			return "Point{x = " + std::to_string(point.x) +
			       ", y = " + std::to_string(point.y) +
			       ", theta = " + std::to_string(point.angle) + "}";
		};

		std::string message =
		    "AsyncLinearMotionProfileController: Path is impossible with "
		    "waypoints: " +
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
		    "AsyncLinearMotionProfileController: Could not allocate trajectory. "
		    "The "
		    "path is probably impossible.";
		logger->error(message);

		if (candidate.laptr) {
			free(candidate.laptr);
		}

		if (candidate.saptr) {
			free(candidate.saptr);
		}

		throw std::runtime_error(message);
	}

	logger->info("AsyncLinearMotionProfileController: Generating path");
	pathfinder_generate(&candidate, trajectory);

	// Free the old path before overwriting it
	removePath(ipathId);

	paths.emplace(ipathId, TrajectoryPair{trajectory, length});
	logger->info(
	    "AsyncLinearMotionProfileController: Completely done generating path");
	logger->info("AsyncLinearMotionProfileController: " + std::to_string(length));
}

void LinearMotionProfileFollower::generatePath(
    std::initializer_list<double> iwaypoints, const std::string& ipathId) {
	generatePath(iwaypoints, ipathId, maxVel, maxJerk, maxAccel);
}

double LinearMotionProfileFollower::controllerGet() {
	return state;
}

std::shared_ptr<TimeUtil> LinearMotionProfileFollower::getTimeUtil() {
	return std::make_shared<TimeUtil>(timeUtil);
}
