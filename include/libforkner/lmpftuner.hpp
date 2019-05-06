/**
 * Linear Motion Profile Follower Tuner. Same idea as Okapi's PID tuner, but
 * modified to use an LMPF.
 */

class LMPFTuner {
	public:
	struct Output {
		double kF, kP, kI, kD;
	};

	/**
	 * Actuates the LMPF Controller back and forth to automatically tune its gain
	 * values.
	 *
	 * \param ilmpf
	 *        The LMPF to tune
	 * \param iturn
	 *        True if the movement is a turn, False if it is forward/backwards
	 * \param itimeout
	 *        The maximum amount of time that a test movement is allowed to run
	 * \param igoal
	 *        The target value for the test movements to move to/from
	 * \param ikFMin
	 *        The minimum value for the kF gain
	 * \param ikFMax
	 *        The maximum value for the kF gain
	 * \param ikPMin
	 *        The minimum value for the kP gain
	 * \param ikPMax
	 *        The maximum value for the kP gain
	 * \param ikIMin
	 *        The minimum value for the kI gain
	 * \param ikIMax
	 *        The maximum value for the kI gain
	 * \param ikDMin
	 *        The minimum value for the kD gain
	 * \param ikDMax
	 *        The maximum value for the kD gain
	 * \param itimeUtil
	 *        The okapi timeUtil that governs the tuner
	 * \param inumIterations
	 *        The number of times that each particle will be updated and tested
	 * \param inumParticles
	 *        The number of gain sets that will be evaluated each iteration
	 * \param ikSettle
	 *        The cost function weighting for the movement's settle time
	 * \param ikITAE
	 *        The cost function weighting for the movement's sum of time adjusted
	 *        error
	 */
	LMPFTuner(const std::shared_ptr<LinearMotionProfileFollower> ilmpf,
	          bool iturn, okapi::QTime itimeout, double igoal, double ikFMin,
	          double ikFMax, double ikPMin, double ikPMax, double ikIMin,
	          double ikIMax, double ikDMin, double ikDMax,
	          const okapi::TimeUtil& itimeUtil = okapi::TimeUtilFactory::create(),
	          std::size_t inumIterations = 5, std::size_t inumParticles = 16,
	          double ikSettle = 1, double ikITAE = 2);

	virtual ~LMPFTuner();

	/**
	 * Runs the LMPF gain autotuning routine. This will actuate the LMPF between
	 * its starting point and the goal point repeatedly until the specified number
	 * of iterations are complete.
	 *
	 * \return A struct containing the tuned set of LMPF gains
	 */
	virtual Output autotune();

	protected:
	static constexpr double inertia = 0.5;    // Particle inertia
	static constexpr double confSelf = 1.1;   // Self confidence
	static constexpr double confSwarm = 1.2;  // Particle swarm confidence
	static constexpr int increment = 5;
	static constexpr int divisor = 5;
	static constexpr okapi::QTime loopDelta =
	    okapi::QTime(static_cast<double>(10 / 1000));

	struct Particle {
		double pos, vel, best;
	};

	struct ParticleSet {
		Particle kF, kP, kI, kD;
		double bestError;
	};

	std::shared_ptr<LinearMotionProfileFollower> lmpf;
	okapi::TimeUtil timeUtil;
	std::unique_ptr<okapi::AbstractRate> rate;

	const okapi::QTime timeout;
	const double goal;
	const double kFMin;
	const double kFMax;
	const double kPMin;
	const double kPMax;
	const double kIMin;
	const double kIMax;
	const double kDMin;
	const double kDMax;
	const std::size_t numIterations;
	const std::size_t numParticles;
	const double kSettle;
	const double kITAE;

	bool turn;
};
