/**
 * Linear Motion Profile Follower Tuner. Same idea as Okapi's PID tuner, but
 * modified to use an LMPF.
 */
#include "main.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

LMPFTuner::LMPFTuner(const std::shared_ptr<LinearMotionProfileFollower> ilmpf,
                     bool iturn, QTime itimeout, double igoal, double ikFMin,
                     double ikFMax, double ikPMin, double ikPMax, double ikIMin,
                     double ikIMax, double ikDMin, double ikDMax,
                     const TimeUtil &itimeUtil, std::size_t inumIterations,
                     std::size_t inumParticles, double ikSettle, double ikITAE)
    : lmpf(ilmpf), turn(iturn), timeUtil(itimeUtil), rate(timeUtil.getRate()),
      timeout(itimeout), goal(igoal), kFMin(ikFMin), kFMax(ikFMax),
      kPMin(ikPMin), kPMax(ikPMax), kIMin(ikIMin), kIMax(ikIMax), kDMin(ikDMin),
      kDMax(ikDMax), numIterations(inumIterations), numParticles(inumParticles),
      kSettle(ikSettle), kITAE(ikITAE) {}

LMPFTuner::~LMPFTuner() = default;

LMPFTuner::Output LMPFTuner::autotune() {
  std::random_device rd;  // Random seed
  std::mt19937 gen(rd()); // Mersenne twister
  std::uniform_real_distribution<double> dist(0, 1);

  std::vector<ParticleSet> particles;
  for (std::size_t i = 0; i < numParticles; i++) {
    ParticleSet set{};
    set.kF.pos = kFMin + (kFMax - kFMin) * dist(gen);
    set.kF.vel = set.kF.pos / increment;
    set.kF.best = set.kF.pos;

    set.kP.pos = kPMin + (kPMax - kPMin) * dist(gen);
    set.kP.vel = set.kP.pos / increment;
    set.kP.best = set.kP.pos;

    set.kI.pos = kIMin + (kIMax - kIMin) * dist(gen);
    set.kI.vel = set.kI.pos / increment;
    set.kI.best = set.kI.pos;

    set.kD.pos = kDMin + (kDMax - kDMin) * dist(gen);
    set.kD.vel = set.kD.pos / increment;
    set.kD.best = set.kD.pos;

    set.bestError = std::numeric_limits<double>::max();
    particles.push_back(set);
  }

  ParticleSet global{};
  global.kF.best = 0;
  global.kP.best = 0;
  global.kI.best = 0;
  global.kD.best = 0;
  global.bestError = std::numeric_limits<double>::max();

  // Run the optimization
  lmpf->generatePath({0, goal}, "Movement");
  bool firstGoal = true;
  for (std::size_t iteration = 0; iteration < numIterations; iteration++) {
    for (std::size_t particleIndex = 0; particleIndex < numParticles;
         particleIndex++) {
      lmpf->setGains(particles.at(particleIndex).kF.pos,
                     particles.at(particleIndex).kP.pos,
                     particles.at(particleIndex).kI.pos,
                     particles.at(particleIndex).kD.pos, 0);

      // Reverse the goal every iteration to stay in the same general area
      lmpf->flipDisable(false);
      odom::state expectedState;
      if (firstGoal) {
        if (turn) {
          expectedState.theta = 0_rad;
          odom::setState(expectedState);
          pros::delay(20);
          lmpf->setTarget("Movement", expectedState.theta.convert(radian));
        } else {
          expectedState.y = 0_m;
          odom::setState(expectedState);
          pros::delay(10);
          lmpf->setTarget("Movement", expectedState.y.convert(meter));
        }
        firstGoal = false;
      } else {
        if (turn) {
          expectedState.theta = QAngle(2 * goal);
          odom::setState(expectedState);
          pros::delay(20);
          lmpf->setTarget("Movement", expectedState.theta.convert(radian),
                          true);
        } else {
          expectedState.y = QLength(2 * goal);
          odom::setState(expectedState);
          pros::delay(20);
          lmpf->setTarget("Movement", expectedState.y.convert(meter), true);
        }
        firstGoal = true;
      }
      pros::lcd::print(0, "%f %f", expectedState.y.convert(meter),
                       odom::getState().y.convert(meter));

      QTime settleTime = 0_ms;
      double itae = 0;
      // Test constants then calculate fitness function
      while (!lmpf->isSettled()) {
        settleTime += loopDelta;
        if (settleTime > timeout)
          break;

        double inputVal;
        if (turn) {
          inputVal =
              lmpf->controllerGet() - expectedState.theta.convert(radian);
        } else {
          inputVal = lmpf->controllerGet() - expectedState.y.convert(meter);
        }
        const double error = lmpf->getError();
        // sum of the error emphasizing later error
        itae += (settleTime.convert(millisecond) * abs((int)error)) / divisor;

        rate->delayUntil(loopDelta);
      }

      const double error =
          kSettle * settleTime.convert(millisecond) + kITAE * itae;

      if (error < particles.at(particleIndex).bestError) {
        particles.at(particleIndex).kF.best =
            particles.at(particleIndex).kF.pos;
        particles.at(particleIndex).kP.best =
            particles.at(particleIndex).kP.pos;
        particles.at(particleIndex).kI.best =
            particles.at(particleIndex).kI.pos;
        particles.at(particleIndex).kD.best =
            particles.at(particleIndex).kD.pos;
        particles.at(particleIndex).bestError = error;

        if (error < global.bestError) {
          global.kF.best = particles.at(particleIndex).kF.pos;
          global.kP.best = particles.at(particleIndex).kP.pos;
          global.kI.best = particles.at(particleIndex).kI.pos;
          global.kD.best = particles.at(particleIndex).kD.pos;
          global.bestError = error;
        }
      }
    }

    // Update particle trajectories
    for (std::size_t i = 0; i < numParticles; i++) {
      // Factor in the particles inertia to keep on the same trajectory
      particles.at(i).kF.vel *= inertia;
      // Move towards particle's best
      particles.at(i).kF.vel +=
          confSelf *
          ((particles.at(i).kF.best - particles.at(i).kF.pos) / increment) *
          dist(gen);
      // Move towards swarm's best
      particles.at(i).kF.vel +=
          confSwarm * ((global.kF.best - particles.at(i).kF.pos) / increment) *
          dist(gen);
      // Kinematics
      particles.at(i).kF.pos += particles.at(i).kF.vel * increment;

      particles.at(i).kP.vel *= inertia;
      particles.at(i).kP.vel +=
          confSelf *
          ((particles.at(i).kP.best - particles.at(i).kP.pos) / increment) *
          dist(gen);
      particles.at(i).kP.vel +=
          confSwarm * ((global.kP.best - particles.at(i).kP.pos) / increment) *
          dist(gen);
      particles.at(i).kP.pos += particles.at(i).kP.vel * increment;

      particles.at(i).kI.vel *= inertia;
      particles.at(i).kI.vel +=
          confSelf *
          ((particles.at(i).kI.best - particles.at(i).kI.pos) / increment) *
          dist(gen);
      particles.at(i).kI.vel +=
          confSwarm * ((global.kI.best - particles.at(i).kI.pos) / increment) *
          dist(gen);
      particles.at(i).kI.pos += particles.at(i).kI.vel * increment;

      particles.at(i).kD.vel *= inertia;
      particles.at(i).kD.vel +=
          confSelf *
          ((particles.at(i).kD.best - particles.at(i).kD.pos) / increment) *
          dist(gen);
      particles.at(i).kD.vel +=
          confSwarm * ((global.kD.best - particles.at(i).kD.pos) / increment) *
          dist(gen);
      particles.at(i).kD.pos += particles.at(i).kD.vel * increment;

      particles.at(i).kF.pos = std::clamp(particles.at(i).kF.pos, kPMin, kPMax);
      particles.at(i).kP.pos = std::clamp(particles.at(i).kP.pos, kPMin, kPMax);
      particles.at(i).kI.pos = std::clamp(particles.at(i).kI.pos, kIMin, kIMax);
      particles.at(i).kD.pos = std::clamp(particles.at(i).kD.pos, kDMin, kDMax);

      lmpf->flipDisable(true);
      pros::delay(10);
    }
  }

  return Output{global.kF.best, global.kP.best, global.kI.best, global.kD.best};
}
