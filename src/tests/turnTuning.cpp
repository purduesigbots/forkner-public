#include "main.h"

using namespace okapi;

namespace test {
void turnTuning() {
  odom::start();
  pros::delay(150);
  odom::state start;
  odom::setState(start);
  LMPFTuner tuner(drive::turnController, true, 4_s,
                  // 1.57,
                  // 0.3, // short turn tuning
                  3.0, // long turn
                  0.6, 0.8, 0.5, 0.7, 0.0005, 0.002, 0.005, 0.02,
                  TimeUtilFactory::withSettledUtilParams(0.01, 0.0005), 5,
                  // 4,
                  20,
                  // 10,
                  2, 1);

  // LMPFTuner::Output out = tuner.autotune();
  // pros::lcd::print(7, "f %f, p %f, i %f, d %f", out.kF, out.kP, out.kI,
  // out.kD); flywheel::start(); pros::delay(10000); drive::setTurnGains(0.789,
  // 0.636, 0.00267, 0.0462, 0); drive::setTurnGainsCA();
  // drive::turnController->generatePath({0, 0.3}, "Test");
  // pros::delay(10000);
  drive::setTurnGainsUnderDamped();
  drive::generateTurnPath({0, 0.3}, "Test");
  drive::setTurnTarget("Test", odom::getState());
  drive::waitUntilSettled(0);
  pros::delay(4000);
  // 0.7, 0.7, 0.003938, 0.04
  drive::setTurnGainsOverDamped();
  drive::generateTurnPath({0, 3.0}, "Test");
  drive::setTurnTarget("Test", odom::getState());
  drive::waitUntilSettled(0);
  // pros::delay(3000);
  // drive::setTurnGains(0.8, 0.7588, 0.003, 0.03, 0);
  // drive::setTurnTarget("Test", odom::getState());
  // drive::waitUntilSettled(0);

  odom::stop();
}
} // namespace test
