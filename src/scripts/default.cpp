#include "main.h"

using namespace okapi;

namespace auton {
void defaultConfig() {
  lcdSelector::titles.push_back("Default");
  lcdSelector::inits.push_back(auton::defaultInit);
  lcdSelector::scripts.push_back(auton::defaultScript);
}

void defaultInit(color side) {}

void defaultScript(color side, uint32_t ign1, uint32_t ign2) {
  flywheel::start();
  pros::delay(5000);
  flywheel::shootBlocking();
  pros::delay(1000);
  flywheel::stop();
}
} // namespace auton
