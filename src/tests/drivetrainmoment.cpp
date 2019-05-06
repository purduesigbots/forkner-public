#include "main.h"

using namespace okapi;

namespace test {
void drivetrainmoment() {
  auto controller = ChassisControllerFactory::create(
      {-DRIVE_LEFT_1, DRIVE_LEFT_2, -DRIVE_LEFT_3, DRIVE_LEFT_4},
      {DRIVE_RIGHT_1, -DRIVE_RIGHT_2, DRIVE_RIGHT_3, -DRIVE_RIGHT_4},
      AbstractMotor::gearset::green, {4_in, 12_in});

  controller.forward(1);
  uint32_t now = pros::millis();
  double prevDist = 0;
  while (pros::millis() - now < 2000) {
    std::valarray enc = controller.getSensorVals();
    double dist = (enc[0] + enc[1]) / 2;
    double vel =
        ((dist - prevDist) * (PI / 180) * DRIVE_DRIVEN_WHEEL_DIAM / 2) / 0.01;
    prevDist = dist;
    printf("%f\n", vel);

    pros::delay(10);
  }
  controller.forward(0);
  pros::delay(2000);
  printf("\n\nangvel\n\n");

  now = pros::millis();
  controller.rotate(1);
  std::valarray p = controller.getSensorVals();
  prevDist = (p[0] - p[1]) / 2;
  while (pros::millis() - now < 2000) {
    std::valarray enc = controller.getSensorVals();
    double dist = (enc[0] - enc[1]) / 2;
    double vel =
        ((dist - prevDist) * (PI / 180) * DRIVE_DRIVEN_WHEEL_DIAM / 2) /
        (ENC_CHASSIS_WIDTH * 0.01);
    prevDist = dist;
    printf("%f\n", vel);

    pros::delay(10);
  }
}
} // namespace test
