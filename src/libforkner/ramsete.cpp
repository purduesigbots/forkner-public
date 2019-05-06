/**
 * RAMSETE Controller Implementation
 *
 * Best source for the math for this is:
 * https://file.tavsys.net/control/state-space-guide.pdf
 */
#include "main.h"

using namespace okapi;

RamseteController::RamseteController(double ib, double izeta)
    : b(ib), zeta(izeta) {}

void RamseteController::setTarget(QLength ix, QLength iy, QAngle itheta,
                                  QSpeed ivel, QAngularSpeed iomega) {
  desX = ix.convert(meter);
  desY = iy.convert(meter);
  desT = itheta.convert(radian);
  velDes = ivel.convert(mps);
  omegaDes = iomega.convert(radps);
}

RamseteController::output RamseteController::step(QLength ix, QLength iy,
                                                  QAngle itheta) {
  // Easier to just do the conversion to ROS coordinates here
  double ey = desX - ix.convert(meter);
  double ex = desY - iy.convert(meter);
  double et = desT - itheta.convert(radian);
  double ct = PI / 2 - itheta.convert(radian);
  ex = cos(-ct) * ex - sin(-ct) * ey;
  ey = sin(-ct) * ex + cos(-ct) * ey;

  double k = 2 * zeta * sqrt(omegaDes * omegaDes + b * velDes * velDes);
  double vel = velDes * cos(et) + k * ex; // ey in odom coords
  double omega = omegaDes + k * et + b * velDes * sin(et) * ey / et;

  output out;
  out.linVel = vel;
  out.angVel = omega;
  return out;
}

RamseteController::output RamseteController::step(odom::state ipose) {
  return step(ipose.x, ipose.y, ipose.theta);
}

void RamseteController::setGains(double ib, double izeta) {
  if (ib)
    b = ib;
  if (izeta)
    zeta = izeta;
}
