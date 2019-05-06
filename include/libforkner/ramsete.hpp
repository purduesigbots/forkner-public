/**
 * Ramsete controller functionality
 */
#ifndef _RAMSETE_HPP_
#define _RAMSETE_HPP_

#include "main.h"

class RamseteController {
	public:
	typedef struct {
		double linVel, angVel;
	} output;

	/**
	 * b and ζ are tuning parameters where b > 0 and ζ ∈ (0, 1). Larger values of
	 * b make convergence more aggressive (like a proportional term), and larger
	 * values of ζ provide more damping
	 */
	RamseteController(double ib, double izeta);

	void setTarget(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta,
	               okapi::QSpeed ivel, okapi::QAngularSpeed iomega);

	output step(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta);
	output step(odom::state ipose);

	void setGains(double ib, double izeta);

	private:
	double b, zeta, desX, desY, desT, velDes, omegaDes;
};
#endif
