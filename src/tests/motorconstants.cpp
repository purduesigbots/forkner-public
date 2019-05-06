#include "main.h"

namespace test {
void motorKt() {
	okapi::Motor m(5);
	m.setGearing(okapi::AbstractMotor::gearset::blue);
	m.moveVoltage(12000);
	while (true) {
		double kt = m.getTorque() / (m.getCurrentDraw() / 1000);
		printf("%f %d %f\n", m.getTorque(), m.getCurrentDraw(), kt);

		pros::delay(10);
	}
}

void motorR() {
	okapi::Motor m(5);
	m.setGearing(okapi::AbstractMotor::gearset::blue);
	m.moveVoltage(12000);
	while (true) {
		double r = (double)m.getVoltage() / (double)m.getCurrentDraw();
		printf("%d %d %f\n", m.getVoltage(), m.getCurrentDraw(), r);

		pros::delay(10);
	}
}

void motorKv() {
	pros::Motor m(5);
	m.set_gearing(MOTOR_GEARSET_06);
	const double r = MOTOR_BLUE_R;
	m.move_voltage(12000);
	while (true) {
		double vel = m.get_actual_velocity();
		double kv = (vel * 0.1047) /
		            ((m.get_voltage() / 1000) - (m.get_current_draw() / 1000) * r);
		printf("%f %d %f\n", vel, m.get_current_draw(), kv);

		pros::delay(10);
	}
}
}  // namespace test
