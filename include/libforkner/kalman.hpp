/**
 * Kalman filter implementation using Eigen. Based on the following
 * introductory paper:
 *
 *     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 *
 * Modifications were made to better support discrete systems, the code
 * for discretization is based on the math found in:
 * https://file.tavsys.net/control/state-space-guide.pdf
 */

#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/MatrixFunctions"

#ifndef _KALMAN_HPP_
#define _KALMAN_HPP_

template <size_t i, size_t s, size_t o>
class KalmanFilter {
	public:
	/**
	 * Create a Kalman filter with the specified matrices.
	 *   A - System dynamics matrix
	 *   C - Output matrix
	 *   Q - Process noise covariance
	 *   R - Measurement noise covariance
	 *   P - Estimate error covariance
	 */
	KalmanFilter(double idt, const Eigen::Matrix<double, s, s>& iA,
	             const Eigen::Matrix<double, i, i>& iB,
	             const Eigen::Matrix<double, o, s>& iC,
	             const Eigen::Matrix<double, s, s>& iQ,
	             const Eigen::Matrix<double, o, o>& iR,
	             const Eigen::Matrix<double, s, s>& iP)
	    : A(iA), B(iB), C(iC), Q(iQ), R(iR), P0(iP), dt(idt), initialized(false) {
		I.setIdentity();
		// Discretize the matrices
		Eigen::Matrix<double, s, s> A_continuous = A;
		A = A * dt;
		A = A.exp();
		// B = A_continuous.inverse() * (A - I) * B;
		Eigen::Matrix<double, s, 2 * s> Q_top;
		Q_top.leftCols(s) = -A.transpose();
		Q_top.rightCols(s) = Q;
		Eigen::Matrix<double, 2 * s, s> Q_bottom;
		Q_bottom.topRows(s) = Eigen::Matrix<double, s, s>::Zero();
		Q_bottom.bottomRows(s) = A;
		Eigen::Matrix<double, 2 * s, 2 * s> Q_exp;
		Q_exp.topRows(s) = Q_top;
		Q_exp.bottomRows(s) = Q_bottom.transpose();
		Q_exp = Q_exp.exp();
		Q = Q_exp.topRightCorner(s, s);
		Q = Q * A;
		R = R / dt;
	}

	/**
	 * Initialize the filter with a guess for initial states.
	 */
	void init(double t0, const Eigen::Matrix<double, s, 1>& x0) {
		x_hat = x0;
		P = P0;
		this->t0 = t0;
		t = t0;
		initialized = true;
	}

	/**
	 * Update the estimated state based on measured values. The
	 * time step is assumed to remain constant.
	 */
	void update(const Eigen::Matrix<double, o, 1>& iY,
	            const Eigen::Matrix<double, i, 1>& iU) {
		if (!initialized) throw std::runtime_error("Filter is not initialized!");

		// Predict
		x_hat_new = A * x_hat + B * iU;
		P = A * P * A.transpose() + Q;
		// Update
		K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
		x_hat_new += K * (iY - C * x_hat_new);
		P = (I - K * C) * P;
		x_hat = x_hat_new;

		t += dt;
	}

	void update(const Eigen::Matrix<double, o, 1>& iY,
	            const Eigen::Matrix<double, i, 1>& iU,
	            const Eigen::Matrix<double, s, s>& iA,
	            const Eigen::Matrix<double, o, s>& iC) {
		this->A = iA;
		this->C = iC;
		update(iY, iU);
	}

	/**
	 * Return the current state and time.
	 */
	Eigen::Matrix<double, o, 1> state() {
		return x_hat;
	};
	double time() {
		return t;
	};

	private:
	// Matrices for computation
	Eigen::Matrix<double, s, s> A;
	Eigen::Matrix<double, i, i> B;
	Eigen::Matrix<double, o, s> C;
	Eigen::Matrix<double, s, s> Q;
	Eigen::Matrix<double, o, o> R;
	Eigen::Matrix<double, s, s> P, P0;
	Eigen::Matrix<double, s, o> K;

	// Initial and current time
	double t0, t;

	// Discrete time step
	double dt;

	// Is the filter initialized?
	bool initialized;

	// n-size identity
	Eigen::Matrix<double, s, s> I;

	// Estimated states
	Eigen::Matrix<double, s, 1> x_hat, x_hat_new;

	// Used for updating signal variance
	int count = 0;
	Eigen::Matrix<double, o, 1> mean, M2;
};

#endif
