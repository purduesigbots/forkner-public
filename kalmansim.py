# Simulates the flywheel's kalman filter
import control as cnt
import numpy as np
import scipy as sp
import csv
import sys
import matplotlib.pyplot as plt

def kalmd(A, C, Q, R):
	"""Solves for the steady state kalman gain and error covariance matrices.
	Keyword arguments:
	sys -- discrete state-space model
	Q -- process noise covariance matrix
	R -- measurement noise covariance matrix
	Returns:
	Kalman gain, error covariance matrix.
	"""
	m = A.shape[0]

	observability_rank = np.linalg.matrix_rank(cnt.obsv(A, C))
	if observability_rank != m:
	    print(
	        "Warning: Observability of %d != %d, unobservable state"
	        % (observability_rank, m)
	    )

	# Compute the steady state covariance matrix
	# P_prior = sp.linalg.solve_discrete_are(a=A.T, b=C.T, q=Q, r=R)
	P_prior = np.array([[R]])
	S = C * P_prior * C.T + R
	K = P_prior * C.T * np.linalg.inv(S)
	P = (np.eye(m) - K * C) * P_prior
	print(str(P_prior[0, 0]) + "  " + str(S[0, 0]) + "  " + str(P[0, 0]) + "   " + str(K[0, 0]))
	return K, P

def updateStats(existingAggregate, newValue):
    (count, mean, M2) = existingAggregate
    count += 1
    delta = newValue - mean
    mean += delta / count
    delta2 = newValue - mean
    M2 += delta * delta2

    return (count, mean, M2)

def main(fname):
	gear_ratio = 5.0
	moment = 0.005
	K_t = 2 * 0.198
	R = 3.84
	K_v = 6.68
	RPM_TO_RADS = 0.1047
	VAR_RADS = 2.75847841614875
	dt = 0.01
	A = np.zeros((1, 1))
	# A[0, 0] = -gear_ratio ** 2 * K_t / (K_v * R * moment)
	# A[0, 0] = 9.8
	A[0, 0] = -163 * dt # 163 is the number of steps for time step
	Ad = A * dt
	Ad = sp.linalg.expm(Ad)
	# 0.9838
	# B = np.array([[gear_ratio * K_t / (R * moment)]])
	# Bd = np.linalg.inv(A) * (Ad - np.identity(1)) * B
	B = np.zeros((1, 1))
	Bd = B
	# divide by -0.0165 to revert discretization
	Bd[0, 0] = float(1) / 12
	C = np.array([[1]])
	D = np.array([[0]])
	Q = np.zeros((A.shape[0], A.shape[1]))
	Q[0, 0] = 1.0
	R = VAR_RADS
	Rd = R / dt
	Qd_tr = np.concatenate(((-A.T), Q), axis=1)
	Qd_br = np.concatenate((np.zeros((A.shape[0], A.shape[1])), A), axis=0)
	Qd_exp = np.concatenate(((Qd_tr, Qd_br.T)), axis=0)
	Qd_exp = sp.linalg.expm(Qd_exp)
	Qd = np.zeros((A.shape[0], A.shape[1]))
	Qd[0, 0] = Qd_exp[0, 1] / np.linalg.inv(Ad)
	# ^ the array positions are temporary assuming the flywheel kalman shape

	raw = []
	filtered = []

	x_hat = np.zeros((A.shape[0], 1))
	U = np.zeros((1, 1))
	U[0, 0] = 12 # we're inputting 12V roughly
	Y = np.zeros((C.shape[0], 1))

	kalman_gain = np.zeros((A.shape[0], C.shape[0]))
	kalman_gain, p0 = kalmd(Ad, C, Qd, Rd)
	# kalman_gain[0] = 0.2 # smaller kalman gain provides better damping

	i = 0
	x = []
	stats = (0, 0, VAR_RADS)
	with open(fname) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		for row in csv_reader:
			if (i < 10):
				i = i + 1
				continue
			if (i is 10):
				x_hat[0, 0] = float(row[0])
			Y[0] = float(row[0])
			raw.append(float(row[0]))

			if (Y[0] > 58.0):
				stats = updateStats(stats, Y[0])
				(count, mean, M2) = stats
				Rd = M2 / (count * dt)
			# Predict
			x_hat = Ad * x_hat + Bd * U
			p0 = Ad * p0 * Ad.T + Qd
			S = C * p0 * C.T + Rd
			# Update
			kalman_gain = p0 * C.T * np.linalg.inv(S)
			x_hat_new = kalman_gain * (Y - C * x_hat) + x_hat
			p0 = (np.identity(1) - kalman_gain * C) * p0
			# print(str(kalman_gain[0, 0]) + "   " + str(p0[0, 0]) + "   " + str(Qd[0, 0]))

			cur = float(x_hat_new.item((0, 0)))
			filtered.append(cur)

			# if (x_hat_new.item(0, 0) > 0.63 * 62.8 and x_hat.item(0, 0) <= 0.63 * 62.8):
				# print(">>>>>>>>>>>>   " + str(i))
			print(str(Rd))
			x_hat = x_hat_new

			x.append(i)

			plt.cla()
			plt.plot(x, raw, ".r", x, filtered, ".b")
			plt.grid(True)
			plt.pause(0.001)

			i = i + 1

		plt.show()
		print(str(kalman_gain[0, 0]))

if __name__ == "__main__":
   main(sys.argv[1])
