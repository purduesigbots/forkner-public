% Script to find the gain vector for a given system.
%
% Requires the 'control' toolbox to be installed and loaded.

gear_ratio = 5.0;
moment = 0.0005;
tau_stall = 2.1;
internal_g = 6.0;
stall_current = 2.5;
K_t = tau_stall / (internal_g * stall_current);
R = 12 / stall_current;
K_v = 62.8 / (12 - 0.01 * R);

A = [((-gear_ratio * gear_ratio * K_t) / (K_v * R * moment))];
B = [((gear_ratio * K_t) / (R * moment))];
Q = [10 / (62.8 * 62.8)];
R = [1 / (12 * 12)];

[K, x, s] = lqr(A, B, Q, R)
