# Forkner - BLRS 15" Turning Point Robot
The PROS 3 Code for the Team BLRS 15" Robot used for the 2018-19 VEX U Turning Point Season.

## Highlights
- Odometry powered by two unpowered encoder wheels
- A modified pure pursuit algorithm and RAMSETE controller acting as feedback control to OkapiLib's AsyncMotionProfileController Spline feedforward paths
- OkapiLib's LinearMotionProfileController 1D feedforward profile coupled with PID feedback for point turns and short forward/backward movements.
- Modified version of the OkapiLib PIDAutotuner for use with the above LinearMotionProfileFollower controllers.
- Kalman Filter library built with Eigen to be capable of use with 1D+ systems
- Flywheel velocity control using 1D Kalman Filter and TBH control, Python script used for Kalman Filter simulation
- LCD Autonomous Selector utility using the PROS LLEMU
- LCD Vision Sensor calibration utility using the PROS LLEMU
- Logging Utility with RNG file naming that writes to uSD card
- libforkner and tests source files are compiled with PROS/OkapiLib into the cold binary while scripts and PROS template files are compiled into the hot binary for faster scripting
- Automatic clang-format script that was set up as a git commit hook
- Python visualization tools for the Pure Pursuit debug info and the odometry logger data
- Python script for exporting code to a LaTeX document for printing and displaying in engineering notebook
- MATLAB/Octave Script for LQR Tuning of a 1 motor state space controller (originally used on flywheel but TBH had better performance)

## Redacted Code
Some of the code in this repository has been removed to preserve the team's competitive edge and to encourage learning the algorithms used instead of copy/pasting code. Most of the redacted code can be rewritten by following along with the math in the papers listed in the header docstring for the given source file.

As a result of this removed code, the project will not compile in its current state.
