/**
 * Robot Port Configuration
 */
#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "libforkner/camera.hpp"

/**
 * Motors
 */
// 1 is the top motor on each side, 4 is the bottom
#define DRIVE_LEFT_1 10
#define DRIVE_LEFT_2 9
#define DRIVE_LEFT_3 6
#define DRIVE_LEFT_4 7

#define DRIVE_RIGHT_1 1
#define DRIVE_RIGHT_2 2
#define DRIVE_RIGHT_3 3
#define DRIVE_RIGHT_4 4

#define FLYWHEEL_TOP 18
#define FLYWHEEL_RIGHT 19
#define FLYWHEEL_BOTTOM 20
#define INDEXER 12
#define INTAKE 13

#define CAP_FLIPPER 16

#define BAND_RELEASE_LEFT 17
#define BAND_RELEASE_RIGHT 14

/**
 * Sensors
 */
#define LINE_TRACKER 'a'
#define INTAKE_LIFT 'b'
#define INDEXER_PHOTOGATE 'c'
#define ANGLE_CHANGER 'd'
#define ENC_LEFT_1 'e'
#define ENC_LEFT_2 'f'
#define ENC_RIGHT_1 'g'
#define ENC_RIGHT_2 'h'
#define CAP_CAMERA 11

#define ANGLE_CHANGER_IN LOW
#define ANGLE_CHANGER_OUT HIGH

/**
 * Robot Params
 */
#define IN_TO_METERS 0.0254f
#define ENC_WHEEL_DIAM (3.25f * IN_TO_METERS)
#define ENC_CHASSIS_WIDTH                                                      \
  (6.75f * IN_TO_METERS) // currently measuring center to center

#define DRIVE_MOTOR_MAX_VEL 200 // in RPM
#define DRIVE_DRIVEN_WHEEL_DIAM (4.125f * IN_TO_METERS)
#define DRIVE_CHASSIS_WIDTH (12.0f * IN_TO_METERS)

#define MOTOR_RED_KT 1.19f
#define MOTOR_RED_R 4.0f
#define MOTOR_RED_KV 1.10f
#define MOTOR_GREEN_KT 0.593f
#define MOTOR_GREEN_R 4.0f
#define MOTOR_GREEN_KV 2.08f
#define MOTOR_BLUE_KT 0.198f
#define MOTOR_BLUE_R 3.84f
#define MOTOR_BLUE_KV 6.68f

#define DRIVE_MOMENT 0.107f

#define DRIVE_ALIGN_CUR 500

#endif
