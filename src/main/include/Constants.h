#pragma once
#include <units/time.h>
#include <units/velocity.h>
#include <rospathmsgs/msg/waypoint.hpp>
#include "robot_lib/util/PIDF.h"
//#define SystemIndependent
#define noRosDebug
//#define PNU_DEBUG

// Canbus ID mappings

// TALON FX
#define DRIVE_FR_DRIVE 1
#define DRIVE_FR_ANGLE 2
#define DRIVE_FR_ENCOD 1

#define DRIVE_FL_DRIVE 3
#define DRIVE_FL_ANGLE 4
#define DRIVE_FL_ENCOD 2

#define DRIVE_RR_DRIVE 5
#define DRIVE_RR_ANGLE 6
#define DRIVE_RR_ENCOD 3

#define DRIVE_RL_DRIVE 7
#define DRIVE_RL_ANGLE 8
#define DRIVE_RL_ENCOD 4

#define LED_BAR 9

#define INTAKE_MOTOR_ID 9
#define DELIVERY_MOTOR_ID 10
#define FLYWHEEL_MOTOR_ID 11
#define CLIMBER_L_MOTOR_ID 12
#define CLIMBER_C_MOTOR_ID 13
#define CLIMBER_R_MOTOR_ID 14
#define HOOD_MOTOR_ID 16

// TALON SRX
#define INDEXER_MOTOR_ID 22

//Motor defines
#define VOLTAGE_COMP 11

#define HOOD_KP .1
#define HOOD_KI 0
#define HOOD_KD 0
#define HOOD_KF 0
#define HOOD_IMAX 0

#define FLYWHEEL_KP 0.2
#define FLYWHEEL_KI 0
#define FLYWHEEL_KD 0
#define FLYWHEEL_KF .048
#define FLYWHEEL_IMAX 0

#define INTAKE_KP 0
#define INTAKE_KI 0
#define INTAKE_KD 0
#define INTAKE_KF 1
#define INTAKE_IMAX 0

#define INDEXER_KP 0
#define INDEXER_KI 0
#define INDEXER_KD 0
#define INDEXER_KF 1
#define INDEXER_IMAX 0

#define CLIMBER_L_KP .05
#define CLIMBER_L_KI 0
#define CLIMBER_L_KD 0
#define CLIMBER_L_KF 0
#define CLIMBER_L_IMAX 0

#define CLIMBER_C_KP .05
#define CLIMBER_C_KI 0
#define CLIMBER_C_KD 0
#define CLIMBER_C_KF 0
#define CLIMBER_C_IMAX 0

#define CLIMBER_R_KP .05
#define CLIMBER_R_KI 0
#define CLIMBER_R_KD 0
#define CLIMBER_R_KF 0
#define CLIMBER_R_IMAX 0

#define CLIMBER_MAX_AMPS 50
#define CLIMBER_MAX_TIME 2
#define CLIMBER_HOLD_AMPS 30

//SOLENOIDS
#define CLIMBER_SOLENOID_L_MAIN_HIGH_ID 0
#define CLIMBER_SOLENOID_L_MAIN_LOW_ID 1
#define CLIMBER_SOLENOID_L_PIN_HIGH_ID 2
#define CLIMBER_SOLENOID_L_PIN_LOW_ID 3
#define CLIMBER_SOLENOID_C_MAIN_HIGH_ID 4 
#define CLIMBER_SOLENOID_C_MAIN_LOW_ID 5
#define CLIMBER_SOLENOID_C_PIN_HIGH_ID 6
#define CLIMBER_SOLENOID_C_PIN_LOW_ID 7
#define CLIMBER_SOLENOID_R_MAIN_HIGH_ID 8
#define CLIMBER_SOLENOID_R_MAIN_LOW_ID 9
#define CLIMBER_SOLENOID_R_PIN_HIGH_ID 10
#define CLIMBER_SOLENOID_R_PIN_LOW_ID 11
#define INTAKE_SOLENOID_HIGH_ID 12
#define INTAKE_SOLENOID_LOW_ID 13

// TOFs
#define EXTERNAL_TOF_ID 1
#define INTERNAL_TOF_ID 2

// Square size of chassis
#define CHASSIS_LENGTH 0.65_m

// Adaptive Pure Pursuit Controller
#define FIXED_LOOKAHEAD .25
#define MAX_ACCEL .5
#define PATH_COMPLETE_TOLERANCE .05

// Controller Axis
#define X_AXIS 1
#define Y_AXIS 0
#define Z_AXIS 4

#define IMU_ID 0

// Which sticks to watch from the driverstation
#define USER_STICKS {0, 1}

#define DRIVE_STICK_TOPIC "/sticks/stick0" 
#define DRIVE_STICK_SCALAR {.75, -.75, .75, .75, .75, .75}
#define DRIVE_STICK_DEADBAND 0.2
#define DRIVE_STICK_POWER 4

/**
 *  Constants for the drivetrain
 **/ 

// How long before the drivetrain locks up after not recieving a new twist packet.
// Applies for both velocity, and open loop twist mode
#define DRIVE_TIMEOUT 0.050 // seconds

// Allowable difference between swerve modules and the average current
#define DELTA_CURRENT_THRESHOLD 10

// IMU covariance matricies (3x3) Row major about x, y, z axes
#define IMU_ORIENT_COVAR {1, 0, 0, 0, 1, 0, 0, 0, 1} // only show variances of data
#define IMU_ACCEL_COVAR {1, 0, 0, 0, 1, 0, 0, 0, 1} // only show variances of data
#define IMU_ANG_VEL_COVAR {1, 0, 0, 0, 1, 0, 0, 0, 1} // only show variances of data

// PID constants for left and right transmission velocity control
#define ANGLE_KF 0
#define ANGLE_KP .7
#define ANGLE_KI 0.00001
#define ANGLE_KD 0 // originally 1, at 2 for debug
#define DRIVE_LEFT_IACCUM 300
#define DRIVE_KF 0.0522
#define DRIVE_KP 0.2
#define DRIVE_KI 0.0
#define DRIVE_KD 2
#define DRIVE_RIGHT_IACCUM 300

// voltage compensation voltage
#define DRIVE_VCOMP_VOLTAGE 11.0

// physical constants
#define DRIVE_TRACK_WIDTH 0.5 // (meters)
#define TICKS_PER_DEGREE (2048.0/360)

#define SWERVE_DRIVE_GEARING 6.12

#define SWERVE_ANGLE_GEARING (5.0 / 64.0)
#define SWERVE_ANGLE_POS_TTR (2 * 3.14159 / 2048.0 * SWERVE_ANGLE_GEARING)
#define SWERVE_ANGLE_VEL_TTR (SWERVE_ANGLE_POS_TTR * 10)

#define FR_ABS_OFFSET -112.1
#define FL_ABS_OFFSET 95.42
#define RR_ABS_OFFSET 145.8
#define RL_ABS_OFFSET -171.0

//LIMELIGHT

#define LIMELIGHT_MAX_ERROR 1.75

#define HEADING_CONTROL_GAINS_AUTO robot::PIDFDiscriptor{.024, .005, 0, 0}
#define HEADING_CONTROL_GAINS_TELE robot::PIDFDiscriptor{.018, .0025, 0, 0}

// PATH FOLLOWING STUFFS (EXPERIEMTAL ;-;)