#pragma once

/* Gets the current FPGA time */
#define GET_TIME_DOUBLE (frc::Timer::GetFPGATimestamp().to<double>())
#define GET_TIME_INT (frc::Timer::GetFPGATimestamp().to<int>())

/* Unpacks arrays to be put in arguments */
#define UNPACK_2(arr) (arr.at(0)), (arr.at(1))
#define UNPACK_3(arr) UNPACK_2(arr), (arr.at(2))
#define UNPACK_4(arr) UNPACK_3(arr), (arr.at(3))
#define UNPACK_5(arr) UNPACK_4(arr), (arr.at(4))

/* Math functions */

/* Converts a -1:1 range value to be 0:1 */
#define MAKE_SCALAR(v) (((v) + 1) / 2)