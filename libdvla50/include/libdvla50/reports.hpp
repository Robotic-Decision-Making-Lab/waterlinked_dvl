// Copyright 2024, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>

/// Shorten the namespace for convenience
using json = nlohmann::json;

namespace libdvla50
{

struct TransducerReport
{
  // ID of the transducer
  std::uint8_t id;

  // Measured velocity (m/s)
  double velocity;

  // Measured distance (m)
  double distance;

  // Transducer signal strength (dBm)
  double rssi;

  // Noise spectral density (dBm)
  double nsd;

  // Whether or not the transducer beam is valid
  bool beam_valid;
};

/// A velocity-and-transducer report is sent for each velocity calculation of the DVL. The rate depends on the altitude
/// of the DVL (distance to the sea bottom or other reflecting surface), but will be in the range 2-15 Hz.
///
/// The X, Y, and Z axes are with respect to body frame of the DVL, or the vehicle frame, if the DVL is mounted on a
/// vehicle at an angle, specified as a 'mounting rotation offset', from the forward axis of the vehicle.
struct VelocityReport
{
  /// Milliseconds since last velocity report (ms)
  std::chrono::milliseconds time;

  /// Velocity in the x direction (m/s)
  double vx;

  /// Velocity in the y direction (m/s)
  double vy;

  /// Velocity in the z direction (m/s)
  double vz;

  /// Figure of merit, a measure of the accuracy of the velocities (m/s)
  double fom;

  /// Covariance matrix for the velocities. The figure of merit is calculated from this (entries in (m/s)^2)
  Eigen::Matrix3d covariance;

  /// Distance to the reflecting surface along the Z axis (m)
  double altitude;

  /// Transducer reports
  std::array<TransducerReport, 4> transducers;

  /// If true, the DVL has a lock on the reflecting surface, and the altitude and velocities are valid (True/False)
  bool velocity_valid;

  /// 8 bit status mask. Bit 0 is set to 1 for high temperature and DVL will soon enter thermal shutdown. Remaining bits
  /// are reserved for future use.
  std::uint8_t status;

  /// Timestamp of the surface reflection, aka 'center of ping' (Unix timestamp in microseconds)
  std::chrono::time_point<std::chrono::system_clock> time_of_validity;

  /// Timestamp from immediately before sending of the report over TCP (Unix timestamp in microseconds)
  std::chrono::time_point<std::chrono::system_clock> time_of_transmission;

  /// Format type and version for this report
  std::string format{"json_v3.1"};

  /// Report type
  std::string type{"velocity"};
};

/// A dead reckoning report outputs the current speed, position, and orientation of the DVL as calculated by dead
/// reckoning, with respect to a frame defined by the axes of the DVL's body frame, or vehicle frame if a mounting
/// rotation offset is set, at the start of the dead reckoning run. The expected update rate is 5 Hz.
struct DeadReckoningReport
{
  /// Timestamp of report (Unix timestamp in seconds)
  std::chrono::time_point<std::chrono::system_clock> ts;

  /// Distance in X direction (m)
  double x;

  /// Distance in Y direction (m)
  double y;

  /// Distance in downward direction (m)
  double z;

  /// Standard deviation (figure of merit) for position (m)
  double std;

  /// Rotation around X axis (degrees)
  double roll;

  /// Rotation around Y axis (degrees)
  double pitch;

  /// Rotation around Z axis, i.e. heading (degrees)
  double yaw;

  /// Report type
  std::string type{"position_local"};

  /// Reports if there are any issues with the DVL (0 if no errors, 1 otherwise)
  std::uint8_t status;

  /// Format type and version for this report
  std::string format{"json_v3"};
};

}  // namespace libdvla50
