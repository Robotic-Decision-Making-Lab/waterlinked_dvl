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

#include <nlohmann/json.hpp>
#include <ranges>

#include "libdvla50/protocol.hpp"

namespace libdvla50
{

namespace
{

/// Parse a JSON string into a TransducerReport
auto parse_transducer_report(const nlohmann::json & data) -> TransducerReport
{
  TransducerReport report;

  report.id = static_cast<std::uint8_t>(data.at("id"));
  report.velocity = static_cast<double>(data.at("velocity"));
  report.distance = static_cast<double>(data.at("distance"));
  report.rssi = static_cast<double>(data.at("rssi"));
  report.nsd = static_cast<double>(data.at("nsd"));
  report.beam_valid = static_cast<bool>(data.at("beam_valid"));

  return report;
}

/// Parse a JSON string into a VelocityReport
auto parse_velocity_report(const nlohmann::json & data) -> VelocityReport
{
  VelocityReport report;

  report.time = std::chrono::milliseconds(data.at("time"));
  report.vx = static_cast<double>(data.at("vx"));
  report.vy = static_cast<double>(data.at("vy"));
  report.vz = static_cast<double>(data.at("vz"));
  report.fom = static_cast<double>(data.at("fom"));
  report.altitude = static_cast<double>(data.at("altitude"));
  report.velocity_valid = static_cast<bool>(data.at("velocity_valid"));
  report.status = static_cast<std::uint8_t>(data.at("status"));
  report.time_of_validity = std::chrono::system_clock::from_time_t(data.at("time_of_validity"));
  report.time_of_transmission = std::chrono::system_clock::from_time_t(data.at("time_of_transmission"));
  std::ranges::transform(data.at("transducers"), std::back_inserter(report.transducers), parse_transducer_report);

  const auto & covariance = data.at("covariance");
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      report.covariance(i, j) = static_cast<double>(covariance[i][j]);
    }
  }

  return report;
}

/// Parse a JSON string into a DeadReckoningReport
auto parse_dead_reckoning_report(const nlohmann::json & data) -> DeadReckoningReport
{
  DeadReckoningReport report;

  report.ts = std::chrono::system_clock::from_time_t(data.at("ts"));
  report.x = static_cast<double>(data.at("x"));
  report.y = static_cast<double>(data.at("y"));
  report.z = static_cast<double>(data.at("z"));
  report.std = static_cast<double>(data.at("std"));
  report.roll = static_cast<double>(data.at("roll"));
  report.pitch = static_cast<double>(data.at("pitch"));
  report.yaw = static_cast<double>(data.at("yaw"));
  report.status = static_cast<std::uint8_t>(data.at("status"));

  return report;
}

}  // namespace

}  // namespace libdvla50
