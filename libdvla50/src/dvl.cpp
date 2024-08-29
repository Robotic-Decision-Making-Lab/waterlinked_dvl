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

#include "libdvla50/dvl.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <nlohmann/json.hpp>
#include <ranges>
#include <stdexcept>

#include "libdvla50/protocol.hpp"

namespace libdvla50
{

namespace
{

/// Message delimiter used by the DVL
constexpr char DELIMITER = '\n';

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

DvlA50Driver::DvlA50Driver(const std::string & addr, std::uint16_t port, std::chrono::seconds session_timeout)
{
  // Open a TCP socket and connect to the DVL
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {
    throw std::runtime_error("Failed to open UDP socket");
  }

  struct sockaddr_in sockaddr;

  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  sockaddr.sin_addr.s_addr = inet_addr(addr.c_str());

  if (sockaddr.sin_addr.s_addr == INADDR_NONE) {
    throw std::runtime_error("Invalid socket address " + addr);
  }

  if (connect(socket_, reinterpret_cast<struct sockaddr *>(&sockaddr), sizeof(sockaddr)) < 0) {
    throw std::runtime_error("Failed to connect to TCP socket");
  }

  // Configure the socket to timeout on failed reads
  struct timeval timeout;
  timeout.tv_sec = session_timeout.count();
  timeout.tv_usec = 0;

  if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
    throw std::runtime_error("Failed to set socket send timeout");
  }

  running_.store(true);
}

DvlA50Driver::~DvlA50Driver() { close(socket_); }

auto DvlA50Driver::send_command(const nlohmann::json & command) -> std::future<CommandResponse>
{
  if (send(socket_, command.dump().c_str(), command.dump().size(), 0) < 0) {
    throw std::runtime_error("Failed to send command to DVL");
  }

  // The promise is non-copyable, so we get the future before moving the request onto the queue
  std::promise<CommandResponse> request;
  auto future = request.get_future();

  pending_requests_[command.at("command")].emplace_back(std::move(request));

  return future;
}

auto DvlA50Driver::set_speed_of_sound(int speed_of_sound) -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "set_config"}, {"parameters", {{"speed_of_sound", speed_of_sound}}}};
  return send_command(command);
}

auto DvlA50Driver::set_mounting_rotation_offset(int degrees) -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "set_config"}, {"parameters", {{"mounting_rotation_offset", degrees}}}};
  return send_command(command);
}

auto DvlA50Driver::enable_acoustics(bool enable) -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "set_config"}, {"parameters", {{"acoustic_enabled", enable}}}};
  return send_command(command);
}

auto DvlA50Driver::enable_dark_mode(bool enable) -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "set_config"}, {"parameters", {{"dark_mode_enabled", enable}}}};
  return send_command(command);
}

auto DvlA50Driver::enable_periodic_cycling(bool enable) -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "set_config"}, {"parameters", {{"periodic_cycling_enabled", enable}}}};
  return send_command(command);
}

auto DvlA50Driver::set_range_mode(const std::string & mode) -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "set_config"}, {"parameters", {{"range_mode", mode}}}};
  return send_command(command);
}

auto DvlA50Driver::get_configuration() -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "get_config"}};
  return send_command(command);
}

auto DvlA50Driver::trigger_ping() -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "trigger_ping"}};
  return send_command(command);
}

auto DvlA50Driver::calibrate_gyro() -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "calibrate_gyro"}};
  return send_command(command);
}

auto DvlA50Driver::reset_dead_reckoning() -> std::future<CommandResponse>
{
  const nlohmann::json command = {{"command", "reset_dead_reckoning"}};
  return send_command(command);
}

auto DvlA50Driver::register_velocity_report_callback(std::function<void(const VelocityReport &)> && callback) -> void
{
  velocity_report_callbacks_.emplace_back(std::move(callback));
}

auto DvlA50Driver::register_dead_reckoning_report_callback(std::function<void(const DeadReckoningReport &)> && callback)
  -> void
{
  dead_reckoning_report_callbacks_.emplace_back(std::move(callback));
}

}  // namespace libdvla50
