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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "libdvla50/protocol.hpp"

namespace libdvla50
{

/// A driver for the WaterLinked DVL-A50.
class DvlA50Driver
{
public:
  /// Create new new interface for the WaterLinked DVL-A50 using:
  /// - The IP address of the DVL (defaults to 192.168.194.95)
  /// - The port of the DVL (defaults to 16171)
  /// - a session timeout (s, defaults to 5s)
  explicit DvlA50Driver(
    const std::string & addr = "192.168.194.95",
    std::uint16_t port = 16171,
    std::chrono::seconds session_timeout = std::chrono::seconds(5));

  ~DvlA50Driver();

  /// Set the speed of sound (1000-2000m/s).
  auto set_speed_of_sound(int speed_of_sound) -> std::future<CommandResponse>;

  /// Set the mounting rotation offset. Typically 0, but can be set to be non-zero if the forward axis of the DVL is not
  /// aligned with the forward axis of a vehicle on which it is mounted (0-360 degrees).
  auto set_mounting_rotation_offset(int degrees) -> std::future<CommandResponse>;

  /// Set to true for normal operation of the DVL, false when the sending of acoustic waves from the DVL is disabled
  /// (e.g. to save power or slow down its heating up in air).
  auto enable_acoustics(bool enable) -> std::future<CommandResponse>;

  /// Set to true for normal LED operation, false for no blinking of the LED (e.g. if the LED is interfering with a
  /// camera).
  auto enable_dark_mode(bool enable) -> std::future<CommandResponse>;

  /// Set to true for normal operation where the DVL periodically searches for bottom lock shorter than the existing
  /// bottom lock, false if periodic cycling is disabled.
  auto enable_periodic_cycling(bool enable) -> std::future<CommandResponse>;

  /// Range mode configuration can be used to instruct the DVL to only search for bottom lock in a limited altitude
  /// range. This can improve performance when operating in environments where you know the DVL minimum and/or maximum
  /// altitude. (e.g., when operating in a river or pool). The range mode can be configured according to the following
  /// format:
  ///
  /// - "auto": The DVL will search for bottom lock in it's full operational area (Default)
  /// - "=a": The DVL is locked to range mode "a" where "a" is a number from 0-4
  /// - "a<=b": The DVL will search for bottom lock within range mode "a" and "b"
  ///
  /// The available range modes are:
  ///
  /// - 0: 0.05m - 0.6m
  /// - 1: 0.3m - 3.0m
  /// - 2: 1.5m - 14m
  /// - 3: 7.7m - 36m
  /// - 4: 15m - 75m
  auto set_range_mode(const std::string & mode) -> std::future<CommandResponse>;

  /// Fetch the current configuration of the DVL.
  auto get_configuration() -> std::future<CommandResponse>;

  /// In setups where multiple acoustic sensors are used it can be useful to control the pinging of each acoustic sensor
  /// individually. By setting the configuration acoustic_enabled = false the pinging of the DVL can be externally
  /// controlled. Up to 15 external trigger commands can be queued by issuing the trigger_ping command. The DVL will
  /// execute each ping in quick succession until no more commands are in the queue.
  auto trigger_ping() -> std::future<CommandResponse>;

  /// Calibrate the gyroscope.
  auto calibrate_gyro() -> std::future<CommandResponse>;

  /// Reset the dead reckoning measurements.
  auto reset_dead_reckoning() -> std::future<CommandResponse>;

  /// Register a callback to receive the velocity report.
  auto register_callback(std::function<void(const VelocityReport &)> && callback) -> void;

  /// Register a callback to receive the dead reckoning report.
  auto register_callback(std::function<void(const DeadReckoningReport &)> && callback) -> void;

private:
  /// Send a command to the DVL.
  auto send_command(const nlohmann::json & command) -> std::future<CommandResponse>;

  /// Poll the connection for new data.
  auto poll_connection() -> void;

  /// Process a vector of JSON objects by triggering callbacks and resolving promises.
  auto process_json_object(const nlohmann::json & json_object) -> void;

  int socket_;

  std::atomic<bool> running_{false};

  std::unordered_map<std::string, std::deque<std::promise<CommandResponse>>> pending_requests_;
  std::mutex request_mutex_;

  std::thread polling_thread_;

  std::vector<std::function<void(const VelocityReport &)>> velocity_report_callbacks_;
  std::vector<std::function<void(const DeadReckoningReport &)>> dead_reckoning_report_callbacks_;
};

}  // namespace libdvla50
