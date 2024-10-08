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

#include "libwaterlinked/client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/select.h>
#include <unistd.h>

#include <ctime>
#include <iostream>
#include <nlohmann/json.hpp>
#include <ranges>
#include <stdexcept>

#include "libwaterlinked/protocol.hpp"

namespace waterlinked
{

namespace
{

/// Read n_bytes from a socket and append them to a queue.
auto read_from_socket(int socket, std::deque<std::uint8_t> & buffer, std::size_t n_bytes) -> ssize_t
{
  std::vector<std::uint8_t> data(n_bytes);
  const ssize_t n_read = recv(socket, data.data(), n_bytes, 0);

  if (n_read < 0) {
    return n_read;
  }

  std::ranges::copy(data | std::views::take(n_read), std::back_inserter(buffer));
  return n_read;
}

/// Parse the byte data from a buffer into a vector of JSON objects
auto parse_bytes(const std::deque<std::uint8_t> & buffer) -> std::vector<nlohmann::json>
{
  std::vector<nlohmann::json> json_objects;

  auto start = buffer.begin();
  auto iter = std::find_if(start, buffer.end(), [](const std::uint8_t & b) { return b == protocol::DELIMITER; });

  while (iter != buffer.end()) {
    const std::vector<std::uint8_t> data(start, iter);

    start = iter + 1;
    iter = std::find_if(start, buffer.end(), [](const std::uint8_t & b) { return b == protocol::DELIMITER; });

    if (data.empty()) {
      continue;
    }

    try {
      json_objects.push_back(nlohmann::json::parse(data));
    }
    catch (const std::exception & e) {
      std::cout << "An error occurred while attempting to parse the DVL message. " << e.what() << "\n";
    }
  }

  return json_objects;
}

/// Establish a connection to a socket with a timeout. Returns 0 on success, -1 on failure.
auto connect(int socket, const struct sockaddr * addr, socklen_t addrlen, std::chrono::seconds timeout) -> int
{
  auto set_socket_flags = [](int socket, int flags) -> int { return fcntl(socket, F_SETFL, flags); };

  const int flags = fcntl(socket, F_GETFL, 0);
  if (flags < 0) {
    return -1;
  }

  // Set the socket to non-blocking mode so that we can create a timeout on the connection attempt
  if (set_socket_flags(socket, flags | O_NONBLOCK) < 0) {
    return -1;
  }

  auto deadline = std::chrono::steady_clock::now() + timeout;

  // Attempt to establish a connection
  int rc = ::connect(socket, addr, addrlen);

  if (rc < 0) {
    if (errno != EINPROGRESS && errno != EWOULDBLOCK) {
      set_socket_flags(socket, flags);
      return rc;
    }

    do {
      const int remaining_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();

      if (remaining_time <= 0) {
        rc = 0;
        break;
      }

      struct pollfd pfds[] = {{.fd = socket, .events = POLLOUT, .revents = 0}};  // NOLINT
      rc = poll(pfds, 1, remaining_time);

      // Verify that the poll was *actually* successful
      // See: https://stackoverflow.com/questions/2597608/c-socket-connection-timeout
      if (rc > 0) {
        int error = 0;
        socklen_t error_len = sizeof(error);

        if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &error, &error_len) < 0) {
          rc = -1;
        } else {
          errno = error;
        }
      }
    } while (rc == -1 && errno == EINTR);

    // A timeout occurred
    if (rc == 0) {
      errno = ETIMEDOUT;
      set_socket_flags(socket, flags);
      return -1;
    }
  }

  // Restore the original socket flags
  return set_socket_flags(socket, flags) < 0 ? -1 : rc;
}

}  // namespace

WaterLinkedClient::WaterLinkedClient(
  const std::string & addr,
  std::uint16_t port,
  std::chrono::seconds connection_timeout)
{
  // Open a TCP socket and connect to the DVL
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {
    throw std::runtime_error("Failed to open TCP socket");
  }

  struct sockaddr_in sockaddr;

  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  sockaddr.sin_addr.s_addr = inet_addr(addr.c_str());

  if (sockaddr.sin_addr.s_addr == INADDR_NONE) {
    throw std::runtime_error("Invalid socket address " + addr);
  }

  if (connect(socket_, reinterpret_cast<struct sockaddr *>(&sockaddr), sizeof(sockaddr), connection_timeout) < 0) {
    throw std::runtime_error(
      "An error occurred while attempting to connect to the DVL. Error: " + std::string(strerror(errno)));
  }

  running_.store(true);
  polling_thread_ = std::thread([this] { poll_connection(); });
}

WaterLinkedClient::~WaterLinkedClient()
{
  running_.store(false);

  if (polling_thread_.joinable()) {
    polling_thread_.join();
  }

  close(socket_);
}

auto WaterLinkedClient::send_command(const nlohmann::json & command) -> std::future<CommandResponse>
{
  const std::string command_str{command.dump()};
  if (send(socket_, command_str.c_str(), command_str.size(), 0) < 0) {
    throw std::runtime_error("Failed to send command to DVL");
  }

  std::promise<CommandResponse> response;
  auto future = response.get_future();

  pending_requests_[command.at("command")].emplace_back(std::move(response));

  return future;
}

auto WaterLinkedClient::set_speed_of_sound(std::uint16_t speed_of_sound) -> std::future<CommandResponse>
{
  return send_command({{"command", "set_config"}, {"parameters", {{"speed_of_sound", speed_of_sound}}}});
}

auto WaterLinkedClient::set_mounting_rotation_offset(std::uint16_t degrees) -> std::future<CommandResponse>
{
  return send_command({{"command", "set_config"}, {"parameters", {{"mounting_rotation_offset", degrees}}}});
}

auto WaterLinkedClient::enable_acoustics(bool enable) -> std::future<CommandResponse>
{
  return send_command({{"command", "set_config"}, {"parameters", {{"acoustic_enabled", enable}}}});
}

auto WaterLinkedClient::enable_dark_mode(bool enable) -> std::future<CommandResponse>
{
  return send_command({{"command", "set_config"}, {"parameters", {{"dark_mode_enabled", enable}}}});
}

auto WaterLinkedClient::enable_periodic_cycling(bool enable) -> std::future<CommandResponse>
{
  return send_command({{"command", "set_config"}, {"parameters", {{"periodic_cycling_enabled", enable}}}});
}

auto WaterLinkedClient::set_range_mode(const std::string & mode) -> std::future<CommandResponse>
{
  return send_command({{"command", "set_config"}, {"parameters", {{"range_mode", mode}}}});
}

auto WaterLinkedClient::get_configuration() -> std::future<CommandResponse>
{
  return send_command({{"command", "get_config"}});
}

auto WaterLinkedClient::set_configuration(const Configuration & configuration) -> std::future<CommandResponse>
{
  const nlohmann::json config_json = configuration;
  return send_command({{"command", "set_config"}, {"parameters", config_json}});
}

auto WaterLinkedClient::trigger_ping() -> std::future<CommandResponse>
{
  return send_command({{"command", "trigger_ping"}});
}

auto WaterLinkedClient::calibrate_gyro() -> std::future<CommandResponse>
{
  return send_command({{"command", "calibrate_gyro"}});
}

auto WaterLinkedClient::reset_dead_reckoning() -> std::future<CommandResponse>
{
  return send_command({{"command", "reset_dead_reckoning"}});
}

auto WaterLinkedClient::register_callback(std::function<void(const VelocityReport &)> && callback) -> void
{
  velocity_report_callbacks_.emplace_back(std::move(callback));
}

auto WaterLinkedClient::register_callback(std::function<void(const DeadReckoningReport &)> && callback) -> void
{
  dead_reckoning_report_callbacks_.emplace_back(std::move(callback));
}

auto WaterLinkedClient::process_json_object(const nlohmann::json & json_object) -> void
{
  // There are only three types of messages sent by the DVL: velocity reports, dead reckoning reports, and command
  // responses.
  if (json_object.at("type") == "velocity") {
    const auto report = json_object.get<VelocityReport>();
    for (const auto & callback : velocity_report_callbacks_) {
      callback(report);
    }
  } else if (json_object.at("type") == "position_local") {
    const auto report = json_object.get<DeadReckoningReport>();
    for (const auto & callback : dead_reckoning_report_callbacks_) {
      callback(report);
    }
  } else if (json_object.at("type") == "response") {
    const auto response = json_object.get<CommandResponse>();
    if (pending_requests_.contains(response.response_to) && !pending_requests_[response.response_to].empty()) {
      pending_requests_[response.response_to].front().set_value(response);
      pending_requests_[response.response_to].pop_front();
    }
  } else {
    throw std::runtime_error("Received an unknown message type from the DVL: " + json_object.dump());
  }
}

auto WaterLinkedClient::poll_connection() -> void
{
  // Maintain a queue to store incoming data
  const std::size_t max_bytes_to_read = 2048;  // Reports can be quite large, so create a large buffer
  std::deque<std::uint8_t> buffer;
  std::size_t n_bytes_to_read = max_bytes_to_read;

  while (running_.load()) {
    if (read_from_socket(socket_, buffer, n_bytes_to_read) < 0) {
      std::cout << "Failed to read from the DVL; the connection was likely lost.\n";
    }

    auto last_delim = std::ranges::find(buffer | std::views::reverse, protocol::DELIMITER);

    if (last_delim != buffer.rend()) {
      try {
        const std::vector<nlohmann::json> json_objects = parse_bytes(buffer);
        if (!json_objects.empty()) {
          for (const auto & report : json_objects) {
            process_json_object(report);
          }
        };

        buffer.erase(buffer.begin(), last_delim.base());
      }
      catch (const std::exception & e) {
        std::cout << "An error occurred while attempting to decode a DVL message: " << e.what() << "\n";
        buffer.clear();
      }
    }

    n_bytes_to_read = max_bytes_to_read - buffer.size();
  }
}

}  // namespace waterlinked
