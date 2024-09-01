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

#include <future>
#include <iostream>
#include <thread>

#include "libwaterlinked/client.hpp"
#include "libwaterlinked/protocol.hpp"

auto main() -> int
{
  // Create a new client using the default IP address (192.168.194.95) and port (16171) with a 5 second session timeout.
  waterlinked::WaterLinkedClient client("192.168.194.95", 16171, std::chrono::seconds(5));

  // Get the current configuration of the DVL
  std::future<waterlinked::CommandResponse> f = client.get_configuration();

  // Wait for the response synchronously
  const waterlinked::CommandResponse response = f.get();

  // Convert the result to a Configuration object using the JSON API
  auto config = response.result.get<waterlinked::Configuration>();

  std::cout << "Current configuration:\n";
  std::cout << "  Speed of sound: " << config.speed_of_sound << " m/s\n";
  std::cout << "  Acoustic enabled: " << config.acoustic_enabled << "\n";
  std::cout << "  Dark mode enabled: " << config.dark_mode_enabled << "\n";
  std::cout << "  Mounting rotation offset: " << config.mounting_rotation_offset << " degrees\n";
  std::cout << "  Range mode: " << config.range_mode << "\n";
  std::cout << "  Periodic cycling enabled: " << config.periodic_cycling_enabled << "\n";

  // Reset the dead reckoning measurements
  std::future<waterlinked::CommandResponse> f2 = client.reset_dead_reckoning();

  // You can also wait for the response asynchronously using a background thread
  std::thread([f2 = std::move(f2)]() mutable {
    std::cout << "Result of command to reset dead reckoning: " << f2.get().success << "\n";
  }).detach();

  // Let the driver run indefinitely
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
