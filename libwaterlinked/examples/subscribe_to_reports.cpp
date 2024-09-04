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

#include <iostream>

#include "libwaterlinked/client.hpp"
#include "libwaterlinked/protocol.hpp"

/// This example demonstrates how to subscribe to velocity and dead reckoning reports from a WaterLinked DVL.
auto main() -> int
{
  // Create a new client using the default IP address (192.168.194.95) and port (16171) with a 5 second connection
  // timeout.
  waterlinked::WaterLinkedClient client("192.168.194.95", 16171, std::chrono::seconds(5));

  // Register a callback to receive velocity reports
  client.register_callback([](const waterlinked::VelocityReport & report) {
    std::cout << "Received velocity report:\n";
    std::cout << "  Time: " << report.time.count() << "ms\n";
    std::cout << "  Velocity: (" << report.vx << ", " << report.vy << ", " << report.vz << ") m/s\n";
    std::cout << "  Figure of merit: " << report.fom << " m/s\n";
    std::cout << "  Altitude: " << report.altitude << " m\n";
    std::cout << "  Velocity valid: " << report.velocity_valid << "\n";
    std::cout << "  Status: " << static_cast<int>(report.status) << "\n";
    std::cout << "  Time of validity: " << report.time_of_validity.time_since_epoch().count() << "us\n";
    std::cout << "  Time of transmission: " << report.time_of_transmission.time_since_epoch().count() << "us\n";
  });

  // Register a callback to receive dead reckoning reports
  client.register_callback([](const waterlinked::DeadReckoningReport & report) {
    std::cout << "Received dead reckoning report:\n";
    std::cout << "  Timestamp: " << report.ts.time_since_epoch().count() << "us\n";
    std::cout << "  Position: (" << report.x << ", " << report.y << ", " << report.z << ") m\n";
    std::cout << "  Standard deviation: " << report.std << " m\n";
    std::cout << "  Roll: " << report.roll << " degrees\n";
    std::cout << "  Pitch: " << report.pitch << " degrees\n";
    std::cout << "  Yaw: " << report.yaw << " degrees\n";
  });

  // Let the driver run indefinitely
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
