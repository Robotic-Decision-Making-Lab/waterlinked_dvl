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

#include "libwaterlinked/protocol.hpp"

namespace waterlinked
{

auto from_json(const nlohmann::json & j, TransducerReport & r) -> void
{
  j.at("id").get_to(r.id);
  j.at("velocity").get_to(r.velocity);
  j.at("distance").get_to(r.distance);
  j.at("rssi").get_to(r.rssi);
  j.at("nsd").get_to(r.nsd);
  j.at("beam_valid").get_to(r.beam_valid);
}

auto from_json(const nlohmann::json & j, Configuration & r) -> void
{
  j.at("speed_of_sound").get_to(r.speed_of_sound);
  j.at("mounting_rotation_offset").get_to(r.mounting_rotation_offset);
  j.at("acoustic_enabled").get_to(r.acoustic_enabled);
  j.at("dark_mode_enabled").get_to(r.dark_mode_enabled);
  j.at("range_mode").get_to(r.range_mode);
  j.at("periodic_cycling_enabled").get_to(r.periodic_cycling_enabled);
}

auto to_json(nlohmann::json & j, const Configuration & r) -> void
{
  j = nlohmann::json{
    {"speed_of_sound", r.speed_of_sound},
    {"acoustic_enabled", r.acoustic_enabled},
    {"dark_mode_enabled", r.dark_mode_enabled},
    {"mounting_rotation_offset", r.mounting_rotation_offset},
    {"range_mode", r.range_mode},
    {"periodic_cycling_enabled", r.periodic_cycling_enabled},
  };
}

auto from_json(const nlohmann::json & j, CommandResponse & r) -> void
{
  j.at("response_to").get_to(r.response_to);
  j.at("success").get_to(r.success);
  j.at("error_message").get_to(r.error_message);
  j.at("result").get_to(r.result);
}

auto from_json(const nlohmann::json & j, DeadReckoningReport & r) -> void
{
  // WaterLinked doesn't use consistent timestamps across reports, so we need to handle this manually
  // Start by getting the ts in seconds as a double to preserve the fractional component, then convert to microseconds.
  auto ts = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>(
    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(j.at("ts").get<double>())));

  r.ts = ts;
  j.at("x").get_to(r.x);
  j.at("y").get_to(r.y);
  j.at("z").get_to(r.z);
  j.at("std").get_to(r.std);
  j.at("roll").get_to(r.roll);
  j.at("pitch").get_to(r.pitch);
  j.at("yaw").get_to(r.yaw);
  j.at("status").get_to(r.status);
}

auto from_json(const nlohmann::json & j, VelocityReport & r) -> void
{
  j.at("time").get_to(r.time);
  j.at("vx").get_to(r.vx);
  j.at("vy").get_to(r.vy);
  j.at("vz").get_to(r.vz);
  j.at("fom").get_to(r.fom);
  j.at("altitude").get_to(r.altitude);
  j.at("velocity_valid").get_to(r.velocity_valid);
  j.at("status").get_to(r.status);
  j.at("time_of_validity").get_to(r.time_of_validity);
  j.at("time_of_transmission").get_to(r.time_of_transmission);
  j.at("covariance").get_to(r.covariance);
  j.at("transducers").get_to(r.transducers);
}

}  // namespace waterlinked
