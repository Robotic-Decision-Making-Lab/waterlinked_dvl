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

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <nlohmann/json.hpp>

#include "libwaterlinked/protocol.hpp"

namespace waterlinked::test
{

/// Reports used have been retrieved from the Waterlinked API documentation:
/// https://waterlinked.github.io/dvl/dvl-protocol

TEST(JsonParsing, ParsesTransducerReport)
{
  const std::string json_string = R"(
    {
      "id": 0,
      "velocity": 0.00010825289791682735,
      "distance": 0.5568000078201294,
      "rssi": -30.494251251220703,
      "nsd": -88.73271179199219,
      "beam_valid": true
    }
  )";

  const auto obj = nlohmann::json::parse(json_string);
  const auto report = obj.get<TransducerReport>();

  EXPECT_EQ(report.id, 0);
  EXPECT_DOUBLE_EQ(report.velocity, 0.00010825289791682735);
  EXPECT_DOUBLE_EQ(report.distance, 0.5568000078201294);
  EXPECT_DOUBLE_EQ(report.rssi, -30.494251251220703);
  EXPECT_DOUBLE_EQ(report.nsd, -88.73271179199219);
  EXPECT_TRUE(report.beam_valid);
}

TEST(JsonParsing, ParsesDeadReckoningReport)
{
  const std::string json_string = R"(
    {
      "ts": 49056.809,
      "x": 12.43563613697886467,
      "y": 64.617631152402609587,
      "z": 1.767641898933798075,
      "std": 0.001959984190762043,
      "roll": 0.6173566579818726,
      "pitch": 0.6173566579818726,
      "yaw": 0.6173566579818726,
      "type": "position_local",
      "status": 0,
      "format": "json_v3.1"
    }
  )";
  const auto obj = nlohmann::json::parse(json_string);
  const auto report = obj.get<DeadReckoningReport>();

  EXPECT_EQ(report.ts.time_since_epoch().count(), 49056809000);
  EXPECT_DOUBLE_EQ(report.x, 12.43563613697886467);
  EXPECT_DOUBLE_EQ(report.y, 64.617631152402609587);
  EXPECT_DOUBLE_EQ(report.z, 1.767641898933798075);
  EXPECT_DOUBLE_EQ(report.std, 0.001959984190762043);
  EXPECT_FLOAT_EQ(report.roll, 0.6173566579818726);
  EXPECT_FLOAT_EQ(report.pitch, 0.6173566579818726);
  EXPECT_FLOAT_EQ(report.yaw, 0.6173566579818726);
}

TEST(JsonParsing, ParseConfiguration)
{
  const std::string json_string = R"(
    {
      "speed_of_sound":1475.00,
      "acoustic_enabled":true,
      "dark_mode_enabled":false,
      "mounting_rotation_offset":20.00,
      "range_mode":"auto",
      "periodic_cycling_enabled":true
    }
  )";

  const auto obj = nlohmann::json::parse(json_string);
  const auto configuration = obj.get<Configuration>();

  EXPECT_FLOAT_EQ(configuration.speed_of_sound, 1475.00);
  EXPECT_TRUE(configuration.acoustic_enabled);
  EXPECT_FALSE(configuration.dark_mode_enabled);
  EXPECT_FLOAT_EQ(configuration.mounting_rotation_offset, 20.00);
  EXPECT_EQ(configuration.range_mode, "auto");
  EXPECT_TRUE(configuration.periodic_cycling_enabled);
}

TEST(JsonParsing, ParseCommandResponse)
{
  const std::string json_string = R"(
    {
      "response_to":"get_config",
      "success":true,
      "error_message":"",
      "result":{
        "speed_of_sound":1475.00,
        "acoustic_enabled":true,
        "dark_mode_enabled":false,
        "mounting_rotation_offset":20.00,
        "range_mode":"auto",
        "periodic_cycling_enabled":true
      },
      "format":"json_v3.1",
      "type":"response"
    }
  )";

  const auto obj = nlohmann::json::parse(json_string);
  const auto response = obj.get<CommandResponse>();

  EXPECT_EQ(response.response_to, "get_config");
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.error_message, "");
  EXPECT_NO_THROW(response.result.get<Configuration>());
}

/*
 * One box of lasagna noodles - usually 12/13 noodles (makes two batches of lasagna) - 9 noodles per lasagna
 * one jar of pasta sauce
 * 1 container riccotta cheese (15 oz)
 * 1 bag mozerella cheese (16 oz)
 * 1 lb ground beef
 * 1 qaurter onion
 *
 * 1. Boil noodles
 * 2. Brown beef and onion
 * 3. Mix pasta sauce with beef and onion
 * 3. Grease pan
 * 4. layer 3 noodles on the bottom
 * 5. Add sauce on each layer of noodles
 * 6. Add half of riccotta cheese on sauce
 * 7. Sprinkle mozerella on that
 * 8. 3 more noodles
 * 9. Sauce, riccotta, mozzerella
 * 10. 3 more noodles
 * 11. Rest of sauce
 * 12. Rest of mozzerella
 * 13. Bake at 350 for 45 minutes with foil on top
 */

TEST(JsonParsing, ParseVelocityReport)
{
  const std::string json_string = R"(
    {
      "time": 106.3935775756836,
      "vx": -3.713480691658333e-05,
      "vy": 5.703703573090024e-05,
      "vz": 2.4990416932269e-05,
      "fom": 0.00016016385052353144,
      "covariance": [
        [
          2.4471841442164077e-08,
          -3.3937477272871774e-09,
          -1.6659699175747278e-09
        ],
        [
          -3.3937477272871774e-09,
          1.4654466085062268e-08,
          4.0409570134514183e-10
        ],
        [
          -1.6659699175747278e-09,
          4.0409570134514183e-10,
          1.5971971523143225e-09
        ]
      ],
      "altitude": 0.4949815273284912,
      "transducers": [
        {
          "id": 0,
          "velocity": 0.00010825289791682735,
          "distance": 0.5568000078201294,
          "rssi": -30.494251251220703,
          "nsd": -88.73271179199219,
          "beam_valid": true
        },
        {
          "id": 1,
          "velocity": -1.4719001228513662e-05,
          "distance": 0.5663999915122986,
          "rssi": -31.095735549926758,
          "nsd": -89.5116958618164,
          "beam_valid": true
        },
        {
          "id": 2,
          "velocity": 2.7863150535267778e-05,
          "distance": 0.537600040435791,
          "rssi": -27.180519104003906,
          "nsd": -96.98075103759766,
          "beam_valid": true
        },
        {
          "id": 3,
          "velocity": 1.9419496311456896e-05,
          "distance": 0.5472000241279602,
          "rssi": -28.006759643554688,
          "nsd": -88.32147216796875,
          "beam_valid": true
        }
      ],
      "velocity_valid": true,
      "status": 0,
      "format": "json_v3.1",
      "type": "velocity",
      "time_of_validity": 1638191471563017,
      "time_of_transmission": 1638191471752336
    }
  )";

  const auto obj = nlohmann::json::parse(json_string);
  const auto response = obj.get<VelocityReport>();

  EXPECT_EQ(response.time.count(), 106);
  EXPECT_DOUBLE_EQ(response.vx, -3.713480691658333e-05);
  EXPECT_DOUBLE_EQ(response.vy, 5.703703573090024e-05);
  EXPECT_DOUBLE_EQ(response.vz, 2.4990416932269e-05);
  EXPECT_DOUBLE_EQ(response.fom, 0.00016016385052353144);
  EXPECT_DOUBLE_EQ(response.altitude, 0.4949815273284912);
  EXPECT_EQ(response.transducers.size(), 4);
  EXPECT_TRUE(response.velocity_valid);
  EXPECT_EQ(response.status, 0);
  EXPECT_EQ(response.time_of_validity.time_since_epoch().count(), 1638191471563017);
  EXPECT_EQ(response.time_of_transmission.time_since_epoch().count(), 1638191471752336);

  // Test the covariance matrix
  Eigen::Matrix3d expected_covariance;
  expected_covariance << 2.4471841442164077e-08, -3.3937477272871774e-09, -1.6659699175747278e-09,
    -3.3937477272871774e-09, 1.4654466085062268e-08, 4.0409570134514183e-10, -1.6659699175747278e-09,
    4.0409570134514183e-10, 1.5971971523143225e-09;
  EXPECT_EQ(response.covariance, expected_covariance);

  // Test the transducer reports
  EXPECT_EQ(response.transducers[0].id, 0);
  EXPECT_DOUBLE_EQ(response.transducers[0].velocity, 0.00010825289791682735);
  EXPECT_DOUBLE_EQ(response.transducers[0].distance, 0.5568000078201294);
  EXPECT_DOUBLE_EQ(response.transducers[0].rssi, -30.494251251220703);
  EXPECT_DOUBLE_EQ(response.transducers[0].nsd, -88.73271179199219);
  EXPECT_TRUE(response.transducers[0].beam_valid);

  EXPECT_EQ(response.transducers[1].id, 1);
  EXPECT_DOUBLE_EQ(response.transducers[1].velocity, -1.4719001228513662e-05);
  EXPECT_DOUBLE_EQ(response.transducers[1].distance, 0.5663999915122986);
  EXPECT_DOUBLE_EQ(response.transducers[1].rssi, -31.095735549926758);
  EXPECT_DOUBLE_EQ(response.transducers[1].nsd, -89.5116958618164);
  EXPECT_TRUE(response.transducers[1].beam_valid);

  EXPECT_EQ(response.transducers[2].id, 2);
  EXPECT_DOUBLE_EQ(response.transducers[2].velocity, 2.7863150535267778e-05);
  EXPECT_DOUBLE_EQ(response.transducers[2].distance, 0.537600040435791);
  EXPECT_DOUBLE_EQ(response.transducers[2].rssi, -27.180519104003906);
  EXPECT_DOUBLE_EQ(response.transducers[2].nsd, -96.98075103759766);
  EXPECT_TRUE(response.transducers[2].beam_valid);

  EXPECT_EQ(response.transducers[3].id, 3);
  EXPECT_DOUBLE_EQ(response.transducers[3].velocity, 1.9419496311456896e-05);
  EXPECT_DOUBLE_EQ(response.transducers[3].distance, 0.5472000241279602);
  EXPECT_DOUBLE_EQ(response.transducers[3].rssi, -28.006759643554688);
  EXPECT_DOUBLE_EQ(response.transducers[3].nsd, -88.32147216796875);
  EXPECT_TRUE(response.transducers[3].beam_valid);
}

};  // namespace waterlinked::test

auto main(int argc, char ** argv) -> int
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
