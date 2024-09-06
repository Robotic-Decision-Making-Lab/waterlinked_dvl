// Copyright 2024, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "waterlinked_dvl_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace waterlinked::ros
{

namespace
{

auto populate_service_response(
  std::shared_ptr<std_srvs::srv::SetBool::Response> & response,
  std::future<CommandResponse> & f) -> void
{
  const CommandResponse command_response = f.get();
  response->success = command_response.success;
  response->message = command_response.error_message;
}

auto populate_service_response(
  std::shared_ptr<std_srvs::srv::Trigger::Response> & response,
  std::future<CommandResponse> & f) -> void
{
  const CommandResponse command_response = f.get();
  response->success = command_response.success;
  response->message = command_response.error_message;
}

}  // namespace

WaterLinkedDvlDriver::WaterLinkedDvlDriver()
: rclcpp_lifecycle::LifecycleNode("waterlinked_dvl_driver")
{
  dvl_msg_.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
  dvl_msg_.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;  // 4-beam convex Janus array

  // The following has been retrieved from:
  // https://github.com/ndahn/dvl_a50/blob/1e6a5304235facf53ecf82043fb5ba4c8569016b/src/dvl_a50_ros2.cpp#L45

  // Each beam points 22.5° away from the center, LED pointing forward.
  // Transducers are rotated 45° around Z.
  // Beam 1 (+135° from X)
  dvl_msg_.beam_unit_vec[0].x = -0.6532814824381883;
  dvl_msg_.beam_unit_vec[0].y = 0.6532814824381883;
  dvl_msg_.beam_unit_vec[0].z = 0.38268343236508984;

  // Beam 2 (-135° from X)
  dvl_msg_.beam_unit_vec[1].x = -0.6532814824381883;
  dvl_msg_.beam_unit_vec[1].y = -0.6532814824381883;
  dvl_msg_.beam_unit_vec[1].z = 0.38268343236508984;

  // Beam 3 (-45° from X)
  dvl_msg_.beam_unit_vec[2].x = 0.6532814824381883;
  dvl_msg_.beam_unit_vec[2].y = -0.6532814824381883;
  dvl_msg_.beam_unit_vec[2].z = 0.38268343236508984;

  // Beam 4 (+45° from X)
  dvl_msg_.beam_unit_vec[3].x = 0.6532814824381883;
  dvl_msg_.beam_unit_vec[3].y = 0.6532814824381883;
  dvl_msg_.beam_unit_vec[3].z = 0.38268343236508984;
}

auto WaterLinkedDvlDriver::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Configuring the WaterLinkedDvlDriver");

  try {
    param_listener_ = std::make_shared<waterlinked_dvl_driver::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get WaterLinkedDvlDriver parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // The client does 99% of the work, this is just a ROS wrapper around its functionality
  try {
    client_ =
      std::make_unique<WaterLinkedClient>(params_.ip_address, params_.port, std::chrono::seconds(params_.timeout));
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to create WaterLinkedClient. %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Set the initial DVL configurations
  // This lets users set the default DVL configurations from a parameters/launch file
  const Configuration config{
    static_cast<std::uint16_t>(params_.speed_of_sound),            // from int64_t
    static_cast<std::uint16_t>(params_.mounting_rotation_offset),  // from int64_t
    params_.acoustic_enabled,
    params_.dark_mode_enabled,
    params_.range_mode,
    params_.periodic_cycling_enabled,
  };
  std::future<CommandResponse> f = client_->set_configuration(config);

  const CommandResponse response = f.get();
  if (!response.success) {
    RCLCPP_ERROR(get_logger(), "Failed to set DVL configuration: %s", response.error_message.c_str());  // NOLINT
    return CallbackReturn::ERROR;
  }

  dvl_msg_.header.frame_id = params_.frame_id;
  dead_reckoning_msg_.header.frame_id = params_.frame_id;
  odom_msg_.header.frame_id = params_.frame_id;

  dvl_pub_ = create_publisher<marine_acoustic_msgs::msg::Dvl>("~/velocity_report", rclcpp::SystemDefaultsQoS());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
  dead_reckoning_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/dead_reckoning_report", rclcpp::SystemDefaultsQoS());

  client_->register_callback([this](const VelocityReport & report) {
    const auto t = std::chrono::time_point_cast<std::chrono::nanoseconds>(report.time_of_validity);
    dvl_msg_.header.stamp = rclcpp::Time(t.time_since_epoch().count());
    dvl_msg_.altitude = report.altitude;
    dvl_msg_.velocity.x = report.vx;
    dvl_msg_.velocity.y = report.vy;
    dvl_msg_.velocity.z = report.vz;
    dvl_msg_.beam_ranges_valid = true;
    dvl_msg_.beam_velocities_valid = report.velocity_valid;
    dvl_msg_.course_gnd = std::atan2(report.vy, report.vx);
    dvl_msg_.speed_gnd = std::sqrt(report.vx * report.vx + report.vy * report.vy);

    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        dvl_msg_.velocity_covar[i * 3 + j] = report.covariance(i, j);
      }
    }

    dvl_msg_.num_good_beams = 0;
    for (std::size_t i = 0; i < report.transducers.size(); ++i) {
      dvl_msg_.beam_quality[i] = report.transducers[i].rssi;
      dvl_msg_.beam_velocity[i] = report.transducers[i].velocity;
      dvl_msg_.range[i] = report.transducers[i].distance;
      dvl_msg_.num_good_beams += report.transducers[i].beam_valid ? 1 : 0;
    }

    dvl_pub_->publish(dvl_msg_);
  });

  // much of the following code could be moved into the above callback, but we separate it to improve readability
  client_->register_callback([this](const VelocityReport & report) {
    const auto t = std::chrono::time_point_cast<std::chrono::nanoseconds>(report.time_of_validity);
    odom_msg_.header.stamp = rclcpp::Time(t.time_since_epoch().count());

    odom_msg_.twist.twist.linear.x = report.vx;
    odom_msg_.twist.twist.linear.y = report.vy;
    odom_msg_.twist.twist.linear.z = report.vz;

    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        odom_msg_.twist.covariance[i * 6 + j] = report.covariance(i, j);
      }
    }

    odom_pub_->publish(odom_msg_);
  });

  client_->register_callback([this](const DeadReckoningReport & report) {
    const auto t = std::chrono::time_point_cast<std::chrono::nanoseconds>(report.ts);
    dead_reckoning_msg_.header.stamp = rclcpp::Time(t.time_since_epoch().count());
    dead_reckoning_msg_.pose.pose.position.x = report.x;
    dead_reckoning_msg_.pose.pose.position.y = report.y;
    dead_reckoning_msg_.pose.pose.position.z = report.z;

    tf2::Quaternion q;
    q.setRPY(report.roll, report.pitch, report.yaw);
    dead_reckoning_msg_.pose.pose.orientation = tf2::toMsg(q);

    dead_reckoning_msg_.pose.covariance[0] = report.std;
    dead_reckoning_msg_.pose.covariance[7] = report.std;
    dead_reckoning_msg_.pose.covariance[14] = report.std;

    // orientation covariance isn't provided by the DVL
    // set to -1 to indicate that it is unknown
    dead_reckoning_msg_.pose.covariance[21] = -1;
    dead_reckoning_msg_.pose.covariance[28] = -1;
    dead_reckoning_msg_.pose.covariance[35] = -1;

    dead_reckoning_pub_->publish(dead_reckoning_msg_);
  });

  client_->register_callback([this](const DeadReckoningReport & report) {
    const auto t = std::chrono::time_point_cast<std::chrono::nanoseconds>(report.ts);
    odom_msg_.header.stamp = rclcpp::Time(t.time_since_epoch().count());

    odom_msg_.pose.pose.position.x = report.x;
    odom_msg_.pose.pose.position.y = report.y;
    odom_msg_.pose.pose.position.z = report.z;

    tf2::Quaternion q;
    q.setRPY(report.roll, report.pitch, report.yaw);
    odom_msg_.pose.pose.orientation = tf2::toMsg(q);

    odom_msg_.pose.covariance[0] = report.std;
    odom_msg_.pose.covariance[7] = report.std;
    odom_msg_.pose.covariance[14] = report.std;

    // same as above: orientation covariance isn't provided by the DVL so set to -1
    odom_msg_.pose.covariance[21] = -1;
    odom_msg_.pose.covariance[28] = -1;
    odom_msg_.pose.covariance[35] = -1;

    odom_pub_->publish(odom_msg_);
  });

  enable_acoustic_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_acoustic",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      std::future<CommandResponse> f = client_->enable_acoustics(request->data);
      populate_service_response(response, f);
    });

  enable_dark_mode_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_dark_mode",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      std::future<CommandResponse> f = client_->enable_dark_mode(request->data);
      populate_service_response(response, f);
    });

  enable_periodic_cycling_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/enable_periodic_cycling",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      std::future<CommandResponse> f = client_->enable_periodic_cycling(request->data);
      populate_service_response(response, f);
    });

  calibrate_gyro_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/calibrate_gyro",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,  // NOLINT
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::future<CommandResponse> f = client_->calibrate_gyro();
      populate_service_response(response, f);
    });

  reset_dead_reckoning_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_dead_reckoning",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,  // NOLINT
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::future<CommandResponse> f = client_->reset_dead_reckoning();
      populate_service_response(response, f);
    });

  trigger_ping_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_ping",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,  // NOLINT
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::future<CommandResponse> f = client_->trigger_ping();
      populate_service_response(response, f);
    });

  RCLCPP_INFO(get_logger(), "WaterLinkedDvlDriver loaded successfully");

  return CallbackReturn::SUCCESS;
}

auto WaterLinkedDvlDriver::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn
{
  std::future<CommandResponse> f = client_->reset_dead_reckoning();
  const CommandResponse response = f.get();
  if (!response.success) {
    RCLCPP_ERROR(get_logger(), "Failed to reset dead reckoning: %s", response.error_message.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace waterlinked::ros

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<waterlinked::ros::WaterLinkedDvlDriver>();
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
