#include <chrono>
#include <memory>
#include <iostream>
#include <stdint.h>
#include <cmath>
#include <array>
#include <vector>
#include <set>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>

#include <nlohmann/json.hpp>

#include "precision_landing/px4_offboard_controller.hpp"
#include "precision_landing/bt_nodes.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
  std::array<float, 3> coords = {0.0f, 0.0f, -5.0f};
  bool found=false;
  bool reached=false; 
  OffboardControl() : Node("offboard_control")
  {
this->declare_parameter<std::string>("checkpoints_file", "checkpoints.json");
load_checkpoints(this->get_parameter("checkpoints_file").as_string());

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    float marker_size = 1.0f; 
    float half_size = marker_size / 2.0f;
    object_points_ = {
      cv::Point3f(-half_size,  half_size, 0.0f),
      cv::Point3f( half_size,  half_size, 0.0f),
      cv::Point3f( half_size, -half_size, 0.0f),
      cv::Point3f(-half_size, -half_size, 0.0f)
    };

    cameraMatrix_ = (cv::Mat_<double>(3, 3) << 
     539.94,    0.0,   640.0,
        0.0, 539.94,   480.0,
        0.0,    0.0,     1.0);
distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      rclcpp::SensorDataQoS(),
      std::bind(&OffboardControl::cameraCallback, this, std::placeholders::_1)
    );
local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
  "/fmu/out/vehicle_local_position_v1",
  rclcpp::SensorDataQoS(),
  [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
      this->current_x_ = msg->x;
      this->current_y_ = msg->y;
      this->current_z_ = msg->z;
      if (!this->reached && std::abs(this->current_z_ - (-5.0f)) < 0.4f) {
      
        this->reached = true; 
        
        RCLCPP_INFO(this->get_logger(), "Reached 5.0 meters, exploration begins");
      }

      if(reached==true && wp_index_ < waypoints_.size()){
        const auto & wp = waypoints_[wp_index_];
        float dx = wp[0] - msg->x;
        float dy = wp[1] - msg->y;
        float dz = wp[2] - msg->z;
        if(std::sqrt(dx*dx + dy*dy + dz*dz) < waypoint_tolerance_){
          RCLCPP_INFO(this->get_logger(), "Checkpoint %zu/%zu reached", wp_index_ + 1, waypoints_.size());
          wp_index_++;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Current Position X: %.2f, Y: %.2f, Z: %.2f", this->current_x_, this->current_y_, this->current_z_);
  }
);
    RCLCPP_INFO(this->get_logger(), "OffboardControl subscribed to '/camera/image_raw'");

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {
      if (offboard_setpoint_counter_ == 10) {
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        this->arm();
      }

      publish_offboard_control_mode();
      publish_trajectory_setpoint();

      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
    };

    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

  void arm();
  void disarm();

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::vector<cv::Point3f> object_points_;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
  float current_x_{0.0f};
  float current_y_{0.0f};
  float current_z_{0.0f};

  std::vector<std::array<float, 3>> waypoints_;
  size_t wp_index_=0;
  float waypoint_tolerance_=0.5f;

  void load_checkpoints(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "File not found: %s", path.c_str());
      return;
    }

  nlohmann::ordered_json j = nlohmann::ordered_json::parse(file, nullptr, false);
    if (j.is_discarded() || !j.contains("checkpoints")) {
      RCLCPP_ERROR(this->get_logger(), "Invalid format: %s", path.c_str());
      return;
    }

    const auto & cp = j["checkpoints"];

 for (const auto & item : cp.items()) {
    const auto & v = item.value();
    waypoints_.push_back({
      static_cast<float>(v.value("x", 0.0)),
      static_cast<float>(v.value("y", 0.0)),
      static_cast<float>(v.value("z", -5.0))
    });
  }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu checkpoints from %s", waypoints_.size(), path.c_str());
  }

  void cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Receiving streaming properly");

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(gray_frame, dictionary_, corners, ids, detector_params_, rejected);

for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == 0) {
          RCLCPP_INFO(this->get_logger(), "Merker Found");
          cv::Mat rvec, tvec;
          bool success = cv::solvePnP(object_points_, corners[i], cameraMatrix_, distCoeffs_, rvec, tvec);

          if (success) {
              float x_cam = static_cast<float>(tvec.at<double>(0, 0));
              float y_cam = static_cast<float>(tvec.at<double>(1, 0)); 

              float offset_body_x = -y_cam; 
              float offset_body_y =  x_cam; 

           
              float current_yaw = 0.0f; 

              float offset_world_x = offset_body_x * std::cos(current_yaw) - offset_body_y * std::sin(current_yaw);
              float offset_world_y = offset_body_x * std::sin(current_yaw) + offset_body_y * std::cos(current_yaw);

              float target_x = this->current_x_ + offset_world_x;
              float target_y = this->current_y_ + offset_world_y;

              if (!this->found) {
                  this->found = true;
                  this->coords[0] = target_x;
                  this->coords[1] = target_y;
                  this->coords[2]=this->current_z_;
              } else {
                 
                  float alpha = 0.2f;
                  this->coords[0] = (1.0f - alpha) * this->coords[0] + alpha * target_x;
                  this->coords[1] = (1.0f - alpha) * this->coords[1] + alpha * target_y;
                  this->coords[2]=0.0f;
              }
          }
        }
    }
  }

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  uint64_t offboard_setpoint_counter_;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}


void OffboardControl::publish_trajectory_setpoint()
{
  TrajectorySetpoint msg{};
  if(found==true){
    RCLCPP_INFO(this->get_logger(), "Going Toward-> X: %.2f, Y: %.2f", this->coords[0], this->coords[1]);
  msg.position = this->coords;
  }
  else if(reached==false){
    msg.position={0.0f, 0.0f, -5.0f};
    
  }
  else if(found==false && reached==true){
    float x, y, z; 
    x=0.0f; 
    y=0.0f; 
    z=-5.0f; 
      if(wp_index_ < waypoints_.size()){
        x=waypoints_[wp_index_][0]; 
        y=waypoints_[wp_index_][1]; 
        z=waypoints_[wp_index_][2]; 
      }
      msg.position = {x, y, z};
    
  }
  msg.yaw = 0.0f; 
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
  std::cout << "Starting offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}










