#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <string>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "klotski_interfaces/msg/move.hpp"
#include "klotski_interfaces/msg/piece.hpp"
#include "klotski_interfaces/msg/cell.hpp"

// Function to generate a collision object
auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z, std::string frame_id, std::string id)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

// Function to generate a target position message
auto generatePoseMsg(float x, float y, float z, float qx, float qy, float qz, float qw)
{
  geometry_msgs::msg::Pose msg;
  msg.orientation.x = qx;
  msg.orientation.y = qy;
  msg.orientation.z = qz;
  msg.orientation.w = qw;
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  return msg;
}

using std::placeholders::_1;

class move_to_marker : public rclcpp::Node
{
public:
  move_to_marker() : Node("move_to_marker")
  {

    // Initalise the transformation listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Look up the transformation ever 200 milliseconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&move_to_marker::tfCallback, this));

    // Generate the movegroup interface
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);

    std::string frame_id = move_group_interface->getPlanningFrame();

    // Generate the objects to avoid
    // Generate a table collision object based on the lab task
    auto col_object_backWall = generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
    auto col_object_sideWall = generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
    auto col_object_table = generateCollisionObject(2.4, 1.2, 0.04, 0.8, 0.3, 0.05, frame_id, "table");
    auto col_object_ceiling = generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.5, frame_id, "ceiling");

    // planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.5, frame_id, "ceiling"));
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Apply table as a collision object
    planning_scene_interface.applyCollisionObject(col_object_backWall);
    planning_scene_interface.applyCollisionObject(col_object_sideWall);
    planning_scene_interface.applyCollisionObject(col_object_table);
    planning_scene_interface.applyCollisionObject(col_object_ceiling);

    move_subscriber_ = this->create_subscription<klotski_interfaces::msg::Move>("/plan/move", 10, std::bind(&move_to_marker::moveCallback, this, _1));
  }

private:
  void tfCallback()
  {
    if (is_moving_)
    {
      RCLCPP_INFO(this->get_logger(), "Moving, skip...");
      return;
    }

    std::string fromFrameRel = "base_link";
    std::string toFrameRel = "OOI";
    geometry_msgs::msg::TransformStamped t;

    try
    {
      t = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);

      RCLCPP_INFO(this->get_logger(), "New Coordinate: [%.3f, %.3f, %.3f]",
                  t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z);

      geometry_msgs::msg::Pose targetPose;
      targetPose.position.x = t.transform.translation.x;
      targetPose.position.y = t.transform.translation.y;
      targetPose.position.z = t.transform.translation.z + 0.1;

      targetPose.orientation.x = 0.0;
      targetPose.orientation.y = 1.0;
      targetPose.orientation.z = 0.0;
      targetPose.orientation.w = 0.0;

      move_group_interface->setPoseTarget(targetPose);

      is_moving_ = true;

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group_interface->plan(plan) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Plan success! Start Executle");
        move_group_interface->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Accive set gol, waiting for new coordinate...");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Plan failure!");
      }

      is_moving_ = false;
    }
    catch (const tf2::TransformException &ex)
    {
      return;
    }
  }

  void moveCallback(const klotski_interfaces::msg::Move::SharedPtr msg)
  {
    if (is_moving_)
    {
      RCLCPP_WARN(this->get_logger(), "Moving");
      return;
    }

    auto piece = msg->piece;
    RCLCPP_INFO(this->get_logger(), "  ID: %s", piece.id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Color: %s", piece.color.c_str());
    RCLCPP_INFO(this->get_logger(), "  Size: %zu", piece.cells.size());

    auto to_cell = msg->to_cell;
    RCLCPP_INFO(this->get_logger(), "  Goal Position: col=%u, row=%u",
                to_cell.col, to_cell.row);

    geometry_msgs::msg::Pose targetPose = gridToRobotPose(to_cell);

    executeMoveToTarget(targetPose);
  }

  geometry_msgs::msg::Pose gridToRobotPose(const klotski_interfaces::msg::Cell &cell)
  {
    geometry_msgs::msg::Pose pose;

    float cellSize = 0.050;

    // I hardcore the palce of the board in base_link coordinate
    float boardOriginX = 0.250;
    float boardOriginY = 0.200;
    float boardHeight = 0.0800;

    pose.position.x = boardOriginX + cell.col * cellSize;
    pose.position.y = boardOriginY + cell.row * cellSize;
    pose.position.z = boardHeight + 0.05;

    pose.orientation.x = 0.0;
    pose.orientation.y = 1.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    return pose;
  }

  void executeMoveToTarget(const geometry_msgs::msg::Pose &targetPose)
  {
    is_moving_ = true;

    move_group_interface->setPoseTarget(targetPose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Plan success! Start Executle");
      move_group_interface->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Accive set gol, waiting for new coordinate...");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "plan failure!");
    }

    is_moving_ = false;
  }

  rclcpp::Subscription<klotski_interfaces::msg::Move>::SharedPtr move_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  bool is_moving_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<move_to_marker>());
  rclcpp::shutdown();
  return 0;
}
