#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "klotski_interfaces/action/move_piece.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>

using namespace std::placeholders;

class MoveToMarker : public rclcpp::Node
{
public:
  using MoveAction = klotski_interfaces::action::MovePiece;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveAction>;

  MoveToMarker() : Node("move_to_marker")
  {
    // ========== Action Server 初始化 ==========
    action_server_ = rclcpp_action::create_server<MoveAction>(
        this,
        "move_piece_action",
        std::bind(&MoveToMarker::handle_goal, this, _1, _2),
        std::bind(&MoveToMarker::handle_cancel, this, _1),
        std::bind(&MoveToMarker::handle_accepted, this, _1));

    // ========== Marker 订阅 ==========
    marker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_single/pose", 10,
        std::bind(&MoveToMarker::marker_callback, this, _1));

    // ========== TF 监听器 ==========
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MoveToMarker::tfCallback, this));

    // ========== MoveIt 初始化 ==========
    // move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
    //     std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    // move_group_interface->setPlanningTime(10.0);

    // std::string frame_id = move_group_interface->getPlanningFrame();
    // ========== MoveIt 初始化 ==========
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");

    //  统一设置规划器参数
    move_group_interface->setPlannerId("RRTConnectkConfigDefault");
    move_group_interface->setPlanningTime(10.0); // 10 秒规划时间
    move_group_interface->setNumPlanningAttempts(10);
    move_group_interface->setMaxVelocityScalingFactor(0.5);     // 50% 速度
    move_group_interface->setMaxAccelerationScalingFactor(0.5); // 50% 加速度

    std::string frame_id = move_group_interface->getPlanningFrame();

    RCLCPP_INFO(this->get_logger(),
                "MoveIt 初始化完成，使用规划器: RRTConnectkConfigDefault");

    // ========== 碰撞物体设置 ==========
    auto col_object_backWall = generateCollisionObject(
        2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
    auto col_object_sideWall = generateCollisionObject(
        0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
    auto col_object_table = generateCollisionObject(
        2.4, 1.2, 0.04, 0.8, 0.3, -0.04, frame_id, "table");
    auto col_object_ceiling = generateCollisionObject(
        2.4, 2.4, 0.04, 0.85, 0.25, 1.5, frame_id, "ceiling");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(col_object_backWall);
    planning_scene_interface.applyCollisionObject(col_object_sideWall);
    planning_scene_interface.applyCollisionObject(col_object_table);
    planning_scene_interface.applyCollisionObject(col_object_ceiling);

    RCLCPP_INFO(this->get_logger(), "✅ MovePiece Action Server 已启动");
  }

private:
  // constant value
  const double BOARD_HEIGHT = 0.008;   // 板子高度 8mm
  const double BLOCK_HEIGHT = 0.015;   // 方块高度 15mm
  const double GRIPPER_HEIGHT = 0.191; // 夹爪高度 191mm

  const double APPROACH_HEIGHT = 0.1 + GRIPPER_HEIGHT + BOARD_HEIGHT + BLOCK_HEIGHT;  // 0.314m
  const double GRIP_HEIGHT = GRIPPER_HEIGHT + BOARD_HEIGHT + BLOCK_HEIGHT;            // 0.214m
  const double RETREAT_HEIGHT = 0.200 + GRIPPER_HEIGHT + BOARD_HEIGHT + BLOCK_HEIGHT; // 0.414m

  // ========== 成员变量 ==========
  rclcpp_action::Server<MoveAction>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  geometry_msgs::msg::PoseStamped latest_marker_pose_;
  bool marker_received_ = false;

  // ========== Action 回调函数 ==========
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
                "收到请求: phase=%d, type=%d, color=%d, to_cell=(%d,%d)",
                goal->phase,
                goal->move.piece.type,
                goal->move.piece.color,
                goal->move.to_cell.col,
                goal->move.to_cell.row);
    (void)uuid;

    if (goal->phase > MoveAction::Goal::PHASE_RETREAT)
    {
      RCLCPP_WARN(this->get_logger(), "无效的 phase: %d", goal->phase);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    std::thread{std::bind(&MoveToMarker::execute, this, _1), goal_handle}.detach();
  }

  // ========== 主执行函数 ==========
  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveAction::Feedback>();
    auto result = std::make_shared<MoveAction::Result>();

    uint8_t phase = goal->phase;
    auto piece_cells = goal->move.piece.cells;
    uint32_t target_col = goal->move.to_cell.col;
    uint32_t target_row = goal->move.to_cell.row;

    bool success = false;

    switch (phase)
    {
    case MoveAction::Goal::PHASE_IDLE:
      RCLCPP_INFO(this->get_logger(), "执行 IDLE");
      success = true;
      break;

    case MoveAction::Goal::PHASE_APPROACH:
    {
      //  移动到棋子中心位置
      if (piece_cells.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "piece.cells 为空！");
        success = false;
        break;
      }

      // 计算棋子中心
      geometry_msgs::msg::PoseStamped piece_center_pose =
          calculatePieceCenterPose(piece_cells);

      RCLCPP_INFO(this->get_logger(),
                  "PHASE_APPROACH: 移动到棋子中心");

      // 先到上方
      success = execute_approach(goal_handle, feedback, piece_center_pose);
      if (!success)
        break;

      // 再下降到棋子中心
      success = execute_pick_place(goal_handle, feedback, piece_center_pose);
      break;
    }

    case MoveAction::Goal::PHASE_PICK_PLACE:
    {
      //  移动到目标位置中心
      // 计算目标位置中心（如果棋子是 2x2，目标中心也需要计算）
      geometry_msgs::msg::PoseStamped target_center_pose;

      if (piece_cells.size() > 1)
      {
        // 多格子棋子：计算目标区域中心
        // 假设 to_cell 是目标的左下角
        double offset_col = 0.0;
        double offset_row = 0.0;

        // 根据棋子类型计算偏移
        uint8_t piece_type = goal->move.piece.type;
        if (piece_type == 1)
        { // TYPE_2_2
          offset_col = 0.5;
          offset_row = 0.5;
        }
        else if (piece_type == 2)
        { // TYPE_1_2 (horizontal)
          offset_col = 0.5;
          offset_row = 0.0;
        }
        else if (piece_type == 3)
        { // TYPE_2_1 (vertical)
          offset_col = 0.0;
          offset_row = 0.5;
        }

        target_center_pose = calculateWorldPose(
            target_col + offset_col,
            target_row + offset_row);
      }
      else
      {
        // 1x1 棋子：直接使用 to_cell
        target_center_pose = calculateWorldPose(target_col, target_row);
      }

      RCLCPP_INFO(this->get_logger(),
                  "PHASE_PICK_PLACE: 移动到目标位置 (%d, %d)",
                  target_col, target_row);

      // 先到上方
      success = execute_approach(goal_handle, feedback, target_center_pose);
      if (!success)
        break;

      // 再下降到目标位置
      success = execute_pick_place(goal_handle, feedback, target_center_pose);
      break;
    }

    case MoveAction::Goal::PHASE_RETREAT:
      RCLCPP_INFO(this->get_logger(), "PHASE_RETREAT: 向上移动");
      success = execute_retreat(goal_handle, feedback);
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "未知的 phase: %d", phase);
      success = false;
    }

    result->success = success;
    if (success)
    {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "✅ Phase %d 完成", phase);
    }
    else
    {
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "❌ Phase %d 失败", phase);
    }
  }

  // ========== 计算棋子中心位置 ==========
  geometry_msgs::msg::PoseStamped calculatePieceCenterPose(
      const std::vector<klotski_interfaces::msg::Cell> &cells)
  {
    if (cells.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "cells 为空！");
      return geometry_msgs::msg::PoseStamped();
    }

    // 计算所有格子的中心
    double sum_col = 0.0;
    double sum_row = 0.0;

    for (const auto &cell : cells)
    {
      sum_col += cell.col;
      sum_row += cell.row;
    }

    double center_col = sum_col / cells.size();
    double center_row = sum_row / cells.size();

    RCLCPP_INFO(this->get_logger(),
                "棋子占据 %zu 个格子，中心位置: (%.2f, %.2f)",
                cells.size(), center_col, center_row);

    return calculateWorldPose(center_col, center_row);
  }

  // ========== 各阶段执行函数 ==========
  bool execute_approach(
      const std::shared_ptr<GoalHandleMove> goal_handle,
      std::shared_ptr<MoveAction::Feedback> feedback,
      geometry_msgs::msg::PoseStamped target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "移动到目标上方 (APPROACH 高度)");

    feedback->progress = 0.1;
    goal_handle->publish_feedback(feedback);

    //  设置 APPROACH 高度 (314mm)
    target_pose.pose.position.z = APPROACH_HEIGHT;

    RCLCPP_INFO(this->get_logger(),
                "目标: (%.3f, %.3f, %.3f)",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);

    feedback->progress = 0.3;
    goal_handle->publish_feedback(feedback);

    move_group_interface->setPoseTarget(target_pose);

    feedback->progress = 0.5;
    goal_handle->publish_feedback(feedback);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "规划失败");
      return false;
    }

    feedback->progress = 0.7;
    goal_handle->publish_feedback(feedback);

    success = (move_group_interface->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    return success;
  }

  bool execute_pick_place(
      const std::shared_ptr<GoalHandleMove> goal_handle,
      std::shared_ptr<MoveAction::Feedback> feedback,
      geometry_msgs::msg::PoseStamped target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "下降到抓取高度 (GRIP 高度)");

    feedback->progress = 0.1;
    goal_handle->publish_feedback(feedback);

    //  设置 GRIP 高度 (214mm)
    target_pose.pose.position.z = GRIP_HEIGHT;

    RCLCPP_INFO(this->get_logger(),
                "目标: (%.3f, %.3f, %.3f)",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);

    move_group_interface->setPoseTarget(target_pose);

    feedback->progress = 0.3;
    goal_handle->publish_feedback(feedback);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "规划失败");
      return false;
    }

    feedback->progress = 0.6;
    goal_handle->publish_feedback(feedback);

    success = (move_group_interface->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    return success;
  }

  bool execute_retreat(const std::shared_ptr<GoalHandleMove> goal_handle, std::shared_ptr<MoveAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "向上撤退 (RETREAT 高度)");

    feedback->progress = 0.2;
    goal_handle->publish_feedback(feedback);

    //  获取当前 x, y 位置，但设置新的 z 高度
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = move_group_interface->getPlanningFrame();
    current_pose.pose = move_group_interface->getCurrentPose().pose;

    //  设置 RETREAT 高度 (414mm)
    current_pose.pose.position.z = RETREAT_HEIGHT;

    RCLCPP_INFO(this->get_logger(),
                "目标: (%.3f, %.3f, %.3f)",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);

    feedback->progress = 0.4;
    goal_handle->publish_feedback(feedback);

    move_group_interface->setPoseTarget(current_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
      RCLCPP_ERROR(this->get_logger(), "规划失败");
      return false;
    }

    feedback->progress = 0.7;
    goal_handle->publish_feedback(feedback);

    success = (move_group_interface->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);

    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    return success;
  }

  // ========== 辅助函数 ==========
  // geometry_msgs::msg::PoseStamped calculateWorldPose(double col, double row)
  // {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = "base_link";
  //   pose.header.stamp = this->now();

  //   // ArUco marker 在棋盘中心
  //   // double board_center_x = 0.13346;  // 测量得到的中心 x (133.45mm)
  //   // double board_center_y = -0.58839; // 测量得到的中心 y (-588.37mm)

  //   double board_center_x = 0.60;  // 测量得到的中心 x
  //   double board_center_y = 0.425; // 测量得到的中心 y

  //   //  计算左下角位置
  //   double board_width = 0.20;                                  // 20 cm
  //   double board_height = 0.25;                                 // 25 cm
  //   double grid_origin_x = board_center_x - board_width / 2.0;  // 0.60 - 0.10 = 0.50
  //   double grid_origin_y = board_center_y - board_height / 2.0; // 0.425 - 0.125 = 0.30

  //   //  计算格子位置
  //   double cell_size = 0.05;
  //   pose.pose.position.x = grid_origin_x + col * cell_size;
  //   pose.pose.position.y = grid_origin_y + row * cell_size;

  //   pose.pose.position.z = BOARD_HEIGHT; // 0.008m
  //   pose.pose.orientation.w = 1.0;

  //   // make the end effector facing down
  //   tf2::Quaternion q;
  //   q.setRPY(0, M_PI, 0);
  //   pose.pose.orientation = tf2::toMsg(q);
  //   return pose;
  // }
  geometry_msgs::msg::PoseStamped calculateWorldPose(double col, double row)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = this->now();

    //  棋盘中心
    double board_center_x = 0.60;
    double board_center_y = 0.425;

    //  计算左下角位置
    double board_width = 0.20;
    double board_height = 0.25;
    double grid_origin_x = board_center_x - board_width / 2.0;
    double grid_origin_y = board_center_y - board_height / 2.0;

    //  计算格子位置
    double cell_size = 0.05;
    pose.pose.position.x = grid_origin_x + col * cell_size;
    pose.pose.position.y = grid_origin_y + row * cell_size;
    pose.pose.position.z = BOARD_HEIGHT; // 0.008m

    //  设置末端执行器朝下
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI, 0.0); // Roll = 180°
    pose.pose.orientation = tf2::toMsg(q);

    return pose;
  }

  void marker_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_marker_pose_ = *msg;
    marker_received_ = true;
  }

  void tfCallback()
  {
    // need somthing to check if is moving
    // if (is_moving_)
    // {
    //   return;
    // }

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

      // is_moving_ = true;

      move_group_interface->setPoseTarget(targetPose);
      auto result = move_group_interface->move();

      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Move success!");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Move failed!");
      }

      // is_moving_ = false;
    }
    catch (const tf2::TransformException &ex)
    {
      return;
    }
  }

  moveit_msgs::msg::CollisionObject generateCollisionObject(
      float size_x, float size_y, float size_z,
      float center_x, float center_y, float center_z,
      const std::string &frame_id, const std::string &id)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size_x;
    primitive.dimensions[1] = size_y;
    primitive.dimensions[2] = size_z;

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = center_x;
    box_pose.position.y = center_y;
    box_pose.position.z = center_z;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToMarker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
