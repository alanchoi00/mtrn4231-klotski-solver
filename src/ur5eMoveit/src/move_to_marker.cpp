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
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);

    std::string frame_id = move_group_interface->getPlanningFrame();

    // ========== 碰撞物体设置 ==========
    auto col_object_backWall = generateCollisionObject(
        2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
    auto col_object_sideWall = generateCollisionObject(
        0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
    auto col_object_table = generateCollisionObject(
        2.4, 1.2, 0.04, 0.8, 0.3, 0.05, frame_id, "table");
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
                "收到移动请求: piece=%s, target=(%d,%d)",
                goal->move.piece.id.c_str(),
                goal->move.to_cell.col,
                goal->move.to_cell.row);
    (void)uuid;

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

  // ========== 主执行函数：执行完整的移动流程 ==========
  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "开始执行完整移动流程");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveAction::Feedback>();
    auto result = std::make_shared<MoveAction::Result>();

    // 从 goal 中提取信息
    std::string piece_id = goal->move.piece.id;
    std::string piece_color = goal->move.piece.color;
    uint32_t target_col = goal->move.to_cell.col;
    uint32_t target_row = goal->move.to_cell.row;

    RCLCPP_INFO(this->get_logger(),
                "移动棋子: piece=%s, color=%s, target=(%d,%d)",
                piece_id.c_str(), piece_color.c_str(),
                target_col, target_row);

    // 计算目标的真实世界坐标
    geometry_msgs::msg::PoseStamped target_pose =
        calculateWorldPose(target_col, target_row);

    RCLCPP_INFO(this->get_logger(),
                "目标世界坐标: x=%.3f, y=%.3f, z=%.3f",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);

    // bool success = true;

    // ========== Phase 1: APPROACH ==========
    feedback->phase = "approach";
    feedback->progress = 0.0;
    goal_handle->publish_feedback(feedback);

    if (!execute_approach(goal_handle, feedback, target_pose))
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Approach 阶段失败");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    // ========== Phase 2: PICK_PLACE (下降到目标) ==========
    feedback->phase = "place";
    feedback->progress = 0.0;
    goal_handle->publish_feedback(feedback);

    if (!execute_pick_place(goal_handle, feedback, target_pose))
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Pick/Place 阶段失败");
      result->success = false;
      goal_handle->abort(result);
      return;
    }
    // ========== 等待 Brain 控制夹爪 ==========
    feedback->phase = "complete";
    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(),
                "✅ MovePiece 完成：机械臂已到达 (%d, %d)，等待 GripPiece",
                target_col, target_row);

    // RCLCPP_INFO(this->get_logger(), "⏸️  等待外部夹爪控制...");
    // // Brain 会在这里控制夹爪

    // ========== Phase 3: RETREAT ==========
    feedback->phase = "retreat";
    feedback->progress = 0.0;
    goal_handle->publish_feedback(feedback);

    if (!execute_retreat(goal_handle, feedback))
    {
      RCLCPP_ERROR(this->get_logger(), "❌ Retreat 阶段失败");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    // ========== 完成 ==========
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "✅ 完整移动流程成功");
  }

  // ========== 各阶段执行函数 ==========
  bool execute_approach(
      const std::shared_ptr<GoalHandleMove> goal_handle,
      std::shared_ptr<MoveAction::Feedback> feedback,
      geometry_msgs::msg::PoseStamped target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "执行 APPROACH 阶段");

    feedback->progress = 0.1;
    goal_handle->publish_feedback(feedback);

    // 在目标位置上方 20cm
    target_pose.pose.position.z += 0.20;

    RCLCPP_INFO(this->get_logger(),
                "移动到 (%.3f, %.3f, %.3f)",
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
    RCLCPP_INFO(this->get_logger(), "执行 PICK_PLACE 阶段");

    feedback->progress = 0.1;
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(),
                "移动到 (%.3f, %.3f, %.3f)",
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

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "✅ 已到达目标位置");
    }

    return success;
  }

  bool execute_retreat(
      const std::shared_ptr<GoalHandleMove> goal_handle,
      std::shared_ptr<MoveAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "执行 RETREAT 阶段");

    feedback->progress = 0.2;
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = move_group_interface->getPlanningFrame();
    current_pose.pose = move_group_interface->getCurrentPose().pose;

    current_pose.pose.position.z += 0.20;

    RCLCPP_INFO(this->get_logger(),
                "移动到 (%.3f, %.3f, %.3f)",
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
  geometry_msgs::msg::PoseStamped calculateWorldPose(uint32_t col, uint32_t row)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = this->now();

    double grid_origin_x = 0.5;
    double grid_origin_y = 0.3;
    double grid_origin_z = 0.1;
    double cell_size = 0.05;

    pose.pose.position.x = grid_origin_x + col * cell_size;
    pose.pose.position.y = grid_origin_y + row * cell_size;
    pose.pose.position.z = grid_origin_z;

    pose.pose.orientation.w = 1.0;

    return pose;
  }

  void marker_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_marker_pose_ = *msg;
    marker_received_ = true;
  }

  void tfCallback()
  {
    // TF 逻辑
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
