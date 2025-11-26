#include "arm_manipulator.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::placeholders;

namespace pkg_manipulation {

ArmManipulator::ArmManipulator() : Node("arm_manipulator") {
  declareAndLoadParameters();

  calculateHeights();

  // ============ new board dynamic============
  this->declare_parameter("use_dynamic_board_pose", true);
  use_dynamic_board_pose_ =
      this->get_parameter("use_dynamic_board_pose").as_bool();
  board_rotation_yaw_ = 0.0;
  if (use_dynamic_board_pose_) {
    board_pose_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/board_pose", 10,
            std::bind(&ArmManipulator::board_pose_callback, this,
                      std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "✓ Subscribed to /board_pose for dynamic board updates");
  } else {
    RCLCPP_INFO(this->get_logger(), "Using static board pose from config file");
  }

  action_server_ = rclcpp_action::create_server<MoveAction>(
      this, "/arm_manipulation/move_piece",
      std::bind(&ArmManipulator::handle_goal, this, _1, _2),
      std::bind(&ArmManipulator::handle_cancel, this, _1),
      std::bind(&ArmManipulator::handle_accepted, this, _1));

  move_group_interface_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          std::shared_ptr<rclcpp::Node>(this), planning_group_);

  // Configure planner parameters from config
  move_group_interface_->setPlannerId(planner_id_);
  move_group_interface_->setPlanningTime(planning_time_);
  move_group_interface_->setNumPlanningAttempts(num_planning_attempts_);
  move_group_interface_->setMaxVelocityScalingFactor(
      max_velocity_scaling_factor_);
  move_group_interface_->setMaxAccelerationScalingFactor(
      max_acceleration_scaling_factor_);

  // Enable trajectory smoothing and optimization
  move_group_interface_->allowReplanning(true);
  move_group_interface_->allowLooking(true);
  move_group_interface_->setReplanAttempts(replan_attempts_);

  std::string frame_id = move_group_interface_->getPlanningFrame();

  RCLCPP_INFO(this->get_logger(), "MoveIt initialized with planner: %s",
              planner_id_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "Height parameters - Board: %.3fm, Block: %.3fm, Gripper: %.3fm",
              board_height_, block_height_, gripper_height_);
  RCLCPP_INFO(
      this->get_logger(),
      "Calculated heights - Approach: %.3fm, Grip: %.3fm, Retreat: %.3fm",
      approach_height_, grip_height_, retreat_height_);

  auto col_object_backWall = generateCollisionObject(
      2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
  auto col_object_sideWall = generateCollisionObject(
      0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
  auto col_object_table = generateCollisionObject(2.4, 1.2, 0.04, 0.8, 0.3,
                                                  -0.04, frame_id, "table");
  auto col_object_ceiling = generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25,
                                                    1.5, frame_id, "ceiling");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(col_object_backWall);
  planning_scene_interface.applyCollisionObject(col_object_sideWall);
  planning_scene_interface.applyCollisionObject(col_object_table);
  planning_scene_interface.applyCollisionObject(col_object_ceiling);

  RCLCPP_INFO(this->get_logger(), "MovePiece Action Server started");
}

void ArmManipulator::declareAndLoadParameters() {
  this->declare_parameter<double>("planning_time");
  this->declare_parameter<int>("num_planning_attempts");
  this->declare_parameter<double>("max_velocity_scaling_factor");
  this->declare_parameter<double>("max_acceleration_scaling_factor");
  this->declare_parameter<std::string>("planner_id");
  this->declare_parameter<std::string>("planning_group");
  this->declare_parameter<std::string>("robot_ip");
  this->declare_parameter<double>("board_height");
  this->declare_parameter<double>("block_height");
  this->declare_parameter<double>("gripper_height");
  this->declare_parameter<double>("approach_offset");
  this->declare_parameter<double>("retreat_offset");
  this->declare_parameter<double>("board_center_x");
  this->declare_parameter<double>("board_center_y");
  this->declare_parameter<double>("board_width");
  this->declare_parameter<double>("board_length");
  this->declare_parameter<double>("cell_size");
  this->declare_parameter<double>("elbow_min_angle");
  this->declare_parameter<double>("elbow_max_angle");
  this->declare_parameter<double>("elbow_constraint_weight");
  this->declare_parameter<double>("shoulder_pan_min_angle");
  this->declare_parameter<double>("shoulder_pan_max_angle");
  this->declare_parameter<double>("shoulder_lift_min_angle");
  this->declare_parameter<double>("shoulder_lift_max_angle");
  this->declare_parameter<double>("wrist_1_min_angle");
  this->declare_parameter<double>("wrist_1_max_angle");
  this->declare_parameter<double>("joint_constraint_weight");
  this->declare_parameter<double>("cartesian_eef_step");
  this->declare_parameter<double>("cartesian_jump_threshold");
  this->declare_parameter<double>("cartesian_fraction_threshold");
  this->declare_parameter<bool>("enable_trajectory_smoothing");
  this->declare_parameter<int>("replan_attempts");

  // Get parameters
  planning_time_ = this->get_parameter("planning_time").as_double();
  num_planning_attempts_ =
      this->get_parameter("num_planning_attempts").as_int();
  max_velocity_scaling_factor_ =
      this->get_parameter("max_velocity_scaling_factor").as_double();
  max_acceleration_scaling_factor_ =
      this->get_parameter("max_acceleration_scaling_factor").as_double();
  planner_id_ = this->get_parameter("planner_id").as_string();
  planning_group_ = this->get_parameter("planning_group").as_string();
  robot_ip_ = this->get_parameter("robot_ip").as_string();
  board_height_ = this->get_parameter("board_height").as_double();
  block_height_ = this->get_parameter("block_height").as_double();
  gripper_height_ = this->get_parameter("gripper_height").as_double();
  approach_offset_ = this->get_parameter("approach_offset").as_double();
  retreat_offset_ = this->get_parameter("retreat_offset").as_double();
  board_center_x_ = this->get_parameter("board_center_x").as_double();
  board_center_y_ = this->get_parameter("board_center_y").as_double();
  board_width_ = this->get_parameter("board_width").as_double();
  board_length_ = this->get_parameter("board_length").as_double();
  cell_size_ = this->get_parameter("cell_size").as_double();
  elbow_min_angle_ = this->get_parameter("elbow_min_angle").as_double();
  elbow_max_angle_ = this->get_parameter("elbow_max_angle").as_double();
  elbow_constraint_weight_ =
      this->get_parameter("elbow_constraint_weight").as_double();
  shoulder_pan_min_angle_ =
      this->get_parameter("shoulder_pan_min_angle").as_double();
  shoulder_pan_max_angle_ =
      this->get_parameter("shoulder_pan_max_angle").as_double();
  shoulder_lift_min_angle_ =
      this->get_parameter("shoulder_lift_min_angle").as_double();
  shoulder_lift_max_angle_ =
      this->get_parameter("shoulder_lift_max_angle").as_double();
  wrist_1_min_angle_ = this->get_parameter("wrist_1_min_angle").as_double();
  wrist_1_max_angle_ = this->get_parameter("wrist_1_max_angle").as_double();
  joint_constraint_weight_ =
      this->get_parameter("joint_constraint_weight").as_double();
  cartesian_eef_step_ = this->get_parameter("cartesian_eef_step").as_double();
  cartesian_jump_threshold_ =
      this->get_parameter("cartesian_jump_threshold").as_double();
  cartesian_fraction_threshold_ =
      this->get_parameter("cartesian_fraction_threshold").as_double();
  enable_trajectory_smoothing_ =
      this->get_parameter("enable_trajectory_smoothing").as_bool();
  replan_attempts_ = this->get_parameter("replan_attempts").as_int();

  RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully");
}

void ArmManipulator::calculateHeights() {
  approach_height_ =
      approach_offset_ + gripper_height_ + board_height_ + block_height_;
  grip_height_ = gripper_height_ + board_height_ + block_height_;
  retreat_height_ =
      retreat_offset_ + gripper_height_ + board_height_ + block_height_;
}

rclcpp_action::GoalResponse ArmManipulator::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const MoveAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received request: phase=%d, type=%d, color=%d, to_cell=(%d,%d)",
              goal->phase, goal->move.piece.type, goal->move.piece.color,
              goal->move.to_cell.col, goal->move.to_cell.row);
  (void)uuid;

  if (goal->phase > MoveAction::Goal::PHASE_RETREAT) {
    RCLCPP_WARN(this->get_logger(), "Invalid phase: %d", goal->phase);
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmManipulator::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmManipulator::handle_accepted(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  std::thread{std::bind(&ArmManipulator::execute, this, _1), goal_handle}
      .detach();
}

void ArmManipulator::execute(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveAction::Feedback>();
  auto result = std::make_shared<MoveAction::Result>();

  uint8_t phase = goal->phase;
  auto piece_cells = goal->move.piece.cells;
  uint32_t target_col = goal->move.to_cell.col;
  uint32_t target_row = goal->move.to_cell.row;

  bool success = false;

  switch (phase) {
    case MoveAction::Goal::PHASE_IDLE:
      RCLCPP_INFO(this->get_logger(), "Executing IDLE");
      success = true;
      break;

    case MoveAction::Goal::PHASE_APPROACH: {
      // Move to piece center position
      if (piece_cells.empty()) {
        RCLCPP_ERROR(this->get_logger(), "piece.cells is empty!");
        success = false;
        break;
      }

      // Calculate piece center
      geometry_msgs::msg::PoseStamped piece_center_pose =
          calculatePieceCenterPose(piece_cells);

      RCLCPP_INFO(this->get_logger(), "PHASE_APPROACH: Moving to piece center");

      // First move to above
      success = execute_approach(goal_handle, feedback, piece_center_pose);
      if (!success) break;

      // Then descend to piece center
      success = execute_pick_place(goal_handle, feedback, piece_center_pose);
      break;
    }

    case MoveAction::Goal::PHASE_PICK_PLACE: {
      // Move to target position center
      geometry_msgs::msg::PoseStamped target_center_pose;

      double offset_col = 0.0;
      double offset_row = 0.0;

      uint8_t piece_type = goal->move.piece.type;
      if (piece_type == Piece::TYPE_2_2) {
        offset_col = 0.5;
        offset_row = 0.5;
      } else if (piece_type == Piece::TYPE_1_2) {
        offset_col = 0.5;
        offset_row = 0.0;
      } else if (piece_type == Piece::TYPE_2_1) {
        offset_col = 0.0;
        offset_row = 0.5;
      }

      target_center_pose =
          calculateWorldPose(target_col + offset_col, target_row + offset_row);

      RCLCPP_INFO(this->get_logger(),
                  "PHASE_PICK_PLACE: Moving to target position (%d, %d)",
                  target_col, target_row);

      // First move to above
      success = execute_approach(goal_handle, feedback, target_center_pose);
      if (!success) break;

      // Then descend to target position
      success = execute_pick_place(goal_handle, feedback, target_center_pose);
      break;
    }

    case MoveAction::Goal::PHASE_RETREAT:
      RCLCPP_INFO(this->get_logger(), "PHASE_RETREAT: Moving upward");
      success = execute_retreat(goal_handle, feedback);
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown phase: %d", phase);
      success = false;
  }

  result->success = success;
  if (success) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Phase %d completed", phase);
  } else {
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Phase %d failed", phase);
  }
}

bool ArmManipulator::execute_approach(
    const std::shared_ptr<GoalHandleMove> goal_handle,
    std::shared_ptr<MoveAction::Feedback> feedback,
    geometry_msgs::msg::PoseStamped target_pose) {
  RCLCPP_INFO(this->get_logger(), "Moving to above target (APPROACH height)");

  feedback->progress = 0.1;
  goal_handle->publish_feedback(feedback);

  target_pose.pose.position.z = approach_height_;

  RCLCPP_INFO(this->get_logger(), "Target: (%.3f, %.3f, %.3f)",
              target_pose.pose.position.x, target_pose.pose.position.y,
              target_pose.pose.position.z);

  // Check if target is reachable
  auto current_pose = move_group_interface_->getCurrentPose();
  double distance =
      sqrt(pow(target_pose.pose.position.x - current_pose.pose.position.x, 2) +
           pow(target_pose.pose.position.y - current_pose.pose.position.y, 2) +
           pow(target_pose.pose.position.z - current_pose.pose.position.z, 2));
  RCLCPP_INFO(this->get_logger(), "Distance to target: %.3fm", distance);

  feedback->progress = 0.3;
  goal_handle->publish_feedback(feedback);

  feedback->progress = 0.5;
  goal_handle->publish_feedback(feedback);

  // Try Cartesian path first for smoother motion
  bool success = planAndExecuteCartesianPath(target_pose, false);

  if (!success) {
    RCLCPP_WARN(this->get_logger(),
                "Cartesian planning failed, trying regular planning");
    success = planAndExecuteSmoothedMotion(target_pose, false);
  }

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "All planning attempts failed");
    RCLCPP_ERROR(this->get_logger(), "Current robot pose: (%.3f, %.3f, %.3f)",
                 move_group_interface_->getCurrentPose().pose.position.x,
                 move_group_interface_->getCurrentPose().pose.position.y,
                 move_group_interface_->getCurrentPose().pose.position.z);
  }

  feedback->progress = 1.0;
  goal_handle->publish_feedback(feedback);

  return success;
}

bool ArmManipulator::execute_pick_place(
    const std::shared_ptr<GoalHandleMove> goal_handle,
    std::shared_ptr<MoveAction::Feedback> feedback,
    geometry_msgs::msg::PoseStamped target_pose) {
  RCLCPP_INFO(this->get_logger(), "Descending to grip height (GRIP height)");

  feedback->progress = 0.1;
  goal_handle->publish_feedback(feedback);

  target_pose.pose.position.z = grip_height_;

  RCLCPP_INFO(this->get_logger(), "Target: (%.3f, %.3f, %.3f)",
              target_pose.pose.position.x, target_pose.pose.position.y,
              target_pose.pose.position.z);

  feedback->progress = 0.3;
  goal_handle->publish_feedback(feedback);

  // For vertical movements like pick/place, Cartesian path is ideal
  bool success = planAndExecuteCartesianPath(target_pose, true);

  if (!success) {
    RCLCPP_WARN(
        this->get_logger(),
        "Cartesian planning failed for pick/place, trying regular planning");
    success = planAndExecuteSmoothedMotion(target_pose, true);
  }

  feedback->progress = 0.6;
  goal_handle->publish_feedback(feedback);

  feedback->progress = 1.0;
  goal_handle->publish_feedback(feedback);

  return success;
}

bool ArmManipulator::execute_retreat(
    const std::shared_ptr<GoalHandleMove> goal_handle,
    std::shared_ptr<MoveAction::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Retreating upward (RETREAT height)");

  feedback->progress = 0.2;
  goal_handle->publish_feedback(feedback);

  // Get current x, y position, but set new z height
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = move_group_interface_->getPlanningFrame();
  current_pose.pose = move_group_interface_->getCurrentPose().pose;

  current_pose.pose.position.z = retreat_height_;

  RCLCPP_INFO(this->get_logger(), "Target: (%.3f, %.3f, %.3f)",
              current_pose.pose.position.x, current_pose.pose.position.y,
              current_pose.pose.position.z);

  feedback->progress = 0.4;
  goal_handle->publish_feedback(feedback);

  // For retreat (vertical movement), Cartesian path is ideal
  bool success = planAndExecuteCartesianPath(current_pose, true);

  if (!success) {
    RCLCPP_WARN(
        this->get_logger(),
        "Cartesian planning failed for retreat, trying regular planning");
    success = planAndExecuteSmoothedMotion(current_pose, true);
  }

  feedback->progress = 0.7;
  goal_handle->publish_feedback(feedback);

  feedback->progress = 1.0;
  goal_handle->publish_feedback(feedback);

  return success;
}

geometry_msgs::msg::PoseStamped ArmManipulator::calculatePieceCenterPose(
    const std::vector<klotski_interfaces::msg::Cell>& cells) {
  if (cells.empty()) {
    RCLCPP_ERROR(this->get_logger(), "cells is empty!");
    return geometry_msgs::msg::PoseStamped();
  }

  // Calculate center of all cells
  double sum_col = 0.0;
  double sum_row = 0.0;

  for (const auto& cell : cells) {
    sum_col += cell.col;
    sum_row += cell.row;
  }

  double center_col = sum_col / cells.size();
  double center_row = sum_row / cells.size();

  RCLCPP_INFO(this->get_logger(),
              "Piece occupies %zu cells, center position: (%.2f, %.2f)",
              cells.size(), center_col, center_row);

  return calculateWorldPose(center_col, center_row);
}

// geometry_msgs::msg::PoseStamped ArmManipulator::calculateWorldPose(double
// col,
//                                                                    double
//                                                                    row) {
//   geometry_msgs::msg::PoseStamped pose;
//   pose.header.frame_id = "base_link";
//   pose.header.stamp = this->now();

//   // Calculate bottom-left corner position from board center
//   double grid_origin_x = board_center_x_ - board_width_ / 2.0;
//   double grid_origin_y = board_center_y_ - board_length_ / 2.0;

//   // Calculate cell position
//   pose.pose.position.x = grid_origin_x + col * cell_size_;
//   pose.pose.position.y = grid_origin_y + row * cell_size_;
//   pose.pose.position.z = board_height_;  // Use configured board height

//   // Set end effector facing down
//   tf2::Quaternion q;
//   q.setRPY(0.0, M_PI, 0.0);  // Roll = 180°
//   pose.pose.orientation = tf2::toMsg(q);

//   return pose;
// }

// geometry_msgs::msg::PoseStamped ArmManipulator::calculateWorldPose(double
// col,
//                                                                    double
//                                                                    row) {
//   geometry_msgs::msg::PoseStamped pose;
//   pose.header.frame_id = "base_link";
//   pose.header.stamp = this->now();

//   // 假设棋盘是4列x5行，编号从0开始
//   double local_x = (col - 1.5) * cell_size_;  // 列方向，中心在1.5
//   double local_y = (2.0 - row) * cell_size_;  // 行方向，中心在2.0

//   double rotated_x, rotated_y;
//   if (use_dynamic_board_pose_) {
//     double cos_theta = std::cos(board_rotation_yaw_);
//     double sin_theta = std::sin(board_rotation_yaw_);

//     rotated_x = local_x * cos_theta - local_y * sin_theta;
//     rotated_y = local_x * sin_theta + local_y * cos_theta;
//   } else {
//     rotated_x = local_x;
//     rotated_y = local_y;
//   }

//   pose.pose.position.x = board_center_x_ + rotated_x;
//   pose.pose.position.y = board_center_y_ + rotated_y;
//   pose.pose.position.z = 0.0;

//   tf2::Quaternion q;
//   if (use_dynamic_board_pose_) {
//     q.setRPY(M_PI, 0, board_rotation_yaw_);
//   } else {
//     q.setRPY(M_PI, 0, 0);
//   }

//   pose.pose.orientation.x = q.x();
//   pose.pose.orientation.y = q.y();
//   pose.pose.orientation.z = q.z();
//   pose.pose.orientation.w = q.w();

//   return pose;
// }

geometry_msgs::msg::PoseStamped ArmManipulator::calculateWorldPose(double col,
                                                                   double row) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.header.stamp = this->now();

  // 计算局部坐标
  double local_x = (col - 1.5) * cell_size_;
  double local_y = (2.0 - row) * cell_size_;

  // 应用旋转
  double rotated_x, rotated_y;
  if (use_dynamic_board_pose_) {
    double cos_theta = std::cos(board_rotation_yaw_);
    double sin_theta = std::sin(board_rotation_yaw_);
    rotated_x = local_x * cos_theta - local_y * sin_theta;
    rotated_y = local_x * sin_theta + local_y * cos_theta;
  } else {
    rotated_x = local_x;
    rotated_y = local_y;
  }

  // 转换到base_link
  pose.pose.position.x = board_center_x_ + rotated_x;
  pose.pose.position.y = board_center_y_ + rotated_y;
  pose.pose.position.z = 0.0;

  // 设置姿态
  tf2::Quaternion q;
  if (use_dynamic_board_pose_) {
    q.setRPY(M_PI, 0, board_rotation_yaw_);
  } else {
    q.setRPY(M_PI, 0, 0);
  }

  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return pose;
}

moveit_msgs::msg::Constraints ArmManipulator::set_joint_constraints() {
  moveit_msgs::msg::Constraints constraints;

  // Helper function to add joint constraint
  auto addJointConstraint = [&](const std::string& joint_name, double min_deg,
                                double max_deg, double weight) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = joint_name;

    const double min_rad = min_deg * M_PI / 180.0;
    const double max_rad = max_deg * M_PI / 180.0;
    const double midpoint = (min_rad + max_rad) / 2.0;

    constraint.position = midpoint;
    constraint.tolerance_below = midpoint - min_rad;
    constraint.tolerance_above = max_rad - midpoint;
    constraint.weight = weight;

    constraints.joint_constraints.push_back(constraint);
  };

  // Apply configurable joint constraints to reduce search space
  addJointConstraint("elbow_joint", elbow_min_angle_, elbow_max_angle_,
                     elbow_constraint_weight_);
  addJointConstraint("shoulder_pan_joint", shoulder_pan_min_angle_,
                     shoulder_pan_max_angle_, joint_constraint_weight_);
  addJointConstraint("shoulder_lift_joint", shoulder_lift_min_angle_,
                     shoulder_lift_max_angle_, joint_constraint_weight_);
  addJointConstraint("wrist_1_joint", wrist_1_min_angle_, wrist_1_max_angle_,
                     joint_constraint_weight_);

  RCLCPP_INFO(this->get_logger(),
              "Applied %zu joint constraints to reduce planning search space",
              constraints.joint_constraints.size());

  return constraints;
}

moveit_msgs::msg::CollisionObject ArmManipulator::generateCollisionObject(
    float size_x, float size_y, float size_z, float center_x, float center_y,
    float center_z, const std::string& frame_id, const std::string& id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
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

bool ArmManipulator::planAndExecuteCartesianPath(
    const geometry_msgs::msg::PoseStamped& target_pose, bool use_constraints) {
  // Get current pose
  auto current_pose = move_group_interface_->getCurrentPose();

  // Create waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Add intermediate waypoint (optional for better path)
  geometry_msgs::msg::Pose intermediate_pose = current_pose.pose;
  intermediate_pose.position.z =
      std::max(current_pose.pose.position.z, target_pose.pose.position.z) +
      0.05;

  // Only add intermediate waypoint if we're moving significantly
  double distance =
      sqrt(pow(target_pose.pose.position.x - current_pose.pose.position.x, 2) +
           pow(target_pose.pose.position.y - current_pose.pose.position.y, 2));

  if (distance > 0.1) {  // If moving more than 10cm horizontally
    waypoints.push_back(intermediate_pose);
  }

  // Add final target
  waypoints.push_back(target_pose.pose);

  // Set constraints if requested
  if (use_constraints) {
    auto constraints = set_joint_constraints();
    move_group_interface_->setPathConstraints(constraints);
  }

  // Plan Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group_interface_->computeCartesianPath(
      waypoints, cartesian_eef_step_, cartesian_jump_threshold_, trajectory);

  RCLCPP_INFO(this->get_logger(),
              "Cartesian path planned: %.2f%% of path achieved",
              fraction * 100.0);

  bool success = false;
  if (fraction > cartesian_fraction_threshold_) {  // Use configurable threshold
    // Execute the Cartesian trajectory
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    success = (move_group_interface_->execute(cartesian_plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Cartesian path planning failed (%.2f%%), falling back to "
                "regular planning",
                fraction * 100.0);

    // Fall back to regular motion planning
    success = planAndExecuteSmoothedMotion(target_pose, use_constraints);
  }

  // Clear constraints
  if (use_constraints) {
    move_group_interface_->clearPathConstraints();
  }

  return success;
}

bool ArmManipulator::planAndExecuteSmoothedMotion(
    const geometry_msgs::msg::PoseStamped& target_pose, bool use_constraints) {
  // Set constraints if requested
  if (use_constraints) {
    auto constraints = set_joint_constraints();
    move_group_interface_->setPathConstraints(constraints);
  }

  move_group_interface_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
    if (use_constraints) {
      move_group_interface_->clearPathConstraints();
    }
    return false;
  }

  // Apply trajectory smoothing and time parameterization if enabled
  if (enable_trajectory_smoothing_) {
    robot_trajectory::RobotTrajectory rt(move_group_interface_->getRobotModel(),
                                         move_group_interface_->getName());
    rt.setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(),
                             plan.trajectory_);

    // Apply smoothing
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool time_param_success = iptp.computeTimeStamps(
        rt, max_velocity_scaling_factor_, max_acceleration_scaling_factor_);

    if (time_param_success) {
      rt.getRobotTrajectoryMsg(plan.trajectory_);
      RCLCPP_INFO(this->get_logger(), "Applied trajectory smoothing");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Trajectory smoothing failed, using original trajectory");
    }
  }
  success = (move_group_interface_->execute(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

  // Clear constraints
  if (use_constraints) {
    move_group_interface_->clearPathConstraints();
  }

  return success;
}

// void ArmManipulator::board_pose_callback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//   // 更新棋盘中心位置
//   board_center_x_ = msg->pose.position.x;
//   board_center_y_ = msg->pose.position.y;

//   // 提取yaw角度（绕z轴的旋转）
//   tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
//                     msg->pose.orientation.z, msg->pose.orientation.w);

//   tf2::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   board_rotation_yaw_ = yaw;

//   RCLCPP_INFO(this->get_logger(), "Board updated: pos=(%.3f, %.3f),
//   yaw=%.1f°",
//               board_center_x_, board_center_y_, yaw * 180.0 / M_PI);
// }
void ArmManipulator::board_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  board_center_x_ = msg->pose.position.x;
  board_center_y_ = msg->pose.position.y;

  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                    msg->pose.orientation.z, msg->pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  board_rotation_yaw_ = yaw;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Board: pos=(%.3f,%.3f), yaw=%.1f°", board_center_x_,
                       board_center_y_, yaw * 180.0 / M_PI);
}

}  // namespace pkg_manipulation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pkg_manipulation::ArmManipulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
