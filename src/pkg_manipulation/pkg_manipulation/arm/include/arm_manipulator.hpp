// #ifndef ARM_MANIPULATOR_HPP_
// #define ARM_MANIPULATOR_HPP_

// #include <tf2/LinearMath/Quaternion.h>

// #include <chrono>
// #include <memory>
// #include <thread>
// #include <vector>

// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "klotski_interfaces/action/move_piece.hpp"
// #include "klotski_interfaces/msg/cell.hpp"
// #include "klotski_interfaces/msg/piece.hpp"
// #include "moveit/move_group_interface/move_group_interface.h"
// #include "moveit/planning_scene_interface/planning_scene_interface.h"
// #include "moveit/robot_trajectory/robot_trajectory.h"
// #include "moveit/trajectory_processing/iterative_time_parameterization.h"
// #include "moveit_msgs/msg/collision_object.hpp"
// #include "moveit_msgs/msg/constraints.hpp"
// #include "moveit_msgs/msg/joint_constraint.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "shape_msgs/msg/solid_primitive.hpp"

// namespace pkg_manipulation {

// /**
//  * @brief ArmManipulator - Controls the UR5e arm using MoveIt for Klotski
//  piece
//  * manipulation
//  *
//  * This class implements an action server that handles moving game pieces on
//  a
//  * Klotski board. It uses MoveIt for motion planning and execution, with
//  support
//  * for approach, pick/place, and retreat phases.
//  */
// class ArmManipulator : public rclcpp::Node {
//  public:
//   using MoveAction = klotski_interfaces::action::MovePiece;
//   using Piece = klotski_interfaces::msg::Piece;
//   using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveAction>;

//   /**
//    * @brief Constructor - Initializes the arm controller node
//    */
//   ArmManipulator();

//   /**
//    * @brief Destructor
//    */
//   ~ArmManipulator() = default;

//  private:
//   // Height parameters
//   double board_height_;
//   double block_height_;
//   double gripper_height_;
//   double approach_offset_;
//   double retreat_offset_;

//   // Calculated heights
//   double approach_height_;
//   double grip_height_;
//   double retreat_height_;

//   // MoveIt planning parameters
//   double planning_time_;
//   int num_planning_attempts_;
//   double max_velocity_scaling_factor_;
//   double max_acceleration_scaling_factor_;
//   std::string planner_id_;
//   std::string planning_group_;
//   std::string robot_ip_;

//   // Board geometry parameters
//   double board_center_x_;
//   double board_center_y_;
//   double board_width_;
//   double board_length_;
//   double cell_size_;

//   // Joint constraint parameters
//   double elbow_min_angle_;
//   double elbow_max_angle_;
//   double elbow_constraint_weight_;
//   double shoulder_pan_min_angle_;
//   double shoulder_pan_max_angle_;
//   double shoulder_lift_min_angle_;
//   double shoulder_lift_max_angle_;
//   double wrist_1_min_angle_;
//   double wrist_1_max_angle_;
//   double joint_constraint_weight_;

//   // Cartesian path planning parameters
//   double cartesian_eef_step_;
//   double cartesian_jump_threshold_;
//   double cartesian_fraction_threshold_;

//   // Trajectory smoothing parameters
//   bool enable_trajectory_smoothing_;
//   int replan_attempts_;

//   rclcpp_action::Server<MoveAction>::SharedPtr action_server_;
//   std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
//       move_group_interface_;

//   /**
//    * @brief Handles incoming goal requests
//    * @param uuid Goal UUID
//    * @param goal Goal request
//    * @return GoalResponse indicating acceptance or rejection
//    */
//   rclcpp_action::GoalResponse handle_goal(
//       const rclcpp_action::GoalUUID& uuid,
//       std::shared_ptr<const MoveAction::Goal> goal);

//   /**
//    * @brief Handles cancel requests
//    * @param goal_handle Goal handle
//    * @return CancelResponse
//    */
//   rclcpp_action::CancelResponse handle_cancel(
//       const std::shared_ptr<GoalHandleMove> goal_handle);

//   /**
//    * @brief Called when a goal is accepted
//    * @param goal_handle Goal handle
//    */
//   void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

//   /**
//    * @brief Main execution function for handling movement phases
//    * @param goal_handle Goal handle
//    */
//   void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

//   /**
//    * @brief Executes approach phase - moves to above target position
//    * @param goal_handle Goal handle
//    * @param feedback Feedback message
//    * @param target_pose Target pose
//    * @return true if successful, false otherwise
//    */
//   bool execute_approach(const std::shared_ptr<GoalHandleMove> goal_handle,
//                         std::shared_ptr<MoveAction::Feedback> feedback,
//                         geometry_msgs::msg::PoseStamped target_pose);

//   /**
//    * @brief Executes pick/place phase - moves to grip height
//    * @param goal_handle Goal handle
//    * @param feedback Feedback message
//    * @param target_pose Target pose
//    * @return true if successful, false otherwise
//    */
//   bool execute_pick_place(const std::shared_ptr<GoalHandleMove> goal_handle,
//                           std::shared_ptr<MoveAction::Feedback> feedback,
//                           geometry_msgs::msg::PoseStamped target_pose);

//   /**
//    * @brief Executes retreat phase - moves upward
//    * @param goal_handle Goal handle
//    * @param feedback Feedback message
//    * @return true if successful, false otherwise
//    */
//   bool execute_retreat(const std::shared_ptr<GoalHandleMove> goal_handle,
//                        std::shared_ptr<MoveAction::Feedback> feedback);

//   /**
//    * @brief Declares and loads all ROS parameters from config file
//    */
//   void declareAndLoadParameters();

//   /**
//    * @brief Calculates derived heights from base parameters
//    */
//   void calculateHeights();

//   /**
//    * @brief Calculates the center pose of a game piece from its cells
//    * @param cells Vector of cells occupied by the piece
//    * @return PoseStamped at the center of the piece
//    */
//   geometry_msgs::msg::PoseStamped calculatePieceCenterPose(
//       const std::vector<klotski_interfaces::msg::Cell>& cells);

//   /**
//    * @brief Converts grid coordinates to world frame pose
//    * @param col Column position (can be fractional for center calculations)
//    * @param row Row position (can be fractional for center calculations)
//    * @return PoseStamped in world frame
//    */
//   geometry_msgs::msg::PoseStamped calculateWorldPose(double col, double row);

//   /**
//    * @brief Sets joint constraints for the elbow joint
//    * @return Constraints message with elbow angle constraints
//    */
//   moveit_msgs::msg::Constraints set_joint_constraints();

//   /**
//    * @brief Generates a collision object for the planning scene
//    * @param size_x Size in x direction
//    * @param size_y Size in y direction
//    * @param size_z Size in z direction
//    * @param center_x Center position x
//    * @param center_y Center position y
//    * @param center_z Center position z
//    * @param frame_id Reference frame
//    * @param id Collision object identifier
//    * @return CollisionObject message
//    */
//   moveit_msgs::msg::CollisionObject generateCollisionObject(
//       float size_x, float size_y, float size_z, float center_x, float
//       center_y, float center_z, const std::string& frame_id, const
//       std::string& id);

//   /**
//    * @brief Plans and executes a Cartesian path for smoother motion
//    * @param target_pose Target pose to reach
//    * @param use_constraints Whether to apply joint constraints
//    * @return true if successful, false otherwise
//    */
//   bool planAndExecuteCartesianPath(
//       const geometry_msgs::msg::PoseStamped& target_pose,
//       bool use_constraints = false);

//   /**
//    * @brief Plans and executes motion with enhanced trajectory smoothing
//    * @param target_pose Target pose to reach
//    * @param use_constraints Whether to apply joint constraints
//    * @return true if successful, false otherwise
//    */
//   bool planAndExecuteSmoothedMotion(
//       const geometry_msgs::msg::PoseStamped& target_pose,
//       bool use_constraints = false);
// };

// }  // namespace pkg_manipulation

// #endif  // ARM_MANIPULATOR_HPP_

#ifndef ARM_MANIPULATOR_HPP_
#define ARM_MANIPULATOR_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "klotski_interfaces/action/move_piece.hpp"
#include "klotski_interfaces/msg/cell.hpp"
#include "klotski_interfaces/msg/piece.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace pkg_manipulation {

/**
 * @brief ArmManipulator - Controls the UR5e arm using MoveIt for Klotski piece
 * manipulation
 *
 * This class implements an action server that handles moving game pieces on a
 * Klotski board. It uses MoveIt for motion planning and execution, with support
 * for approach, pick/place, and retreat phases.
 *
 * Now supports dynamic board position updates from camera vision system.
 */
class ArmManipulator : public rclcpp::Node {
 public:
  using MoveAction = klotski_interfaces::action::MovePiece;
  using Piece = klotski_interfaces::msg::Piece;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveAction>;

  /**
   * @brief Constructor - Initializes the arm controller node
   */
  ArmManipulator();

  /**
   * @brief Destructor
   */
  ~ArmManipulator() = default;

 private:
  // Height parameters
  double board_height_;
  double block_height_;
  double gripper_height_;
  double approach_offset_;
  double retreat_offset_;

  // Calculated heights
  double approach_height_;
  double grip_height_;
  double retreat_height_;

  // MoveIt planning parameters
  double planning_time_;
  int num_planning_attempts_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  std::string planner_id_;
  std::string planning_group_;
  std::string robot_ip_;

  // Board geometry parameters
  double board_center_x_;
  double board_center_y_;
  double board_width_;
  double board_length_;
  double cell_size_;

  // Dynamic board position flag
  bool use_dynamic_board_position_;
  bool board_position_received_;

  // Joint constraint parameters
  double elbow_min_angle_;
  double elbow_max_angle_;
  double elbow_constraint_weight_;
  double shoulder_pan_min_angle_;
  double shoulder_pan_max_angle_;
  double shoulder_lift_min_angle_;
  double shoulder_lift_max_angle_;
  double wrist_1_min_angle_;
  double wrist_1_max_angle_;
  double joint_constraint_weight_;

  // Cartesian path planning parameters
  double cartesian_eef_step_;
  double cartesian_jump_threshold_;
  double cartesian_fraction_threshold_;

  // Trajectory smoothing parameters
  bool enable_trajectory_smoothing_;
  int replan_attempts_;

  rclcpp_action::Server<MoveAction>::SharedPtr action_server_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
      move_group_interface_;

  // TF2 for coordinate transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriber for board position from camera
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      board_position_sub_;

  /**
   * @brief Callback for receiving board center position from camera vision
   * @param msg PointStamped message containing board center in camera frame
   */
  void boardPositionCallback(
      const geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
   * @brief Transforms a point from source frame to target frame using TF2
   * @param point_in Input point in source frame
   * @param target_frame Target reference frame
   * @return Transformed point in target frame, or empty if transform fails
   */
  geometry_msgs::msg::PointStamped transformPoint(
      const geometry_msgs::msg::PointStamped& point_in,
      const std::string& target_frame);

  /**
   * @brief Handles incoming goal requests
   * @param uuid Goal UUID
   * @param goal Goal request
   * @return GoalResponse indicating acceptance or rejection
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveAction::Goal> goal);

  /**
   * @brief Handles cancel requests
   * @param goal_handle Goal handle
   * @return CancelResponse
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMove> goal_handle);

  /**
   * @brief Called when a goal is accepted
   * @param goal_handle Goal handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);

  /**
   * @brief Main execution function for handling movement phases
   * @param goal_handle Goal handle
   */
  void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

  /**
   * @brief Executes approach phase - moves to above target position
   * @param goal_handle Goal handle
   * @param feedback Feedback message
   * @param target_pose Target pose
   * @return true if successful, false otherwise
   */
  bool execute_approach(const std::shared_ptr<GoalHandleMove> goal_handle,
                        std::shared_ptr<MoveAction::Feedback> feedback,
                        geometry_msgs::msg::PoseStamped target_pose);

  /**
   * @brief Executes pick/place phase - moves to grip height
   * @param goal_handle Goal handle
   * @param feedback Feedback message
   * @param target_pose Target pose
   * @return true if successful, false otherwise
   */
  bool execute_pick_place(const std::shared_ptr<GoalHandleMove> goal_handle,
                          std::shared_ptr<MoveAction::Feedback> feedback,
                          geometry_msgs::msg::PoseStamped target_pose);

  /**
   * @brief Executes retreat phase - moves upward
   * @param goal_handle Goal handle
   * @param feedback Feedback message
   * @return true if successful, false otherwise
   */
  bool execute_retreat(const std::shared_ptr<GoalHandleMove> goal_handle,
                       std::shared_ptr<MoveAction::Feedback> feedback);

  /**
   * @brief Declares and loads all ROS parameters from config file
   */
  void declareAndLoadParameters();

  /**
   * @brief Calculates derived heights from base parameters
   */
  void calculateHeights();

  /**
   * @brief Calculates the center pose of a game piece from its cells
   * @param cells Vector of cells occupied by the piece
   * @return PoseStamped at the center of the piece
   */
  geometry_msgs::msg::PoseStamped calculatePieceCenterPose(
      const std::vector<klotski_interfaces::msg::Cell>& cells);

  /**
   * @brief Converts grid coordinates to world frame pose
   * @param col Column position (can be fractional for center calculations)
   * @param row Row position (can be fractional for center calculations)
   * @return PoseStamped in world frame
   */
  geometry_msgs::msg::PoseStamped calculateWorldPose(double col, double row);

  /**
   * @brief Sets joint constraints for the elbow joint
   * @return Constraints message with elbow angle constraints
   */
  moveit_msgs::msg::Constraints set_joint_constraints();

  /**
   * @brief Generates a collision object for the planning scene
   * @param size_x Size in x direction
   * @param size_y Size in y direction
   * @param size_z Size in z direction
   * @param center_x Center position x
   * @param center_y Center position y
   * @param center_z Center position z
   * @param frame_id Reference frame
   * @param id Collision object identifier
   * @return CollisionObject message
   */
  moveit_msgs::msg::CollisionObject generateCollisionObject(
      float size_x, float size_y, float size_z, float center_x, float center_y,
      float center_z, const std::string& frame_id, const std::string& id);

  /**
   * @brief Plans and executes a Cartesian path for smoother motion
   * @param target_pose Target pose to reach
   * @param use_constraints Whether to apply joint constraints
   * @return true if successful, false otherwise
   */
  bool planAndExecuteCartesianPath(
      const geometry_msgs::msg::PoseStamped& target_pose,
      bool use_constraints = false);

  /**
   * @brief Plans and executes motion with enhanced trajectory smoothing
   * @param target_pose Target pose to reach
   * @param use_constraints Whether to apply joint constraints
   * @return true if successful, false otherwise
   */
  bool planAndExecuteSmoothedMotion(
      const geometry_msgs::msg::PoseStamped& target_pose,
      bool use_constraints = false);
};

}  // namespace pkg_manipulation

#endif  // ARM_MANIPULATOR_HPP_
