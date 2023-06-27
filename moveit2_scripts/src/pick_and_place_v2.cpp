#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
using namespace std::chrono_literals;
#define OFFSET 0.07
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickPlace : public rclcpp::Node {
public:
  using FindGraspableObjects = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFindGraspableObjects =
      rclcpp_action::ClientGoalHandle<FindGraspableObjects>;

  PickPlace(std::shared_ptr<rclcpp::Node> move_group_node)
      : Node("pick_place"), move_group_arm(move_group_node, PLANNING_GROUP_ARM),
        joint_model_group_arm(
            move_group_arm.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_ARM)),
        move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER),
        joint_model_group_gripper(
            move_group_gripper.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_GRIPPER)) {

    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PickPlace::timer_callback, this), callback_group_);

    this->client_ptr_ = rclcpp_action::create_client<FindGraspableObjects>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_objects");

  } // end of constructor

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    // this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(LOGGER, "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = FindGraspableObjects::Goal();
    goal_msg.plan_grasps = false;
    RCLCPP_INFO(LOGGER, "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<FindGraspableObjects>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&PickPlace::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&PickPlace::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&PickPlace::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // Getting Basic Information
  void get_info() {

    RCLCPP_INFO(LOGGER, "Planning frame: %s",
                move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s",
                move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(),
              move_group_arm.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
  }

  void current_state() {
    RCLCPP_INFO(LOGGER, "Get Robot Current State");

    current_state_arm = move_group_arm.getCurrentState(10);
    current_state_gripper = move_group_gripper.getCurrentState(10);

    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);
    current_state_gripper->copyJointGroupPositions(
        this->joint_model_group_gripper, this->joint_group_positions_gripper);
  }

  void go_home() {

    RCLCPP_INFO(LOGGER, "Going Home");

    joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
    joint_group_positions_arm[1] = -1.57; // Shoulder Lift
    joint_group_positions_arm[2] = 0.00;  // Elbow
    joint_group_positions_arm[3] = -1.57; // Wrist 1
    joint_group_positions_arm[4] = 0.00;  // Wrist 2
    joint_group_positions_arm[5] = 0.00;  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
    }
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void pregrasp() {

    RCLCPP_INFO(LOGGER, "Pregrasp Position");

    target_pose1.orientation.x = -1.0;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 0.00;
    target_pose1.position.x = pose_x;
    target_pose1.position.y = pose_y;
    target_pose1.position.z = 0.06 + OFFSET + 0.2;
    move_group_arm.setPoseTarget(target_pose1);
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setPoseTarget(target_pose1);
    }
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void approach() {

    RCLCPP_INFO(LOGGER, "Approach to object");

    target_pose1.position.z = target_pose1.position.z - OFFSET - 0.03;
    move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setPoseTarget(target_pose1);
    }
    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void retreat() {

    RCLCPP_INFO(LOGGER, "Retreat from object");

    target_pose1.position.z = target_pose1.position.z + OFFSET;
    move_group_arm.setPoseTarget(target_pose1);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setPoseTarget(target_pose1);
    }
    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void rotate_arm() {

    RCLCPP_INFO(LOGGER, "Rotating Arm");

    joint_group_positions_arm[0] = 2.01;  // Shoulder Pan
    joint_group_positions_arm[1] = -1.03; // Shoulder lif
    joint_group_positions_arm[2] = 0.89;  // Elbow joint
    joint_group_positions_arm[3] = -1.43; // Wrist 1
    joint_group_positions_arm[4] = -1.57; // Wrist 2
    joint_group_positions_arm[5] = -1.18; // Wrist 3
    // joint_group_positions_arm[1] = -2.50;  // Shoulder Lift

    move_group_arm.setJointValueTarget(joint_group_positions_arm);
    while (move_group_arm.plan(my_plan_arm) !=
           moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
    }
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void open_gripper() {
    RCLCPP_INFO(LOGGER, "Open Gripper!");

    joint_group_positions_gripper[0] = 0.00;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_gripper.execute(my_plan_gripper);
  }

  void close_gripper() {
    RCLCPP_INFO(LOGGER, "Close Gripper!");

    joint_group_positions_gripper[0] = -0.65;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);

    bool success_gripper =
        (move_group_gripper.plan(my_plan_gripper) ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_gripper.execute(my_plan_gripper);
  }

  // Timer Callback function
  void timer_callback() {

    this->timer_->cancel();
    get_info();
    current_state();
    go_home();
    send_goal();
    while (!got_perception_data) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    current_state();
    pregrasp();
    current_state();
    open_gripper();
    current_state();
    approach();
    current_state();
    close_gripper();
    current_state();
    retreat();
    current_state();
    rotate_arm();
    /*rclcpp::Duration duration(5s);
    rclcpp::Time startTime = this->now();

    rclcpp::Rate r(1);
    while (this->now() - startTime < duration) {
      r.sleep();
      RCLCPP_INFO(LOGGER, "wait");
    }*/
    current_state();
    open_gripper();
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  geometry_msgs::msg::Pose target_pose1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  moveit::planning_interface::MoveGroupInterface move_group_arm;
  moveit::planning_interface::MoveGroupInterface move_group_gripper;

  const moveit::core::JointModelGroup *joint_model_group_arm;
  const moveit::core::JointModelGroup *joint_model_group_gripper;

  moveit::core::RobotStatePtr current_state_arm;
  moveit::core::RobotStatePtr current_state_gripper;

  float delta = 0.04;

  bool got_perception_data = false;
  bool goal_done_;
  float pose_x = 0.0;
  float pose_y = 0.0;
  float pose_z = 0.0;

  rclcpp_action::Client<FindGraspableObjects>::SharedPtr client_ptr_;

  void goal_response_callback(
      std::shared_future<GoalHandleFindGraspableObjects::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
    } else {
      RCLCPP_INFO(LOGGER,
                  "[PERCEPTION] Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleFindGraspableObjects::SharedPtr,
      const std::shared_ptr<const FindGraspableObjects::Feedback> feedback) {}

  void
  result_callback(const GoalHandleFindGraspableObjects::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(LOGGER, "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(LOGGER, "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(LOGGER, "Unknown result code");
      return;
    }
    for (auto object : result.result->objects) {
      if (object.object.primitives[0].type == 1 &&
          object.object.primitives[0].dimensions[0] < 0.1 &&
          object.object.primitives[0].dimensions[1] < 0.1 &&
          object.object.primitives[0].dimensions[2] < 0.1) {

        pose_x = object.object.primitive_poses[0].position.x;
        pose_y = object.object.primitive_poses[0].position.y;
        pose_z = object.object.primitive_poses[0].position.z;
      }
    }
    RCLCPP_INFO(LOGGER,
                "[PERCEPTION] Result received:\n[PERCEPTION] X: "
                "%f\n[PERCEPTION] Y: %f\n[PERCEPTION] Z: %f",
                pose_x, pose_y, pose_z);

    got_perception_data = true;
  }

}; // End of Class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_demo", node_options);

  rclcpp::executors::MultiThreadedExecutor planner_executor;
  std::shared_ptr<PickPlace> planner_node =
      std::make_shared<PickPlace>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}
