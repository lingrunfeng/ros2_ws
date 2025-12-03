/**
 * @file mtc_grasp_pose_node.cpp
 * @brief MTC node that grasps an object at a received pose.
 *
 * This node subscribes to /object_pose, adds a collision object at that pose,
 * and then executes a pick and place task using MoveIt Task Constructor.
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;

class MTCGraspPoseNode : public rclcpp::Node
{
public:
  MTCGraspPoseNode(const rclcpp::NodeOptions& options);

private:
  // Subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_sub_;
  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // MTC Task
  mtc::Task task_;
  mtc::Task createTask();
  void doTask();
  bool executeSolutionWithMoveGroup();

  // Helper
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped& pose);

  // State
  geometry_msgs::msg::PoseStamped target_pose_;
  bool has_pose_ = false;
  std::string object_name_ = "target_object";
};

MTCGraspPoseNode::MTCGraspPoseNode(const rclcpp::NodeOptions& options)
  : Node("mtc_grasp_pose_node", options)
{
  // Declare parameters
  this->declare_parameter("execute", true);
  this->declare_parameter("object_type", "cylinder"); // "box" or "cylinder"
  this->declare_parameter("object_dimensions", std::vector<double>{0.1, 0.0125}); // [height, radius] or [x, y, z]
  
  // Robot configuration parameters
  this->declare_parameter("arm_group_name", "arm");
  this->declare_parameter("gripper_group_name", "gripper");
  this->declare_parameter("gripper_frame", "link6_flange");
  this->declare_parameter("arm_home_pose", "home");
  this->declare_parameter("gripper_open_pose", "open");
  this->declare_parameter("gripper_close_pose", "half_closed");
  this->declare_parameter("world_frame", "base_link");

  // Grasp/Place parameters
  this->declare_parameter("grasp_frame_transform", std::vector<double>{0.0, 0.0, 0.096, 1.5708, 0.0, 0.0});
  this->declare_parameter("place_pose", std::vector<double>{-0.183, -0.14, 0.0, 0.0, 0.0, 0.0});
  
  // Motion parameters (defaults)
  this->declare_parameter("approach_object_min_dist", 0.0015);
  this->declare_parameter("approach_object_max_dist", 0.3);
  this->declare_parameter("lift_object_min_dist", 0.005);
  this->declare_parameter("lift_object_max_dist", 0.3);
  this->declare_parameter("lower_object_min_dist", 0.005);
  this->declare_parameter("lower_object_max_dist", 0.4);
  this->declare_parameter("retreat_min_distance", 0.025);
  this->declare_parameter("retreat_max_distance", 0.25);
  
  // Create subscription
  object_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/object_pose", 10,
    std::bind(&MTCGraspPoseNode::objectPoseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Waiting for object pose on /object_pose topic...");
}

void MTCGraspPoseNode::objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (has_pose_) {
    RCLCPP_INFO(this->get_logger(), "Already received a pose, ignoring new one for now.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received object pose: [%.3f, %.3f, %.3f]",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  target_pose_ = *msg;
  has_pose_ = true;

  // Run task in a separate thread to not block the callback
  std::thread([this]() {
    this->setupPlanningScene(target_pose_);
    // Wait for planning scene to synchronize (applyCollisionObjects is async)
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    this->doTask();
    // Reset for next run if needed, or just exit
    // has_pose_ = false; 
  }).detach();
}

void MTCGraspPoseNode::setupPlanningScene(const geometry_msgs::msg::PoseStamped& pose)
{
  moveit::planning_interface::PlanningSceneInterface psi;
  
  // Remove existing object if any
  psi.removeCollisionObjects({object_name_});

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = pose.header.frame_id;
  collision_object.id = object_name_;

  std::string type = this->get_parameter("object_type").as_string();
  std::vector<double> dims = this->get_parameter("object_dimensions").as_double_array();

  shape_msgs::msg::SolidPrimitive primitive;
  if (type == "cylinder") {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    if (dims.size() >= 2) {
      primitive.dimensions.resize(2);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = dims[0];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = dims[1];
    }
  } else if (type == "box") {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    if (dims.size() >= 3) {
      primitive.dimensions.resize(3);
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = dims[0];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = dims[1];
      primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = dims[2];
    }
  }

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose.pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  psi.applyCollisionObjects({collision_object});
  RCLCPP_INFO(this->get_logger(), "Added collision object '%s' to planning scene", object_name_.c_str());
}

// Helper to convert vector to Eigen Isometry
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
}

mtc::Task MTCGraspPoseNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("grasp_pose_task");
  task.loadRobotModel(shared_from_this(), "robot_description");

  // Get parameters
  auto arm_group_name = this->get_parameter("arm_group_name").as_string();
  auto gripper_group_name = this->get_parameter("gripper_group_name").as_string();
  auto gripper_frame = this->get_parameter("gripper_frame").as_string();
  auto world_frame = this->get_parameter("world_frame").as_string();
  auto grasp_frame_transform = this->get_parameter("grasp_frame_transform").as_double_array();

  // Planners
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  std::unordered_map<std::string, std::string> ompl_map_arm = {
    {"ompl", arm_group_name + "[RRTConnectkConfigDefault]"}
  };
  auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), ompl_map_arm);
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", gripper_group_name);
  task.setProperty("ik_frame", gripper_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto stage_open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  stage_open_gripper->setGroup(gripper_group_name);
  stage_open_gripper->setGoal(this->get_parameter("gripper_open_pose").as_string());
  task.add(std::move(stage_open_gripper));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group_name, ompl_planner}});
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  // Pick Object Serial Container
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", gripper_frame);
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names({"arm_controller", "gripper_action_controller"}));
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(this->get_parameter("approach_object_min_dist").as_double(),
                               this->get_parameter("approach_object_max_dist").as_double());
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = gripper_frame;
      vec.vector.z = 1.0; // Approach direction matches original mtc_node.cpp
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // Generate Grasp Pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(this->get_parameter("gripper_open_pose").as_string());
      stage->setObject(object_name_);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(20);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // Allow Collision (gripper, object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions(object_name_,
        task.getRobotModel()->getJointModelGroup(gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    // Close Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal(this->get_parameter("gripper_close_pose").as_string());
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names({"arm_controller", "gripper_action_controller"}));
      grasp->insert(std::move(stage));
    }

    // Attach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name_, gripper_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Lift Object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(this->get_parameter("lift_object_min_dist").as_double(),
                               this->get_parameter("lift_object_max_dist").as_double());
      stage->setIKFrame(gripper_frame);
      stage->properties().set("marker_ns", "lift_object");
      stage->properties().set("trajectory_execution_info",
                      mtc::TrajectoryExecutionInfo().set__controller_names({"arm_controller", "gripper_action_controller"}));
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = 1.0; // Lift up (world z)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // Move to Place (simplified for now, just move to a pose)
  // You can add Place logic here if needed, similar to the original node
  
  return task;
}

void MTCGraspPoseNode::doTask()
{
  task_ = createTask();

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR(this->get_logger(), "Task init failed: %s", e.what());
    return;
  }

  if (!task_.plan(10)) {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task planning succeeded");
  task_.introspection().publishSolution(*task_.solutions().front());

  if (this->get_parameter("execute").as_bool()) {
    executeSolutionWithMoveGroup();
  }
}

bool MTCGraspPoseNode::executeSolutionWithMoveGroup()
{
  if (task_.solutions().empty()) return false;
  
  const auto& solution = *task_.solutions().front();
  moveit_task_constructor_msgs::msg::Solution solution_msg;
  solution.toMsg(solution_msg);

  auto arm_group_name = this->get_parameter("arm_group_name").as_string();
  auto gripper_group_name = this->get_parameter("gripper_group_name").as_string();

  try {
    auto arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), arm_group_name);
    auto gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), gripper_group_name);

    size_t stage_num = 0;
    for (const auto& sub_traj_msg : solution_msg.sub_trajectory) {
      const auto& trajectory_msg = sub_traj_msg.trajectory;
      
      if (trajectory_msg.joint_trajectory.points.empty() && 
          trajectory_msg.multi_dof_joint_trajectory.points.empty()) {
        continue;
      }

      std::string stage_info = "Stage " + std::to_string(stage_num++);
      RCLCPP_INFO(this->get_logger(), "Executing %s", stage_info.c_str());

      // Determine which group to use
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> current_group;
      std::string group_name;

      if (!trajectory_msg.joint_trajectory.joint_names.empty()) {
        const std::string& first_joint = trajectory_msg.joint_trajectory.joint_names[0];
        const auto& arm_joints = arm_move_group->getJointNames();
        
        bool is_arm = false;
        for (const auto& j : arm_joints) {
          if (j == first_joint) {
            is_arm = true;
            break;
          }
        }

        if (is_arm) {
          current_group = arm_move_group;
          group_name = arm_group_name;
        } else {
          current_group = gripper_move_group;
          group_name = gripper_group_name;
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Trajectory has no joints, skipping");
        continue;
      }

      // Create plan with modified trajectory
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto modified_trajectory = trajectory_msg;

      // CRITICAL: Update first point to match current robot state
      if (!modified_trajectory.joint_trajectory.points.empty()) {
        std::vector<double> current_joint_values;
        const auto& joint_names = modified_trajectory.joint_trajectory.joint_names;
        
        auto current_state = current_group->getCurrentState();
        bool update_success = true;
        
        for (const auto& name : joint_names) {
          const double* pos = current_state->getJointPositions(name);
          if (pos) {
            current_joint_values.push_back(*pos);
          } else {
            RCLCPP_WARN(this->get_logger(), "Could not get current position for joint %s", name.c_str());
            update_success = false;
            break;
          }
        }

        if (update_success && current_joint_values.size() == modified_trajectory.joint_trajectory.points[0].positions.size()) {
          RCLCPP_INFO(this->get_logger(), "Updating trajectory start point to match current state");
          modified_trajectory.joint_trajectory.points[0].positions = current_joint_values;
          modified_trajectory.joint_trajectory.points[0].velocities.clear();
          modified_trajectory.joint_trajectory.points[0].accelerations.clear();
          modified_trajectory.joint_trajectory.points[0].effort.clear();
        }
      }

      plan.trajectory = modified_trajectory;

      // Execute
      auto result = current_group->execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Execution failed for %s (error code: %d)", stage_info.c_str(), result.val);
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "Successfully executed %s", stage_info.c_str());
      
      // Delay between stages
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(this->get_logger(), "All stages executed successfully");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during execution: %s", e.what());
    return false;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // options.automatically_declare_parameters_from_overrides(true); // Removed to avoid double declaration
  auto node = std::make_shared<MTCGraspPoseNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
