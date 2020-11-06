/*
 * Author: Nicola Castaman
 */

// ROS
#include <ros/ros.h>

#include <chrono>
#include <iostream>

#include <moveit_tmp/pick.h>
#include <moveit_tmp/place.h>
#include <moveit_tmp/task_planner.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_tmp/current_state.h>
#include <moveit_tmp/modify_planning_scene.h>
#include <moveit_tmp/move.h>
#include <moveit_tmp/place_pose_generator.h>
#include <moveit_tmp/planner.h>

#include <moveit_tmp/knowledge_base.h>

#include <moveit_tmp_msgs/ExecuteTMPSolutionAction.h>

#include <actionlib/client/simple_action_client.h>

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_tmp/action_cache.h>
#include <moveit_tmp/pick.h>
#include <moveit_tmp/place.h>
#include <moveit_tmp/task_planner.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef std::pair<moveit_tmp::Action, std::vector<double>> ActionSolution;

static const std::string LOGNAME = "sorting_navigation";

std::string planner_command_;
std::string domain_path_;
std::string problem_path_;
std::string data_path_;

int horizon_;

// Arm
const robot_model::JointModelGroup* arm_jmg_;

// Robot
robot_model::RobotModelPtr robot_model_;

// Planner
moveit_tmp::PlannerPtr planner_;

// Choose which arm to use
std::string hand_group_name_;
std::string arm_group_name_;
std::string eef_name_;
std::string hand_frame_;
std::string world_frame_;
Eigen::Isometry3d ik_frame_;

std::string hand_open_pose_;
std::string hand_close_pose_;
std::string arm_home_pose_;

int load_ = 0;

// KB
moveit_tmp::KnowledgeBase kb;

moveit::planning_interface::PlanningSceneInterfacePtr psi_;

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

void loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
  ros::NodeHandle pnh("~");

  // Planning group properties
  size_t errors = 0;
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name",
                                     hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "ik_frame", ik_frame_);

  // Predefined pose targets
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose",
                                     hand_close_pose_);
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);

  // Horizon size
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "horizon", horizon_);

  // File Path
  rosparam_shortcuts::get(LOGNAME, pnh, "domain_path", domain_path_);
  rosparam_shortcuts::get(LOGNAME, pnh, "problem_path", problem_path_);
  rosparam_shortcuts::get(LOGNAME, pnh, "data_path", data_path_);
  rosparam_shortcuts::get(LOGNAME, pnh, "planner_command", planner_command_);

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void initialize()
{
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description");

  // Load the robot model
  robot_model_ = robot_model_loader->getModel();
  arm_jmg_ = robot_model_->getJointModelGroup(arm_group_name_);

  psi_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
      robot_model_->getModelFrame());
}

void createTaskProblem()
{

  std::cout << problem_path_ << std::endl;

  std::ofstream problem_file;
  problem_file.open((problem_path_).c_str());

  problem_file << "(define (problem sort_clutter)(:domain navigation)"
               << std::endl;

  problem_file << "(:objects" << kb.getObjectsPDDL(false) << ")" << std::endl;

  problem_file << "(:init" << kb.getInitPDDL(false);

  problem_file << "(= (manipulation-time) 1)(= (load) " << load_
               << ")(= (max-load) 4)(= (speed) "
                  "2)(= (time) 0)";

  problem_file
      << "(= (distance table_1 table_2) 10)(= (distance table_2 table_1) 10)";
  problem_file
      << "(= (distance table_1 table_3) 10)(= (distance table_3 table_1) 10)";

  problem_file
      << "(= (distance table_2 table_3) 10)(= (distance table_3 table_2) 10)";

  problem_file << ")" << std::endl;

  problem_file
      << "(:goal (and (on red_0 red_target_0)(on red_1 red_target_1) "
         "(on green_0 green_target_0) (on green_1 green_target_1)"
         "(on blue_0 blue_target_0) (on blue_1 blue_target_1)"
         "(on yellow_0 yellow_target_0) (on yellow_1 yellow_target_1)))";
  problem_file << "(:metric minimize (time))";
  problem_file << ")";

  /*
    problem_file
        << "(:goal (and (on object_0 surface_8) (on object_1 surface_9) "
           "(on object_2 surface_10) (on object_3 surface_11) (on object_4 "
           "surface_0) (on object_5 surface_1) (on object_6 surface_2) (on
    object_7 surface_3))))";
  */
  problem_file.close();

  std::cout << kb.getPDDL();
}

bool convertDoublesToEigen(std::vector<double> values,
                           Eigen::Isometry3d& transform)
{
  if (values.size() == 6)
  {
    // This version is correct RPY
    Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

    transform =
        Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;

    return true;
  }
  else if (values.size() == 7)
  {
    // Quaternion
    transform = Eigen::Translation3d(values[0], values[1], values[2]) *
                Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Invalid number of doubles provided for transform, size="
                     << values.size());
    return false;
  }
}

void spawnObject(const moveit_msgs::CollisionObject& object,
                 const rviz_visual_tools::colors& color)
{

  psi_->applyCollisionObject(object, visual_tools_->getColor(color));
}

void spawnObject(const moveit_msgs::CollisionObject& object)
{

  psi_->applyCollisionObject(object);
}

moveit_msgs::CollisionObject createTable(std::string table_name, double x,
                                         double y, double z, double width,
                                         double depth, double height,
                                         std::string table_reference_frame)
{
  std::vector<double> table_dimensions = {width, depth, height};
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  moveit_msgs::CollisionObject object;
  object.id = table_name;
  object.header.stamp = ros::Time::now();
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = table_dimensions;
  pose.position.z -= 0.5 * table_dimensions[2] +
                     0.005; // align surface with world TODO SISTEMARE
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCylinder(std::string object_name, double x,
                                            double y, double z, double radius,
                                            double height,
                                            std::string object_reference_frame)
{
  std::vector<double> object_dimensions = {height, radius};
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z += 0.5 * object_dimensions[0] + z;
  pose.orientation.w = 1.0;

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject
moveCollisionObject(const geometry_msgs::PoseStamped& pose,
                    const std::string& name)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = pose.header.frame_id;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::MOVE;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose.pose;

  return collision_obj;
}

void loadTarget(const std::string& param, const std::string& prefix,
                const rviz_visual_tools::colors& color)
{
  ros::NodeHandle pnh("~");

  int n;
  rosparam_shortcuts::get(LOGNAME, pnh, param, n);
  for (int i = 0; i < n; i++)
  {
    std::string surface_id = prefix + std::to_string(i);
    std::vector<double> object_pose;
    std::string frame_id;
    rosparam_shortcuts::get(LOGNAME, pnh, surface_id + "/pose", object_pose);
    rosparam_shortcuts::get(LOGNAME, pnh, surface_id + "/frame_id", frame_id);

    Eigen::Isometry3d p_min = Eigen::Isometry3d::Identity();
    p_min.translation().z() = 0.001;
    Eigen::Isometry3d p_max = Eigen::Isometry3d::Identity();
    p_max.translation().z() = 0.001;

    moveit_tmp::Surface surface = {surface_id, p_min, p_max};
    moveit_tmp::Object object = {prefix + std::to_string(i),
                                 moveit_tmp::ObjectType::FIXED, -1, true,
                                 surface};

    spawnObject(createCylinder(surface_id, object_pose[0], object_pose[1],
                               object_pose[2], 0.03, 0.001, frame_id),
                color);

    kb.addObject(object);
    kb.getIsStackable()->add(surface_id, true);
    kb.getLocatedIn()->add(surface_id, frame_id);
  }
}

void loadObject(const std::string& param, const std::string& prefix,
                const rviz_visual_tools::colors& color)
{
  ros::NodeHandle pnh("~");

  int n;
  rosparam_shortcuts::get(LOGNAME, pnh, param, n);
  for (int i = 0; i < n; i++)
  {
    std::string object_name = prefix + std::to_string(i);
    std::vector<double> object_pose;
    std::string frame_id;
    rosparam_shortcuts::get(LOGNAME, pnh, object_name + "/pose", object_pose);
    rosparam_shortcuts::get(LOGNAME, pnh, object_name + "/frame_id", frame_id);

    spawnObject(createCylinder(object_name, object_pose[0], object_pose[1],
                               object_pose[2], 0.015, 0.20, frame_id),
                color);
    moveit_tmp::Object obj = {object_name, moveit_tmp::ObjectType::MOVABLE, 1,
                              false};
    kb.addObject(obj);
    kb.getOn()->add(object_name, frame_id);

    std::string locate = kb.getLocatedIn()->getLocationById(frame_id);
    if (locate == "")
      locate = frame_id;
    kb.getLocatedIn()->add(object_name, locate);
  }
}

void generateGraspingPoints(const Eigen::Isometry3d& target_offset,
                            const int num_points,
                            std::vector<Eigen::Isometry3d>& grasping_points)
{
  double current_angle_ =
      -M_PI + (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                  (2.0 * M_PI);

  for (int i = 0; i < num_points; i++)
  {
    // rotate object pose about z-axis
    Eigen::Isometry3d target_pose(target_offset);
    target_pose.rotate(
        Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()));

    grasping_points.push_back(target_pose);

    current_angle_ += 2 * M_PI / num_points;

    if (current_angle_ > M_PI)
      current_angle_ -= (2 * M_PI);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_tmp_baseline");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  // TODO Spostare fuori da main
  // Execution
  actionlib::SimpleActionClient<moveit_tmp_msgs::ExecuteTMPSolutionAction>
      execute_("execute_tmp_solution", true);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",
                                                                   true);

  loadParameters();

  initialize();

  srand(time(NULL));

  // moveit_tmp::Surface sur = {"world", p_min, p_max};
  // moveit_tmp::Object obj = {"robot", moveit_tmp::ObjectType::SURFACE, true,
  //                          sur};
  // kb.addObject(obj);

  // Create scene

  std::map<std::string, std::vector<double>> docking_station;

  spawnObject(
      createTable("table_1", 0.0, 2.0, 0.2, 1.2, 0.8, 0.2, world_frame_),
      rviz_visual_tools::colors::BROWN);

  moveit_tmp::Object table_1 = {"table_1", moveit_tmp::ObjectType::PLACE,
                                false};
  kb.addObject(table_1);
  docking_station.insert(
      std::make_pair("table_1", std::vector<double>{0.0, 0.9, 0.0, 0.0, 0.0,
                                                    0.7071068, 0.7071068}));

  spawnObject(
      createTable("table_2", 0.0, -2.0, 0.2, 1.2, 0.8, 0.2, world_frame_),
      rviz_visual_tools::colors::BROWN);

  moveit_tmp::Object table_2 = {"table_2", moveit_tmp::ObjectType::PLACE,
                                false};
  kb.addObject(table_2);
  docking_station.insert(
      std::make_pair("table_2", std::vector<double>{0.0, -0.9, 0.0, 0.0, 0.0,
                                                    -0.7071068, 0.7071068}));

  spawnObject(
      createTable("table_3", 2.5, 0.0, 0.2, 0.8, 1.2, 0.2, world_frame_),
      rviz_visual_tools::colors::BROWN);

  moveit_tmp::Object table_3 = {"table_3", moveit_tmp::ObjectType::PLACE,
                                false};
  docking_station.insert(std::make_pair(
      "table_3", std::vector<double>{1.55, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}));

  kb.addObject(table_3);

  std::vector<double> min = {-0.4, -0.7, 0.102, 0.0, 0.0, 0.0};
  std::vector<double> max = {0.0, 0.7, 0.102, 0.0, 0.0, 0.0};
  Eigen::Isometry3d p_min, p_max;
  convertDoublesToEigen(min, p_min);
  convertDoublesToEigen(max, p_max);

  moveit_tmp::Surface surface = {"table_3", p_min, p_max};
  moveit_tmp::Object surface_1 = {"surface_1", moveit_tmp::ObjectType::SURFACE,
                                  -1, true, surface};
  spawnObject(
      createTable("surface_1", 2.5, 0.0, 0.2, 0.8, 1.2, 0.2, world_frame_),
      rviz_visual_tools::colors::BROWN);

  kb.addObject(surface_1);
  kb.getLocatedIn()->add("surface_1", "table_3");

  // robot surface
  min = {-0.1, -0.15, 0.007, 0.0, 0.0, 0.0};
  max = {-0.3, 0.15, 0.007, 0.0, 0.0, 0.0};
  convertDoublesToEigen(min, p_min);
  convertDoublesToEigen(max, p_max);

  moveit_tmp::Surface surface_robot = {"top_plate_link", p_min, p_max};
  moveit_tmp::Object surface_robot_1 = {"robot", moveit_tmp::ObjectType::ROBOT,
                                        -1, true, surface_robot};
  kb.addObject(surface_robot_1);

  kb.getNear()->add("table_1");

  //  spawnObject(
  //      createCylinder("robot", 0.0, 0.0, 0.02, 0.1, 0.01, "top_plate_link"),
  //      rviz_visual_tools::colors::BROWN);

  // TODO load targets
  loadTarget("red_targets", "red_target_", rviz_visual_tools::colors::RED);
  loadTarget("green_targets", "green_target_",
             rviz_visual_tools::colors::GREEN);
  loadTarget("blue_targets", "blue_target_", rviz_visual_tools::colors::BLUE);
  loadTarget("yellow_targets", "yellow_target_",
             rviz_visual_tools::colors::YELLOW);

  // TODO load objects
  loadObject("red_objects", "red_", rviz_visual_tools::colors::RED);
  loadObject("green_objects", "green_", rviz_visual_tools::colors::GREEN);
  loadObject("blue_objects", "blue_", rviz_visual_tools::colors::BLUE);
  loadObject("yellow_objects", "yellow_", rviz_visual_tools::colors::YELLOW);

  std::ofstream myfile;
  myfile.open("/home/nicola/" + std::to_string(horizon_) + "_" +
                  std::to_string(10) + ".csv",
              std::ios_base::app);

  // Inizializzo libreria
  moveit_tmp::TaskPlanner task_planner;
  task_planner.setDomainPath(domain_path_);
  task_planner.setProblemPath(problem_path_);
  task_planner.setPath(data_path_);
  task_planner.setPlannerCommand(planner_command_);

  moveit_tmp::ModifyPlanningScene mps;
  moveit_tmp::PlacePoseGenerator ppg;
  ppg.setFixedLink("base_link"); // TODO creare parametro

  planner_ = std::make_shared<moveit_tmp::Planner>(robot_model_);

  moveit_tmp::Pick pick("pick", planner_);
  pick.setIKFrame(ik_frame_, hand_frame_);
  pick.setGroup(arm_group_name_);
  pick.setEndEffector(eef_name_);

  moveit_tmp::Place place("place", planner_);
  place.setIKFrame(ik_frame_, hand_frame_);
  place.setGroup(arm_group_name_);
  place.setEndEffector(eef_name_);

  planning_scene::PlanningScenePtr ps;

  moveit_tmp::CurrentState cs(robot_model_);
  cs.compute(ps);

  moveit_msgs::PlanningScene psm;
  ps->getPlanningSceneMsg(psm);
  psi_->applyPlanningScene(psm);

  std::vector<moveit_msgs::ObjectColor> object_colors = psm.object_colors;

  const auto begin = std::chrono::system_clock::now();

  moveit_tmp::ActionCache cache;

  bool f = false;
  bool status_changed = true;
  int j = 0;

  std::vector<moveit_tmp::Action> task_solution;

  double reasoning_time = 0.0;

  while (ros::ok())
  {

    moveit_tmp::CurrentState current_state(robot_model_);
    planning_scene::PlanningScenePtr current_scene;
    current_state.compute(current_scene);

    current_scene->getPlanningSceneMsg(psm);
    psm.object_colors = object_colors;

    psi_->applyPlanningScene(psm);

    // se lo stato è inaspettato resetto
    if (status_changed)
    {
      ROS_INFO("State is changed");

      // Remove constraints
      kb.getObstruct()->reset();
      kb.getLeaveClean()->reset();

      cache.reset();
      f = false;
      status_changed = false;
    }

    // se no ho una soluzione valida rigenero il task plan
    if (!f)
    {
      std::vector<const moveit::core::AttachedBody*> attached_bodies;
      current_scene->getCurrentState().getAttachedBodies(attached_bodies);

      for (int i = 0; i < attached_bodies.size(); i++)
      {

        std::cout << "ATTACH LINK "
                  << attached_bodies[i]->getAttachedLink()->getName()
                  << std::endl;

        if (attached_bodies[i]->getAttachedLink()->getName() == hand_frame_)
        {
          std::string in_hand = attached_bodies[i]->getName();

          // std::cout << "check in-hand: " << in_hand << std::endl;

          kb.getInHand()->add(in_hand);
        }
      }

      createTaskProblem();

      if (!task_planner.plan(task_solution))
      {
        ROS_INFO("Task Unsolvable");
        break;
      }
      f = true;
      j = 0;
    }

    ROS_INFO_STREAM("j: " << j << " size: " << task_solution.size());

    if (j >= task_solution.size())
    {
      ROS_INFO("Task Complete");
      break;
    }

    ROS_INFO("Start Reasoning");

    clock_t reasoning_start = clock();

    moveit_tmp::KnowledgeBase sandbox_kb(kb);

    std::vector<ActionSolution> action_solutions;
    current_state.compute(current_scene);

    ROS_INFO("Start Loop");

    for (int i = j; i < task_solution.size() && i < j + horizon_; i++)
    {
      ROS_INFO_STREAM(task_solution[i]);

      planning_scene::PlanningScenePtr scene_out;

      std::string action_name = task_solution[i].name;

      if (action_name == "pickup" || action_name == "unstack" ||
          action_name == "unload")
      {
        bool found = false;
        std::string object_name = task_solution[i].parameters[0];

        std::string surface_name;
        if (action_name == "unload")
          task_solution[i].parameters.push_back("robot");

        surface_name = task_solution[i].parameters[1];

        if (action_name == "unload")
        {
          mps.detachObject(object_name, "top_plate_link");
          mps.apply(current_scene, scene_out);

          scene_out->decoupleParent();
          current_scene = scene_out;
        }

        // search solution in cache
        std::vector<double> joint_values;
        moveit_tmp::ActionCache::ActionID id =
            std::make_tuple(action_name, object_name, surface_name);
        if (cache.retrieve(id, joint_values))
        {
          scene_out = current_scene->diff();

          robot_state::RobotState& robot_state =
              scene_out->getCurrentStateNonConst();

          robot_state.setJointGroupPositions(arm_group_name_, joint_values);
          robot_state.update();

          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          scene_out->checkCollision(req, res);

          found = !res.collision;

          if (!found)
          {
            cache.remove(id);
          }
          else
          {
            action_solutions.emplace_back(task_solution[i], joint_values);
          }
        }

        // solution not founded in cache
        if (!found)
        {
          std::vector<std::string> collisions;
          std::vector<Eigen::Isometry3d> grasping_poses;

          generateGraspingPoints(Eigen::Isometry3d::Identity(), 40,
                                 grasping_poses);

          bool res = pick.reason(current_scene, object_name, grasping_poses,
                                 scene_out, collisions);

          // std::cout << "collisions: " << collisions.size() << std::endl;

          // std::vector<double> joint_values;
          scene_out->getCurrentState().copyJointGroupPositions(arm_group_name_,
                                                               joint_values);

          // cache[action_name + "_" + object_name] = joint_values;

          cache.insert(id, joint_values);

          action_solutions.emplace_back(task_solution[i], joint_values);

          if (!res)
          {

            for (int i = 0; i < collisions.size(); i++)
            {

              // avoid object collide with themself
              if (collisions[i] == object_name)
              {
                ROS_WARN("Object collide with themself.");
                continue;
              }

              moveit_tmp::Object coll_obj;
              if (kb.getObjectById(collisions[i], coll_obj))
              {
                if (coll_obj.type == moveit_tmp::ObjectType::MOVABLE ||
                    coll_obj.type == moveit_tmp::ObjectType::FIXED)
                {
                  // kb.addObject(collisions[i], moveit_tmp::MOVABLE);
                  kb.getObstruct()->add(collisions[i], object_name);
                }
              }

              moveit_tmp::Object surface;
              if (sandbox_kb.getOn()->getUnderById(collisions[i], surface))
              {
                if (surface.type != moveit_tmp::ObjectType::SURFACE)
                  kb.getLeaveClean()->add(surface.id, object_name);
              }
            }
            f = false;
            ROS_WARN("Collisions found. Update contraints and replan.");
            break; // continue
          }
        }

        sandbox_kb.getOn()->removeObject(object_name);

        scene_out->decoupleParent();
        current_scene = scene_out;

        mps.attachObject(object_name, hand_frame_);
        mps.apply(current_scene, scene_out);

        scene_out->decoupleParent();
        current_scene = scene_out;
      }
      else if (action_name == "putdown" || action_name == "stack" ||
               action_name == "load")
      {
        bool found = false;
        std::string object_name = task_solution[i].parameters[0];

        std::string surface_name;
        if (action_name == "load")
          task_solution[i].parameters.push_back("robot");

        surface_name = task_solution[i].parameters[1];

        // search solution in cache
        std::vector<double> joint_values;
        moveit_tmp::ActionCache::ActionID id =
            std::make_tuple(action_name, object_name, surface_name);
        if (cache.retrieve(id, joint_values))
        {
          scene_out = current_scene->diff();

          robot_state::RobotState& robot_state =
              scene_out->getCurrentStateNonConst();

          robot_state.setJointGroupPositions(arm_group_name_, joint_values);
          robot_state.update();

          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          scene_out->checkCollision(req, res);

          found = !res.collision;

          if (!found)
          {
            cache.remove(id);
          }
          else
          {
            action_solutions.emplace_back(task_solution[i], joint_values);
          }
        }

        // solution not founded in cache
        if (!found)
        {

          moveit_tmp::Object surface;
          if (!kb.getObjectById(surface_name, surface))
          {
            ROS_ERROR_STREAM("Surface " << surface_name << " not found.");
            return 1; // TODO
          }

          geometry_msgs::PoseStamped target;
          Eigen::Isometry3d pose;

          if (!ppg.compute(current_scene, object_name, surface.surface.frame_id,
                           surface.surface.p_min, surface.surface.p_max,
                           target))
          {
            // kb.getIsStackable()->add(surface.id, false);
            f = false;
            ROS_WARN("Place surface not empty. Update contraints and replan.");
            break;
          }

          tf::poseMsgToEigen(target.pose, pose);

          std::vector<Eigen::Isometry3d> poses;
          generateGraspingPoints(pose, 40, poses);

          std::vector<std::string> collisions;
          bool res = place.reason(current_scene, target.header.frame_id, poses,
                                  scene_out, collisions);

          std::vector<double> joint_values;
          scene_out->getCurrentState().copyJointGroupPositions(arm_group_name_,
                                                               joint_values);

          cache.insert(id, joint_values);

          action_solutions.emplace_back(task_solution[i], joint_values);

          if (!res)
          {

            for (int i = 0; i < collisions.size(); i++)
            {

              // avoid object collide with themself
              if (collisions[i] == object_name)
              {
                ROS_WARN("Object collide with themself.");
                continue;
              }

              moveit_tmp::Object coll_obj;
              if (kb.getObjectById(collisions[i], coll_obj))
              {
                if (coll_obj.type == moveit_tmp::ObjectType::MOVABLE ||
                    coll_obj.type == moveit_tmp::ObjectType::FIXED)
                {
                  // kb.addObject(collisions[i], moveit_tmp::MOVABLE);
                  kb.getObstruct()->add(collisions[i], surface_name);
                }
              }

              moveit_tmp::Object surface;
              if (sandbox_kb.getOn()->getUnderById(collisions[i], surface))
              {
                if (surface.type != moveit_tmp::ObjectType::SURFACE)
                  kb.getLeaveClean()->add(surface.id, object_name);
              }
            }
            f = false;
            ROS_WARN("Collisions found. Update contraints and replan.");
            break; // continue
          }
        }

        sandbox_kb.getOn()->add(object_name, surface_name);

        scene_out->decoupleParent();
        current_scene = scene_out;

        mps.detachObject(object_name, hand_frame_);
        mps.apply(current_scene, scene_out);

        scene_out->decoupleParent();
        current_scene = scene_out;

        if (action_name == "load")
        {
          mps.attachObject(object_name, "top_plate_link");
          mps.apply(current_scene, scene_out);

          scene_out->decoupleParent();
          current_scene = scene_out;
        }
      }
      else if (action_name == "navigate")
      {

        robot_state::RobotState& state =
            current_scene->getCurrentStateNonConst();
        state.setToDefaultValues(arm_jmg_, arm_home_pose_);
        state.update();

        moveit_msgs::PlanningScene ps_msg;

        current_scene->getPlanningSceneMsg(ps_msg);

        std::vector<double> place =
            docking_station[task_solution[i].parameters[1]];

        ps_msg.robot_state.multi_dof_joint_state.transforms[0].translation.x =
            place[0];
        ps_msg.robot_state.multi_dof_joint_state.transforms[0].translation.y =
            place[1];
        ps_msg.robot_state.multi_dof_joint_state.transforms[0].translation.z =
            place[2];

        /*
        ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.x = 0;
        ps_msg.robot_sclock();tate.multi_dof_joint_state.transforms[0].rotation.y
        = 0; ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.z =
        0; ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.w =
        1;
        */

        tf2::Quaternion quat_tf;
        quat_tf.setX(place[3]);
        quat_tf.setY(place[4]);
        quat_tf.setZ(place[5]);
        quat_tf.setW(place[6]);
        quat_tf.normalize();

        ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.x =
            quat_tf.getX();
        ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.y =
            quat_tf.getY();
        ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.z =
            quat_tf.getZ();
        ps_msg.robot_state.multi_dof_joint_state.transforms[0].rotation.w =
            quat_tf.getW();

        action_solutions.emplace_back(task_solution[i], place);

        current_scene->setPlanningSceneDiffMsg(ps_msg);
      }
      else
      {
        ROS_ERROR("Undefined action name.");
        return -1;
      }

      // std::cout << kb.getObstruct()->getPDDL() << std::endl;
    }

    clock_t reasoning_end = clock();
    double reasoning_secs =
        double(reasoning_end - reasoning_start) / CLOCKS_PER_SEC;
    reasoning_time += reasoning_secs;
    ROS_INFO_STREAM("Reasoning Time: " << reasoning_secs);

    ROS_INFO("End Geometry Reasoning");

    // se ho una soluzione valida ...
    if (f)
    {

      ROS_INFO("Execution");

      planning_scene::PlanningScenePtr scene;
      current_state.compute(scene);

      if (action_solutions[0].first.name == "navigate")
      {
        planning_scene::PlanningScenePtr scene_goal;
        moveit_tmp::Move move(planner_);
        move.setGroup(arm_group_name_);
        robot_trajectory::RobotTrajectoryPtr traj;
        move.compute(scene, arm_home_pose_, scene_goal, traj);

        moveit_tmp_msgs::ExecuteTMPSolutionGoal execute_goal;

        execute_goal.solution.emplace_back();
        moveit_tmp_msgs::SubTrajectory& t = execute_goal.solution.back();
        scene_goal->getPlanningSceneDiffMsg(t.scene_diff);
        traj->getRobotTrajectoryMsg(t.trajectory);

        ROS_INFO("Executing solution trajectory");
        execute_.sendGoal(execute_goal);
        execute_.waitForResult();
        moveit_msgs::MoveItErrorCodes execute_result =
            execute_.getResult()->error_code;

        if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_ERROR_STREAM("Task execution failed and returned: "
                           << execute_.getState().toString());
        }

        move_base_msgs::MoveBaseGoal goal;

        // we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "odom";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = action_solutions[0].second[0];
        goal.target_pose.pose.position.y = action_solutions[0].second[1];
        goal.target_pose.pose.position.z = action_solutions[0].second[2];

        /*
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;
        */

        goal.target_pose.pose.orientation.x = action_solutions[0].second[3];
        goal.target_pose.pose.orientation.y = action_solutions[0].second[4];
        goal.target_pose.pose.orientation.z = action_solutions[0].second[5];
        goal.target_pose.pose.orientation.w = action_solutions[0].second[6];

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        if (ac.waitForResult())
          kb.getNear()->add(action_solutions[0].first.parameters[1]);

        status_changed = true;
      }
      else
      {
        moveit_tmp_msgs::ExecuteTMPSolutionGoal execute_goal;

        std::vector<moveit_tmp::ActionPipeline> execute;

        ROS_INFO_STREAM("Planning: "
                        << action_solutions[0].first.name << " "
                        << action_solutions[0].first.parameters[0]);

        std::vector<double> joint_values = action_solutions[0].second;

        robot_trajectory::RobotTrajectoryPtr traj;

        execute.emplace_back();
        moveit_tmp::ActionPipeline& p = execute.back();

        bool s = false;
        int count = 0;
        while (!s && count < 3)
        {
          bool attach = false;

          if (action_solutions[0].first.name == "pickup" ||
              action_solutions[0].first.name == "unstack" ||
              action_solutions[0].first.name == "unload")
          {

            if (action_solutions[0].first.name == "unload")
              attach = true;

            s = pick.plan(scene, action_solutions[0].first.parameters[0],
                          joint_values, attach, p);
          }
          else if (action_solutions[0].first.name == "putdown" ||
                   action_solutions[0].first.name == "stack" ||
                   action_solutions[0].first.name == "load")
          {

            if (action_solutions[0].first.name == "load")
              attach = true;

            s = place.plan(scene, action_solutions[0].first.parameters[0],
                           joint_values, attach, p);
          }
          else
          {
            ROS_ERROR("Undefined action name.");
            return -1;
          }
          count++;
        }

        ROS_INFO("Done");

        if (!s)
        {
          ros::shutdown();
          return 0;
        }

        moveit_tmp::ActionCache::ActionID id =
            std::make_tuple(action_solutions[0].first.name,
                            action_solutions[0].first.parameters[0],
                            action_solutions[0].first.parameters[1]);
        cache.remove(id);

        ROS_INFO("Create execution message");
        for (int i = 0; i < execute.size(); i++)
        {
          moveit_tmp::ActionPipeline p = execute[i];
          for (int j = 0; j < p.size(); j++)
          {
            execute_goal.solution.emplace_back();
            moveit_tmp_msgs::SubTrajectory& t = execute_goal.solution.back();
            p[j].first->getPlanningSceneDiffMsg(t.scene_diff);
            if (p[j].second)
              p[j].second->getRobotTrajectoryMsg(t.trajectory);
          }
        }

        ROS_INFO("Executing solution trajectory");
        execute_.sendGoal(execute_goal);
        execute_.waitForResult();
        moveit_msgs::MoveItErrorCodes execute_result =
            execute_.getResult()->error_code;

        if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_ERROR_STREAM("Task execution failed and returned: "
                           << execute_.getState().toString());

          status_changed = true;
          continue;
        }
        ROS_INFO("DONE");

        // UPDATE STATUS
        if (action_solutions[0].first.name == "pickup" ||
            action_solutions[0].first.name == "unstack")
        {
          kb.getOn()->removeObject(action_solutions[0].first.parameters[0]);
          kb.getObstruct()->removeObject(
              action_solutions[0].first.parameters[0]);
          kb.getLeaveClean()->removeObject(
              action_solutions[0].first.parameters[0]);
        }
        else if (action_solutions[0].first.name == "unload")
        {
          ROS_WARN_STREAM(
              "UNLOADED: " << action_solutions[0].first.parameters[0]);
          kb.getLoaded()->removeObject(action_solutions[0].first.parameters[0]);
          load_--;

          std::cout << "TEST:" << std::endl << kb.getPDDL() << std::endl;
        }
        else if (action_solutions[0].first.name == "putdown" ||
                 action_solutions[0].first.name == "stack")
        {
          kb.getOn()->add(action_solutions[0].first.parameters[0],
                          action_solutions[0].first.parameters[1]);
          kb.getInHand()->empty();

          std::string location = kb.getLocatedIn()->getLocationById(
              action_solutions[0].first.parameters[1]);
          std::cout << "location: " << location << std::endl;
          kb.getLocatedIn()->add(action_solutions[0].first.parameters[0],
                                 location);
        }
        else if (action_solutions[0].first.name == "load")
        {
          kb.getLoaded()->add(action_solutions[0].first.parameters[0]);
          ROS_WARN_STREAM(
              "LOADED: " << action_solutions[0].first.parameters[0]);
          kb.getInHand()->empty();
          load_++;
        }

        ROS_INFO("UPDATED KB");
      }

      j++;
      // } //tentativo movimento dopo ogni ciclo

      // Genero movimento casuale

      double r = (double)rand() / RAND_MAX;
      ROS_INFO_STREAM("RAND: " << r);
      if (r < 0.1)
      {

        ros::NodeHandle pnh("~");

        std::vector<std::string> ob = {"red_0",    "red_1",   "green_0",
                                       "green_1",  "blue_0",  "blue_1",
                                       "yellow_0", "yellow_1"};

        int obj = rand() % 8; // TODO inserire num_objects
        // int sur = rand() % num_surfaces;

        std::string object_name = ob[obj];
        // std::string surface_name = "surface_" + std::to_string(sur);

        std::vector<double> object_pose;
        rosparam_shortcuts::get(LOGNAME, pnh, object_name + "/pose",
                                object_pose);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "surface_1";
        pose.pose.position.x = object_pose[0] + 0.02;
        pose.pose.position.y = object_pose[1] + 0.02;
        pose.pose.position.z = 0.5 * 0.2 + object_pose[2];

        spawnObject(moveCollisionObject(pose, object_name));

        kb.getOn()->add(object_name, "surface_1");
        kb.getLocatedIn()->add(object_name, "table_3");

        ROS_WARN_STREAM("Moved Object: " << object_name);

        status_changed = true;
      }

    } // tentativo movimento dopo ogni movimento
  }

  const std::chrono::duration<double> elapsed_secs =
      std::chrono::system_clock::now() - begin;
  ROS_INFO_STREAM("Elapsed Time: " << elapsed_secs.count()
                                   << " Reasoning Time: " << reasoning_time);
  // ROS_INFO_STREAM("Removed Objects: " << tot_removed);
  myfile << elapsed_secs.count() << ";" /*<< tot_removed * 2 + 1 << ";"*/ << f
         << "\n";
  myfile.close();

  //
  // begin = clock();

  // end = clock();
  // elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  // ROS_INFO_STREAM("Elapsed Time: " << elapsed_secs);

  /* Wait for user input */
  // visual_tools.prompt(
  // b    "Press 'next' in the RvizVisualToolsGui window to finish the demo");

  ros::shutdown();
  return 0;
}
