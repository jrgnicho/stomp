/*
 * stomp_planner.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <stomp/stomp.h>
#include <stomp_moveit_interface/stomp_optimization_task.h>
#include <stomp_moveit_interface/stomp_planner.h>
#include <stomp/stomp_utils.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <class_loader/class_loader.h>

namespace stomp_moveit_interface
{

const static int DEFAULT_MAX_ITERATIONS = 100;
const static int DEFAULT_ITERATIONS_AFTER_COLLISION_FREE = 10;
const static double DEFAULT_CONTROL_COST_WEIGHT = 0.001;
const static double DEFAULT_PADDING = 0.001;
const static double DEFAULT_SCALE = 1.0;

StompPlanner::StompPlanner(const std::string& group):
    PlanningContext("STOMP",group),
    node_handle_("~")
{
  trajectory_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("marker", 20);
  robot_body_viz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("marker_array", 20);

}

StompPlanner::~StompPlanner()
{
}

void StompPlanner::init(const moveit::core::RobotModelConstPtr& model)
{
  kinematic_model_ = model;

  // read distance field params
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_x", df_size_x_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_y", df_size_y_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/size_z", df_size_z_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/resolution", df_resolution_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/collision_tolerance", df_collision_tolerance_));
  STOMP_VERIFY(node_handle_.getParam("collision_space/max_propagation_distance", df_max_propagation_distance_));
}

void StompPlanner::getPlanningAlgorithms(std::vector<std::string> &algs) const
{
  algs.clear();
  algs.push_back("STOMP");
  algs.push_back("CHOMP");
}

bool StompPlanner::solve(planning_interface::MotionPlanResponse &res)
{
  ROS_INFO("STOMP: solve() started.");
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);

  // construct the compact response from the detailed one
  res.trajectory_ = detailed_res.trajectory_.back();
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = ros::Duration(wd.sec, wd.nsec).toSec();
  res.error_code_ = detailed_res.error_code_;

  ROS_INFO("STOMP: solve() finished.");
  return success;
}

void StompPlanner::printCollisionRobot()
{
  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot = planning_scene_->getCollisionRobot();
  const std::map<std::string,double>& padding_map = collision_robot->getLinkPadding();
  for(std::map<std::string,double>::const_iterator iter = padding_map.begin();
      iter != padding_map.end();iter++)
  {
    ROS_INFO_STREAM("Collision Robot Link: "<<iter->first);
  }

}

bool StompPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  ros::WallTime start_time = ros::WallTime::now();
  boost::shared_ptr<StompOptimizationTask> stomp_task;
  boost::shared_ptr<stomp::STOMP> stomp;

  int max_rollouts;
  int num_threads=1;
  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts));

  // prepare the collision checkers
  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot; /**< standard robot collision checker */
  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world; /**< standard robot -> world collision checker */
  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> collision_robot_df;    /**< distance field robot collision checker */
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> collision_world_df;    /**< distance field robot -> world collision checker */

  printCollisionRobot();

  collision_robot = planning_scene_->getCollisionRobot();
  collision_world = planning_scene_->getCollisionWorld();
  std::map<std::string, std::vector<collision_detection::CollisionSphere> > link_body_decompositions;
  bool use_signed_distance_field = true;
  double padding = DEFAULT_PADDING;
  double scale = DEFAULT_SCALE;

  ros::Time start = ros::Time::now();
  ROS_DEBUG_STREAM("Stomp Planner collision setup started");
  collision_robot_df.reset(new collision_detection::CollisionRobotDistanceField(kinematic_model_,
                                                                                link_body_decompositions,
                                                                                df_size_x_, df_size_y_, df_size_z_,
                                                                                use_signed_distance_field,
                                                                                df_resolution_, df_collision_tolerance_,
                                                                                df_max_propagation_distance_,
                                                                                padding, scale));

  collision_world_df.reset(new collision_detection::CollisionWorldDistanceField(df_size_x_, df_size_y_, df_size_z_,
                                                                                use_signed_distance_field,
                                                                                df_resolution_, df_collision_tolerance_,
                                                                                df_max_propagation_distance_));

  copyObjects(collision_world, collision_world_df);
  ROS_DEBUG_STREAM("Stomp Planner collision setup completed after "<<(ros::Time::now() - start).toSec()<<" seconds");


  // first setup the task
  stomp_task.reset(new StompOptimizationTask(node_handle_, request_.group_name,
                                             kinematic_model_,
                                             collision_robot, collision_world,
                                             collision_robot_df, collision_world_df));

  STOMP_VERIFY(stomp_task->initialize(num_threads, max_rollouts));

  XmlRpc::XmlRpcValue features_xml;
  STOMP_VERIFY(node_handle_.getParam("features", features_xml));
  stomp_task->setFeaturesFromXml(features_xml);
  stomp_task->setControlCostWeight(DEFAULT_CONTROL_COST_WEIGHT);
  stomp_task->setTrajectoryVizPublisher(const_cast<ros::Publisher&>(trajectory_viz_pub_));
  stomp_task->setDistanceFieldVizPublisher((const_cast<ros::Publisher&>(trajectory_viz_pub_)));
  stomp_task->setRobotBodyVizPublisher((const_cast<ros::Publisher&>(robot_body_viz_pub_)));
  stomp_task->setMotionPlanRequest(planning_scene_, request_);

  stomp.reset(new stomp::STOMP());
  stomp->initialize(node_handle_, stomp_task);

  // TODO: don't hardcode these params
  start = ros::Time::now();
  ROS_DEBUG_STREAM("Stomp Planner planning optimization started");
  bool success = stomp->runUntilValid(DEFAULT_MAX_ITERATIONS,DEFAULT_ITERATIONS_AFTER_COLLISION_FREE);
  ROS_DEBUG_STREAM("Stomp Planner planning optimization completed after "<<(ros::Time::now() - start).toSec()<<" seconds");

  std::vector<Eigen::VectorXd> best_params;
  double best_cost;
  stomp->getBestNoiselessParameters(best_params, best_cost);
  stomp_task->publishTrajectoryMarkers(const_cast<ros::Publisher&>(trajectory_viz_pub_), best_params);

  trajectory_msgs::JointTrajectory trajectory;
  stomp_task->parametersToJointTrajectory(best_params, trajectory);
  ROS_DEBUG_STREAM(__FUNCTION__<< " Stomp analysis completed");

  moveit::core::RobotState robot_state(planning_scene_->getRobotModel());
  res.trajectory_.resize(1,robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(kinematic_model_,request_.group_name)));
  res.trajectory_.back()->setRobotTrajectoryMsg( robot_state,trajectory);
  res.description_.resize(1);
  res.description_[0] = "STOMP";
  res.processing_time_.resize(1);
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.processing_time_[0] = ros::Duration(wd.sec, wd.nsec).toSec();

  if (!success)
  {
    ROS_WARN("STOMP: failed to find a collision-free plan");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;


  return true;
}

bool StompPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  // check if planner is available
  if (req.planner_id != "STOMP" && req.planner_id != "CHOMP")
  {
    ROS_ERROR("STOMP: Planner %s not available.", req.planner_id.c_str());
    return false;
  }

  // check for single goal region
  if (req.goal_constraints.size() != 1)
  {
    ROS_ERROR("STOMP: Can only handle a single goal region.");
    return false;
  }

  // check that we have only joint constraints at the goal
  if (req.goal_constraints[0].position_constraints.size() > 0
      || req.goal_constraints[0].orientation_constraints.size() > 0
      || req.goal_constraints[0].visibility_constraints.size() > 0
      || req.goal_constraints[0].joint_constraints.size() == 0)
  {
    ROS_ERROR("STOMP: Can only handle joint space goals.");
    return false;
  }

  return true;
}

bool StompPlanner::terminate()
{
  ROS_ERROR("STOMP: terminate() not implemented yet!");
  return false;
}

void StompPlanner::clear()
{
  ROS_ERROR("STOMP: clear() not implemented yet!");
}



void StompPlanner::copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                 const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const
{
  std::vector<std::string> object_ids = from_world->getWorld()->getObjectIds();
  for (size_t i=0; i<object_ids.size(); ++i)
  {
    collision_detection::CollisionWorld::ObjectConstPtr obj = from_world->getWorld()->getObject(object_ids[i]);
    to_world->getWorld()->addToObject(object_ids[i], obj->shapes_, obj->shape_poses_);
  }
}

} /* namespace stomp_moveit_interface */

