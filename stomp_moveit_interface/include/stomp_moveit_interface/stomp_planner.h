/*
 * stomp_planner.h
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#ifndef STOMP_PLANNER_H_
#define STOMP_PLANNER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <ros/ros.h>

namespace stomp_moveit_interface
{

class StompPlanner: public planning_interface::PlanningContext
{
public:
  StompPlanner(const std::string& group);
  virtual ~StompPlanner();

  virtual void init(const moveit::core::RobotModelConstPtr& model);

  /// Get a short string that identifies the planning interface
  virtual std::string getDescription(void) const { return "STOMP"; }

  /// Get the names of the known planning algorithms (values that can be filled as planner_id in the planning request)
  virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const;

  /// Subclass must implement methods below
  virtual bool solve(planning_interface::MotionPlanResponse &res);

  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res);

  /// Determine whether this plugin instance is able to represent this planning request
  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;

  /// Request termination, if a solve() function is currently computing plans
  virtual bool terminate() ;

  virtual void clear();



private:
  ros::NodeHandle node_handle_;

  ros::Publisher trajectory_viz_pub_;
  ros::Publisher robot_body_viz_pub_;
  //boost::shared_ptr<stomp::STOMP> stomp_;
  //std::map<std::string, boost::shared_ptr<StompOptimizationTask> > stomp_tasks_;
  moveit::core::RobotModelConstPtr kinematic_model_;

  // distance field params
  double df_size_x_, df_size_y_, df_size_z_;
  double df_resolution_;
  double df_collision_tolerance_;
  double df_max_propagation_distance_;

//  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot_; /**< standard robot collision checker */
//  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world_; /**< standard robot -> world collision checker */
//  boost::shared_ptr<const collision_detection::CollisionRobotDistanceField> collision_robot_df_;    /**< distance field robot collision checker */
//  boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> collision_world_df_;    /**< distance field robot -> world collision checker */

  void copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                   const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const;
};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_PLANNER_H_ */
