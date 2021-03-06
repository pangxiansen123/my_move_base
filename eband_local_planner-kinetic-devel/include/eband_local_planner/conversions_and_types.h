

#ifndef EBAND_CONVERSIONS_AND_TYPES_H_
#define EBAND_CONVERSIONS_AND_TYPES_H_

#include <ros/ros.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>


namespace eband_local_planner{


  // typedefs

  ///<@brief defines a bubble - pose of center & radius of according hypersphere (expansion)
  // 中心，以及半径
  struct Bubble
  {
    geometry_msgs::PoseStamped center;
    double expansion;
  };

  enum AddAtPosition {add_front, add_back};

  // functions

  /**
   * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from quaternions to euler angles)
   * @param Pose which shall be converted
   * @param References to converted ROS Pose2D frmae
   */
  void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D);


  /**
   * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from euler angles to quaternions, -> z-coordinate is set to zero)
   * @param References to converted ROS Pose2D frame
   * @param Pose2D which shall be converted
   */
  void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D);


  /**
   * @brief  Transforms the global plan of the robot from the planner frame to the local frame. This replaces the transformGlobalPlan as defined in the base_local_planner/goal_functions.h main difference is that it additionally outputs counter indicating which part of the plan has been transformed.
   将机器人的全局计划从计划器框架转换为本地框架。这取代了base_local_planner / goal_functions.h中定义的transformGlobalPlan。
   主要区别在于，它另外输出了指示计划的哪个部分已被转换的计数器。
   这个函数的作用就是实时的存储局部地图中的全局点(放在transformed_plan),并且start_end_counts存储的是局部地图中开始的点和将要出去的点
   * @param tf A reference to a transform listener
   * @param global_plan The plan to be transformed
   * @param costmap A reference to the costmap being used so the window size for transforming can be computed
   * @param global_frame The frame to transform the plan to
   * @param transformed_plan Populated with the transformed plan填充了转换后的计划
   * @param number of start and end frame counted from the end of the global plan
   */
  bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
      costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts_from_end);

  /**
   * @brief Gets the footprint of the robot and computes the circumscribed radius for the eband approach
   * @param costmap A reference to the costmap from which the radius is computed
   * @return radius in meters
   */
  double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap);

};
#endif

