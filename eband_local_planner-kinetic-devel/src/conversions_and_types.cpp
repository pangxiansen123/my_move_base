// 第一部分负责数据类型的转换

#include <eband_local_planner/conversions_and_types.h>

namespace eband_local_planner{

  void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D)
  {
    // use tf-pkg to convert angles
    tf::Pose pose_tf;

    // convert geometry_msgs::PoseStamped to tf::Pose
    tf::poseMsgToTF(pose, pose_tf);

    // now get Euler-Angles from pose_tf
    double useless_pitch, useless_roll, yaw;
    pose_tf.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

    // normalize angle
    yaw = angles::normalize_angle(yaw);

    // and set to pose2D
    pose2D.x = pose.position.x;
    pose2D.y = pose.position.y;
    pose2D.theta = yaw;

    return;
  }


  void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D)
  {
    // use tf-pkg to convert angles
    tf::Quaternion frame_quat;

    // transform angle from euler-angle to quaternion representation
    frame_quat = tf::createQuaternionFromYaw(pose2D.theta);

    // set position
    pose.position.x = pose2D.x;
    pose.position.y = pose2D.y;
    pose.position.z = 0.0;

    // set quaternion
    pose.orientation.x = frame_quat.x();
    pose.orientation.y = frame_quat.y();
    pose.orientation.z = frame_quat.z();
    pose.orientation.w = frame_quat.w();

    return;
  }

  // eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, start_end_counts)
  // 将机器人的全局计划从计划器框架转换为本地框架。这取代了base_local_planner / goal_functions.h中定义的transformGlobalPlan。
  // 主要区别在于，它另外输出了指示计划的哪个部分已被转换的计数器。
  // 这个函数的作用就是实时的存储局部地图中的全局点(放在transformed_plan),并且start_end_counts存储的是局部地图中开始的点和将要出去的点
  bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
      costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
      std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts)
  {
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    // initiate refernce variables
    transformed_plan.clear();

    try
    {
      if (!global_plan.size() > 0)
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      tf::StampedTransform transform;
      // 可以获得两个坐标系之间转换的关系transform，包括旋转和平移
      // 从plan_pose到global_frame（也就是odom）
      tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, 
          plan_pose.header.frame_id, transform);

      //let's get the pose of the robot in the frame of the plan
      // 返回机器人的坐标转化到plan_pose所对应的坐标系(我感觉应该是map)
      tf::Stamped<tf::Pose> robot_pose;
      robot_pose.setIdentity();
      // 返回机器人的坐标系，也就是footprint
      robot_pose.frame_id_ = costmap.getBaseFrameID();
      robot_pose.stamp_ = ros::Time();
      // 参数名字的定义对功能的说明还是很明显的，target_frame就是你要把源pose转换成哪个frame上的pose。
      // 假如你的源pose的frame_id是"odom"，你想转到"map"上，那么target_frame写成“map”就可以了。
      // stamped_in就是源pose，而stamped_out就是目标数据了，也就是转换完成的数据。
      // 需要注意的是，从参数上来看，转换时是不需要指定源frame_id的，这是因为它已经包含在了stamped_in中，
      // 换句话说，就是这个函数一个隐含的使用条件是，stamped_in中必须指明源pose属于哪个frame。
      tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);

      //we'll keep points on the plan that are within the window that we're looking at
      // 得到局部地图的最大值
      double dist_threshold = std::max(
          costmap.getCostmap()->getSizeInMetersX() / 2.0,
          costmap.getCostmap()->getSizeInMetersY() / 2.0
          );

      unsigned int i = 0;
      // 这个就是地图最长边一半的平方
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = DBL_MAX;

      // --- start - modification w.r.t. base_local_planner
      // initiate start_end_count
      std::vector<int> start_end_count;
      // 就是给两个0,这两个数是有具体含义的
      // 第一个数就是局部地图中刚进去的那个全局路径点
      // 第二个是局部地图中将要出去的全局路径点
      start_end_count.assign(2, 0);

      // we know only one direction and that is forward! - initiate search with previous start_end_counts
      // this is neccesserry to work with the sampling based planners - path may severall time enter and leave moving window
      // std::vector<int> start_end_counts (2, (int) global_plan_.size());
      ROS_ASSERT( (start_end_counts.at(0) > 0) && (start_end_counts.at(0) <= (int) global_plan.size()) );
      // 我们只知道一个方向，那就是前进方向！ 
      // -使用以前的start_end_counts开始搜索，
      // 这对于与基于采样的计划者一起工作是必不可少的-路径可能需要数次进入和离开移动窗口
      // 这个一开始是0
      i = (unsigned int) global_plan.size() - (unsigned int) start_end_counts.at(0);
      // --- end - modification w.r.t. base_local_planner

      //we need to loop to a point on the plan that is within a certain distance of the robot
      //我们需要循环到计划中距机器人一定距离内的一点
      while(i < (unsigned int)global_plan.size() && sq_dist > sq_dist_threshold)
      {
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        // --- start - modification w.r.t. base_local_planner
        // not yet in reach - get next frame
        // 尚未触及-获取下一帧
        if( sq_dist > sq_dist_threshold )
          ++i;
        else
          // set counter for start of transformed intervall - from back as beginning of plan might be prunned
          //为转换的intervall的开始设置计数器-从后面开始，因为计划的开始可能会被修剪
          start_end_count.at(0) = (int) (((unsigned int) global_plan.size()) - i);
        // --- end - modification w.r.t. base_local_planner
      }


      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      //now we'll transform until points are outside of our distance threshold
      while(i < (unsigned int)global_plan.size() && sq_dist < sq_dist_threshold)
      {
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        const geometry_msgs::PoseStamped& pose = global_plan[i];
        poseStampedMsgToTF(pose, tf_pose);
        // 进行坐标转化，转化到局部路径规划中
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        // --- start - modification w.r.t. base_local_planner
        // set counter for end of transformed intervall - from back as beginning of plan might be prunned
        start_end_count.at(1) = int (((unsigned int) global_plan.size()) - i);
        // --- end - modification w.r.t. base_local_planner

        ++i;
      }

      // --- start - modification w.r.t. base_local_planner
      // write to reference variable
      // 重新写参考变量
      start_end_counts = start_end_count;
      // --- end - modification w.r.t. base_local_planner
    }
    catch(tf::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex)
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex)
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap) {
    std::vector<geometry_msgs::Point> footprint(costmap.getRobotFootprint());
    double max_distance_sqr = 0;
    for (size_t i = 0; i < footprint.size(); ++i) {
      geometry_msgs::Point& p = footprint[i];
      double distance_sqr = p.x*p.x + p.y*p.y;
      if (distance_sqr > max_distance_sqr) {
        max_distance_sqr = distance_sqr;
      }
    }
    return sqrt(max_distance_sqr);
  }


}
