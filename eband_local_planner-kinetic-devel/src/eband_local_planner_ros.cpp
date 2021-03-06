// 第三部分是move_base局部规划器和本局部规划器具体实现的中间转换接口；

#include <eband_local_planner/eband_local_planner_ros.h>

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>


// register this planner as a BaseGlobalPlanner plugin
// (see http://www.ros.org/wiki/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
PLUGINLIB_EXPORT_CLASS(eband_local_planner::EBandPlannerROS, nav_core::BaseLocalPlanner)


  namespace eband_local_planner{

    EBandPlannerROS::EBandPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}


    EBandPlannerROS::EBandPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
      : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
      // initialize planner
      initialize(name, tf, costmap_ros);
    }


    EBandPlannerROS::~EBandPlannerROS() {}

    // tc_->initialize(blp_loader_.getName(local_planner_name_), &tf_, controller_costmap_ros_);
    void EBandPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
      // check if the plugin is already initialized
      if(!initialized_)
      {
        // copy adress of costmap and Transform Listener (handed over from move_base)
        // 复制费用地图和tf转化树
        costmap_ros_ = costmap_ros;
        tf_ = tf;


        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle pn("~/" + name);

        // advertise topics (adapted global plan and predicted local trajectory)
        // 发布话题
        g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);


        // subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
        ros::NodeHandle gn;
        // 订阅里程计信息
        odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&EBandPlannerROS::odomCallback, this, _1));


        // create the actual planner that we'll use. Pass Name of plugin and pointer to global costmap to it.
        // (configuration is done via parameter server)
        // 创建我们将使用的实际计划程序。将插件名称和指向全局成本图的指针传递给它。 （通过参数服务器完成配置）
        eband_ = boost::shared_ptr<EBandPlanner>(new EBandPlanner(name, costmap_ros_));

        // create the according controller
        // 创建产生速度的控制器
        eband_trj_ctrl_ = boost::shared_ptr<EBandTrajectoryCtrl>(new EBandTrajectoryCtrl(name, costmap_ros_));

        // create object for visualization
        // 可视化对象
        eband_visual_ = boost::shared_ptr<EBandVisualization>(new EBandVisualization);

        // pass visualization object to elastic band
        eband_->setVisualization(eband_visual_);

        // pass visualization object to controller
        eband_trj_ctrl_->setVisualization(eband_visual_);

        // initialize visualization - set node handle and pointer to costmap
        eband_visual_->initialize(pn, costmap_ros);

        // create and initialize dynamic reconfigure
        // 动态参数设置
        drs_.reset(new drs(pn));
        drs::CallbackType cb = boost::bind(&EBandPlannerROS::reconfigureCallback, this, _1, _2);
        drs_->setCallback(cb);

        // set initialized flag
        initialized_ = true;

        // this is only here to make this process visible in the rxlogger right from the start
        ROS_DEBUG("Elastic Band plugin initialized.");
      }
      else
      {
        ROS_WARN("This planner has already been initialized, doing nothing.");
      }
    }


    void EBandPlannerROS::reconfigureCallback(EBandPlannerConfig& config,
      uint32_t level)
    {
      xy_goal_tolerance_ = config.xy_goal_tolerance;
      yaw_goal_tolerance_ = config.yaw_goal_tolerance;
      rot_stopped_vel_ = config.rot_stopped_vel;
      trans_stopped_vel_ = config.trans_stopped_vel;

      if (eband_)
        eband_->reconfigure(config);
      else
        ROS_ERROR("Reconfigure CB called before eband planner initialization");

      if (eband_trj_ctrl_)
        eband_trj_ctrl_->reconfigure(config);
      else
        ROS_ERROR("Reconfigure CB called before trajectory controller initialization!");

      if (eband_visual_)
        eband_visual_->reconfigure(config);
      else
        ROS_ERROR("Reconfigure CB called before eband visualizer initialization");
    }


    // set global plan to wrapper and pass it to eband
    // 通过使用对象eband_的成员函数（方法）setPlan，将eband_local_planner_ros得到的transformed_plan_传递到eband_local_planner 中
    // 并且对弹性带进行优化,优化结果放在elastic_band_
    bool EBandPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {

      // check if plugin initialized
      if(!initialized_)
      {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }

      //reset the global plan
      global_plan_.clear();
      global_plan_ = orig_global_plan;

      // transform global plan to the map frame we are working in
      // this also cuts the plan off (reduces it to local window)
      // 将全局计划转换为我们正在使用的地图框架，这也会切断该计划（将其减少到本地窗口）
      // 生成两个数(都是全局点的总数)
      std::vector<int> start_end_counts (2, (int) global_plan_.size()); // counts from the end() of the plan
      // 这个函数的作用就是实时的存储局部地图中的全局点(放在transformed_plan),并且start_end_counts存储的是局部地图中开始的点和将要出去的点
      // transformed_plan中点的frame是odom
      if(!eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, start_end_counts))
      {
        // if plan could not be tranformed abort control and local planning
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
      }

      // also check if there really is a plan
      if(transformed_plan_.empty())
      {
        // if global plan passed in is empty... we won't do anything
        ROS_WARN("Transformed plan is empty. Aborting local planner!");
        return false;
      }

      // set plan - as this is fresh from the global planner robot pose should be identical to start frame
      //设置计划-因为这是从全局计划程序中重新获得的，机器人的姿势应该与开始帧相同
      // global_plan这个就是处理过的路径，记住这个是局部地图中的点，也就是frame是odom
      // 然后根据global_plan进行弹性带的生成，点就是路径点或者扩充删减的点，
      // 半径就是就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
      // 这个函数的作用就是得到处理后的弹性到，放在elastic_band_
      if(!eband_->setPlan(transformed_plan_))
      {
        // We've had some difficulty where the global planner keeps returning a valid path that runs through an obstacle
        // in the local costmap. See issue #5. Here we clear the local costmap and try one more time.
        // 全局规划不断返回一条通过本地成本图中障碍物的有效路径时，我们遇到了一些困难。请参阅问题5。在这里，我们清除本地成本图，然后再尝试一次。
        costmap_ros_->resetLayers();
        // global_plan这个就是处理过的路径，记住这个是局部地图中的点，也就是frame是odom
        // 然后根据global_plan进行弹性带的生成，点就是路径点或者扩充删减的点，
        // 半径就是就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
        // 这个函数的作用就是得到处理后的弹性到，放在elastic_band_
        if (!eband_->setPlan(transformed_plan_)) {
          ROS_ERROR("Setting plan to Elastic Band method failed!");
          return false;
        }
      }
      ROS_DEBUG("Global plan set to elastic band for optimization");

      // plan transformed and set to elastic band successfully - set counters to global variable
      // 计划已转换并成功设置为弹性带-将计数器设置为全局变量
      // start_end_counts存储的是局部地图中开始的点和将要出去的点
      plan_start_end_counter_ = start_end_counts;

      // let eband refine the plan before starting continuous operation (to smooth sampling based plans)
      // 更新位置，对得到的band进行修正（对点进行增加或者删除），进行迭代优化
      eband_->optimizeBand();


      // display result
      // 这个就是展示
      std::vector<eband_local_planner::Bubble> current_band;
      if(eband_->getBand(current_band))
        eband_visual_->publishBand("bubbles", current_band);

      // set goal as not reached
      goal_reached_ = false;

      return true;
    }


    bool EBandPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
      // check if plugin initialized
      if(!initialized_)
      {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }

      // instantiate local variables
      //std::vector<geometry_msgs::PoseStamped> local_plan;
      tf::Stamped<tf::Pose> global_pose;
      geometry_msgs::PoseStamped global_pose_msg;
      std::vector<geometry_msgs::PoseStamped> tmp_plan;

      // get curent robot position
      ROS_DEBUG("Reading current robot Position from costmap and appending it to elastic band.");
      // 得到机器人的位置(局部坐标系)
      if(!costmap_ros_->getRobotPose(global_pose))
      {
        ROS_WARN("Could not retrieve up to date robot pose from costmap for local planning.");
        return false;
      }

      // convert robot pose to frame in plan and set position in band at which to append
      tf::poseStampedTFToMsg(global_pose, global_pose_msg);
      tmp_plan.assign(1, global_pose_msg);
      eband_local_planner::AddAtPosition add_frames_at = add_front;

      // set it to elastic band and let eband connect it
      // 将其设置为松紧带，然后让eband连接
      // tmp_plan只保存着机器人当前位置坐标
      // 这个函数的作用就是将elastic_band_进行处理,机器人气泡以前的气泡全部删除
      if(!eband_->addFrames(tmp_plan, add_frames_at))
      {
        ROS_WARN("Could not connect robot pose to existing elastic band.");
        return false;
      }

      // get additional path-frames which are now in moving window
      ROS_DEBUG("Checking for new path frames in moving window");
      // plan_start_end_counter_存储的是局部地图中开始的点和将要出去的点
      std::vector<int> plan_start_end_counter = plan_start_end_counter_;
      std::vector<geometry_msgs::PoseStamped> append_transformed_plan;
      // transform global plan to the map frame we are working in - careful this also cuts the plan off (reduces it to local window)
      // 将全局计划转换为我们正在使用的地图框架-小心，这也会切断计划（将其减少到本地窗口）
      // 这个函数的作用就是实时的存储局部地图中的全局点(放在transformed_plan),并且plan_start_end_counter存储的是局部地图中开始的点和将要出去的点
      if(!eband_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan_, plan_start_end_counter))
      {
        // if plan could not be tranformed abort control and local planning
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
      }

      // also check if there really is a plan
      if(transformed_plan_.empty())
      {
        // if global plan passed in is empty... we won't do anything
        ROS_WARN("Transformed plan is empty. Aborting local planner!");
        return false;
      }

      ROS_DEBUG("Retrieved start-end-counts are: (%d, %d)", plan_start_end_counter.at(0), plan_start_end_counter.at(1));
      ROS_DEBUG("Current start-end-counts are: (%d, %d)", plan_start_end_counter_.at(0), plan_start_end_counter_.at(1));

      // identify new frames - if there are any
      append_transformed_plan.clear();
      // did last transformed plan end futher away from end of complete plan than this transformed plan?
      // 上一个转变后的计划比这个转变后的计划结束得更远吗
      // plan_start_end_counter_.at(1)为上一时刻处理地图时候得到的出去局部地图的点
      // plan_start_end_counter.at(1)为本时刻处理地图时候得到的出去局部地图的点
      // 这个if就是判断机器人是在前进还是后退,如果是在后退,那么就把屁股后面的点删除
      // 最后处理的结果还是在elastic_band_
        if(plan_start_end_counter_.at(1) > plan_start_end_counter.at(1)) // counting from the back (as start might be pruned)从后面开始计数(开始可能被修剪)
        {
          // new frames in moving window
          if(plan_start_end_counter_.at(1) > plan_start_end_counter.at(0)) // counting from the back (as start might be pruned)从后面开始计数(开始可能被修剪)
          {
            // append everything
            append_transformed_plan = transformed_plan_;
          }
          else
          {
            // append only the new portion of the plan
            int discarded_frames = plan_start_end_counter.at(0) - plan_start_end_counter_.at(1);
            ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 >= transformed_plan_.begin());
            ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 < transformed_plan_.end());
            append_transformed_plan.assign(transformed_plan_.begin() + discarded_frames + 1, transformed_plan_.end());
          }

          // set it to elastic band and let eband connect it
          ROS_DEBUG("Adding %d new frames to current band", (int) append_transformed_plan.size());
          add_frames_at = add_back;
          // 这个说明是在后退
          // 这个函数的作用就是将elastic_band_进行处理,机器人气泡以前的气泡全部删除(可能是前进,也可能是后退)
          // 无论传进去的plan_to_add是什么,最后处理的变量只是elastic_band_
          if(eband_->addFrames(append_transformed_plan, add_back))
          {
            // appended frames succesfully to global plan - set new start-end counts
            ROS_DEBUG("Sucessfully added frames to band");
            plan_start_end_counter_ = plan_start_end_counter;
          }
          else {
            ROS_WARN("Failed to add frames to existing band");
            return false;
          }
        }
        else
          ROS_DEBUG("Nothing to add");

      // update Elastic Band (react on obstacle from costmap, ...)
      ROS_DEBUG("Calling optimization method for elastic band");
      std::vector<eband_local_planner::Bubble> current_band;
      // 更新位置，对得到的band进行修正（对点进行增加或者删除），进行迭代优化
      if(!eband_->optimizeBand())
      {
        ROS_WARN("Optimization failed - Band invalid - No controls availlable");
        // display current band
        if(eband_->getBand(current_band))
          eband_visual_->publishBand("bubbles", current_band);
        return false;
      }

      // get current Elastic Band and
      // 函数的作用是输出弹性带,放在elastic_band(也就是将全局变量elastic_band_赋值给elastic_band)
      eband_->getBand(current_band);
      // set it to the controller
      // 把输入变量current_band设置到自己的全局变量中
      if(!eband_trj_ctrl_->setBand(current_band))
      {
        ROS_DEBUG("Failed to to set current band to Trajectory Controller");
        return false;
      }

      // set Odometry to controller
      // base_odom_这里就只是使用了速度,没有使用位移
      // 更新速度变量,因为base_odom_只有速度
      if(!eband_trj_ctrl_->setOdometry(base_odom_))
      {
        ROS_DEBUG("Failed to to set current odometry to Trajectory Controller");
        return false;
      }

      // get resulting commands from the controller
      geometry_msgs::Twist cmd_twist;
      // 里面会调用差速模型
      // getTwistDifferentialDrive函数里面会执行三步(三步都是相斥的)
        // 第一步,判断是否到达xy目标
        // 第二步,判断是否第一个气泡和第二个差距很大
        // 第三步,正常计算速度
      if(!eband_trj_ctrl_->getTwist(cmd_twist, goal_reached_))
      {
        ROS_DEBUG("Failed to calculate Twist from band in Trajectory Controller");
        return false;
      }


      // set retrieved commands to reference variable
      ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd_twist.linear.x, cmd_twist.linear.y, cmd_twist.angular.z);
      cmd_vel = cmd_twist;


      // publish plan
      // 显示
      std::vector<geometry_msgs::PoseStamped> refined_plan;
      if(eband_->getPlan(refined_plan))
        // TODO publish local and current gloabl plan
        base_local_planner::publishPlan(refined_plan, g_plan_pub_);
      //base_local_planner::publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

      // display current band
      if(eband_->getBand(current_band))
        eband_visual_->publishBand("bubbles", current_band);

      return true;
    }


    bool EBandPlannerROS::isGoalReached()
    {
      // check if plugin initialized
      if(!initialized_)
      {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
      }

      return goal_reached_;

      // // copy odometry information to local variable
      // nav_msgs::Odometry base_odom;
      // {
      // 	// make sure we do not read new date from topic right at the moment
      // 	boost::mutex::scoped_lock lock(odom_mutex_);
      // 	base_odom = base_odom_;
      // }

      // tf::Stamped<tf::Pose> global_pose;
      // costmap_ros_->getRobotPose(global_pose);

      // // analogous to dwa_planner the actual check uses the routine implemented in trajectory_planner (trajectory rollout)
      // bool is_goal_reached = base_local_planner::isGoalReached(
      //     *tf_, global_plan_, *(costmap_ros_->getCostmap()),
      //     costmap_ros_->getGlobalFrameID(), global_pose, base_odom,
      // 		rot_stopped_vel_, trans_stopped_vel_, xy_goal_tolerance_,
      //     yaw_goal_tolerance_
      // );

      // return is_goal_reached;

    }


    void EBandPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      // lock Callback while reading data from topic
      boost::mutex::scoped_lock lock(odom_mutex_);

      // get odometry and write it to member variable (we assume that the odometry is published in the frame of the base)
      // 这里就只是使用了速度,没有使用位移
      base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
      base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
      base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    }


  }


