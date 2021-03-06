// 第二部分负责具体的局部规划的具体实现，是核心代码；

#include <eband_local_planner/eband_local_planner.h>


namespace eband_local_planner{


  EBandPlanner::EBandPlanner() : costmap_ros_(NULL), initialized_(false) {}


  EBandPlanner::EBandPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false)
  {
    // initialize planner
    initialize(name, costmap_ros);
  }


  EBandPlanner::~EBandPlanner()
  {
    delete world_model_;
  }


  void EBandPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    // check if the plugin is already initialized
    if(!initialized_)
    {
      // copy adress of costmap (handed over from move_base via eband wrapper)
      costmap_ros_ = costmap_ros;

      // get a pointer to the underlying costmap
      costmap_ = costmap_ros_->getCostmap();

      // create world model from costmap
      world_model_ = new base_local_planner::CostmapModel(*costmap_);

      // get footprint of the robot
      footprint_spec_ = costmap_ros_->getRobotFootprint();

      // create Node Handle with name of plugin (as used in move_base for loading)
      ros::NodeHandle pn("~/" + name);

      // clean up band
      elastic_band_.clear();

      // set initialized flag
      initialized_ = true;

      // set flag whether visualization availlable to false by default
      visualization_ = false;
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  void EBandPlanner::reconfigure(
    eband_local_planner::EBandPlannerConfig& config)
  {
    // connectivity checking
    min_bubble_overlap_ = config.eband_min_relative_overlap;

    // bubble geometric bounds
    tiny_bubble_distance_ = config.eband_tiny_bubble_distance;
    tiny_bubble_expansion_ = config.eband_tiny_bubble_expansion;

    // optimization - force calculation
    internal_force_gain_ = config.eband_internal_force_gain;
    external_force_gain_ = config.eband_external_force_gain;
    num_optim_iterations_ = config.num_iterations_eband_optimization;

    // recursive approximation of bubble equilibrium position based
    max_recursion_depth_approx_equi_ = config.eband_equilibrium_approx_max_recursion_depth;
    equilibrium_relative_overshoot_ = config.eband_equilibrium_relative_overshoot;
    significant_force_ = config.eband_significant_force_lower_bound;

    // use this parameter if a different weight is supplied to the costmap in dyn reconfigure
    // Costmap权重因子，用于计算到障碍物的距离。
    costmap_weight_ = config.costmap_weight;
  }

  void EBandPlanner::setVisualization(boost::shared_ptr<EBandVisualization> eband_visual)
  {
    eband_visual_ = eband_visual;

    visualization_ = true;
  }

  // eband_->setPlan(transformed_plan_)
  // global_plan这个就是处理过的路径，记住这个是局部地图中的点，也就是frame是odom
  // 然后根据global_plan进行弹性带的生成，点就是路径点或者扩充删减的点，
  // 半径就是就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
  // 这个函数的作用就是得到处理后的弹性到，放在elastic_band_
  bool EBandPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }


    // check if plan valid (minimum 2 frames)
    if(global_plan.size() < 2)
    {
      ROS_ERROR("Attempt to pass empty path to optimization. Valid path needs to have at least 2 Frames. This one has %d.", ((int) global_plan.size()) );
      return false;
    }
    // copy plan to local member variable
    global_plan_ = global_plan;


    // check whether plan and costmap are in the same frame
    if(global_plan.front().header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("Elastic Band expects plan for optimization in the %s frame, the plan was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), global_plan.front().header.frame_id.c_str());
      return false;
    }


    // convert frames in path into bubbles in band -> sets center of bubbles and calculates expansion
    // 转换帧路径到皮筋中的气泡形式，也就是设置气泡中心和计算膨胀
    ROS_DEBUG("Converting Plan to Band");
    // 进行初步处理，得到plan中各个点对应的半径,放在elastic_band_
    if(!convertPlanToBand(global_plan_, elastic_band_))
    {
      ROS_WARN("Conversion from plan to elastic band failed. Plan probably not collision free. Plan not set for optimization");
      // TODO try to do local repairs of band
      return false;
    }


    // close gaps and remove redundant bubbles
    ROS_DEBUG("Refining Band");
    // 对得到的band进行修正（对点进行增加或者删除）
    if(!refineBand(elastic_band_))
    {
      ROS_WARN("Band is broken. Could not close gaps in converted path. Path not set. Global replanning needed");
      return false;
    }


    ROS_DEBUG("Refinement done - Band set.");
    return true;
  }


  bool EBandPlanner::getPlan(std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // check if there is a band
    if(elastic_band_.empty())
    {
      ROS_WARN("Band is empty. There was no path successfully set so far.");
      return false;
    }

    // convert band to plan
    if(!convertBandToPlan(global_plan, elastic_band_))
    {
      ROS_WARN("Conversion from Elastic Band to path failed.");
      return false;
    }

    return true;
  }

  // 函数的作用是输出弹性带,放在elastic_band(也就是将全局变量elastic_band_赋值给elastic_band)
  bool EBandPlanner::getBand(std::vector<Bubble>& elastic_band)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    elastic_band = elastic_band_;

    // check if there is a band
    if(elastic_band_.empty())
    {
      ROS_WARN("Band is empty.");
      return false;
    }

    return true;
  }

  // eband_->addFrames(tmp_plan, add_frames_at)
  // tmp_plan只保存着机器人当前位置坐标
  // 这个函数的作用就是将elastic_band_进行处理,机器人气泡以前的气泡全部删除(可能是前进,也可能是后退)
  // 无论传进去的plan_to_add是什么,最后处理的变量只是elastic_band_
  bool EBandPlanner::addFrames(const std::vector<geometry_msgs::PoseStamped>& plan_to_add, const AddAtPosition& add_frames_at)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // check that there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
    if(elastic_band_.size() < 1)
    {
      ROS_WARN("Attempt to connect path to empty band. path not connected. Use SetPath instead");
      return false;
    }

    //check that plan which shall be added is not empty
    if(plan_to_add.empty())
    {
      ROS_WARN("Attempt to connect empty path to band. Nothing to do here.");
      return false;
    }

    // check whether plan and costmap are in the same frame
    if(plan_to_add.at(0).header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("Elastic Band expects robot pose for optimization in the %s frame, the pose was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), plan_to_add.at(0).header.frame_id.c_str());
      return false;
    }


    // convert plan to band
    std::vector<Bubble> band_to_add;
    // 进行初步处理，得到plan中各个点对应的半径
    if(!convertPlanToBand(plan_to_add, band_to_add))
    {
      ROS_DEBUG("Conversion from plan to elastic band failed. Plan not appended");
      // TODO try to do local repairs of band
      return false;
    }


    // connect frames to existing band
    // 连接框架到存在的弹性带
    ROS_DEBUG("Checking for connections between current band and new bubbles");
    bool connected = false;
    int bubble_connect = -1;
    if(add_frames_at == add_front)
    {
      // add frames at the front of the current band
      // - for instance to connect band and current robot position
      // 这个应该是判断机器人的坐标值在其中一个气泡里面
      for(int i = ((int) elastic_band_.size() - 1); i >= 0; i--)
      {
        // cycle over bubbles from End - connect to bubble furthest away but overlapping
        // 判断两个气泡重合度是否满足，min_bubble_overlap_这个参数说如果重合度不满足也是不行的
        if(checkOverlap(band_to_add.back(), elastic_band_.at(i)))
        {
          bubble_connect = i;
          connected = true;
          break;
        }
      }
    }
    else
    {
      // add frames at the end of the current band
      // - for instance to connect new frames entering the moving window
      for(int i = 0; i < ((int) elastic_band_.size() - 1); i++)
      {
        // cycle over bubbles from Start - connect to bubble furthest away but overlapping
        if(checkOverlap(band_to_add.front(), elastic_band_.at(i)))
        {
          // 与机器人当前位置想联通的气泡
          bubble_connect = i;
          connected = true;
          break;
        }
      }
    }

    // intanstiate local copy of band
    std::vector<Bubble> tmp_band;
    std::vector<Bubble>::iterator tmp_iter1, tmp_iter2;
    // copy new frames to tmp_band
    // 将区间[first,last)的元素赋值到当前的vector容器中
    // 应该就是机器人位置所在的气泡
    tmp_band.assign(band_to_add.begin(), band_to_add.end());

    if(connected)
    {
      ROS_DEBUG("Connections found - composing new band by connecting new frames to bubble %d", bubble_connect);
      if(add_frames_at == add_front)
      {
        // compose new vector by appending elastic_band to new frames
        // 通过将elastic_band预先附加到新帧来组成新矢量
        tmp_iter1 = elastic_band_.begin() + bubble_connect;
        ROS_ASSERT( (tmp_iter1 >= elastic_band_.begin()) && (tmp_iter1 < elastic_band_.end()) );
        // 在tmp_band.end()也就是tmp_band尾部插入tmp_iter1个elastic_band_.end()
        // 就是将机器人气泡 再加上上弹性带后面的气泡
        tmp_band.insert(tmp_band.end(), tmp_iter1, elastic_band_.end());
      }
      else
      {
        // compose new vector by pre-appending elastic_band to new frames
        tmp_iter1 = elastic_band_.begin() + bubble_connect + 1; // +1 - as insert only appends [start, end)
        ROS_ASSERT( (tmp_iter1 > elastic_band_.begin()) && (tmp_iter1 <= elastic_band_.end()) );
        tmp_band.insert(tmp_band.begin(), elastic_band_.begin(), tmp_iter1);
      }

      // done
      elastic_band_ = tmp_band;
      return true;
    }

    // otherwise, we need to do some more work - add complete band to tmp_band
    ROS_DEBUG("No direct connection found - Composing tmp band and trying to fill gap");
    if(add_frames_at == add_front)
    {
      // compose new vector by appending elastic_band to new frames
      tmp_band.insert(tmp_band.end(), elastic_band_.begin(), elastic_band_.end());
      // and get iterators to connecting bubbles
      tmp_iter1 = tmp_band.begin() + ((int) band_to_add.size()) - 1;
      tmp_iter2 = tmp_iter1 + 1;
    }
    else
    {
      // compose new vector by pre-appending elastic_band to new frames
      tmp_band.insert(tmp_band.begin(), elastic_band_.begin(), elastic_band_.end());
      // and get iterators to connecting bubbles
      tmp_iter1 = tmp_band.begin() + ((int) elastic_band_.size()) - 1;
      tmp_iter2 = tmp_iter1 + 1;
    }

    // just in case
    ROS_ASSERT( tmp_iter1 >= tmp_band.begin() );
    ROS_ASSERT( tmp_iter2 < tmp_band.end() );
    ROS_ASSERT( tmp_iter1 < tmp_iter2 );
    if(!fillGap(tmp_band, tmp_iter1, tmp_iter2))
    {
      // we could not connect band and robot at its current position
      ROS_DEBUG("Could not connect robot pose to band - Failed to fill gap.");
      return false;
    }

    // otherwise - done
    elastic_band_ = tmp_band;

    return true;
  }

  // 更新位置，对得到的band进行修正（对点进行增加或者删除），进行迭代优化
  bool EBandPlanner::optimizeBand()
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // check if there is a band
    if(elastic_band_.empty())
    {
      ROS_ERROR("Band is empty. Probably Band has not been set yet");
      return false;
    }

    // call optimization with member elastic_band_
    ROS_DEBUG("Starting optimization of band");
    // setPlan函数已经进行了处理，就是得到处理后的弹性带，放在elastic_band_
    // 更新位置，对得到的band进行修正（对点进行增加或者删除），进行迭代优化
    if(!optimizeBand(elastic_band_))
    {
      ROS_DEBUG("Aborting Optimization. Changes discarded.");
      return false;
    }

    ROS_DEBUG("Elastic Band - Optimization successfull!");
    return true;
  }

  // 这个band变量就是进行setPlan函数进行处理后得到的
  // 更新位置，对得到的band进行修正（对点进行增加或者删除），进行迭代优化
  bool EBandPlanner::optimizeBand(std::vector<Bubble>& band)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // check whether band and costmap are in the same frame
    if(band.front().center.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("Elastic Band expects plan for optimization in the %s frame, the plan was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), band.front().center.header.frame_id.c_str());
      return false;
    }

    double distance;
    for(int i = 0; i < ((int) band.size()); i++)
    {
      // update Size of Bubbles in band by calculating Dist to nearest Obstacle [depends kinematic, environment]
      //通过计算距离到最近的障碍物来更新带中气泡的大小[取决于运动学，环境]
      if(!calcObstacleKinematicDistance(band.at(i).center.pose, distance))
      {
        ROS_DEBUG("Optimization (Elastic Band) - Calculation of Distance failed. Frame %d of %d Probably outside map coordinates.",
            i, ((int) band.size()) );
        return false;
      }

      if(distance == 0.0)
      {
        // frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
        ROS_DEBUG("Optimization (Elastic Band) - Calculation of Distance failed. Frame %d of %d in collision. Plan invalid. Trying to refine band.",
            i, ((int) band.size()) );
        // TODO if frame in collision try to repair band instead of aborting everything
        return false;
      }

      band.at(i).expansion = distance;
    }

    // close gaps and remove redundant bubbles
    // 对得到的band进行修正（对点进行增加或者删除）
    if(!refineBand(band))
    {
      ROS_DEBUG("Elastic Band is broken. Could not close gaps in band. Global replanning needed.");
      return false;
    }

    // get a copy of current (valid) band
    std::vector<Bubble> tmp_band = band;

    // now optimize iteratively (for instance by miminizing the energy-function of the full system)
    //现在进行迭代优化（例如，通过最小化整个系统的能量函数）
    //弹性带优化的迭代次数。
    for(int i = 0; i < num_optim_iterations_; i++)
    {
      ROS_DEBUG("Inside optimization: Cycle no %d", i);

      // calculate forces and apply changes
      // 计算内力，计算外力，去除力的切向部分，确定气泡的位置（重新确定）
      if(!modifyBandArtificialForce(tmp_band))
      {
        ROS_DEBUG("Optimization failed while trying to modify Band.");
        // something went wrong -> discard changes and stop process
        return false;
      }

      // check whether band still valid - refine if neccesarry
      // 对得到的band进行修正（对点进行增加或者删除）
      if(!refineBand(tmp_band))
      {
        ROS_DEBUG("Optimization failed while trying to refine modified band");
        // modified band is not valid anymore -> discard changes and stop process
        return false;
      }
    }

    // copy changes back to band
    band = tmp_band;
    return true;
  }


  // private methods
  // 对得到的band进行修正（对点进行增加或者删除）
  bool EBandPlanner::refineBand(std::vector<Bubble>& band)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // check if band valid (minimum 2 bubbles)
    if(band.size() < 2)
    {
      ROS_WARN("Attempt to convert empty band to plan. Valid band needs to have at least 2 Frames. This one has %d.", ((int) band.size()) );
      return false;
    }

    // instantiate local variables
    bool success;
    std::vector<Bubble> tmp_band;
    std::vector<Bubble>::iterator start_iter, end_iter;

    // remove redundant Bubbles and fill gabs recursively
    tmp_band = band;
    // 临时地图的开始点
    start_iter = tmp_band.begin();
    end_iter = (tmp_band.end() - 1); // -1 because .end() points "past the end"!
    // 这个函数的作用就是采用递归的方法进行对路径点的处理（删除和插入）
    success = removeAndFill(tmp_band, start_iter, end_iter);

    if(!success)
      ROS_DEBUG("Band is broken. Could not close gaps.");
    else
    {
      #ifdef DEBUG_EBAND_
            ROS_DEBUG("Recursive filling and removing DONE");
      #endif
      band = tmp_band;
    }

    return success;
  }

  // 这个函数的作用就是采用递归的方法进行对路径点的处理（删除和插入）
  bool EBandPlanner::removeAndFill(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,std::vector<Bubble>::iterator& end_iter)
  {
    // instantiate local variables
    bool overlap;
    std::vector<Bubble>::iterator tmp_iter;
    int mid_int, diff_int;

    #ifdef DEBUG_EBAND_
        int debug_dist_start, debug_dist_iters;
        debug_dist_start = std::distance(band.begin(), start_iter);
        debug_dist_iters = std::distance(start_iter, end_iter);
        ROS_DEBUG("Refining Recursive - Check if Bubbles %d and %d overlapp. Total size of band %d.", debug_dist_start, (debug_dist_start + debug_dist_iters), ((int) band.size()) );
    #endif

    // check that iterators are still valid
    ROS_ASSERT( start_iter >= band.begin() );
    ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT( start_iter < end_iter );

    // check whether start and end bubbles of this intervall overlap
    // 检查此时间间隔的开始气泡和结束气泡是否重叠
    // 判断两个气泡重合度是否满足，min_bubble_overlap_这个参数说如果重合度不满足也是不行的
    overlap = checkOverlap(*start_iter, *end_iter);
    // 说明首末位置很近
    if(overlap)
    {

      #ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles overlapp, check for redundancies");
      #endif

      // if there are bubbles between start and end of intervall remove them (they are redundant as start and end of intervall do overlap)
      // 如果在路径的开始和结束之间有气泡，则将其删除（它们是多余的，因为路径的开始和结束就已经重叠了）
      if((start_iter + 1) < end_iter)
      {
        #ifdef DEBUG_EBAND_
                ROS_DEBUG("Refining Recursive - Bubbles overlapp, removing Bubbles %d to %d.", (debug_dist_start + 1), (debug_dist_start + debug_dist_iters -1));
        #endif

        // erase bubbles between start and end (but not start and end themself) and get new iterator to end (old one is invalid)
        tmp_iter = band.erase((start_iter+1), end_iter);

        // write back changed iterator pointing to the end of the intervall
        end_iter = tmp_iter;
      }

      #ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Bubbles overlapp - DONE");
      #endif

      // we are done here (leaf of this branch is reached)
      // 就是直接退出，已经经过处理了
      return true;
    }


    // if bubbles do not overlap -> check whether there are still bubbles between start and end
    // 如果气泡不重叠->检查开始和结束之间是否仍然存在气泡
    if((start_iter + 1) < end_iter)
    {
    #ifdef DEBUG_EBAND_
          ROS_DEBUG("Refining Recursive - Bubbles do not overlapp, go one recursion deeper");
    #endif

      // split remaining sequence of bubbles
      // get distance between start and end iterator for this intervall
      mid_int = std::distance(start_iter, end_iter);
      mid_int = mid_int/2; // division by integer implies floor (round down)

      // now get iterator pointing to the middle (roughly)
      tmp_iter = start_iter + mid_int;
      // and realative position of end_iter to tmp_iter
      diff_int = (int) std::distance(tmp_iter, end_iter);

      // after all this arithmetics - check that iterators are still valid
      ROS_ASSERT( start_iter >= band.begin() );
      ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
      ROS_ASSERT( start_iter < end_iter );


      // and call removeAndFill recursively for the left intervall
      // 这个是递归，真的厉害
      // 这个处理之后end_iter就发生改变，就是错误的迭代器，所以下面要重新赋值
      if(!removeAndFill(band, start_iter, tmp_iter))
      {
        // band is broken in this intervall and could not be fixed
        return false;
      }

      // carefull at this point!!! if we filled in or removed bubbles end_iter is not valid anymore
      // but the relative position towards tmp_iter is still the same and tmp_iter was kept valid in the lower recursion steps
      //此时要小心！！！如果我们填充或删除了气泡，则end_iter不再有效，
      // 但相对于tmp_iter的相对位置仍然相同，并且在较低的递归步骤中tmp​​_iter保持有效
      end_iter = tmp_iter + diff_int;

      // check that iterators are still valid - one more time
      ROS_ASSERT( start_iter >= band.begin() );
      ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
      ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );


      // o.k. we are done with left hand intervall now do the same for the right hand intervall
      // but first get relative position of start and tmp iter
      // 上面是处理完左边，那么现在我们是处理右边
      diff_int = (int) std::distance(start_iter, tmp_iter);
      if(!removeAndFill(band, tmp_iter, end_iter))
      {
        // band is broken in this intervall and could not be fixed
        return false;
      }

      // if we filled in bubbles vector might have been reallocated -> start_iter might be invalid
      // 同样的左边的迭代器发生改变，所以我们要做的就是更改
      start_iter = tmp_iter - diff_int;

      // check that iterators are still valid - almost done
      ROS_ASSERT( start_iter >= band.begin() );
      ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
      ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );


      // we reached the leaf but we are not yet done
      // -> we know that there are no redundant elements in the left intervall taken on its own
      // -> and we know the same holds for the right intervall
      // but the middle bubble itself might be redundant -> check it    
      //我们到达了叶子但是还没有完成
      //->我们知道，左间隔中没有多余的元素
      //->而且我们知道正确的间隔也成立
      //但是中间的气泡本身可能是多余的->检查它
      // 判断两个气泡重合度是否满足，min_bubble_overlap_这个参数说如果重合度不满足也是不行的
      if(checkOverlap(*(tmp_iter-1), *(tmp_iter+1)))
      {
    #ifdef DEBUG_EBAND_
            ROS_DEBUG("Refining Recursive - Removing middle bubble");
    #endif

        // again: get distance between (tmp_iter + 1) and end_iter, (+1 because we will erase tmp_iter itself)
        diff_int = (int) std::distance((tmp_iter + 1), end_iter);

        // remove middle bubble and correct end_iter
        tmp_iter = band.erase(tmp_iter);
        end_iter = tmp_iter + diff_int;
      }

      // check that iterators are still valid - almost almost
      ROS_ASSERT( start_iter >= band.begin() );
      ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
      ROS_ASSERT( start_iter < end_iter );

    #ifdef DEBUG_EBAND_
          ROS_DEBUG("Refining Recursive - Bubbles do not overlapp, go one recursion deeper DONE");
    #endif

      //now we are done with this case
      return true;
    }


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Gap detected, fill recursive");
    #endif

    // last possible case -> bubbles do not overlap AND there are nor bubbles in between -> try to fill gap recursively
    //最后一种可能的情况->气泡不重叠，并且两者之间也没有气泡->尝试递归地填补空白
    // 在两点之间进行插点，然后判断这个点和左右两点是否重合，如果没有在进行递归
    if(!fillGap(band, start_iter, end_iter))
    {
      // band is broken in this intervall and could not be fixed (this should only be called on a leaf, so we put a log_out here;)
      ROS_DEBUG("Failed to fill gap between bubble %d and %d.", (int) distance(band.begin(), start_iter), (int) distance(band.begin(), end_iter) );
      return false;
    }

    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Refining Recursive - Gap detected, fill recursive DONE");
    #endif

    // we could fill the gap (reached leaf of this branch)
    return true;
  }

  // 在两点之间进行插点，然后判断这个点和左右两点是否重合，如果没有在进行递归
  bool EBandPlanner::fillGap(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,std::vector<Bubble>::iterator& end_iter)
  {
    // insert bubbles in the middle between not-overlapping bubbles (e.g. (Dist > Size Bub1) && (Dist > Size Bub2) )
    // repeat until gaps are closed

    // instantiate local variables
    double distance = 0.0;
    Bubble interpolated_bubble;
    geometry_msgs::PoseStamped interpolated_center;
    std::vector<Bubble>::iterator tmp_iter;
    int diff_int, start_num, end_num;

    // make sure this method was called for a valid element in the forces or bubbles vector
    ROS_ASSERT( start_iter >= band.begin() );
    ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT( start_iter < end_iter );


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - interpolate");
    #endif

    // interpolate between bubbles [depends kinematic]
    // 在他们中间插入一个气泡
    if(!interpolateBubbles(start_iter->center, end_iter->center, interpolated_center))
    {
      // interpolation failed (for whatever reason), so return with false
      start_num = std::distance(band.begin(), start_iter);
      end_num = std::distance(band.begin(), end_iter);
      ROS_DEBUG("Interpolation failed while trying to fill gap between bubble %d and %d.", start_num, end_num);
      return false;
    }


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - calc expansion of interpolated bubble");
    #endif

    // calc Size of Bubbles by calculating Dist to nearest Obstacle [depends kinematic, environment]
    // 计算这个新加入的这个点距离障碍物的距离
    // 这个函数的意思就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
    if(!calcObstacleKinematicDistance(interpolated_center.pose, distance))
    {
      // pose probably outside map coordinates
      start_num = std::distance(band.begin(), start_iter);
      end_num = std::distance(band.begin(), end_iter);
      ROS_DEBUG("Calculation of Distance failed for interpolated bubble - failed to fill gap between bubble %d and %d.", start_num, end_num);
      return false;
    }

    if(distance <= tiny_bubble_expansion_)
    {
      // band broken! frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
      start_num = std::distance(band.begin(), start_iter);
      end_num = std::distance(band.begin(), end_iter);
      ROS_DEBUG("Interpolated Bubble in Collision - failed to fill gap between bubble %d and %d.", start_num, end_num);
      // TODO this means only that there is an obstacle on the direct interconnection between the bubbles - think about repair or rescue strategies -
      return false;
    }


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - inserting interpolated bubble at (%f, %f), with expansion %f", interpolated_center.pose.position.x, interpolated_center.pose.position.y, distance);
    #endif

    // insert bubble and assign center and expansion
    interpolated_bubble.center = interpolated_center;
    interpolated_bubble.expansion = distance;
    // insert bubble (vector.insert() inserts elements before given iterator) and get iterator pointing to it
    tmp_iter = band.insert(end_iter, interpolated_bubble);
    // insert is a little bit more tricky than erase, as it may require reallocation of the vector -> start and end iter could be invalid
    start_iter = tmp_iter - 1;
    end_iter = tmp_iter + 1;

    // check that iterators are still valid - just in case :)
    ROS_ASSERT( start_iter >= band.begin() );
    ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - check overlap interpolated bubble and first bubble");
    #endif

    // we have now two intervalls (left and right of inserted bubble) which need to be checked again and filled if neccessary
    if(!checkOverlap(*start_iter, *tmp_iter))
    {

    #ifdef DEBUG_EBAND
          ROS_DEBUG("Fill recursive - gap btw. interpolated and first bubble - fill recursive");
    #endif

      // gap in left intervall -> try to fill
      if(!fillGap(band, start_iter, tmp_iter))
      {
        // band is broken in this intervall and could not be fixed
        return false;
      }
      // bubbles were inserted -> make sure to keep end_iter iterator valid
      end_iter = tmp_iter + 1;
    }

    // check that iterators are still valid - just in case :)
    ROS_ASSERT( start_iter >= band.begin() );
    ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - check overlap interpolated bubble and second bubble");
    #endif

    if(!checkOverlap(*tmp_iter, *end_iter))
    {

    #ifdef DEBUG_EBAND_
          ROS_DEBUG("Fill recursive - gap btw. interpolated and second bubble - fill recursive");
    #endif

      // get distance between start_iter and tmp_iter before filling right intervall (in case of reallocation of vector)
      diff_int = (int) std::distance(start_iter, tmp_iter);

      // gap in left intervall -> try to fill
      if(!fillGap(band, tmp_iter, end_iter))
      {
        // band is broken in this intervall and could not be fixed
        return false;
      }
      // bubbles were inserted -> make sure to keep start_iter iterator valid
      start_iter = tmp_iter - diff_int;
    }

    // check that iterators are still valid - just in case :)
    ROS_ASSERT( start_iter >= band.begin() );
    ROS_ASSERT( end_iter < band.end() ); // "<" because .end() points _behind_ last element of vector
    ROS_ASSERT( (start_iter < tmp_iter) && (tmp_iter < end_iter) );


    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Fill recursive - gap closed");
    #endif

    // bubbles overlap, iterators are kept valid -> done
    return true;
  }


  // optimization
  // 计算内力，计算外力，去除力的切向部分，确定气泡的位置（重新确定）
  bool EBandPlanner::modifyBandArtificialForce(std::vector<Bubble>& band)
  {
    if(band.empty())
    {
      ROS_ERROR("Trying to modify an empty band.");
      return false;
    }

    if(band.size() <= 2)
    {
      // nothing to do here -> we can stop right away
      return true;
    }

    std::vector<geometry_msgs::WrenchStamped> internal_forces, external_forces, forces;
    geometry_msgs::WrenchStamped wrench;

    #ifdef DEBUG_EBAND_
        //publish original band
        if(visualization_)
          eband_visual_->publishBand("bubbles", band);
    #endif

    // init variables to calm down debug warnings
    wrench.header.stamp = ros::Time::now();
    wrench.header.frame_id = band[0].center.header.frame_id;
    wrench.wrench.force.x = 0.0;
    wrench.wrench.force.y = 0.0;
    wrench.wrench.force.z = 0.0;
    wrench.wrench.torque.x = 0.0;
    wrench.wrench.torque.y = 0.0;
    wrench.wrench.torque.z = 0.0;
    // 每个点上都有wrench
    internal_forces.assign(band.size(), wrench);
    external_forces = internal_forces;
    forces = internal_forces;

    // TODO log timigs of planner
    // instantiate variables for timing
    //ros::Time time_stamp1, time_stamp2;
    //ros::Duration duration;
    //time_stamp1 = ros::Time::now();

    // due to refinement band might change its size -> use while loop
    int i = 1;
    bool forward = true; // cycle 1xforwards and 1xbackwards through band
    // 计算内力，计算外力，去除力的切向部分，确定气泡的位置
    while( (i>0) && (i < ((int) band.size() - 1)) )
    {
      ROS_DEBUG("Modifying bubble %d.", i);


      #ifdef DEBUG_EBAND_
            ROS_DEBUG("Calculating internal force for bubble %d.", i);
      #endif
      // 根据论文中得计算公式计算这个点的内部力
      if(!calcInternalForces(i, band, band.at(i), internal_forces.at(i)))
      {
        // calculation of internal forces failed - stopping optimization
        ROS_DEBUG("Calculation of internal forces failed");
        return false;
      }

      #ifdef DEBUG_EBAND_
            if(visualization_)
              // publish internal forces
              eband_visual_->publishForce("internal_forces", i, eband_visual_->blue, internal_forces[i], band[i]);
            // Log out debug info about next step
            ROS_DEBUG("Calculating external force for bubble %d.", i);
      #endif


      //if(!calcExternalForces(i, band, external_forces))
      // 根据论文中的公式进行外部力的计算（只计算二维的）
      if(!calcExternalForces(i, band.at(i), external_forces.at(i)))
      {
        // calculation of External Forces failed - stopping optimization
        ROS_DEBUG("Calculation of external forces failed");
        return false;
      }

      #ifdef DEBUG_EBAND_
            if(visualization_)
              //publish external forces
              eband_visual_->publishForce("external_forces", i, eband_visual_->red, external_forces[i], band[i]);
            // Log out debug info about next step
            ROS_DEBUG("Superposing internal and external forces");
      #endif


      // sum up external and internal forces over all bubbles
      forces.at(i).wrench.force.x = internal_forces.at(i).wrench.force.x + external_forces.at(i).wrench.force.x;
      forces.at(i).wrench.force.y = internal_forces.at(i).wrench.force.y + external_forces.at(i).wrench.force.y;
      forces.at(i).wrench.force.z = internal_forces.at(i).wrench.force.z + external_forces.at(i).wrench.force.z;

      forces.at(i).wrench.torque.x = internal_forces.at(i).wrench.torque.x + external_forces.at(i).wrench.torque.x;
      forces.at(i).wrench.torque.y = internal_forces.at(i).wrench.torque.y + external_forces.at(i).wrench.torque.y;
      forces.at(i).wrench.torque.z = internal_forces.at(i).wrench.torque.z + external_forces.at(i).wrench.torque.z;

    #ifdef DEBUG_EBAND_
          ROS_DEBUG("Superpose forces: (x, y, theta) = (%f, %f, %f)", forces.at(i).wrench.force.x, forces.at(i).wrench.force.y, forces.at(i).wrench.torque.z);
          ROS_DEBUG("Supressing tangential forces");
    #endif
      // 表示在i点的合力forces.at(i)
      // 计算力的切向部分
      // 论文中说，在某些情况下，插入新气泡和去除多余气泡会产生不良的副作用，
      // 气泡可以在一处插入，沿橡皮筋移动，然后在另一处去除。
      // 这个序列可以无限地继续下去，因此松紧带以一种不稳定的方式振荡。
      // 解决这个问题的一种方法是修改施加在橡皮筋上的总力，这样就可以去掉切向分量。
      if(!suppressTangentialForces(i, band, forces.at(i)))
      {
        // suppression of tangential forces failed
        ROS_DEBUG("Supression of tangential forces failed");
        return false;
      }

      #ifdef DEBUG_EBAND_
            if(visualization_)
              //publish resulting forces
              eband_visual_->publishForce("resulting_forces", i, eband_visual_->green, forces[i], band[i]);
      #endif


      ROS_DEBUG("Applying forces to modify band");
      // 施加力以移动气泡并重新计算气泡的膨胀
      // 重新计算位置半径等等
      if(!applyForces(i, band, forces))
      {
        // band invalid
        ROS_DEBUG("Band is invalid - Stopping Modification");
        return false;
      }

      #ifdef DEBUG_EBAND_
        if(visualization_)
        {
          // publish band with changed bubble at resulting position
          eband_visual_->publishBand("bubbles", band);
          ros::Duration(0.01).sleep();
        }
      #endif


      //next bubble
      if(forward)
      {
        i++;
        if(i == ((int) band.size() - 1))
        {
          // reached end of band - start backwards cycle until at start again - then stop
          forward = false;
          i--;
          ROS_DEBUG("Optimization Elastic Band - Forward cycle done, starting backward cycle");
        }
      }
      else
      {
        i--;
      }
    }

    return true;
  }

  // 施加力以移动气泡并重新计算气泡的膨胀
  // 重新计算位置半径等等
  bool EBandPlanner::applyForces(int bubble_num, std::vector<Bubble>& band, std::vector<geometry_msgs::WrenchStamped> forces)
  {
    //cycle over all bubbles except first and last (these are fixed)
    if(band.size() <= 2)
    {
      // nothing to do here -> we can stop right away - no forces calculated
      return true;
    }

    geometry_msgs::Pose2D bubble_pose2D, new_bubble_pose2D;
    geometry_msgs::Pose bubble_pose, new_bubble_pose;
    geometry_msgs::Twist bubble_jump;
    Bubble new_bubble = band.at(bubble_num);
    double distance;


    // move bubble
    bubble_pose = band.at(bubble_num).center.pose;
    PoseToPose2D(bubble_pose, bubble_pose2D);
    // 这个就是论文中的概念
    // move according to bubble_new = bubble_old + alpha*force -> we choose alpha to be the current expansion of the modified bubble
    // 选择alpha为这个气泡的半径
    bubble_jump.linear.x = band.at(bubble_num).expansion*forces.at(bubble_num).wrench.force.x;
    bubble_jump.linear.y = band.at(bubble_num).expansion*forces.at(bubble_num).wrench.force.y;
    bubble_jump.linear.z = 0.0;
    bubble_jump.angular.x = 0.0;
    bubble_jump.angular.y = 0.0;
    bubble_jump.angular.z = band.at(bubble_num).expansion/getCircumscribedRadius(*costmap_ros_) * forces.at(bubble_num).wrench.torque.z;
    bubble_jump.angular.z = angles::normalize_angle(bubble_jump.angular.z);

    // apply changes to calc tmp bubble position
    new_bubble_pose2D.x = bubble_pose2D.x + bubble_jump.linear.x;
    new_bubble_pose2D.y = bubble_pose2D.y + bubble_jump.linear.y;
    new_bubble_pose2D.theta = bubble_pose2D.theta + bubble_jump.angular.z;
    new_bubble_pose2D.theta = angles::normalize_angle(new_bubble_pose2D.theta);

    // apply changes to local copy
    Pose2DToPose(new_bubble_pose, new_bubble_pose2D);
    new_bubble.center.pose = new_bubble_pose;

    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Try moving bubble %d at (%f, %f, %f) by (%f, %f, %f).", bubble_num, bubble_pose2D.x, bubble_pose2D.y, bubble_pose2D.theta,
            bubble_jump.linear.x, bubble_jump.linear.y, bubble_jump.angular.z);
    #endif


    // check validity of moved bubble

    // recalc expansion of bubble -> calc Size of Bubbles by calculating Dist to nearest Obstacle [depends kinematic, environment]
    // 这个函数的意思就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
    if(!calcObstacleKinematicDistance(new_bubble_pose, distance))
    {
      ROS_DEBUG("Calculation of Distance failed. Frame %d of %d Probably outside map. Discarding Changes", bubble_num, ((int) band.size()) );

    #ifdef DEBUG_EBAND_
          if(visualization_)
            eband_visual_->publishBubble("bubble_hypo", bubble_num, eband_visual_->red, new_bubble);
    #endif

      // this bubble must not be changed, but band is still valid -> continue with other bubbles
      return true;
    }

    if(distance <= tiny_bubble_expansion_)
    {
      // frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
      ROS_DEBUG("Calculation of Distance failed. Frame %d of %d in collision. Plan invalid. Discarding Changes", bubble_num, ((int) band.size()) );

    #ifdef DEBUG_EBAND_
          if(visualization_)
            eband_visual_->publishBubble("bubble_hypo", bubble_num, eband_visual_->red, new_bubble);
    #endif

      // this bubble must not be changed, but band is still valid -> continue with other bubbles
      return true;
    }

    // so far o.k. -> assign distance to new bubble
    new_bubble.expansion = distance;


    // check whether step was reasonable

    geometry_msgs::WrenchStamped new_bubble_force = forces.at(bubble_num);

    // check whether we get a valid force calculation here
    // 就是更新力
    if(!getForcesAt(bubble_num, band, new_bubble, new_bubble_force))
    {
      // error during calculation of forces for the new position - discard changes
      ROS_DEBUG("Cannot calculate forces on bubble %d at new position - discarding changes", bubble_num);
      return true;
    }

    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Check for zero-crossings in force on bubble %d", bubble_num);
    #endif

    // check for zero-crossings in the force-vector
    // 检查力矢量中的零交叉
    double checksum_zero, abs_new_force, abs_old_force;

    // project force-vectors onto each other
    // 将力矢量进行映射
    checksum_zero = (new_bubble_force.wrench.force.x * forces.at(bubble_num).wrench.force.x) +
      (new_bubble_force.wrench.force.y * forces.at(bubble_num).wrench.force.y) +
      (new_bubble_force.wrench.torque.z * forces.at(bubble_num).wrench.torque.z);

    // if sign changes and ...
    if(checksum_zero < 0.0)
    {
      ROS_DEBUG("Detected zero-crossings in force on bubble %d. Checking total change in force.", bubble_num);
      // check the absolute values of the two vectors
      abs_new_force = sqrt( (new_bubble_force.wrench.force.x * new_bubble_force.wrench.force.x) +
          (new_bubble_force.wrench.force.y * new_bubble_force.wrench.force.y) +
          (new_bubble_force.wrench.torque.z * new_bubble_force.wrench.torque.z) );
      abs_old_force = sqrt( (forces.at(bubble_num).wrench.force.x * forces.at(bubble_num).wrench.force.x) +
          (forces.at(bubble_num).wrench.force.x * forces.at(bubble_num).wrench.force.x) +
          (forces.at(bubble_num).wrench.torque.z * forces.at(bubble_num).wrench.torque.z) );

      // force still has a significant high value (> ~75% of old force by default)
      if( (abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) && (abs_new_force > significant_force_) )
      {
        ROS_DEBUG("Detected significante change in force (%f to %f) on bubble %d. Entering Recursive Approximation.", abs_old_force, abs_new_force, bubble_num);
        // o.k. now we really have to take a closer look -> start recursive approximation to equilibrium-point
        int curr_recursion_depth = 0;
        geometry_msgs::Twist new_step_width;
        Bubble curr_bubble = band.at(bubble_num);
        geometry_msgs::WrenchStamped curr_bubble_force = forces.at(bubble_num);

        // half step size
        new_step_width.linear.x = 0.5*bubble_jump.linear.x;
        new_step_width.linear.y = 0.5*bubble_jump.linear.y;
        new_step_width.linear.z = 0.5*bubble_jump.linear.z;
        new_step_width.angular.x = 0.5*bubble_jump.angular.x;
        new_step_width.angular.y = 0.5*bubble_jump.angular.y;
        new_step_width.angular.z = 0.5*bubble_jump.angular.z;

        // one step deeper into the recursion
        if(moveApproximateEquilibrium(bubble_num, band, curr_bubble, curr_bubble_force, new_step_width, curr_recursion_depth))
        {
          // done with recursion - change bubble and hand it back
          new_bubble = curr_bubble;

    #ifdef DEBUG_EBAND_
              geometry_msgs::Pose2D curr_bubble_pose2D;
              PoseToPose2D(curr_bubble.center.pose, curr_bubble_pose2D);
              ROS_DEBUG("Instead - Try moving bubble %d at (%f, %f, %f) by (%f, %f, %f) to (%f, %f, %f).",
                  bubble_num, bubble_pose2D.x, bubble_pose2D.y, bubble_pose2D.theta,
                  new_step_width.linear.x, new_step_width.linear.y, new_step_width.angular.z,
                  curr_bubble_pose2D.x, curr_bubble_pose2D.y, curr_bubble_pose2D.theta);
    #endif
        }
      }
    }


    // check validity of resulting band (given the moved bubble)

    // TODO use this routine not only to check whether gap can be filled but also to fill gap (if possible)
    // get local copy of band, set new position of moved bubble and init iterators
    std::vector<Bubble> tmp_band = band;
    std::vector<Bubble>::iterator start_iter, end_iter;
    // 更新气泡
    tmp_band.at(bubble_num) = new_bubble;
    start_iter = tmp_band.begin();

    // check left connection (bubble and bubble-1)
    // 检查左边的 (bubble-1 and bubble)
    start_iter = start_iter + bubble_num - 1;
    end_iter = start_iter + 1;

    // check Overlap - if bubbles do not overlap try to fill gap
    // 判断两个气泡重合度是否满足，min_bubble_overlap_这个参数说如果重合度不满足也是不行的
    if(!checkOverlap(*start_iter, *end_iter))
    {
      // 在两点之间进行插点，然后判断这个点和左右两点是否重合，如果没有在进行递归
      if(!fillGap(tmp_band, start_iter, end_iter))
      {
        ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
        // this bubble must not be changed, but band is still valid -> continue with other bubbles
        return true;
      }
    }


    // get fresh copy of band, set new position of bubble again and reinit iterators
    tmp_band = band;
    tmp_band.at(bubble_num) = new_bubble;
    start_iter = tmp_band.begin();

    // check right connection (bubble and bubble +1)
    start_iter = start_iter + bubble_num;
    end_iter = start_iter + 1;

    // check Overlap - if bubbles do not overlap try to fill gap
    if(!checkOverlap(*start_iter, *end_iter))
    {
      if(!fillGap(tmp_band, start_iter, end_iter))
      {
        ROS_DEBUG("Bubble at new position cannot be connected to neighbour. Discarding changes.");
        // this bubble must not be changed, but band is still valid -> continue with other bubbles
        return true;
      }
    }


    // check successful - bubble and band valid apply changes

    #ifdef DEBUG_EBAND_
        ROS_DEBUG("Frame %d of %d: Check successful - bubble and band valid. Applying Changes", bubble_num, ((int) band.size()) );
    #endif

    band.at(bubble_num) = new_bubble;

    return true;
  }


  bool EBandPlanner::moveApproximateEquilibrium(const int& bubble_num, const std::vector<Bubble>& band, Bubble& curr_bubble,
      const geometry_msgs::WrenchStamped& curr_bubble_force, geometry_msgs::Twist& curr_step_width, const int& curr_recursion_depth)
  {

    double distance;
    Bubble new_bubble = curr_bubble;
    geometry_msgs::Pose2D new_bubble_pose2D, curr_bubble_pose2D;
    geometry_msgs::WrenchStamped new_bubble_force = curr_bubble_force;

    // move bubble
    PoseToPose2D(curr_bubble.center.pose, curr_bubble_pose2D);
    PoseToPose2D(new_bubble.center.pose, new_bubble_pose2D);

    // apply changes to calculate tmp bubble position
    new_bubble_pose2D.x = curr_bubble_pose2D.x + curr_step_width.linear.x;
    new_bubble_pose2D.y = curr_bubble_pose2D.y + curr_step_width.linear.y;
    new_bubble_pose2D.theta = curr_bubble_pose2D.theta + curr_step_width.angular.z;
    new_bubble_pose2D.theta = angles::normalize_angle(new_bubble_pose2D.theta);

    // apply changes to local copy
    Pose2DToPose(new_bubble.center.pose, new_bubble_pose2D);


    // check validity of moved bubble

    // recalc expansion of bubble -> calc Size of Bubbles by calculating Dist to nearest Obstacle [depends kinematic, environment]
    if(!calcObstacleKinematicDistance(new_bubble.center.pose, distance))
      return false;

    // we wont be able to calculate forces later on
    if(distance == 0.0)
      return false;


    // so far o.k. -> assign distance to new bubble
    new_bubble.expansion = distance;

    // check whether we get a valid force calculation here
    if(!getForcesAt(bubble_num, band, new_bubble, new_bubble_force))
      return false;

    // great - lets store this - this is better then what we had so far
    curr_bubble = new_bubble;

    // if everything is fine and we reached our maximum recursion depth
    if(curr_recursion_depth >= max_recursion_depth_approx_equi_)
      // break recursion at this point
      return true;


    // now - let's check for zero-crossing

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Check for zero-crossings in force on bubble %d - Recursion %d", bubble_num, curr_recursion_depth);
#endif

    double checksum_zero, abs_new_force, abs_old_force;
    int new_recursion_depth;
    geometry_msgs::Twist new_step_width;

    // check zero-crossing by projecting force-vectors onto each other
    checksum_zero = (new_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x) +
      (new_bubble_force.wrench.force.y * curr_bubble_force.wrench.force.y) +
      (new_bubble_force.wrench.torque.z * curr_bubble_force.wrench.torque.z);

    if(checksum_zero < 0.0)
    {
#ifdef DEBUG_EBAND_
      ROS_DEBUG("Detected zero-crossings in force on bubble %d - Recursion %d. Checking total change in force.", bubble_num, curr_recursion_depth);
#endif

      // check the absolute values of the two vectors
      abs_new_force = sqrt( (new_bubble_force.wrench.force.x * new_bubble_force.wrench.force.x) +
          (new_bubble_force.wrench.force.y * new_bubble_force.wrench.force.y) +
          (new_bubble_force.wrench.torque.z * new_bubble_force.wrench.torque.z) );
      abs_old_force = sqrt( (curr_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x) +
          (curr_bubble_force.wrench.force.x * curr_bubble_force.wrench.force.x) +
          (curr_bubble_force.wrench.torque.z * curr_bubble_force.wrench.torque.z) );

      if( (abs_new_force > equilibrium_relative_overshoot_ * abs_old_force) && (abs_new_force > significant_force_) )
      {
#ifdef DEBUG_EBAND_
        ROS_DEBUG("Detected significant change in force (%f to %f) on bubble %d - Recursion %d. Going one Recursion deeper.", abs_old_force, abs_new_force, bubble_num, curr_recursion_depth);
#endif

        // o.k. now we really have to take a closer look -> start recursive approximation to equilibrium-point
        new_recursion_depth = curr_recursion_depth + 1;
        // half step size - backward direction
        new_step_width.linear.x = -0.5*curr_step_width.linear.x;
        new_step_width.linear.y = -0.5*curr_step_width.linear.y;
        new_step_width.linear.z = -0.5*curr_step_width.linear.z;
        new_step_width.angular.x = -0.5*curr_step_width.angular.x;
        new_step_width.angular.y = -0.5*curr_step_width.angular.y;
        new_step_width.angular.z = -0.5*curr_step_width.angular.z;

        // one step deeper into the recursion
        if(moveApproximateEquilibrium(bubble_num, band, new_bubble, new_bubble_force, new_step_width, new_recursion_depth))
          // done with recursion - change bubble and hand it back
          curr_bubble = new_bubble;

        // otherwise - could not get a better value - return without change (curr_bubble as asigned above)
      }

      // otherwise - this is good enough for us - return with this value (curr_bubble as asigned above)
    }
    else
    {
#ifdef DEBUG_EBAND_
      ROS_DEBUG("No zero-crossings in force on bubble %d - Recursion %d. Continue walk in same direction. Going one recursion deeper.", bubble_num, curr_recursion_depth);
#endif

      // continue walk in same direction
      new_recursion_depth = curr_recursion_depth + 1;
      // half step size - backward direction
      new_step_width.linear.x = 0.5*curr_step_width.linear.x;
      new_step_width.linear.y = 0.5*curr_step_width.linear.y;
      new_step_width.linear.z = 0.5*curr_step_width.linear.z;
      new_step_width.angular.x = 0.5*curr_step_width.angular.x;
      new_step_width.angular.y = 0.5*curr_step_width.angular.y;
      new_step_width.angular.z = 0.5*curr_step_width.angular.z;

      // one step deeper into the recursion
      if(moveApproximateEquilibrium(bubble_num, band, new_bubble, new_bubble_force, new_step_width, new_recursion_depth))
        // done with recursion - change bubble and hand it back
        curr_bubble = new_bubble;

      // otherwise - could not get a better value - return without change (curr_bubble as asigned above)
    }

    // done
    return true;
  }

  // 就是更新力
  bool EBandPlanner::getForcesAt(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble, geometry_msgs::WrenchStamped& forces)
  {
    geometry_msgs::WrenchStamped internal_force, external_force;
    // 还是三步走，内力，外力，去除切应力
    if(!calcInternalForces(bubble_num, band, curr_bubble, internal_force))
    {
      // calculation of internal forces failed - stopping optimization
      ROS_DEBUG("Calculation of internal forces failed");
      return false;
    }

    if(!calcExternalForces(bubble_num, curr_bubble, external_force))
    {
      // calculation of External Forces failed - stopping optimization
      ROS_DEBUG("Calculation of external forces failed");
      return false;
    }

    // sum up external and internal forces over all bubbles
    forces.wrench.force.x = internal_force.wrench.force.x + external_force.wrench.force.x;
    forces.wrench.force.y = internal_force.wrench.force.y + external_force.wrench.force.y;
    forces.wrench.force.z = internal_force.wrench.force.z + external_force.wrench.force.z;

    forces.wrench.torque.x = internal_force.wrench.torque.x + external_force.wrench.torque.x;
    forces.wrench.torque.y = internal_force.wrench.torque.y + external_force.wrench.torque.y;
    forces.wrench.torque.z = internal_force.wrench.torque.z + external_force.wrench.torque.z;
    // forces代表这个点的合力
    if(!suppressTangentialForces(bubble_num, band, forces))
    {
      // suppression of tangential forces failed
      ROS_DEBUG("Supression of tangential forces failed");
      return false;
    }

    return true;
  }

  // 根据论文中得计算公式计算内部力
  bool EBandPlanner::calcInternalForces(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble, geometry_msgs::WrenchStamped& forces)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //cycle over all bubbles except first and last (these are fixed)
    if(band.size() <= 2)
    {
      // nothing to do here -> we can stop right away - no forces calculated
      return true;
    }

    // init tmp variables
    double distance1, distance2;
    geometry_msgs::Twist difference1, difference2;
    geometry_msgs::Wrench wrench;

    // make sure this method was called for a valid element in the forces or bubbles vector
    ROS_ASSERT( bubble_num > 0 );
    ROS_ASSERT( bubble_num < ((int) band.size() - 1) );


    // get distance between bubbles
    // 德到前一个点距离这个点的距离
    if(!calcBubbleDistance(curr_bubble.center.pose, band[bubble_num-1].center.pose, distance1))
    {
      ROS_ERROR("Failed to calculate Distance between two bubbles. Aborting calculation of internal forces!");
      return false;
    }
    // 得到后一个点距离这个点的距离
    if(!calcBubbleDistance(curr_bubble.center.pose, band[bubble_num+1].center.pose, distance2))
    {
      ROS_ERROR("Failed to calculate Distance between two bubbles. Aborting calculation of internal forces!");
      return false;
    }

    // get (elementwise) difference bewtween bubbles
    // 矢量差异
    if(!calcBubbleDifference(curr_bubble.center.pose, band[bubble_num-1].center.pose, difference1))
    {
      ROS_ERROR("Failed to calculate Difference between two bubbles. Aborting calculation of internal forces!");
      return false;
    }
    // 矢量差异
    if(!calcBubbleDifference(curr_bubble.center.pose, band[bubble_num+1].center.pose, difference2))
    {
      ROS_ERROR("Failed to calculate Difference between two bubbles. Aborting calculation of internal forces!");
      return false;
    }

    // make sure to avoid division by  (almost) zero during force calculation (avoid numerical problems)
    // 确保在力计算过程中避免除以（几乎）零（避免数值问题）
    // 就是说distance1很小的话,那么difference也是很小的,所以可能等于0,所以我们要做的就是把distance设置为很大就行了
    // -> if difference/distance is (close to) zero then the force in this direction should be zero as well
    if(distance1 <= tiny_bubble_distance_)
      distance1 = 1000000.0;
    if(distance2 <= tiny_bubble_distance_)
      distance2 = 1000000.0;

    // now calculate wrench - forces model an elastic band and are normed (distance) to render forces for small and large bubbles the same  
    // 现在计算wrench-力模型弹性带并进行规范化（距离）以使大小气泡的力相同
    // 内力的增益，也就是论文中得Kc
    wrench.force.x = internal_force_gain_*(difference1.linear.x/distance1 + difference2.linear.x/distance2);
    wrench.force.y = internal_force_gain_*(difference1.linear.y/distance1 + difference2.linear.y/distance2);
    wrench.force.z = internal_force_gain_*(difference1.linear.z/distance1 + difference2.linear.z/distance2);
    wrench.torque.x = internal_force_gain_*(difference1.angular.x/distance1 + difference2.angular.x/distance2);
    wrench.torque.y = internal_force_gain_*(difference1.angular.y/distance1 + difference2.angular.y/distance2);
    wrench.torque.z = internal_force_gain_*(difference1.angular.z/distance1 + difference2.angular.z/distance2);

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Calculating internal forces: (x, y, theta) = (%f, %f, %f)", wrench.force.x, wrench.force.y, wrench.torque.z);
#endif

    // store wrench in vector
    forces.wrench = wrench;

    return true;
  }

  // 根据论文中的公式进行外部力的计算（只计算二维的）
  bool EBandPlanner::calcExternalForces(int bubble_num, Bubble curr_bubble, geometry_msgs::WrenchStamped& forces)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // init tmp variables
    double distance1, distance2;
    geometry_msgs::Pose edge;
    geometry_msgs::Pose2D edge_pose2D;
    geometry_msgs::Wrench wrench;


    // calculate delta-poses (on upper edge of bubble) for x-direction
    // 计算x方向的delta-poses（在气泡的上边缘）
    edge = curr_bubble.center.pose;
    edge.position.x = edge.position.x + curr_bubble.expansion;
    // get expansion on bubble at this point
    // center_posej就是中心位置，distance就是计算的半径
    // 这个函数的意思就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
    if(!calcObstacleKinematicDistance(edge, distance1))
    {
      ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
      // we cannot calculate external forces for this bubble - but still continue for the other bubbles
      return true;
    }
    // calculate delta-poses (on lower edge of bubble) for x-direction
    // 计算x方向的delta-poses（在气泡的下边缘）
    // 为什么减去二，因为上面已经进行加一了
    edge.position.x = edge.position.x - 2.0*curr_bubble.expansion;
    // get expansion on bubble at this point
    if(!calcObstacleKinematicDistance(edge, distance2))
    {
      ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
      // we cannot calculate external forces for this bubble - but still continue for the other bubbles
      return true;
    }

    // calculate difference-quotient (approx. of derivative) in x-direction
    // 一般不会进去，就是说离得很近，那么力就应该很大很大
    // 这个函数计算的就是论文中锋的关于半径的导数
    if(curr_bubble.expansion <= tiny_bubble_expansion_)
    {
      // avoid division by (almost) zero to avoid numerical problems
      wrench.force.x = -external_force_gain_*(distance2 - distance1)/(2.0*tiny_bubble_expansion_);
      // actually we should never end up here - band should have been considered as broken
      ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably ill configured");
    }
    else
      wrench.force.x = -external_force_gain_*(distance2 - distance1)/(2.0*curr_bubble.expansion);
    // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponding term


    // calculate delta-poses (on upper edge of bubble) for y-direction
    edge = curr_bubble.center.pose;
    edge.position.y = edge.position.y + curr_bubble.expansion;
    // get expansion on bubble at this point
    if(!calcObstacleKinematicDistance(edge, distance1))
    {
      ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
      // we cannot calculate external forces for this bubble - but still continue for the other bubbles
      return true;
    }
    // calculate delta-poses (on lower edge of bubble) for x-direction
    edge.position.y = edge.position.y - 2.0*curr_bubble.expansion;
    // get expansion on bubble at this point
    if(!calcObstacleKinematicDistance(edge, distance2))
    {
      ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
      // we cannot calculate external forces for this bubble - but still continue for the other bubbles
      return true;
    }

    // calculate difference-quotient (approx. of derivative) in x-direction
    if(curr_bubble.expansion <= tiny_bubble_expansion_)
    {
      // avoid division by (almost) zero to avoid numerical problems
      wrench.force.y = -external_force_gain_*(distance2 - distance1)/(2.0*tiny_bubble_expansion_);
      // actually we should never end up here - band should have been considered as broken
      ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably ill configured");
    }
    else
      wrench.force.y = -external_force_gain_*(distance2 - distance1)/(2.0*curr_bubble.expansion);
    // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponsing term


    // no force in z-direction
    wrench.force.z = 0.0;


    // no torque around x and y axis
    wrench.torque.x = 0.0;
    wrench.torque.y = 0.0;


    // calculate delta-poses (on upper edge of bubble) for x-direction
    PoseToPose2D(curr_bubble.center.pose, edge_pose2D);
    edge_pose2D.theta = edge_pose2D.theta + (curr_bubble.expansion/getCircumscribedRadius(*costmap_ros_));
    edge_pose2D.theta = angles::normalize_angle(edge_pose2D.theta);
    PoseToPose2D(edge, edge_pose2D);
    // get expansion on bubble at this point
    if(!calcObstacleKinematicDistance(edge, distance1))
    {
      ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
      // we cannot calculate external forces for this bubble - but still continue for the other bubbles
      return true;
    }
    // calculate delta-poses (on lower edge of bubble) for x-direction
    edge_pose2D.theta = edge_pose2D.theta - 2.0*(curr_bubble.expansion/getCircumscribedRadius(*costmap_ros_));
    edge_pose2D.theta = angles::normalize_angle(edge_pose2D.theta);
    PoseToPose2D(edge, edge_pose2D);
    // get expansion on bubble at this point
    if(!calcObstacleKinematicDistance(edge, distance2))
    {
      ROS_DEBUG("Bubble %d probably at edge of map - cannot retrieve distance information to calculate external forces", bubble_num);
      // we cannot calculate external forces for this bubble - but still continue for the other bubbles
      return true;
    }

    // calculate difference-quotient (approx. of derivative) in x-direction
    if(curr_bubble.expansion <= tiny_bubble_expansion_)
    {
      // avoid division by (almost) zero to avoid numerical problems
      wrench.torque.z = -external_force_gain_*(distance2 - distance1)/(2.0*tiny_bubble_expansion_);
      // actually we should never end up here - band should have been considered as broken
      ROS_DEBUG("Calculating external forces on broken band. Bubble should have been removed. Local Planner probably ill configured");
    }
    else
      wrench.torque.z = -external_force_gain_*(distance2 - distance1)/(2.0*curr_bubble.expansion);
    // TODO above equations skip term to make forces continuous at end of influence region - test to add corresponsing term


#ifdef DEBUG_EBAND_
    ROS_DEBUG("Calculating external forces: (x, y, theta) = (%f, %f, %f)", wrench.force.x, wrench.force.y, wrench.torque.z);
#endif

    // assign wrench to forces vector
    forces.wrench = wrench;

    return true;
  }

  // 计算力的切向部分
  // 论文中说，在某些情况下，插入新气泡和去除多余气泡会产生不良的副作用，
  // 气泡可以在一处插入，沿橡皮筋移动，然后在另一处去除。
  // 这个序列可以无限地继续下去，因此松紧带以一种不稳定的方式振荡。
  // 解决这个问题的一种方法是修改施加在橡皮筋上的总力，这样就可以去掉切向分量。
  bool EBandPlanner::suppressTangentialForces(int bubble_num, std::vector<Bubble> band, geometry_msgs::WrenchStamped& forces)
  {
    //cycle over all bubbles except first and last (these are fixed)
    if(band.size() <= 2)
    {
      // nothing to do here -> we can stop right away - no forces calculated
      return true;
    }

    double scalar_fd, scalar_dd;
    geometry_msgs::Twist difference;

    // make sure this method was called for a valid element in the forces or bubbles vector
    ROS_ASSERT( bubble_num > 0 );
    ROS_ASSERT( bubble_num < ((int) band.size() - 1) );


    // get pose-difference from following to preceding bubble -> "direction of the band in this bubble"
    // 矢量差异
    if(!calcBubbleDifference(band[bubble_num+1].center.pose, band[bubble_num-1].center.pose, difference))
      return false;

    // "project wrench" in middle bubble onto connecting vector scalar wrench*difference
    // 中间气泡中的“project wrench”到连接矢量标量wrench上*差异
    scalar_fd = forces.wrench.force.x*difference.linear.x + forces.wrench.force.y*difference.linear.y +
      forces.wrench.force.z*difference.linear.z + forces.wrench.torque.x*difference.angular.x +
      forces.wrench.torque.y*difference.angular.y + forces.wrench.torque.z*difference.angular.z;

    // abs of difference-vector: scalar difference*difference
    // 差向量的绝对值：标量差*差
    scalar_dd = difference.linear.x*difference.linear.x + difference.linear.y*difference.linear.y + difference.linear.z*difference.linear.z +
      difference.angular.x*difference.angular.x + difference.angular.y*difference.angular.y + difference.angular.z*difference.angular.z;

    // avoid division by (almost) zero -> check if bubbles have (almost) same center-pose
    if(scalar_dd <= tiny_bubble_distance_)
    {
      // there are redundant bubbles, this should normally not hapen -> probably error in band refinement
      ROS_DEBUG("Calculating tangential forces for redundant bubbles. Bubble should have been removed. Local Planner probably ill configured");
    }

    // calculate orthogonal components
    // 计算正交分量
    forces.wrench.force.x = forces.wrench.force.x - scalar_fd/scalar_dd * difference.linear.x;
    forces.wrench.force.y = forces.wrench.force.y - scalar_fd/scalar_dd * difference.linear.y;
    forces.wrench.force.z = forces.wrench.force.z - scalar_fd/scalar_dd * difference.linear.z;
    forces.wrench.torque.x = forces.wrench.torque.x - scalar_fd/scalar_dd * difference.angular.x;
    forces.wrench.torque.y = forces.wrench.torque.y - scalar_fd/scalar_dd * difference.angular.y;
    forces.wrench.torque.z = forces.wrench.torque.z - scalar_fd/scalar_dd * difference.angular.z;

#ifdef DEBUG_EBAND_
    ROS_DEBUG("Supressing tangential forces: (x, y, theta) = (%f, %f, %f)",
        forces.wrench.force.x, forces.wrench.force.y, forces.wrench.torque.z);
#endif

    return true;
  }


  // problem (geometry) dependant functions
  // 在他们中间插入一个气泡
  bool EBandPlanner::interpolateBubbles(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center, geometry_msgs::PoseStamped& interpolated_center)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // instantiate local variables
    geometry_msgs::Pose2D start_pose2D, end_pose2D, interpolated_pose2D;
    double delta_theta;

    // copy header
    interpolated_center.header = start_center.header;

    // interpolate angles
    // TODO make this in a better way
    // - for instance use "slerp" to interpolate directly between quaternions
    // - or work with pose2D right from the beginnning
    // convert quaternions to euler angles - at this point this no longer works in 3D !!
    PoseToPose2D(start_center.pose, start_pose2D);
    PoseToPose2D(end_center.pose, end_pose2D);
    // calc mean of theta angle
    delta_theta = end_pose2D.theta - start_pose2D.theta;
    delta_theta = angles::normalize_angle(delta_theta) / 2.0;
    interpolated_pose2D.theta = start_pose2D.theta + delta_theta;
    interpolated_pose2D.theta = angles::normalize_angle(interpolated_pose2D.theta);
    // convert back to quaternion
    interpolated_pose2D.x = 0.0;
    interpolated_pose2D.y = 0.0;
    Pose2DToPose(interpolated_center.pose, interpolated_pose2D);

    // interpolate positions
    interpolated_center.pose.position.x = (end_center.pose.position.x + start_center.pose.position.x)/2.0;
    interpolated_center.pose.position.y = (end_center.pose.position.y + start_center.pose.position.y)/2.0;
    interpolated_center.pose.position.z = (end_center.pose.position.z + start_center.pose.position.z)/2.0;

    // TODO ideally this would take into account kinematics of the robot and for instance use splines

    return true;
  }

  // 判断两个气泡重合度是否满足，min_bubble_overlap_这个参数说如果重合度不满足也是不行的
  bool EBandPlanner::checkOverlap(Bubble bubble1, Bubble bubble2)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // calc (kinematic) Distance between bubbles
    double distance = 0.0;
    // 就是计算两个中心间的距离，把距离返回到distance
    if(!calcBubbleDistance(bubble1.center.pose, bubble2.center.pose, distance))
    {
      ROS_ERROR("failed to calculate Distance between two bubbles. Aborting check for overlap!");
      return false;
    }

    // compare with size of the two bubbles
    // 最小相对重叠，必须将两个气泡视为已连接min_bubble_overlap_
    // 也就是说如果重合度不满足也是不行的
    if(distance >= min_bubble_overlap_ * (bubble1.expansion + bubble2.expansion))
      return false;

    // TODO this does not account for kinematic properties -> improve

    // everything fine - bubbles overlap
    return true;
  }

  // 就是计算两个中心间的距离，把距离返回到distance
  bool EBandPlanner::calcBubbleDistance(geometry_msgs::Pose start_center_pose, geometry_msgs::Pose end_center_pose, double& distance)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::Pose2D start_pose2D, end_pose2D, diff_pose2D;

    // TODO make this in a better way
    // - or work with pose2D right from the beginnning
    // convert quaternions to euler angles - at this point this no longer works in 3D !!
    PoseToPose2D(start_center_pose, start_pose2D);
    PoseToPose2D(end_center_pose, end_pose2D);

    // get rotational difference
    diff_pose2D.theta = end_pose2D.theta - start_pose2D.theta;
    diff_pose2D.theta = angles::normalize_angle(diff_pose2D.theta);
    // get translational difference
    diff_pose2D.x = end_pose2D.x - start_pose2D.x;
    diff_pose2D.y = end_pose2D.y - start_pose2D.y;

    // calc distance
    double angle_to_pseudo_vel = diff_pose2D.theta * getCircumscribedRadius(*costmap_ros_);
    //distance = sqrt( (diff_pose2D.x * diff_pose2D.x) + (diff_pose2D.y * diff_pose2D.y) + (angle_to_pseudo_vel * angle_to_pseudo_vel) );
    distance = sqrt( (diff_pose2D.x * diff_pose2D.x) + (diff_pose2D.y * diff_pose2D.y));

    // TODO take into account kinematic properties of body

    return true;
  }

  // 矢量差异
  bool EBandPlanner::calcBubbleDifference(geometry_msgs::Pose start_center_pose, geometry_msgs::Pose end_center_pose, geometry_msgs::Twist& difference)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::Pose2D start_pose2D, end_pose2D, diff_pose2D;

    // TODO make this in a better way
    // - or work with pose2D right from the beginnning
    // convert quaternions to euler angles - at this point this no longer works in 3D !!
    PoseToPose2D(start_center_pose, start_pose2D);
    PoseToPose2D(end_center_pose, end_pose2D);

    // get rotational difference
    diff_pose2D.theta = end_pose2D.theta - start_pose2D.theta;
    diff_pose2D.theta = angles::normalize_angle(diff_pose2D.theta);
    // get translational difference
    diff_pose2D.x = end_pose2D.x - start_pose2D.x;
    diff_pose2D.y = end_pose2D.y - start_pose2D.y;

    difference.linear.x = diff_pose2D.x;
    difference.linear.y = diff_pose2D.y;
    difference.linear.z = 0.0;
    // multiply by inscribed radius to math calculation of distance
    difference.angular.x = 0.0;
    difference.angular.y = 0.0;
    difference.angular.z = diff_pose2D.theta*getCircumscribedRadius(*costmap_ros_);

    // TODO take into account kinematic properties of body

    return true;
  }

  // center_posej就是中心位置，distance就是计算的半径
  // 这个函数的意思就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
  bool EBandPlanner::calcObstacleKinematicDistance(geometry_msgs::Pose center_pose, double& distance)
  {
    // calculate distance to nearest obstacle [depends kinematic, shape, environment]

    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    unsigned int cell_x, cell_y;
    unsigned char disc_cost;
    double weight = costmap_weight_;

    // read distance to nearest obstacle directly from costmap
    // (does not take into account shape and kinematic properties)
    // get cell for coordinates of bubble center
    // 直接从成本图读取到最近障碍物的距离（不考虑形状和运动学特性）获取气泡中心坐标的单元格
    // 这个就是用costmap的好处，他帮你维护好一个费用地图，直接使用就可以
    if(!costmap_->worldToMap(center_pose.position.x, center_pose.position.y, cell_x, cell_y)) {
      // probably at the edge of the costmap - this value should be recovered soon
      disc_cost = 1;
    } else {
      // get cost for this cell
      disc_cost = costmap_->getCost(cell_x, cell_y);
    }

    // calculate distance to nearest obstacel from this cost (see costmap_2d in wiki for details)

    // For reference: here comes an excerpt of the cost calculation within the costmap function
    /*if(distance == 0)
      cost = LETHAL_OBSTACLE;
      else if(distance <= cell_inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;
      else {
    //make sure cost falls off by Euclidean distance
    double euclidean_distance = distance * resolution_;
    double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
    cost = (unsigned char) ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }*/

    if (disc_cost == costmap_2d::LETHAL_OBSTACLE) {
      // pose is inside an obstacle - very bad
      distance = 0.0;
    }	else if (disc_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      // footprint is definitely inside an obstacle - still bad
      distance = 0.0;
    } else {
      // 我们把没有信息的和自由的费用都设置为1
      if (disc_cost == 0) { // freespace, no estimate of distance
        disc_cost = 1; // lowest non freespace cost
      } else if (disc_cost == 255) { // unknown space, we should never be here
        disc_cost = 1;
      }
      // factor这个因子，我的见解就是越小说明离障碍物越远，如果disc_cost是252
      // 也就是说这个路径是在障碍物旁边的
      double factor = ((double) disc_cost) / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
      // -log(factor)也就是factor越趋近于0，那么这个值也就越大，也就是说半径越大，然后防止他远离的很大，这里除以一个系数
      distance = -log(factor) / weight;
    }

    return true;
  }


  // type conversions
  // 进行初步处理，得到plan中各个点对应的半径
  bool EBandPlanner::convertPlanToBand(std::vector<geometry_msgs::PoseStamped> plan, std::vector<Bubble>& band)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // create local variables
    double distance = 0.0;
    std::vector<Bubble> tmp_band;

    ROS_DEBUG("Copying plan to band - Conversion started: %d frames to convert.", ((int) plan.size()) );

    // get local copy of referenced variable
    // 得到引用
    tmp_band = band;

    // adapt band to plan
    tmp_band.resize(plan.size());
    // 将所有的路径进行处理，就是计算点对应的半径
    for(int i = 0; i < ((int) plan.size()); i++)
    {
      #ifdef DEBUG_EBAND_
        ROS_DEBUG("Checking Frame %d of %d", i, ((int) plan.size()) );
      #endif

      // set poses in plan as centers of bubbles
      // 将路径点设置为中心
      tmp_band[i].center = plan[i];

      // calc Size of Bubbles by calculating Dist to nearest Obstacle [depends kinematic, environment]
      // 这个函数的意思就是根据中心的cell所对应的costmap中的费用，来计算距离（服从对数分布，越小距离越大）
      if(!calcObstacleKinematicDistance(tmp_band[i].center.pose, distance))
      {
        // frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
        ROS_WARN("Calculation of Distance between bubble and nearest obstacle failed. Frame %d of %d outside map", i, ((int) plan.size()) );
        return false;
      }

      if(distance <= 0.0)
      {
        // frame must not be immediately in collision -> otherwise calculation of gradient will later be invalid
        ROS_WARN("Calculation of Distance between bubble and nearest obstacle failed. Frame %d of %d in collision. Plan invalid", i, ((int) plan.size()) );
        // TODO if frame in collision try to repair band instaed of aborting averything
        return false;
      }


      // assign to expansion of bubble
      tmp_band[i].expansion = distance;
    }

    // write to referenced variable
    band = tmp_band;

    ROS_DEBUG("Successfully converted plan to band");
    return true;
  }


  bool EBandPlanner::convertBandToPlan(std::vector<geometry_msgs::PoseStamped>& plan, std::vector<Bubble> band)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // create local variables
    std::vector<geometry_msgs::PoseStamped> tmp_plan;

    // adapt plan to band
    tmp_plan.resize(band.size());
    for(int i = 0; i < ((int) band.size()); i++)
    {
      // set centers of bubbles to StampedPose in plan
      tmp_plan[i] = band[i].center;
    }

    //write to referenced variable and done
    plan = tmp_plan;

    return true;
  }


}
