class TrajectoryManager{
  private:
    ros::NodeHandle node;

    tf::TransformListener tfListener;
    tf::TransformBroadcaster tfBroadcaster;

    tf::StampedTransform tfActual;
    tf::Transform tfGoal;

    geometry_msgs::PoseArray traj;
    geometry_msgs::Twist vel_lim;
    geometry_msgs::Point real_pos;

    ros::Publisher pose_pub;
    ros::Publisher vel_lim_pub;
    ros::Publisher real_pos_pub;
    ros::ServiceClient motor_enable_srv;

    ros::Subscriber gnd_truth_pos;
    ros::Subscriber enable_movement;

    bool move_enabled;

    // Randomness
    std::random_device rd;
    typedef std::mt19937 myGen;
    myGen generator;
    std::normal_distribution<double> distribution;

  public:
    TrajectoryManager(): generator(rd()), distribution(std::normal_distribution<>(0.0, 0.2)){
      // Subscribtions
      pose_pub = node.advertise<geometry_msgs::PoseStamped>("/command/pose", 20);
      vel_lim_pub = node.advertise<geometry_msgs::Twist>("/command/twist_limit", 20);
      real_pos_pub = node.advertise<geometry_msgs::Point>("/raw_pos", 20);

      gnd_truth_pos = node.subscribe("/ground_truth/state", 10, &TrajectoryManager::pos_faker, this);
      enable_movement = node.subscribe("/moveit", 10, &TrajectoryManager::moveCallback, this);

      motor_enable_srv = node.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

      traj = TrajectoryManager::genTraj();

      TrajectoryManager::enableMotors();

      vel_lim.linear.x = 1;
      vel_lim.linear.y = 1;
      vel_lim.linear.z = 1;
      vel_lim.angular.x = 5;
      vel_lim.angular.y = 5;
      vel_lim.angular.z = 5;


      //ROS_INFO("Broadcasting world->goal tf");
      tf::poseMsgToTF(genPose(0,0,0,0), tfGoal);
      tfBroadcaster.sendTransform(tf::StampedTransform(tfGoal, ros::Time::now(), "world", "goal"));

      move_enabled = false;
      //generator(rd());
      //generator.seed(::time(NULL))
      //distribution(std::normal_distribution<>(0.0, 0.2));
    }


    void spin(){
      //spitPos();

      //ROS_INFO("TM Spinner");
      try{
        tfListener.lookupTransform("base_link", "world", ros::Time(0), tfActual);
      }catch (tf::TransformException &ex){
        ROS_ERROR("1 %s",ex.what());
        ros::Duration(1.0).sleep();
      }

      //ROS_INFO("Broadcasting world->goal tf");
      tfBroadcaster.sendTransform(tf::StampedTransform(tfGoal, ros::Time::now(), "world", "goal"));

      if (TrajectoryManager::reachedGoal() && move_enabled){
        move_enabled = false;
        if (traj.poses.size() < 1){
          ROS_INFO("Reached Final WP");
        }else{
          ROS_INFO("Reached Goal, updating WP");
          TrajectoryManager::setNewGoal();
          ros::Duration(5).sleep();
        }
      }else{
        //ROS_INFO("Not yet reached Goal");
      }
    }

    geometry_msgs::PoseArray genTraj(){
      geometry_msgs::PoseArray traj;

      // genPose(x, y, z, heading)
      traj.poses.push_back(TrajectoryManager::genPose(  0.0, 0.0, 1.0, M_PI));  // Levitate
      traj.poses.push_back(TrajectoryManager::genPose(-16.0, 0.0, 1.0, M_PI));  // Move to WP1
      traj.poses.push_back(TrajectoryManager::genPose(-16.0, 0.0, 1.0, 3*M_PI/2)); // Align to WP2
      traj.poses.push_back(TrajectoryManager::genPose(-16.0,-8.5, 1.0, 3*M_PI/2)); // Move to WP2
      traj.poses.push_back(TrajectoryManager::genPose(-16.0,-8.5, 1.0, 2*M_PI)); // Align to WP2
      traj.poses.push_back(TrajectoryManager::genPose(  9.0,-8.5, 1.0, 2*M_PI)); // Move to WP3
      traj.poses.push_back(TrajectoryManager::genPose(  9.0,-8.5, 1.0, M_PI/2)); // Align to WP4
      traj.poses.push_back(TrajectoryManager::genPose(  9.0,-6.5, 1.0, M_PI/2)); // Move to WP4
      traj.poses.push_back(TrajectoryManager::genPose(  9.0,-6.5, 1.0, 3*M_PI/4)); // Align to WP5
      traj.poses.push_back(TrajectoryManager::genPose(  3.0,-0.0, 1.0, 3*M_PI/4)); // Move to WP5
      traj.poses.push_back(TrajectoryManager::genPose(  0.0, 0.0, 1.0, M_PI)); // Move to WP1

      return traj;
    }

    geometry_msgs::Pose genPose(float x_pos, float y_pos, float z_pos, float heading){
      geometry_msgs::Pose pose;

      pose.position.x = x_pos;
      pose.position.y = y_pos;
      pose.position.z = z_pos;
      pose.orientation = tf::createQuaternionMsgFromYaw(heading);

      return pose;
    }

    void setNewGoal(){
      geometry_msgs::PoseStamped newGoal;

      // Fill in PoseStamped message
      newGoal.header.stamp = ros::Time::now();
      newGoal.header.frame_id = "world";
      newGoal.pose = traj.poses.front();

      // Publish message
      pose_pub.publish(newGoal);
      vel_lim_pub.publish(vel_lim);
      // Update goal tf
      tf::poseMsgToTF(newGoal.pose, tfGoal);

      // Delete current goal from array
      traj.poses.erase(traj.poses.begin());

      ROS_INFO("Waypoints left: %d", traj.poses.size());
      return;
    }

    bool reachedGoal(){
      float dist;
      double heading;
      tf::StampedTransform tfRel;

      //tfRel = tfActual.inverseTimes(tfGoal);

      try{
        tfListener.lookupTransform("goal","base_link",ros::Time(0),tfRel);
      }catch (tf::TransformException &ex){
        ROS_ERROR("2 %s",ex.what());
        ros::Duration(1.0).sleep();
      }
      dist = tfRel.getOrigin().length();
      heading = getYaw(tfRel.getRotation());


      ROS_INFO("Distance to next waypoint: %.2f [m]", dist);

      if ((abs(dist) <= 0.05)&&(abs(heading) <= 5.0*M_PI/180.0)){
        return true;
      }else{
        return false;
      }
    }

    void enableMotors(){
      // Enable Motors and check
      hector_uav_msgs::EnableMotors srv;
      srv.request.enable = true;

      if(TrajectoryManager::motor_enable_srv.call(srv)){
        ROS_INFO("Motors Enabled");
      }else{
        ROS_ERROR("Motor Enabling FAILED");
      }

      return;
    }

    void pos_faker(const nav_msgs::Odometry& gnd_truth){
      real_pos.x = gnd_truth.pose.pose.position.x + distribution(generator);
      real_pos.y = gnd_truth.pose.pose.position.y + distribution(generator);
      real_pos.z = gnd_truth.pose.pose.position.z + distribution(generator);

      real_pos_pub.publish(real_pos);

      return;
    }

    void spitPos(){
      real_pos.x = tfActual.getOrigin().x();
      real_pos.y = tfActual.getOrigin().y();
      real_pos.z = tfActual.getOrigin().z();

      real_pos_pub.publish(real_pos);

      return;
    }

    void moveCallback(const std_msgs::String::ConstPtr& enable){
      // rostopic pub /moveit std_msgs/String "y"
      if (enable->data == "y"){
        move_enabled = true;
        ROS_INFO("Enabling Movement");
      }
      return;
    }
};
