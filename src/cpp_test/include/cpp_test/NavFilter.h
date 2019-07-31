class NavFilter{
  private:
    // ROS Stuff
    ros::NodeHandle node;
    ros::Subscriber sub_pos;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_imu;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tfBroadcaster;

    std::chrono::high_resolution_clock::time_point prev_time;
    double Ts;

    // Filter matrices/vector
    Eigen::Matrix<double,STATES,1> x_k;         // State vector [q,p,v,b_w,b_a]
    Eigen::Matrix<double,STATES,1> x_k_p;       // Prev time-step state vector
    Eigen::Matrix<double,INPUTS,1> u_k;         // Input vector [w,a]
    Eigen::Matrix<double,STATES,STATES> P_k;    // State covariance matrix
    Eigen::Matrix<double,STATES,STATES> P_k_p;  // Prev time-step state covariance matrix
    Eigen::Matrix<double,STATES,STATES> F_k;    // State transition model
    Eigen::Matrix<double,STATES,INPUTS> B_k;    // Input transition model
    Eigen::Matrix<double,MSRMTS,STATES> H_k;    // Observation model for position measurement
    Eigen::Matrix<double,STATES,STATES> Q_k;    // Process noise covariance
    Eigen::Matrix<double,MSRMTS,MSRMTS> R_k;    // Observation noise covariance
    Eigen::Matrix<double,MSRMTS,MSRMTS> S_k;    // Innovation matrix
    Eigen::Matrix<double,STATES,MSRMTS> K_k;    // Kalman gains
    Eigen::Matrix<double,MSRMTS,1> y_k;         // Innovation
    Eigen::Matrix<double,MSRMTS,1> z_k;         // Sensor measurement [pos; vel]
    Eigen::Matrix<double,3,3> C_k;              // Inertial-to-Body Rotation
    Eigen::Matrix<double,4,4> Omega_k;          // Skew symmetric matrix associated with ω

    // For easy access
    Eigen::Matrix<double,3,1> w_k;    // Rotational velocity vector [rad/s]
    Eigen::Matrix<double,3,1> v_k;    // Linear velocity vector [m/s]
    Eigen::Matrix<double,3,1> a_k;    // Linear acceleration vector [m/s2]
    Eigen::Matrix<double,3,3> I3;     // 3x3 Identity matrix
    Eigen::Matrix<double,4,4> I4;     // 4x4 Identity matrix


  public:
    // Just for curiosity
    int posCallbacks;
    int velCallbacks;
    int imuCallbacks;

    NavFilter(){
      // Initialise everything
      I3 = Eigen::Matrix<double,3,3>::Identity();
      I4 = Eigen::Matrix<double,4,4>::Identity();

      x_k.setZero();  // States all start at 0
      x_k_p.setZero();
      x_k(0) = 1;

      u_k.setZero();

      P_k.setIdentity();
      P_k.block(0,0,4,4).diagonal() << 0.5, 0.5, 0.5, 0.5;  // Quaternion covariance
      P_k.block(4,4,3,3).diagonal() << 1, 1, 1;     // Position covariance
      P_k.block(7,7,3,3).diagonal() << 1, 1, 1;     // Linear velocity covariance
      P_k_p = P_k;

      F_k.setIdentity();
      B_k.setZero();
      H_k.setZero();
      H_k.block(0,4,3,3).setIdentity();

      // Process noise covariance (tuning parameters)
      Q_k.setIdentity();
      Q_k.block(0,0,4,4).diagonal().setConstant(1);     // Quaternion related-covariance
      Q_k.block(4,4,3,3).diagonal().setConstant(1);     // Position-related covariance
      Q_k.block(7,7,3,3).diagonal().setConstant(1);     // Velocity-related covariance

      //Measurement noise covariance
      R_k.setIdentity();
      /*R_k.block( 0, 0, 4, 4).diagonal() << 1, 1, 1, 1;  // IMU Quaternion noise
      R_k.block( 4, 4, 3, 3).diagonal() << 0.05, 0.05, 0.015;     // IMU Gyro noise
      R_k.block( 7, 7, 3, 3).diagonal() << 1, 1, 1;     // Position estimation noise
      R_k.block(10,10, 3, 3).diagonal() << 1, 1, 1;   // Optical Flow vel noise
      R_k.block(13,13, 3, 3).diagonal() << 0.35, 0.35, 0.3;   // IMU acceleration noise*/
      R_k.block(0, 0, 3, 3).diagonal() << 0.2, 0.2, 0.2;    // Position measurement noise
      R_k.block(3, 3, 3, 3).diagonal() << 1, 1, 1;    // Velocity measurement noise


      // Subscribtions
      sub_pos = node.subscribe("/raw_pos", 50, &NavFilter::posCallback, this);
      sub_vel = node.subscribe("/raw_vel", 50, &NavFilter::velCallback, this);
      sub_imu = node.subscribe("/raw_imu/bias", 500, &NavFilter::imuCallback, this);
      // Publications
      odom_pub = node.advertise<nav_msgs::Odometry>("/fused_state/odom", 20);

      posCallbacks = 0;
      velCallbacks = 0;
      imuCallbacks = 0;

      ROS_INFO("Initialised NavFilter");
    }

    //Eigen::Matrix<double,STATES,1> propagateFilter(){
    void propagateFilter(){
      // Update omega with current timestep
      updateOmega();

      // Calculate time step that has occured
      std::chrono::high_resolution_clock::time_point now_time = std::chrono::high_resolution_clock::now();
      Ts = (double)std::chrono::duration_cast<std::chrono::duration<double>>(now_time - prev_time).count();
      prev_time = now_time;

      // Update F_k
      F_k.setIdentity();
      F_k.block(0,0,4,4) = Eigen::Matrix<double,4,4>::Identity() + Ts * Omega_k;  // Quat. propagation
      F_k.block(4,4,3,3) = Eigen::Matrix<double,3,3>::Identity();   // Position propagation
      F_k.block(4,7,3,3) = Ts * Eigen::Matrix<double,3,3>::Identity();  // Vel. component of Pos. prop.
      F_k.block(7,7,3,3) = Eigen::Matrix<double,3,3>::Identity();   // Velocity Propagation

      // Update B_k
      B_k.setZero();
      B_k.block(4,3,3,3) = 0.5*Ts*Ts*C_k.transpose(); // Acceleration component of position propagation
      B_k.block(7,3,3,3) = Ts*C_k.transpose();    // Acceleration component of velocity propagation
      /*std::cout << C_k << std::endl;
      std::cout << F_k << std::endl;
      std::cout << Omega_k << std::endl;
      std::cout << B_k << std::endl;
      std::cout << u_k << std::endl;
      std::cout << x_k.transpose() << std::endl;*/

      // Propagate state and covariances
      //ROS_INFO("HERE 1");
      x_k_p = F_k * x_k + B_k * u_k;
      P_k_p = F_k * P_k * F_k.transpose() + Q_k;    // B_k * Q_k * B_k.transpose() ??????

      // IMU propagation, will be fixed when measurement occurs
      x_k = x_k_p;
      P_k = P_k_p;

      // Update C_k with previous timestep
      updateRotationMatrix();

      // Publish state
      publishState();

      return;// x_k;
    }

    void posCorrection(){
      // Correction for Position measurement
      y_k.block(0,0,3,1) = z_k.block(0,0,3,1) - H_k.block(0,0,3,STATES) * x_k_p;
      S_k.block(0,0,3,3) = R_k.block(0,0,3,3) + H_k.block(0,0,3,STATES) * P_k_p * H_k.block(0,0,3,STATES).transpose();
      K_k.block(0,0,STATES,3) = P_k_p * H_k.block(0,0,3,STATES).transpose() * S_k.block(0,0,3,3).inverse();

      // Update
      x_k = x_k_p + K_k.block(0,0,STATES,3) * y_k.block(0,0,3,1);
      P_k = (Eigen::Matrix<double,STATES,STATES>::Identity() - K_k.block(0,0,STATES,3) * H_k.block(0,0,3,STATES)) * P_k;

      // Publish State
      NavFilter::publishState();

      return;
    }

    void velCorrection(){
      // Correction for Position measurement
      y_k.block(3,0,3,1) = z_k.block(3,0,3,1) - H_k.block(3,0,3,STATES) * x_k_p;
      S_k.block(3,3,3,3) = R_k.block(3,3,3,3) + H_k.block(3,0,3,STATES) * P_k_p * H_k.block(3,0,3,STATES).transpose();
      K_k.block(0,3,STATES,3) = P_k_p * H_k.block(3,0,3,STATES).transpose() * S_k.block(3,3,3,3).inverse();

      // Update
      x_k = x_k_p + K_k.block(0,3,STATES,3) * y_k.block(3,0,3,1);
      P_k = (Eigen::Matrix<double,STATES,STATES>::Identity() - K_k.block(0,3,STATES,3) * H_k.block(3,0,3,STATES)) * P_k;

      // Publish State
      NavFilter::publishState();

      return;
    }

    void posCallback(const geometry_msgs::Point& pos_meas){
      z_k(0) = pos_meas.x;
      z_k(1) = pos_meas.y;
      z_k(2) = pos_meas.z;

      posCallbacks++;

      posCorrection();
      return;
    }

    void velCallback(const geometry_msgs::TwistWithCovariance& vel_meas){
      // Linear velocity measurement
      z_k(3) = vel_meas.twist.linear.x;
      z_k(4) = vel_meas.twist.linear.y;
      z_k(5) = vel_meas.twist.linear.z;

      // Linear velocity covariance
      /*R_k(1,1) = vel_meas.covariance[1];
      R_k(2,2) = vel_meas.covariance[8];
      R_k(3,3) = vel_meas.covariance[16];*/

      velCallbacks++;

      velCorrection();
      return;
    }

    void imuCallback(const sensor_msgs::Imu& imu_meas){
      // Quaterions
      /*z_k(0) = imu_meas.orientation.x;
      z_k(1) = imu_meas.orientation.y;
      z_k(2) = imu_meas.orientation.z;
      z_k(3) = imu_meas.orientation.w;*/

      // Angular velocity
      w_k(0) = imu_meas.angular_velocity.x - x_k(10);
      w_k(1) = imu_meas.angular_velocity.y - x_k(11);
      w_k(2) = imu_meas.angular_velocity.z - x_k(12);

      // Linear acceleration
      a_k(0) = imu_meas.linear_acceleration.x - x_k(13);
      a_k(1) = imu_meas.linear_acceleration.y - x_k(14);
      a_k(2) = imu_meas.linear_acceleration.z - x_k(15);

      if ( ( w_k.hasNaN() ) || ( a_k.hasNaN() ) ){
        // Ignore Measurement
        ROS_INFO("Ignoring IMU measurement");
        return;
      }else{
        // Update input vector
        u_k(0) = w_k(0);
        u_k(1) = w_k(1);
        u_k(2) = w_k(2);
        u_k(3) = a_k(0);
        u_k(4) = a_k(1);
        u_k(5) = a_k(2);
      }

      //imuCallbacks++;

      /* imu_meas also provides covariances */
      // imu_meas.orientation_covariance is a 3x3 matrix of Quaternion covariance

      /* Doesn't work :/ */
      //R_k.block(4,4,3,3) << imu_meas.angular_velocity_covariance;  // IMU Gyro noise
      //R_k.block(4,4,3,3) = Eigen::Map<Eigen::MatrixXd> (imu_meas.angular_velocity_covariance);
      //R_k.block(13,13,3,3) << Eigen::Map<Eigen::MatrixXd> (imu_meas.linear_acceleration_covariance,3,3);

      // Unfortunately need to create temp arrays
      /*double vel_cov_f[9], acc_cov_f[9];
      for (int i = 0; i < 9; i++){
        vel_cov_f[i] = imu_meas.angular_velocity_covariance[i];
        acc_cov_f[i] = imu_meas.linear_acceleration_covariance[i];
      }
      R_k.block(4,4,3,3) << Eigen::Map<Eigen::MatrixXd> (vel_cov_f,3,3);
      R_k.block(13,13,3,3) << Eigen::Map<Eigen::MatrixXd> (acc_cov_f,3,3);*/

      propagateFilter();
      return;
    }

    void publishState(){
      //ROS_INFO("Publishing Fused State");
      tf::Quaternion quat_tf = tf::Quaternion(x_k(0), x_k(1), x_k(2), x_k(3));
      tf::Vector3 position_tf = tf::Vector3(x_k(4), x_k(5), x_k(6));

      // Publish TF
      tf::Transform tf_fused;

      tf_fused.setOrigin(position_tf);
      tf_fused.setRotation(quat_tf.normalize());

      tfBroadcaster.sendTransform( tf::StampedTransform(tf_fused, ros::Time::now(), "world", "fused_frame"));

      // Publish Odometry
      nav_msgs::Odometry odom;

      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "fused_frame";

      //set the position
      tf::pointTFToMsg(position_tf, odom.pose.pose.position);
      tf::quaternionTFToMsg(quat_tf, odom.pose.pose.orientation);

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = x_k(7);
      odom.twist.twist.linear.y = x_k(8);
      odom.twist.twist.linear.z = x_k(9);
      odom.twist.twist.angular.x = u_k(0);
      odom.twist.twist.angular.y = u_k(1);
      odom.twist.twist.angular.z = u_k(2);

      odom_pub.publish(odom);
      return;
    }

    void updateRotationMatrix(){
      Eigen::Quaterniond q;
      // Be careful here. I specify x_k(0) as the real part of the quaternion,
      // whereas for an Eigen::Quaterniond the real element is considered the
      // fourth element (q.w())
      q.w() = x_k(0);
      q.x() = x_k(1);
      q.y() = x_k(2);
      q.z() = x_k(3);

      C_k = q.normalized().toRotationMatrix();

      //Manual implementation, don't fuck with..
      /*C_k << 1 - 2*x_k(2)*x_k(2) - 2*x_k(3)*x_k(3),
             2*x_k(1)*x_k(2) - 2*x_k(3)*x_k(0),
             2*x_k(1)*x_k(3) + 2*x_k(2)*x_k(0),
             2*x_k(1)*x_k(2) + 2*x_k(3)*x_k(0),
             1 - 2*x_k(1)*x_k(1) - 2*x_k(3)*x_k(3),
             2*x_k(2)*x_k(3) - 2*x_k(1)*x_k(0),
             2*x_k(1)*x_k(3) - 2*x_k(2)*x_k(0),
             2*x_k(2)*x_k(3) + 2*x_k(1)*x_k(0),
             1 - 2*x_k(1)*x_k(1) - 2*x_k(2)*x_k(2);*/
      //ROS_INFO("HERE 3");
      return;
    }

    void updateOmega(){
      //ROS_INFO("HERE Om");
      Omega_k <<     0,  w_k(2), -w_k(1), w_k(0),
               -w_k(2),       0,  w_k(0), w_k(1),
                w_k(1), -w_k(0),       0, w_k(2),
               -w_k(0), -w_k(1), -w_k(2),      0;
      return;
    }
};
