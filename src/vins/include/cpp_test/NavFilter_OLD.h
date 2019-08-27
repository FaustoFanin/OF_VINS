class NavFilter{
  private:
    // ROS Stuff
    ros::NodeHandle node;
    ros::Subscriber sub_pos;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_imu;
    tf::TransformBroadcaster tfBroadcaster;

    // Timestep
    float dt;

    // Filter matrices/vector
    Eigen::Matrix<double,STATES,1> x_k;         // State vector
    Eigen::Matrix<double,STATES,1> x_k_p;       // Prev time-step state vector
    Eigen::Matrix<double,STATES,STATES> P_k;    // State covariance matrix
    Eigen::Matrix<double,STATES,STATES> P_k_p;  // Prev time-step state covariance matrix
    Eigen::Matrix<double,STATES,STATES> F_k;    // State transition model
    Eigen::Matrix<double,MSRMTS,STATES> H_k;    // Observation model
    Eigen::Matrix<double,STATES,STATES> Q_k;    // Process noise covariance
    Eigen::Matrix<double,MSRMTS,MSRMTS> R_k;    // Observation noise covariance
    Eigen::Matrix<double,MSRMTS,MSRMTS> S_k;    // Innovation matrix
    Eigen::Matrix<double,STATES,MSRMTS> K_k;    // Kalman gains
    Eigen::Matrix<double,MSRMTS,1> y_k;         // Innovation
    Eigen::Matrix<double,MSRMTS,1> z_k;         // Sensor measurement
    Eigen::Matrix<double,4,4> Omega_k;          // Skew symmetric matrix associated with ω

  public:
    NavFilter(int Hz): dt(1.0/Hz){
      // Initialise everything
      x_k.setZero();  // States all start at 0

      P_k.setIdentity();
      P_k.block(0,0,4,4).diagonal() << 1, 1, 1, 1;  // Quaternion covariance
      P_k.block(4,4,3,3).diagonal() << 1, 1, 1;     // Rotational velocity covariance
      P_k.block(7,7,3,3).diagonal() << 1, 1, 1;     // Position covariance
      P_k.block(10,10,3,3).diagonal() << 1, 1, 1;  // Linear velocity covariance
      P_k.block(13,13,3,3).diagonal() << 1, 1, 1;  // Linear acceleration covariance

      Omega_k <<     1, -x_k(4), -x_k(5), -x_k(6),
                x_k(4),       1,  x_k(6), -x_k(5),
                x_k(5), -x_k(6),       1, -x_k(4),
                x_k(6),  x_k(5), -x_k(4),       1;
      Omega_k *= 0.5 * dt;

      F_k.setIdentity();
      F_k.block(0,0,4,4) = Omega_k;
      F_k.block(7,10,3,3).diagonal().setConstant(dt);     // Timestep diagonal for lin_vel integration
      F_k.block(10,13,3,3).diagonal().setConstant(dt);    // Timestep diagonal for lin_acc integration
      F_k.block(7,13,3,3).diagonal().setConstant(dt*dt);  // Timestep diagonal for lin_acc double integration

      // Observation Model
      H_k.setIdentity();

      // Process noise covariance
      Q_k.setIdentity();
      Q_k.block(0,0,4,4).diagonal().setConstant(1);     // Quaternionrelated-covariance
      Q_k.block(4,4,3,3).diagonal().setConstant(1);     // Gyro-related covairance
      Q_k.block(7,7,3,3).diagonal().setConstant(1);     // Position estimation noise
      Q_k.block(10,10,3,3).diagonal().setConstant(1);   // Optical Flow vel noise
      Q_k.block(13,13,3,3).diagonal().setConstant(1);   // IMU acceleration noise

      //Measurement noise covariance
      R_k.setIdentity();
      R_k.block( 0, 0, 4, 4).diagonal() << 1, 1, 1, 1;  // IMU Quaternion noise
      R_k.block( 4, 4, 3, 3).diagonal() << 0.05, 0.05, 0.015;     // IMU Gyro noise
      R_k.block( 7, 7, 3, 3).diagonal() << 1, 1, 1;     // Position estimation noise
      R_k.block(10,10, 3, 3).diagonal() << 1, 1, 1;   // Optical Flow vel noise
      R_k.block(13,13, 3, 3).diagonal() << 0.35, 0.35, 0.3;   // IMU acceleration noise

      // Subscribtions
      sub_pos = node.subscribe("/raw_pos", 10, &NavFilter::posCallback, this);
      sub_vel = node.subscribe("/raw_vel", 10, &NavFilter::velCallback, this);
      sub_imu = node.subscribe("/raw_imu/bias", 10, &NavFilter::imuCallback, this);
    }

    Eigen::Matrix<double,STATES,1> propagateFilter(){
      // Prediction
      x_k_p = F_k * x_k;
      P_k_p = F_k * P_k * F_k.transpose() + Q_k;

      // Innovation
      y_k = z_k - H_k * x_k_p;
      S_k = R_k + H_k * P_k_p * H_k.transpose();
      K_k = P_k_p * H_k.transpose() * S_k.inverse();

      // Update
      x_k = x_k_p + K_k * y_k;
      P_k = (Eigen::Matrix<double,STATES,STATES>::Identity() - K_k * H_k) * P_k;

      // Publish State
      NavFilter::publishState();

      // KF finshed, update Omega and in turn F_k
      Omega_k <<     1, -x_k(4), -x_k(5), -x_k(6),
                x_k(4),       1,  x_k(6), -x_k(5),
                x_k(5), -x_k(6),       1, -x_k(4),
                x_k(6),  x_k(5), -x_k(4),       1;
      Omega_k *= 0.5 * dt;
      F_k.block(0,0,4,4) = Omega_k;

      return x_k;
    }

    void posCallback(const geometry_msgs::Point& pos_meas){
      z_k(7) = pos_meas.x;
      z_k(8) = pos_meas.y;
      z_k(9) = pos_meas.z;

      return;
    }

    void velCallback(const geometry_msgs::TwistWithCovariance& vel_meas){
      // Linear velocity measurement
      z_k(10) = vel_meas.twist.linear.x;
      z_k(11) = vel_meas.twist.linear.y;
      z_k(12) = vel_meas.twist.linear.z;

      // Linear velocity covariance
      R_k(1,1) = vel_meas.covariance[1];
      R_k(2,2) = vel_meas.covariance[8];
      R_k(3,3) = vel_meas.covariance[16];

      return;
    }

    void imuCallback(const sensor_msgs::Imu& imu_meas){
      // Quaterions
      z_k(0) = imu_meas.orientation.x;
      z_k(1) = imu_meas.orientation.y;
      z_k(2) = imu_meas.orientation.z;
      z_k(3) = imu_meas.orientation.w;

      // Angular velocity
      z_k(4) = imu_meas.angular_velocity.x;
      z_k(5) = imu_meas.angular_velocity.y;
      z_k(6) = imu_meas.angular_velocity.z;

      // Linear acceleration
      z_k(13) = imu_meas.linear_acceleration.x;
      z_k(14) = imu_meas.linear_acceleration.y;
      z_k(15) = imu_meas.linear_acceleration.z;

      /* imu_meas also provides covariances */
      // imu_meas.orientation_covariance is a 3x3 matrix of Quaternion covariance

      /* Doesn't work :/ */
      //R_k.block(4,4,3,3) << imu_meas.angular_velocity_covariance;  // IMU Gyro noise
      //R_k.block(4,4,3,3) = Eigen::Map<Eigen::MatrixXd> (imu_meas.angular_velocity_covariance);
      //R_k.block(13,13,3,3) << Eigen::Map<Eigen::MatrixXd> (imu_meas.linear_acceleration_covariance,3,3);

      // Unfortunately need to create temp arrays
      double vel_cov_f[9], acc_cov_f[9];
      for (int i = 0; i < 9; i++){
        vel_cov_f[i] = imu_meas.angular_velocity_covariance[i];
        acc_cov_f[i] = imu_meas.linear_acceleration_covariance[i];
      }
      R_k.block(4,4,3,3) << Eigen::Map<Eigen::MatrixXd> (vel_cov_f,3,3);
      R_k.block(13,13,3,3) << Eigen::Map<Eigen::MatrixXd> (acc_cov_f,3,3);

      return;
    }

    void publishState(){
      tf::Transform tf_fused;

      tf_fused.setOrigin( tf::Vector3(x_k(7), x_k(8), x_k(9)) );
      tf_fused.setRotation( tf::Quaternion(x_k(0), x_k(1), x_k(2), x_k(3)) );

      tfBroadcaster.sendTransform( tf::StampedTransform(tf_fused, ros::Time::now(), "world", "fused_frame"));

      return;
    }
};
