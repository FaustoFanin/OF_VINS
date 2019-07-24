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
    time_t prev_time;
    double Ts;

    // Filter matrices/vector
    Eigen::Matrix<double,STATES,1> x_k;         // State vector [q,p,v]
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
    Eigen::Matrix<double,MSRMTS,1> z_k;         // Sensor measurement
    Eigen::Matrix<double,3,3> C_k;              // Inertial-to-Body Rotation
    Eigen::Matrix<double,4,4> Omega_k;          // Skew symmetric matrix associated with ω

    // For easy access
    Eigen::Matrix<double,1,3> w_k;    // Rotational velocity vector [rad/s]
    Eigen::Matrix<double,1,3> v_k;    // Linear velocity vector [m/s]
    Eigen::Matrix<double,1,3> a_k;    // Linear acceleration vector [m/s2]
    Eigen::Matrix<double,3,3> I3 = Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix<double,4,4> I4 = Eigen::Matrix<double,4,4>::Identity();

  public:
    NavFilter(int Hz): dt(1.0/Hz){
      // Initialise everything
      x_k.setZero();  // States all start at 0

      P_k.setIdentity();
      P_k.block(0,0,4,4).diagonal() << 0.5, 0.5, 0.5, 0.5;  // Quaternion covariance
      P_k.block(4,4,3,3).diagonal() << 1, 1, 1;     // Position covariance
      P_k.block(7,7,3,3).diagonal() << 1, 1, 1;     // Linear velocity covariance

      F_k.setZero();

      // Observation Model
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
      R_k.block( 0, 0, 3, 3).diagonal() << 1, 1, 1;

      // Subscribtions
      sub_pos = node.subscribe("/raw_pos", 10, &NavFilter::posCallback, this);
      sub_vel = node.subscribe("/raw_vel", 10, &NavFilter::velCallback, this);
      sub_imu = node.subscribe("/raw_imu/bias", 10, &NavFilter::imuCallback, this);
    }

    Eigen::Matrix<double,STATES,1> propagateFilter(){
      // Update omega with current timestep
      updateOmega();

      Ts = difftime(time(NULL),prev_time);    // Calculate timestep
      prev_time = time(NULL);
      double mag_w = sqrt(w_k(0)*w_k(0) + w_k(1)*w_k(1) + w_k(2)*w_k(2));

      // Update F_k
      F_k.setZero();
      F_k.block(0,0,4,4) = (cos(0.5*Ts*mag_w)*I4 + sin(0.5*Ts*mag_w)*Omega_k/mag_w);  // Quat. propagation
      F_k.block(4,4,3,3) = Eigen::Matrix<double,3,3>::Identity();   // Position propagation
      F_k.block(4,7,3,3) = Ts * Eigen::Matrix<double,3,3>::Identity();  // Vel. component of Pos. prop.
      F_k.block(7,7,3,3) = Eigen::Matrix<double,3,3>::Identity();   // Velocity Propagation

      // Update B_k
      B_k.setZero();
      B_k.block(4,3,3,3) = 0.5*Ts*Ts*C_k.transpose(); // Acceleration component of position propagation
      B_k.block(7,3,3,3) = Ts*C_k.transpose();    // Acceleration component of velocity propagation


      /*// Propagate state manually
      x_k_p.block(0,0,4,1) = (cos(0.5*Ts*mag_w)*I4 + sin(0.5*Ts*mag_w)*Omega_k/mag_w) * x_k.block(0,0,4,1);   // Quaternion propagation
      x_k_p.block(4,0,3,1) = x_k.block(4,0,3,1) + Ts*x_k.block(7,0,3,1) + 0.5*Ts*Ts*(C_k*a_k);  // Position propagation
      x_k_p.block(7,0,3,1) = x_k.block(7,0,3,1) + Ts*(C_k*a_k);   // Velocity propagation*/

      // Propagate state and covariances
      x_k_p = F_k * x_k + B_k * u_k;
      P_k_p = F_k * P_k * F_k.transpose() + Q_k;    // B_k * Q_k * B_k.transpose() ??????

      // IMU propagation, will be fixed when measurement occurs
      x_k = x_k_p;
      P_k = P_k_p;

      // Update C_k with previous timestep
      updateRotationMatrix();

      return x_k;
    }

    void correctFilter(){
      // Innovation
      y_k = z_k - H_k * x_k_p;
      S_k = R_k + H_k * P_k_p * H_k.transpose();
      K_k = P_k_p * H_k.transpose() * S_k.inverse();

      // Update
      x_k = x_k_p + K_k * y_k;
      P_k = (Eigen::Matrix<double,STATES,STATES>::Identity() - K_k * H_k) * P_k;

      // Publish State
      NavFilter::publishState();

      return;
    }

    void posCallback(const geometry_msgs::Point& pos_meas){
      z_k(0) = pos_meas.x;
      z_k(1) = pos_meas.y;
      z_k(2) = pos_meas.z;

      return;
    }

    void velCallback(const geometry_msgs::TwistWithCovariance& vel_meas){
      // Linear velocity measurement
      v_k(0) = vel_meas.twist.linear.x;
      v_k(1) = vel_meas.twist.linear.y;
      v_k(2) = vel_meas.twist.linear.z;

      // Linear velocity covariance
      /*R_k(1,1) = vel_meas.covariance[1];
      R_k(2,2) = vel_meas.covariance[8];
      R_k(3,3) = vel_meas.covariance[16];*/

      return;
    }

    void imuCallback(const sensor_msgs::Imu& imu_meas){
      // Quaterions
      /*z_k(0) = imu_meas.orientation.x;
      z_k(1) = imu_meas.orientation.y;
      z_k(2) = imu_meas.orientation.z;
      z_k(3) = imu_meas.orientation.w;*/

      // Angular velocity
      w_k(0) = imu_meas.angular_velocity.x;
      w_k(1) = imu_meas.angular_velocity.y;
      w_k(2) = imu_meas.angular_velocity.z;

      // Linear acceleration
      a_k(0) = imu_meas.linear_acceleration.x;
      a_k(1) = imu_meas.linear_acceleration.y;
      a_k(2) = imu_meas.linear_acceleration.z;

      // Update input vector
      u_k.block(0,0,3,1) = w_k;
      u_k.block(0,3,3,1) = a_k;

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

      return;
    }

    void publishState(){
      tf::Transform tf_fused;

      tf_fused.setOrigin( tf::Vector3(x_k(4), x_k(5), x_k(6)) );
      tf_fused.setRotation( tf::Quaternion(x_k(0), x_k(1), x_k(2), x_k(3)) );

      tfBroadcaster.sendTransform( tf::StampedTransform(tf_fused, ros::Time::now(), "world", "fused_frame"));

      return;
    }

    void updateRotationMatrix(){
      C_k << 1 - 2*x_k(2)*x_k(2) - 2*x_k(3)*x_k(3),
             2*x_k(1)*x_k(2) - 2*x_k(3)*x_k(0),
             2*x_k(1)*x_k(3) + 2*x_k(2)*x_k(0),
             2*x_k(1)*x_k(2) + 2*x_k(3)*x_k(0),
             1 - 2*x_k(1)*x_k(1) - 2*x_k(3)*x_k(3),
             2*x_k(2)*x_k(3) - 2*x_k(1)*x_k(0),
             2*x_k(1)*x_k(3) - 2*x_k(2)*x_k(0),
             2*x_k(2)*x_k(3) + 2*x_k(1)*x_k(0),
             1 - 2*x_k(1)*x_k(1) - 2*x_k(2)*x_k(2);
      return;
    }

    void updateOmega(){
      /*Omega_k <<     1, -x_k(4), -x_k(5), -x_k(6),
                x_k(4),       1,  x_k(6), -x_k(5),
                x_k(5), -x_k(6),       1, -x_k(4),
                x_k(6),  x_k(5), -x_k(4),       1;*/
      Omega_k <<     0,  w_k(2), -w_k(1), w_k(0),
               -w_k(2),       0,  w_k(0), w_k(1),
                w_k(1), -w_k(0),       0, w_k(2),
               -w_k(0), -w_k(1), -w_k(2),      0;
      return;
    }
};
