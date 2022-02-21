#include <controller/controller.h>
#include <stdio.h>
#include <string.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~"),
  nh_param_("/" + nh_private_.param<std::string>("param_namespace", nh_private_.getNamespace()) ),
  _server(nh_param_)
{
  if (!nh_param_.getParam("equilibrium_throttle", throttle_eq_))
    ROS_ERROR("[Controller] MAV equilibrium_throttle not found!");
    
  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  max_accel_z_ = 1.0 / throttle_eq_;
  max_accel_xy_ = sin(acos(throttle_eq_)) / throttle_eq_ / sqrt(2.);

  is_flying_ = false;
  received_cmd_ = false;

  nh_param_.getParam("max_roll", max_.roll);
  nh_param_.getParam("max_pitch", max_.pitch);
  nh_param_.getParam("max_yaw_rate", max_.yaw_rate);
  nh_param_.getParam("max_throttle", max_.throttle);
  nh_param_.getParam("max_n_dot", max_.n_dot);
  nh_param_.getParam("max_e_dot", max_.e_dot);
  nh_param_.getParam("max_d_dot", max_.d_dot);

  nh_param_.getParam("min_altitude", min_altitude_);

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
  is_flying_sub_ =
      nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  cmd_sub_ =
      nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &Controller::statusCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
}


void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

  // This should already be coming in NED
  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = msg->pose.pose.position.y;
  xhat_.pd = msg->pose.pose.position.z;

  xhat_.u = msg->twist.twist.linear.x;
  xhat_.v = msg->twist.twist.linear.y;
  xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(xhat_.phi, xhat_.theta, xhat_.psi);
  xhat_.theta = xhat_.theta;
  xhat_.psi = xhat_.psi;

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = msg->twist.twist.angular.y;
  xhat_.r = msg->twist.twist.angular.z;

  if(is_flying_ && armed_ && received_cmd_)
  {
    ROS_WARN_ONCE("CONTROLLER ACTIVE");
    computeControl(dt);
    publishCommand();
  }
  else
  {
    resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
}


void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
}

void Controller::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
  armed_ = msg->armed;
}


void Controller::cmdCallback(const roscopter_msgs::CommandConstPtr &msg)
{
  switch(msg->mode)
  {
    case roscopter_msgs::Command::MODE_NPOS_EPOS_DPOS_YAW:
      xc_.pn = msg->cmd1;
      xc_.pe = msg->cmd2;
      xc_.pd = msg->cmd3;
      xc_.psi = msg->cmd4;
      control_mode_ = msg->mode;
      break;
    case roscopter_msgs::Command::MODE_NPOS_EPOS_DVEL_YAW:
      xc_.pn = msg->cmd1;
      xc_.pe = msg->cmd2;
      xc_.z_dot = msg->cmd3;
      xc_.psi = msg->cmd4;
      control_mode_ = msg->mode;
      break;
    // case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
    case roscopter_msgs::Command::MODE_NVEL_EVEL_DPOS_YAWRATE:
      xc_.x_dot = msg->cmd1;
      xc_.y_dot = msg->cmd2;
      xc_.pd = msg->cmd3;
      xc_.r = msg->cmd4;
      control_mode_ = msg->mode;
      break;
    // case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_Z_VEL:
    case roscopter_msgs::Command::MODE_NVEL_EVEL_DVEL_YAWRATE:
      xc_.x_dot = msg->cmd1;
      xc_.y_dot = msg->cmd2;
      xc_.z_dot = msg->cmd3;
      xc_.r = msg->cmd4;
      control_mode_ = msg->mode;
      break;
    // case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
    case roscopter_msgs::Command::MODE_NACC_EACC_DACC_YAWRATE:
      xc_.ax = msg->cmd1;
      xc_.ay = msg->cmd2;
      xc_.az = msg->cmd3;
      xc_.r = msg->cmd4;
      control_mode_ = msg->mode;
      break;
    default:
      ROS_ERROR("roscopter/controller: Unhandled command message of type %d",
                msg->mode);
      break;
  }

  if (!received_cmd_)
    received_cmd_ = true;
}

void Controller::reconfigure_callback(roscopter::ControllerConfig& config,
                                      uint32_t level)
{
  double P, I, D, tau;
  tau = config.tau;
  P = config.x_dot_P;
  I = config.x_dot_I;
  D = config.x_dot_D;
  PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.y_dot_P;
  I = config.y_dot_I;
  D = config.y_dot_D;
  PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.z_dot_P;
  I = config.z_dot_I;
  D = config.z_dot_D;
  // set max z accelerations so that we can't fall faster than 1 gravity
  PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);

  P = config.north_P;
  I = config.north_I;
  D = config.north_D;
  max_.n_dot = config.max_n_dot;
  PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);

  P = config.east_P;
  I = config.east_I;
  D = config.east_D;
  max_.e_dot = config.max_e_dot;
  PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);

  P = config.down_P;
  I = config.down_I;
  D = config.down_D;
  max_.d_dot = config.max_d_dot;
  PID_d_.setGains(P, I, D, tau, max_.d_dot, -max_.d_dot);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  PID_psi_.setGains(P, I, D, tau);

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;

  max_.n_dot = config.max_n_dot;
  max_.e_dot = config.max_e_dot;
  max_.d_dot = config.max_d_dot;

  throttle_eq_ = config.equilibrium_throttle;

  ROS_INFO("new gains");

  resetIntegrators();
}


void Controller::computeControl(double dt)
{
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    return;
  }

  uint8_t mode_flag = control_mode_;

  if(mode_flag == roscopter_msgs::Command::MODE_NPOS_EPOS_DPOS_YAW)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);
    double pddot_c = PID_d_.computePID(xc_.pd, xhat_.pd, dt);

    // Calculate desired yaw rate
    // First, determine the shortest direction to the commanded psi
    if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi += 2*M_PI;
    }
    else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi -= 2*M_PI;
    }
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

    xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
    xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);
    xc_.z_dot = pddot_c;

    mode_flag = roscopter_msgs::Command::MODE_NVEL_EVEL_DVEL_YAWRATE;
  }

if(mode_flag == roscopter_msgs::Command::MODE_NPOS_EPOS_DVEL_YAW)
{
  // Figure out desired velocities (in inertial frame)
  // By running the position controllers
  double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
  double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);

  // Calculate desired yaw rate
  // First, determine the shortest direction to the commanded psi
  if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
  {
    xc_.psi += 2*M_PI;
  }
  else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
  {
    xc_.psi -= 2*M_PI;
  }
  xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

  xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
  xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);

  mode_flag = roscopter_msgs::Command::MODE_NVEL_EVEL_DVEL_YAWRATE;
}

  if(mode_flag == roscopter_msgs::Command::MODE_NVEL_EVEL_DVEL_YAWRATE)
  {
    // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
    // Rotate body frame velocities to vehicle 1 frame velocities
    double sinp = sin(xhat_.phi);
    double cosp = cos(xhat_.phi);
    double sint = sin(xhat_.theta);
    double cost = cos(xhat_.theta);
    double pxdot =
        cost * xhat_.u + sinp * sint * xhat_.v + cosp * sint * xhat_.w;
    double pydot = cosp * xhat_.v - sinp * xhat_.w;
    double pddot =
        -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;

    xc_.ax = PID_x_dot_.computePID(xc_.x_dot, pxdot, dt);
    xc_.ay = PID_y_dot_.computePID(xc_.y_dot, pydot, dt);
    xc_.az = PID_z_dot_.computePID(xc_.z_dot, pddot, dt);

    mode_flag = roscopter_msgs::Command::MODE_NACC_EACC_DACC_YAWRATE;
  }

  if(mode_flag == roscopter_msgs::Command::MODE_NVEL_EVEL_DPOS_YAWRATE)
  {
    // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
    // Rotate body frame velocities to vehicle 1 frame velocities
    double sinp = sin(xhat_.phi);
    double cosp = cos(xhat_.phi);
    double sint = sin(xhat_.theta);
    double cost = cos(xhat_.theta);
    double pxdot =
        cost * xhat_.u + sinp * sint * xhat_.v + cosp * sint * xhat_.w;
    double pydot = cosp * xhat_.v - sinp * xhat_.w;
    double pddot =
        -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;

    xc_.ax = PID_x_dot_.computePID(xc_.x_dot, pxdot, dt);
    xc_.ay = PID_y_dot_.computePID(xc_.y_dot, pydot, dt);

    // Nested Loop for Altitude
    double pddot_c = PID_d_.computePID(xc_.pd, xhat_.pd, dt, pddot);
    xc_.az = PID_z_dot_.computePID(pddot_c, pddot, dt);
    mode_flag = roscopter_msgs::Command::MODE_NACC_EACC_DACC_YAWRATE;
  }

  if(mode_flag == roscopter_msgs::Command::MODE_NACC_EACC_DACC_YAWRATE)
  {
    /*
    std::cout << "xc_.x_dot " << xc_.x_dot << std::endl;
    std::cout << "xc_.z_dot " << xc_.z_dot << std::endl;
    std::cout << "xc_.ax " << xc_.ax << std::endl;
    std::cout << "xc_.az " << xc_.az << std::endl;
*/

    bool skimming_controller = true;
    if (skimming_controller){
      // New Model inversion (Forces compared in V1 frame)

      // Init useful variables
      double m = 3.69;
      double g = 9.81;
      double rho = 997.0;
      double R = 0.05;
      double h_max = .25;
      Eigen::Vector3d Cd(0.1, 0.1,0.01);

      Eigen::Matrix<double, 4, 3> rotor_positions;
      rotor_positions << 0.1907,  0.205, 0.0,
                    -0.1907,  0.205, 0.0,
                    -0.1907, -0.205, 0.0,
                     0.1907, -0.205, 0.0;

      Eigen::Matrix3d V1RB;
      V1RB << sin(xhat_.theta), sin(xhat_.phi)*sin(xhat_.theta),-cos(xhat_.phi)*sin(xhat_.theta),
                                0.0, cos(xhat_.phi), sin(xhat_.phi),
                                sin(xhat_.theta), -cos(xhat_.theta)*sin(xhat_.phi),cos(xhat_.phi)*cos(xhat_.theta);

      // Calculate Gravity Force 
      Eigen::Vector3d gravity(0.0, 0.0, m*g);

      // Calculate Inertial Command Force
      Eigen::Vector3d inertail(m*g*xc_.ax, m*g*xc_.ay, m*g*xc_.az);

      // Calculate Hydrostatic Forces for each bouy
      double V_tot = 0.0;
      for (int i = 0; i < 4; i++){ 
        // 1st find the state of bouy
        Eigen::Vector3d r_fi_pi(0.0, 0.0, 0.3);

        Eigen::Vector3d r_pi_g; 
        r_pi_g << rotor_positions(i,0), rotor_positions(i,1), rotor_positions(i,2);

        Eigen::Vector3d r_g_o(xhat_.pn, xhat_.pe,xhat_.pd); 

        Eigen::Vector3d r_fi_g = r_pi_g + r_fi_pi;
        Eigen::Vector3d r_fi_o = r_g_o + V1RB * r_fi_g;

        // 2nd find height of bouy
        double h = (r_fi_o[2] / V1RB(2,2));
        double sat_h = saturate(h,h_max,0.0);

        // 3rd find volume of displaced water
        double V = M_PI * pow(R,2) *sat_h;

        // 4th sum the dispalced volume accross all bouys 
        V_tot += V;
/*  
        std::cout << "h " << sat_h << std::endl;
        std::cout << "fi z " << r_fi_o[2] << std::endl;
        std::cout << "g z " << r_g_o[2] << std::endl;
*/  
      }
      // Finalize hydrostatic force
      Eigen::Vector3d hydrostatic; 
      hydrostatic << 0.0, 0.0, -rho*g*V_tot;

      // Calculate Hydrodynamic Forces for each bouy
      Eigen::Vector3d hydrodynamic(0.0, 0.0, 0.0);
      for (int i = 0; i < 4; i++){ 
        // 1st find the state of bouy
        Eigen::Vector3d r_fi_pi(0.0, 0.0, 0.3);

        Eigen::Vector3d r_pi_g; 
        r_pi_g << rotor_positions(i,0), rotor_positions(i,1), rotor_positions(i,2);

        Eigen::Vector3d r_g_o(xhat_.pn, xhat_.pe,xhat_.pd); 

        Eigen::Vector3d r_fi_g = r_pi_g + r_fi_pi;
        Eigen::Vector3d r_fi_o = r_g_o + V1RB * r_fi_g;

        // 2nd find height of bouy
        double h = (r_fi_o[2] / V1RB(2,2));
        double sat_h = saturate(h,h_max,0.0);

        // 3rd Find center of displaced water
        Eigen::Vector3d r_ci_fi(0.0, 0.0, h/2);
        Eigen::Vector3d r_ci_g = r_ci_fi + r_fi_g; 
        Eigen::Vector3d r_ci_o = r_g_o + V1RB * r_ci_g;

        // 4th find velocity of bouy
        Eigen::Vector3d w(xhat_.p,xhat_.q,xhat_.r);
        double sinp = sin(xhat_.phi);
        double cosp = cos(xhat_.phi);
        double sint = sin(xhat_.theta);
        double cost = cos(xhat_.theta);
        double pxdot = cost * xhat_.u + sinp * sint * xhat_.v + cosp * sint * xhat_.w;
        double pydot = cosp * xhat_.v - sinp * xhat_.w;
        double pddot = -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;
        Eigen::Vector3d v_g_o(pxdot, pydot, pddot);
        Eigen::Vector3d v_ci_o = v_g_o + w.cross(r_ci_g);

        // 5th calculate hydrodynamic drag
        double A = 2*M_PI*sat_h;
        Eigen::Vector3d D_body;
        D_body << 0.5*A*Cd(0)*pow(v_ci_o(0),2), 0.5*A*Cd(1)*pow(v_ci_o(1),2), 0.5*A*Cd(2)*pow(v_ci_o(2),2) ;

        Eigen::Vector3d D_v1 = V1RB * D_body;

        // 6th sum hydrodynamic forces
        hydrodynamic += D_v1;
      }

      // Sum forces
      Eigen::Vector3d T = gravity - inertail + hydrostatic + hydrodynamic;
      //Eigen::Vector3d T = gravity - inertail;
      Eigen::Vector3d acc_c = T / m / g;
      double total_acc_c = sqrt(pow(acc_c(0),2) + pow(acc_c(1),2) + pow(acc_c(2),2));  // (in g's)

      if (total_acc_c > 0.001)
      {
        xc_.phi = asin(-acc_c(1) / total_acc_c);
        xc_.theta = -1.0*asin(-acc_c(0) / total_acc_c);
      }
      else
      {
        xc_.phi = 0;
        xc_.theta = 0;
      }

      // Compute desired thrust based on current pose
      double cosp = cos(xhat_.phi);
      double cost = cos(xhat_.theta);
      xc_.throttle = acc_c(2) * throttle_eq_ / cosp / cost;
    }
    else{
      // Old Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
      double total_acc_c = sqrt((1.0 - xc_.az) * (1.0 - xc_.az) +
                                xc_.ax * xc_.ax + xc_.ay * xc_.ay);  // (in g's)
      if (total_acc_c > 0.001)
      {
        xc_.phi = asin(xc_.ay / total_acc_c);
        xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
      }
      else
      {
        xc_.phi = 0;
        xc_.theta = 0;
      }

      // Compute desired thrust based on current pose
      double cosp = cos(xhat_.phi);
      double cost = cos(xhat_.theta);
      xc_.throttle = (1.0 - xc_.az) * throttle_eq_ / cosp / cost;
    }

/*
    std::cout << "z/g " << (T(2)/g) << std::endl;
    std::cout << "z/g/c/c " << (T(2)/g) / cosp / cost << std::endl;
    std::cout << "Phi_c " << xc_.phi << std::endl;
    std::cout << "Theta_c " << xc_.theta << std::endl;
    std::cout << "Phi " << xhat_.phi << std::endl;
    std::cout << "Theta " << xhat_.theta << std::endl;
    std::cout << "Thottle " << T(2) / cosp / cost << std::endl;
    std::cout << "Sum Forces " << T(0) << " " << T(1) << " " << T(2) << std::endl;
    std::cout << "Gravity " << gravity(0) << " " << gravity(1) << " " << gravity(2) << std::endl;
    std::cout << "inertial " << inertail(0) << " " << inertail(1) << " " << inertail(2) << std::endl;
    std::cout << "hydrostatic " << hydrostatic(0) << " " << hydrostatic(1) << " " << hydrostatic(2) << std::endl;
*/



    mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command
    command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = saturate(xc_.phi, max_.roll, -max_.roll);
    command_.y = saturate(xc_.theta, max_.pitch, -max_.pitch);
    command_.z = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
  }
}

void Controller::publishCommand()
{
  command_.header.stamp = ros::Time::now();
  command_pub_.publish(command_);
}

void Controller::resetIntegrators()
{
  PID_x_dot_.clearIntegrator();
  PID_y_dot_.clearIntegrator();
  PID_z_dot_.clearIntegrator();
  PID_n_.clearIntegrator();
  PID_e_.clearIntegrator();
  PID_d_.clearIntegrator();
  PID_psi_.clearIntegrator();
}

double Controller::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

}  // namespace controller
