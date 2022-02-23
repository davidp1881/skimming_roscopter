/*
 * Copyright 2016 James Jackson, Brigham Young University, Provo UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "roscopter_sim/multirotor_forces_and_moments.h"

namespace gazebo
{

MultiRotorForcesAndMoments::MultiRotorForcesAndMoments()
{

}


MultiRotorForcesAndMoments::~MultiRotorForcesAndMoments()
{
  GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}


void MultiRotorForcesAndMoments::SendForces()
{
  // apply the forces and torques to the joint
  // Gazebo is in NWU, while we calculate forces in NED, hence the negatives
  link_->AddRelativeForce(GazeboVector(actual_forces_.Fx, -actual_forces_.Fy, -actual_forces_.Fz));
  link_->AddRelativeTorque(GazeboVector(actual_forces_.l, -actual_forces_.m, -actual_forces_.n));
}


void MultiRotorForcesAndMoments::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[multirotor_forces_and_moments] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);
  nh_private_ = ros::NodeHandle(namespace_ + "/dynamics");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[multirotor_forces_and_moments] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[multirotor_forces_and_moments] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "windTopic", wind_topic_, "wind");
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");
  getSdfParam<std::string>(_sdf, "attitudeTopic", attitude_topic_, "attitude");

  /* Load Params from ROS Server */
  mass_ = nh_private_.param<double>("mass", 3.856);
  linear_mu_ = nh_private_.param<double>("linear_mu", 0.1);
  angular_mu_ = nh_private_.param<double>("angular_mu", 0.5);

  // Drag Constant
  linear_mu_ = nh_private_.param<double>( "linear_mu", 0.8);
  angular_mu_ = nh_private_.param<double>( "angular_mu", 0.5);

  /* Ground Effect Coefficients */
  std::vector<double> ground_effect_list = {-55.3516, 181.8265, -203.9874, 85.3735, -7.6619};
  nh_private_.getParam("ground_effect", ground_effect_list);
  ground_effect_.a = ground_effect_list[0];
  ground_effect_.b = ground_effect_list[1];
  ground_effect_.c = ground_effect_list[2];
  ground_effect_.d = ground_effect_list[3];
  ground_effect_.e = ground_effect_list[4];

  // Build Actuators Container
  actuators_.l.max = nh_private_.param<double>("max_l", .2); // N-m
  actuators_.m.max = nh_private_.param<double>("max_m", .2); // N-m
  actuators_.n.max = nh_private_.param<double>("max_n", .2); // N-m
  actuators_.F.max = nh_private_.param<double>("max_F", 1.0); // N
  actuators_.l.tau_up = nh_private_.param<double>("tau_up_l", .25);
  actuators_.m.tau_up = nh_private_.param<double>("tau_up_m", .25);
  actuators_.n.tau_up = nh_private_.param<double>("tau_up_n", .25);
  actuators_.F.tau_up = nh_private_.param<double>("tau_up_F", 0.25);
  actuators_.l.tau_down = nh_private_.param<double>("tau_down_l", .25);
  actuators_.m.tau_down = nh_private_.param<double>("tau_down_m", .25);
  actuators_.n.tau_down = nh_private_.param<double>("tau_down_n", .25);
  actuators_.F.tau_down = nh_private_.param<double>("tau_down_F", 0.35);

  // Get PID Gains
  double rollP, rollI, rollD;
  double pitchP, pitchI, pitchD;
  double yawP, yawI, yawD;
  double altP, altI, altD;
  rollP = nh_private_.param<double>("roll_P", 0.1);
  rollI = nh_private_.param<double>("roll_I", 0.0);
  rollD = nh_private_.param<double>("roll_D", 0.0);
  pitchP = nh_private_.param<double>("pitch_P", 0.1);
  pitchI = nh_private_.param<double>("pitch_I", 0.0);
  pitchD = nh_private_.param<double>("pitch_D", 0.0);
  yawP = nh_private_.param<double>("yaw_P", 0.1);
  yawI = nh_private_.param<double>("yaw_I", 0.0);
  yawD = nh_private_.param<double>("yaw_D", 0.0);
  altP = nh_private_.param<double>("alt_P", 0.1);
  altI = nh_private_.param<double>("alt_I", 0.0);
  altD = nh_private_.param<double>("alt_D", 0.0);

  roll_controller_.setGains(rollP, rollI, rollD);
  pitch_controller_.setGains(pitchP, pitchI, pitchD);
  yaw_controller_.setGains(yawP, yawI, yawD);
  alt_controller_.setGains(altP, altI, altD);

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MultiRotorForcesAndMoments::OnUpdate, this, _1));

  // Connect Subscribers
  command_sub_ = nh_->subscribe(command_topic_, 1, &MultiRotorForcesAndMoments::CommandCallback, this);
  wind_sub_ = nh_->subscribe(wind_topic_, 1, &MultiRotorForcesAndMoments::WindCallback, this);

  // Connect Publishers
  attitude_pub_ = nh_->advertise<rosflight_msgs::Attitude>(attitude_topic_, 1);

  // Initialize State
  this->Reset();
}

// This gets called by the world update event.
void MultiRotorForcesAndMoments::OnUpdate(const common::UpdateInfo& _info) {

  if (!cmd_valid_)
    return;

  if (prev_sim_time_ > 0.)
  {
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    UpdateForcesAndMoments();
    SendForces();
  }

  prev_sim_time_ = _info.simTime.Double();
}

void MultiRotorForcesAndMoments::WindCallback(const geometry_msgs::Vector3 &wind){
  GZ_COMPAT_SET_X(W_wind_ , wind.x);
  GZ_COMPAT_SET_Y(W_wind_ , wind.y);
  GZ_COMPAT_SET_Z(W_wind_ , wind.z);
}

void MultiRotorForcesAndMoments::CommandCallback(const rosflight_msgs::Command msg)
{
  command_ = msg;

  if (!cmd_valid_)
    cmd_valid_ = true;
}

void MultiRotorForcesAndMoments::Reset()
{
  cmd_valid_ = false;

  // Re-Initialize Memory Variables
  applied_forces_.Fx = 0;
  applied_forces_.Fy = 0;
  applied_forces_.Fz = 0;
  applied_forces_.l = 0;
  applied_forces_.m = 0;
  applied_forces_.n = 0;

  actual_forces_.Fx = 0;
  actual_forces_.Fy = 0;
  actual_forces_.Fz = 0;
  actual_forces_.l = 0;
  actual_forces_.m = 0;
  actual_forces_.n = 0;

  prev_sim_time_ = -1.0;
  sampling_time_ = -1.0;

  command_.mode = -1;

  // teleport the MAV to the initial position and reset it
  // link_->SetWorldPose(initial_pose_);
  // link_->ResetPhysicsStates();
}


void MultiRotorForcesAndMoments::UpdateForcesAndMoments()
{
  /* Get state information from Gazebo                          *
   * C denotes child frame, P parent frame, and W world frame.  *
   * Further C_pose_W_P denotes pose of P wrt. W expressed in C.*/
  // all coordinates are in standard aeronatical frame NED
  GazeboPose W_pose_W_C = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  double pn = GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(W_pose_W_C));
  double pe = -GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(W_pose_W_C));
  double pd = -GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(W_pose_W_C));
  GazeboVector euler_angles = GZ_COMPAT_GET_EULER(GZ_COMPAT_GET_ROT(W_pose_W_C));
  double phi = GZ_COMPAT_GET_X(euler_angles);
  double theta = -GZ_COMPAT_GET_Y(euler_angles);
  double psi = -GZ_COMPAT_GET_Z(euler_angles);
  GazeboVector C_linear_velocity_W_C = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  double u = GZ_COMPAT_GET_X(C_linear_velocity_W_C);
  double v = -GZ_COMPAT_GET_Y(C_linear_velocity_W_C);
  double w = -GZ_COMPAT_GET_Z(C_linear_velocity_W_C);
  GazeboVector C_angular_velocity_W_C = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);
  double p = GZ_COMPAT_GET_X(C_angular_velocity_W_C);
  double q = -GZ_COMPAT_GET_Y(C_angular_velocity_W_C);
  double r = -GZ_COMPAT_GET_Z(C_angular_velocity_W_C);

  // wind info is available in the wind_ struct
  // Rotate into body frame and relative velocity
  GazeboVector C_wind_speed = GZ_COMPAT_GET_ROT(W_pose_W_C).RotateVector(W_wind_);
  double ur = u - GZ_COMPAT_GET_X(C_wind_speed);
  double vr = v - GZ_COMPAT_GET_Y(C_wind_speed);
  double wr = w - GZ_COMPAT_GET_Z(C_wind_speed);

  // calculate the appropriate control <- Depends on Control type (which block is being controlled)
  if (command_.mode < 0)
  {
    // We have not received a command yet.  This is not an error, but needs to be handled
  }
  else if (command_.mode == rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
  {
    desired_forces_.l = roll_controller_.computePID(command_.x, p, sampling_time_);
    desired_forces_.m = pitch_controller_.computePID(command_.y, q, sampling_time_);
    desired_forces_.n = yaw_controller_.computePID(command_.z, r, sampling_time_);
    desired_forces_.Fz = command_.F*actuators_.F.max; // this comes in normalized between 0 and 1
  }
  else if (command_.mode == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    desired_forces_.l = roll_controller_.computePID(command_.x, phi, sampling_time_, p);
    desired_forces_.m = pitch_controller_.computePID(command_.y, theta, sampling_time_, q);
    desired_forces_.n = yaw_controller_.computePID(command_.z, r, sampling_time_);
    desired_forces_.Fz = command_.F*actuators_.F.max;
  }

  // calculate the actual output force using low-pass-filters to introduce a first-order
  // approximation of delay in motor reponse
  // x(t+1) = Ce^(-t/tau)dt <- transfer to z-domain using backward differentiation

  // first get the appropriate tau for this situation
  double taul = (desired_forces_.l > applied_forces_.l ) ? actuators_.l.tau_up : actuators_.l.tau_down;
  double taum = (desired_forces_.m > applied_forces_.m ) ? actuators_.m.tau_up : actuators_.m.tau_down;
  double taun = (desired_forces_.n > applied_forces_.n ) ? actuators_.n.tau_up : actuators_.n.tau_down;
  double tauF = (desired_forces_.Fz > applied_forces_.Fz ) ? actuators_.F.tau_up : actuators_.F.tau_down;

  // calulate the alpha for the filter
  double alphal = sampling_time_/(taul + sampling_time_);
  double alpham = sampling_time_/(taum + sampling_time_);
  double alphan = sampling_time_/(taun + sampling_time_);
  double alphaF = sampling_time_/(tauF + sampling_time_);

  // Apply the discrete first-order filter
  applied_forces_.l = sat((1 - alphal)*applied_forces_.l + alphal *desired_forces_.l, actuators_.l.max, -1.0*actuators_.l.max);
  applied_forces_.m = sat((1 - alpham)*applied_forces_.m + alpham *desired_forces_.m, actuators_.m.max, -1.0*actuators_.m.max);
  applied_forces_.n = sat((1 - alphan)*applied_forces_.n + alphan *desired_forces_.n, actuators_.n.max, -1.0*actuators_.n.max);
  applied_forces_.Fz = sat((1 - alphaF)*applied_forces_.Fz + alphaF *desired_forces_.Fz, actuators_.F.max, 0.0);

  bool skimming_physics = true;
  if (skimming_physics){
    // Init useful variables
    double m = 3.69;
    double g = 9.81;
    double rho = 997.0;
    double R = 0.05;
    double r_prop = 0.1;
    double h_max = .25;
    double Ca = .44;
    double Cb = 1.8;
    Eigen::Vector3d Cd(0.1, 0.1,0.01);

    Eigen::Matrix<double, 4, 3> rotor_positions;
    rotor_positions << 0.1907,  0.205, 0.0,
                  -0.1907,  0.205, 0.0,
                  -0.1907, -0.205, 0.0,
                   0.1907, -0.205, 0.0;

    Eigen::Matrix3d V1RB;
    V1RB << sin(theta), sin(phi)*sin(theta),-cos(phi)*sin(theta),
                        0.0, cos(phi), sin(phi),
                        sin(theta), -cos(theta)*sin(phi),cos(phi)*cos(theta);

    // Calculate Gravity Force 
    Eigen::Vector3d gravity(0.0, 0.0, m*g);

    // Calculate Aerodynamic Drag
    Eigen::Vector3d aerodrag_B(-1.0*linear_mu_*ur, -1.0*linear_mu_*vr, -1.0*linear_mu_*wr);
    Eigen::Vector3d aerodrag = V1RB * aerodrag_B;
    Eigen::Vector3d aerodrag_moment(-1.0*angular_mu_*p, -1.0*angular_mu_*q, -1.0*angular_mu_*r);
    Eigen::Vector3d rotor_drag_moment(applied_forces_.l, applied_forces_.m, applied_forces_.n);

    // Calculate Surface Effect
    Eigen::Vector3d T_B(0, 0, - applied_forces_.Fz);
    double SE = 0.0;
    for (int i = 0; i < 4; i++){ 
      // 1st find the state of bouy    
      Eigen::Vector3d r_pi_g; 
      r_pi_g << rotor_positions(i,0), rotor_positions(i,1), rotor_positions(i,2);

      Eigen::Vector3d r_g_o(pn, pe,pd); 
      Eigen::Vector3d r_pi_o = r_g_o + V1RB * r_pi_g;

      // 2nd find height of rotor
      double z = abs(r_pi_o[2] / V1RB(2,2));

      // 3rd sum the surface effects 
      SE += 1+Ca*exp(-Cb*z/r_prop);
    }
    //Eigen::Vector3d T = V1RB * (T_B*(SE/4));
    Eigen::Vector3d T = V1RB * T_B;

    // Calculate Hydrostatic Forces for each bouy
    Eigen::Vector3d hydrostatic(0.0, 0.0, 0.0);
    Eigen::Vector3d hydrostatic_moment(0.0, 0.0, 0.0);
    for (int i = 0; i < 4; i++){ 
      // 1st find the state of bouy
      Eigen::Vector3d r_fi_pi(0.0, 0.0, 0.3);
      Eigen::Vector3d r_pi_g; 
      r_pi_g << rotor_positions(i,0), rotor_positions(i,1), rotor_positions(i,2);
      Eigen::Vector3d r_g_o(pn, pe,pd); 
      Eigen::Vector3d r_fi_g = r_pi_g + r_fi_pi;
      Eigen::Vector3d r_fi_o = r_g_o + V1RB * r_fi_g;

      // 2nd find height of bouy
      double h = (r_fi_o[2] / V1RB(2,2));
      double sat_h = sat(h,h_max,0.0);

      // 3rd Define other points
      Eigen::Vector3d r_ci_fi(0.0, 0.0, h/2);
      Eigen::Vector3d r_ci_g = r_ci_fi + r_fi_g; 
      Eigen::Vector3d r_ci_o = r_g_o + V1RB * r_ci_g;

      // 4th find volume of displaced water
      double V = M_PI * pow(R,2) *sat_h;

      // 5th sum the dispalced volume accross all bouys 
      Eigen::Vector3d hydrostatic_I(0.0, 0.0, -rho*g*V);
      Eigen::Vector3d hydrostatic_B = V1RB.transpose() * hydrostatic_I;
      hydrostatic += hydrostatic_I;
      hydrostatic_moment += r_ci_g.cross(hydrostatic_B);
    }

    // Calculate Hydrodynamic Forces for each bouy
    Eigen::Vector3d hydrodynamic(0.0, 0.0, 0.0);
    Eigen::Vector3d hydrodynamic_moment(0.0, 0.0, 0.0);
    for (int i = 0; i < 4; i++){ 
      // 1st find the state of bouy
      Eigen::Vector3d r_fi_pi(0.0, 0.0, 0.3);
      Eigen::Vector3d r_pi_g; 
      r_pi_g << rotor_positions(i,0), rotor_positions(i,1), rotor_positions(i,2);
      Eigen::Vector3d r_g_o(pn, pe, pd); 
      Eigen::Vector3d r_fi_g = r_pi_g + r_fi_pi;
      Eigen::Vector3d r_fi_o = r_g_o + V1RB * r_fi_g;

      // 2nd find height of bouy
      double h = (r_fi_o[2] / V1RB(2,2));
      double sat_h = sat(h,h_max,0.0);

      // 3rd Find center of displaced water
      Eigen::Vector3d r_ci_fi(0.0, 0.0, h/2);
      Eigen::Vector3d r_ci_g = r_ci_fi + r_fi_g; 
      Eigen::Vector3d r_ci_o = r_g_o + V1RB * r_ci_g;

      // 4th find velocity of bouy
      Eigen::Vector3d angular_rates(p,q,r);
      double sinp = sin(phi);
      double cosp = cos(phi);
      double sint = sin(theta);
      double cost = cos(theta);
      double pxdot = cost * u + sinp * sint * v + cosp * sint * w;
      double pydot = cosp * v - sinp * w;
      double pddot = -sint * u + sinp * cost * v + cosp * cost * w;
      Eigen::Vector3d v_g_o(pxdot, pydot, pddot);
      Eigen::Vector3d v_ci_o = v_g_o + angular_rates.cross(r_ci_g);

      // 5th calculate hydrodynamic drag
      double A = 2*M_PI*sat_h;
      Eigen::Vector3d D_body;
      D_body << 0.5*A*Cd(0)*pow(v_ci_o(0),2), 0.5*A*Cd(1)*pow(v_ci_o(1),2), 0.5*A*Cd(2)*pow(v_ci_o(2),2) ;
      Eigen::Vector3d D_v1 = V1RB * D_body;

      // 6th sum hydrodynamic forces
      hydrodynamic += D_v1;
      hydrodynamic_moment += r_ci_g.cross(D_body);
    }

    // Sum forces (Note, gravity handled in gazebo, not in this script.)
    Eigen::Vector3d sum_force =  T + hydrostatic + hydrodynamic + aerodrag;
    //Eigen::Vector3d sum_force =  T + aerodrag;
    Eigen::Vector3d sum_force_B = V1RB.transpose() * sum_force;

    // Sum Moments
    Eigen::Vector3d sum_moments = aerodrag_moment + rotor_drag_moment + hydrostatic_moment + hydrodynamic_moment;
    //Eigen::Vector3d sum_moments = aerodrag_moment + rotor_drag_moment;


    actual_forces_.Fx = sum_force_B(0);
    actual_forces_.Fy = sum_force_B(1);
    actual_forces_.Fz = sum_force_B(2);
    actual_forces_.l = sum_moments(0);
    actual_forces_.m = sum_moments(1);
    actual_forces_.n = sum_moments(2);

    /*
    std::cout << "Phi " << phi << std::endl;
    std::cout << "Theta " << theta << std::endl;
    std::cout << "Psi " << psi << std::endl;
    std::cout << "Sum Forces " << sum_force(0) << " " << sum_force(1) << " " << sum_force(2) << std::endl;
    std::cout << "Gravity " << gravity(0) << " " << gravity(1) << " " << gravity(2) << std::endl;
    std::cout << "hydrostatic " << hydrostatic(0) << " " << hydrostatic(1) << " " << hydrostatic(2) << std::endl;
    std::cout << "aerodrag " << aerodrag(0) << " " << aerodrag(1) << " " << aerodrag(2) << std::endl;
    std::cout << "Thrust " << T(0) << " " << T(1) << " " << T(2) << std::endl;  
    std::cout << "hydrostatic " << hydrostatic(0) << " " << hydrostatic(1) << " " << hydrostatic(2) << std::endl;
    std::cout << "hydrodynamic " << hydrodynamic(0) << " " << hydrodynamic(1) << " " << hydrodynamic(2) << std::endl;
    std::cout << "aerodrag " << aerodrag(0) << " " << aerodrag(1) << " " << aerodrag(2) << std::endl;
    std::cout << "hydrostatic " << hydrostatic(0) << " " << hydrostatic(1) << " " << hydrostatic(2) << std::endl;
  */
  }
  else{
    // Apply other forces (wind) <- follows "Quadrotors and Accelerometers - State Estimation With an Improved Dynamic Model"
    // By Rob Leishman et al. (Remember NED)
    
    // calculate ground effect
    double z = -pd;
    double ground_effect = max(ground_effect_.a*z*z*z*z + ground_effect_.b*z*z*z + ground_effect_.c*z*z + ground_effect_.d*z + ground_effect_.e, 0);

    actual_forces_.Fx = -1.0*linear_mu_*ur;
    actual_forces_.Fy = -1.0*linear_mu_*vr;
    actual_forces_.Fz = -1.0*linear_mu_*wr - applied_forces_.Fz - ground_effect;
    actual_forces_.l = -1.0*angular_mu_*p + applied_forces_.l;
    actual_forces_.m = -1.0*angular_mu_*q + applied_forces_.m;
    actual_forces_.n = -1.0*angular_mu_*r + applied_forces_.n;
  }
  



  // publish attitude like ROSflight
  rosflight_msgs::Attitude attitude_msg;
  common::Time current_time  = GZ_COMPAT_GET_SIM_TIME(world_);
  attitude_msg.header.stamp.sec = current_time.sec;
  attitude_msg.header.stamp.nsec = current_time.nsec;
  attitude_msg.attitude.w =  GZ_COMPAT_GET_W(GZ_COMPAT_GET_ROT(W_pose_W_C));
  attitude_msg.attitude.x =  GZ_COMPAT_GET_X(GZ_COMPAT_GET_ROT(W_pose_W_C));
  attitude_msg.attitude.y = -GZ_COMPAT_GET_Y(GZ_COMPAT_GET_ROT(W_pose_W_C));
  attitude_msg.attitude.z = -GZ_COMPAT_GET_Z(GZ_COMPAT_GET_ROT(W_pose_W_C));

  attitude_msg.angular_velocity.x = p;
  attitude_msg.angular_velocity.y = q;
  attitude_msg.angular_velocity.z = r;

  attitude_pub_.publish(attitude_msg);
}

double MultiRotorForcesAndMoments::sat(double x, double max, double min)
{
  if(x > max)
    return max;
  else if(x < min)

    return min;
  else
    return x;
}

double MultiRotorForcesAndMoments::max(double x, double y)
{
  return (x > y) ? x : y;
}

GZ_REGISTER_MODEL_PLUGIN(MultiRotorForcesAndMoments);
}
