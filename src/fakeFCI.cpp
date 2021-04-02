/*
 * fakeFCI.cpp
 *
 *  Created on: Apr 16, 2020
 *      Author: oliva
 */

#include "fakeFCI.h"
using namespace KDL;

fakeFCI::fakeFCI():_q(7,0),_dq(7,0),_tau_J(7,0), _q_(7), _q_des_(7),_dq_des_(7),
                   _q_des(7,0),_p_cart_des(6,0),_dq_des(7,0),_dq_des_filt(7,0),
                   _v_cart_des(6,0),_tau_J_des(7,0), _tau_J_des_filt(7,0),
                   _ft_cart_des(6,0),_q_min_(7),_q_max_(7),_eMc(){

//  _chain_.addSegment(Segment(Joint(Joint::None),Frame(Vector(0, 0, 0.333))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation(1,0,0,0,0,1,0,-1,0))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation(1,0,0,0,0,-1,0,1,0),Vector(0.0,-0.316,0.0))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation(1,0,0,0,0,-1,0,1,0),Vector(0.0825,0.0,0.0))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation(1,0,0,0,0,1,0,-1,0),Vector(-0.0825,0.384,0.0))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation(1,0,0,0,0,-1,0,1,0))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation(1,0,0,0,0,-1,0,1,0),Vector(0.088,0.0,0.0))));
//  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.107))));

  _chain_.addSegment(Segment(Joint(Joint::None),Frame::DH_Craig1989( 0.0   ,  0.0  ,0.333,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989( 0.0   ,-M_PI_2,0.0  ,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989( 0.0   , M_PI_2,0.316,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989( 0.0825, M_PI_2,0.0  ,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989(-0.0825,-M_PI_2,0.384,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989( 0.0   , M_PI_2,0.0  ,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989( 0.088 , M_PI_2,0.0  ,0.0)));
  _chain_.addSegment(Segment(Joint(Joint::RotZ),Frame::DH_Craig1989( 0.0   ,  0.0  ,0.107,0.0)));

  _q_min_(0) = -2.8973; _q_min_(1) = -1.7628; _q_min_(2) = -2.8973;
  _q_min_(3) = -3.0718; _q_min_(4) = -2.8973; _q_min_(5) = -0.0175;
  _q_min_(6) = -2.8973;
  _q_max_(0) =  2.8973; _q_max_(1) =  1.7628; _q_max_(2) =  2.8973;
  _q_max_(3) = -0.0698; _q_max_(4) =  2.8973; _q_max_(5) =  3.7525;
  _q_max_(6) =  2.8973;

  _fksolver_ = new ChainFkSolverPos_recursive(_chain_);
  _JacobianSolver_ = new ChainJntToJacSolver(_chain_);
//  _DiffIkSolver = new ChainIkSolverVel_pinv_givens (_chain_);
  _DiffIkSolver_ = new ChainIkSolverVel_pinv (_chain_);
//  _DiffIkSolver_ = new ChainIkSolverVel_pinv_nso (_chain_);
  _iksolver_ = new ChainIkSolverPos_NR(_chain_,*(_fksolver_),*(_DiffIkSolver_),500,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
  _iksolver_JL_ = new ChainIkSolverPos_NR_JL(_chain_,_q_min_,_q_max_,*(_fksolver_),*(_DiffIkSolver_),100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
//  _DiffIkSolver = new ChainIkSolverVel_pinv_nso (_chain_);
//  _DiffIkSolver = new ChainIkSolverVel_wdls (_chain_);

  _connected = false;
  _stateRobot = vpRobot::STATE_STOP ;

  _posControlThreadIsRunning = false;
  _posControlThreadStopAsked = false;
  _velControlThreadIsRunning = false;
  _velControlThreadStopAsked = false;
  _ftControlThreadIsRunning = false;
  _ftControlThreadStopAsked = false;
  _posControlLock = false;
  _posControlNewCmd = false;
  //  _q_= JntArray(_chain_.getNrOfJoints());
  //  for(unsigned int i=0;i<_chain_.getNrOfJoints();i++){
  //    _q_(i)=_q[i];
  //  }
//    _fksolver_.ChainFkSolverPos_recursive(_chain_);
//    _JacobianSolver_.ChainJntToJacSolver(_chain_);
  //  _fJe_.data.setZero(6,7);
  //
  //  _JacobianSolver_.JntToJac(_q_,_fJe_);


}

fakeFCI::~fakeFCI() {
  std::cout << "~fakeFCI() destructor called" << std::endl;
  _mutex.lock();
  _connected = false;
  _posControlThreadStopAsked = true;
  _velControlThreadStopAsked = true;
  _ftControlThreadStopAsked = true;
  _mutex.unlock();
  if (_acquisitionThread.joinable()) {
    _acquisitionThread.join();
  }
  if (_controlThread.joinable()) {
    _controlThread.join();
    _posControlThreadStopAsked = false;
    _posControlThreadIsRunning = false;
    _velControlThreadStopAsked = false;
    _velControlThreadIsRunning = false;
    _ftControlThreadStopAsked = false;
    _ftControlThreadIsRunning = false;
  }
}

vpColVector fakeFCI::getVelDes(){
  return this->_dq_des_filt;
};

void fakeFCI::connect(){
  _mutex.lock();
  _connected = true;
  _acquisitionThread = std::thread([this] {this->readingLoop();});
  _mutex.unlock();
};

void fakeFCI::set_eMc(const vpHomogeneousMatrix &eMc){
  std::lock_guard<std::mutex> lock(_mutex);
  _eMc = eMc;
  _eTc.buildFrom(_eMc);
};

vpRobot::vpRobotStateType fakeFCI::getRobotState(void){

  return this->_stateRobot;
};

vpRobot::vpRobotStateType fakeFCI::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is velocity or force/torque
    if (vpRobot::STATE_POSITION_CONTROL == getRobotState()) {
          std::cout << "Change the control mode from position control to stop.\n";
          _posControlThreadStopAsked = true;
        }
    else if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity control to stop.\n";
      _velControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from force/torque control to stop.\n";
      _ftControlThreadStopAsked = true;
    }

    if(_controlThread.joinable()) {
      _controlThread.join();
      _posControlThreadStopAsked = false;
      _posControlThreadIsRunning = false;
      _velControlThreadStopAsked = false;
      _velControlThreadIsRunning = false;
      _ftControlThreadStopAsked = false;
      _ftControlThreadIsRunning = false;
    }
    this->_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_STOP == getRobotState()) {
        std::cout << "Change the control mode from stop to position control.\n";
    }
    else if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      // Stop the velocity control loop
      _velControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from force/torque to position control.\n";
      // Stop the force control loop
      _ftControlThreadStopAsked = true;
    }
    if(_controlThread.joinable()) {
      _controlThread.join();
      _velControlThreadStopAsked = false;
      _velControlThreadIsRunning = false;
      _ftControlThreadStopAsked = false;
      _ftControlThreadIsRunning = false;
    }
//    _controlThread = std::thread([this] {this->positionContolLoop();});
    this->_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_STOP == getRobotState()) {
        std::cout << "Change the control mode from stop to velocity control.\n";
    }
    else if (vpRobot::STATE_POSITION_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from position to velocity control.\n";
      _posControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_FORCE_TORQUE_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from force/torque to velocity control.\n";
      // Stop the force control loop
      _ftControlThreadStopAsked = true;
    }
    if(_controlThread.joinable()) {
      _controlThread.join();
      _posControlThreadStopAsked = false;
      _posControlThreadIsRunning = false;
      _ftControlThreadStopAsked = false;
      _ftControlThreadIsRunning = false;
    }
    _controlThread = std::thread([this] {this->velocityContolLoop();});
    this->_stateRobot = newState;
    break;
  }
  case vpRobot::STATE_FORCE_TORQUE_CONTROL: {
    if (vpRobot::STATE_STOP == getRobotState()) {
      std::cout << "Change the control mode from stop to force/torque control.\n";
    }
    else if (vpRobot::STATE_POSITION_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from position to force/torque control.\n";
      _posControlThreadStopAsked = true;
    }
    else if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to force/torque control.\n";
      _velControlThreadStopAsked = true;
    }
    if(_controlThread.joinable()) {
      _controlThread.join();
      _posControlThreadStopAsked = false;
      _posControlThreadIsRunning = false;
      _velControlThreadStopAsked = false;
      _velControlThreadIsRunning = false;
    }
    _controlThread = std::thread([this] {this->torqueControlLoop();});
    this->_stateRobot = newState;
    break;
  }

  default:
    break;
  }

  return newState;
}

void fakeFCI::readingLoop(){

  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate loop_rate(500); // Hz

  ros::Subscriber sub_franka_jointState = n->subscribe("/franka/joint_state", 1, &fakeFCI::JointState_callback,this);

  while (ros::ok() && _connected)
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  std::lock_guard<std::mutex> lock(_mutex);
  _connected = false;

  std::cout << "fakeFCI::readingLoop() thread is finished." << std::endl;

};
void fakeFCI::positionContolLoop(){
  std::cout << "fakeFCI::positionContolLoop(): q_des = " << _q_des.t() << std::endl;
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate pos_loop_rate(500); // Hz
  ros::Publisher joint_cmd_pub = n->advertise<sensor_msgs::JointState>("/fakeFCI/joint_commands", 1);
  sensor_msgs::JointState joint_command_msg;
  joint_command_msg.velocity.resize(7);
  joint_command_msg.name.resize(7);
  joint_command_msg.header.frame_id = "Joint_velocity_cmd";
  for(int i=0; i<7; i++){
    joint_command_msg.name[i] = "J" + std::to_string(i);
  }
  vpColVector vel_max(7, 0), dq_sat(7,0), gains(7,0);
  vel_max = {2.1750 ,  2.1750 , 2.1750 , 2.1750 , 2.6100 , 2.6100 , 2.6100};

//  const int order = 2; // 4th order (=2 biquads)
//  std::array<Iir::Butterworth::LowPass<order>, 7> MultiChannelFilter;
//  const float samplingrate = 500; // Hz
//  const float cutoff_frequency = 10; // Hz
//  for(int i=0;i<7;i++){
//    MultiChannelFilter[i].setup (samplingrate, cutoff_frequency);
//  }
  vpMatrix Kp(7,7), Kd(7,7);

  gains = {6.0, 6.0, 6.0, 6.0, 3.5, 2.5, 2.0};
  Kp.diag(gains);
  gains = {0.5, 0.5, 0.5, 0.5, 0.3, 0.25, 0.2};
  Kd.diag(gains);

  _posControlThreadIsRunning = true;
  while (ros::ok() && !_posControlThreadStopAsked && _posControlNewCmd)
  {
    pos_loop_rate.sleep();
    _mutex.lock();
    _dq_des = Kp*(_q_des - _q) -Kd*_dq;
    dq_sat = vpRobot::saturateVelocities(_dq_des, vel_max, false);
    if(std::sqrt(((180/M_PI)*(_q_des - _q)).sumSquare()) > 0.1 ){
//      _posControlLock = true;
      for(int i=0; i<7; i++){
//        _dq_des_filt[i] = MultiChannelFilter[i].filter((float)dq_sat[i]);
//        joint_command_msg.velocity[i] = _dq_des_filt[i];

        joint_command_msg.velocity[i] = dq_sat[i];
      }
    }else{
      for(int i=0; i<7; i++){
        joint_command_msg.velocity[i] = 0;
      }
//      _posControlLock = false;
      _posControlNewCmd = false;
    }

    _mutex.unlock();
    joint_cmd_pub.publish(joint_command_msg);
  }

  for(int i=0; i<7; i++){
    joint_command_msg.velocity[i] = 0;
  }
  joint_cmd_pub.publish(joint_command_msg);
  _posControlThreadIsRunning = false;
  std::cout << "fakeFCI::positionContolLoop() position reached!. \n";

};

void fakeFCI::velocityContolLoop(){

  std::cout << "fakeFCI::velocityContolLoop() thread launched. \n";
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate vel_loop_rate(500); // Hz
  ros::Publisher joint_cmd_pub = n->advertise<sensor_msgs::JointState>("/fakeFCI/joint_commands", 1);
  sensor_msgs::JointState joint_command_msg;
  joint_command_msg.velocity.resize(7);
  joint_command_msg.name.resize(7);
  joint_command_msg.header.frame_id = "Joint_velocity_cmd";
  for(int i=0; i<7; i++){
    joint_command_msg.name[i] = "J" + std::to_string(i);
  }
  vpColVector vel_max(7, 0), dq_sat(7,0);
  vel_max = {2.1750 ,  2.1750 , 2.1750 , 2.1750 , 2.6100 , 2.6100 , 2.6100};

//  const int order = 2; // 4th order (=2 biquads)
//  std::array<Iir::Butterworth::LowPass<order>, 7> MultiChannelFilter;
//  const float samplingrate = 500; // Hz
//  const float cutoff_frequency = 10; // Hz
//  for(int i=0;i<7;i++){
//    MultiChannelFilter[i].setup (samplingrate, cutoff_frequency);
//  }

  _velControlThreadIsRunning = true;
  while (ros::ok() && !_velControlThreadStopAsked)
  {
    vel_loop_rate.sleep();
    _mutex.lock();
    dq_sat = vpRobot::saturateVelocities(_dq_des, vel_max, true);
    for(int i=0; i<7; i++){
//      _dq_des_filt[i] = MultiChannelFilter[i].filter((float)dq_sat[i]);
//      joint_command_msg.velocity[i] = _dq_des_filt[i];
      joint_command_msg.velocity[i] = dq_sat[i];
    }
    _mutex.unlock();
    joint_cmd_pub.publish(joint_command_msg);
  }

  for(int i=0; i<7; i++){
    joint_command_msg.velocity[i] = 0;
  }
  joint_cmd_pub.publish(joint_command_msg);
  _velControlThreadIsRunning = false;
  std::cout << "fakeFCI::velocityContolLoop() thread is finished. \n";
};

void fakeFCI::torqueControlLoop(){
  std::cout << "fakeFCI::torqueControlLoop() thread launched. \n";
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate ft_loop_rate(500); // Hz
  ros::Publisher joint_cmd_pub = n->advertise<sensor_msgs::JointState>("/fakeFCI/joint_commands", 1);
  sensor_msgs::JointState joint_command_msg;
  joint_command_msg.effort.resize(7);
  joint_command_msg.name.resize(7);
  joint_command_msg.header.frame_id = "Joint_torque_cmd";
  for(int i=0; i<7; i++){
    joint_command_msg.name[i] = "J" + std::to_string(i);
  }
  vpColVector g(7,0), tau_sat(7,0), tau_max(7,0);
  tau_max = {87, 87,87, 87, 12, 12, 12};
  const int order = 1; // ex: 4th order (=2 biquads)
  std::array<Iir::Butterworth::LowPass<order>, 7> TorqueFilter;
  const float samplingrate = 1000; // Hz
  const float cutoff_frequency = 10; // Hz
  for(int i=0;i<7;i++){
    TorqueFilter[i].setup (samplingrate, cutoff_frequency);
  }

  _ftControlThreadIsRunning = true;
  while (ros::ok() && !_ftControlThreadStopAsked)
  {
    ft_loop_rate.sleep();
    this->getGravity(g);
//    tau_sat = vpRobot::saturateVelocities(_tau_J_des, tau_max, true);
    std::lock_guard<std::mutex> lock(_mutex);
    for(int i=0; i<7; i++){
      if(std::abs(tau_sat[i]) >= 0.0){ // this simulates the static friction
        _tau_J_des_filt[i] = TorqueFilter[i].filter((float)_tau_J_des[i]);
//        _mutex.lock();
        joint_command_msg.effort[i] = _tau_J_des_filt[i] + g[i];
//        _mutex.unlock();
      }else{
        joint_command_msg.effort[i] = g[i];
      }
    }

    joint_cmd_pub.publish(joint_command_msg);
  }

  for(int i=0; i<7; i++){
    joint_command_msg.effort[i] = 0;
  }
  joint_cmd_pub.publish(joint_command_msg);
  _ftControlThreadIsRunning = false;
  std::cout << "fakeFCI::torqueControlLoop() thread is finished. \n";

};

void fakeFCI::JointState_callback(const sensor_msgs::JointState& J_state)
{
  std::lock_guard<std::mutex> lock(_mutex);
  for(int i=0; i<7; i++){
    _q_(i) = _q[i] = J_state.position[i];
    _dq[i] = J_state.velocity[i];
    _tau_J[i] = J_state.effort[i];
  }

  return;
}
void fakeFCI::get_fJe(vpMatrix &fJe){
  fJe.reshape(6,7);
  Jacobian Jac(7);

  std::lock_guard<std::mutex> lock(_mutex);
  _JacobianSolver_->JntToJac(_q_,Jac);

  for(unsigned int i=0;i<6;i++){
    for(unsigned int j=0;j<7;j++){
      fJe[i][j] = Jac.data(i,j);
    }
  }
};

void fakeFCI::get_fJe(const vpColVector &q, vpMatrix &fJe){
  fJe.reshape(6,7);
  KDL::JntArray jnts = JntArray(7);
  Jacobian Jac(7);

  for(unsigned int i=0;i<7;i++){
    jnts(i)=q[i];
  }
  _JacobianSolver_->JntToJac(jnts,Jac);

  for(unsigned int i=0;i<6;i++){
    for(unsigned int j=0;j<7;j++){
      fJe[i][j] = Jac.data(i,j);
    }
  }

};

void fakeFCI::get_eJe(vpMatrix &eJe){
//  std::cout << "fakeFCI::get_eJe: entering...\n";
  vpMatrix fJe(6,7);
  this->get_fJe(fJe);
  vpHomogeneousMatrix fMe(this->get_fMe());
  vpMatrix RR(6,6);
  RR.insert(fMe.getRotationMatrix().t(),0,0);
  RR.insert(fMe.getRotationMatrix().t(),3,3);

  eJe = RR*fJe;
//  std::cout << "fakeFCI::get_eJe: exit...\n";
};

void fakeFCI::get_eJe(const vpColVector &q, vpMatrix &eJe){

  vpMatrix fJe(6,7);
  this->get_fJe(q,fJe);
  vpHomogeneousMatrix fMe(this->get_fMe(q));
  vpMatrix RR(6,6);
  RR.insert(fMe.getRotationMatrix().t(),0,0);
  RR.insert(fMe.getRotationMatrix().t(),3,3);

  eJe = RR*fJe;
};


void fakeFCI::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position){

  switch(frame) {
  case vpRobot::JOINT_STATE: {
    position.resize(7);
    std::lock_guard<std::mutex> lock(_mutex);
    position = _q;
    break;
  }
//  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME: {
//    std::cout << "fakeFCI::getPosition() END_EFFECTOR_FRAME \n" ;
      position.resize(6);

      vpPoseVector fPc(get_fMe());
      for (size_t i=0; i < 6; i++) {
        position[i] = fPc[i];
      }

    break;
  }
    case vpRobot::CAMERA_FRAME: { // same as CAMERA_FRAME
      position.resize(6);
      vpPoseVector fPc(get_fMe()*_eMc);
      for (size_t i=0; i < 6; i++) {
        position[i] = fPc[i];
      }
      break;
    }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: wrong method"));
  }
  }
};

void fakeFCI::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position){
	vpColVector pos(6,0);
	if(frame == vpRobot::END_EFFECTOR_FRAME || frame == vpRobot::CAMERA_FRAME){
		getPosition(frame, pos);
		for (size_t i=0; i < 6; i++) {
			position[i] = pos[i];
		}
	}else{
		throw(vpException(vpException::fatalError, "Cannot get a cartesian position for the specified frame"));
	}
};

void fakeFCI::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position){
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    //    std::lock_guard<std::mutex> lock(_mutex);
    _mutex.lock();
    for(int i=0;i<7;i++){
      _q_des[i] = position[i];
    }
    _posControlNewCmd = true;
    _mutex.unlock();

    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    std::cout << " fakeFCI::setPosition(END_EFFECTOR_FRAME, pos):  \n";
    vpHomogeneousMatrix H;
    H.buildFrom(position[0],position[1],position[2],position[3],position[4],position[5]);
    _mutex.lock();
    _q_des = this->SolveIK(H);
    _posControlNewCmd = true;
    _mutex.unlock();

    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot set Franka position for the specified frame \n"));
  }
  }

  if(vpRobot::STATE_POSITION_CONTROL  == getRobotState()){
    _controlThread = std::thread([this] {this->positionContolLoop();});

    if(_controlThread.joinable()){
      _controlThread.join();
    }
  }else{
    std::cout << "Robot is not in STATE_POSITION_CONTROL, you should change to position control before. \n" ;
  }

};

vpColVector fakeFCI::SolveIK(const vpHomogeneousMatrix &edMw){
  vpColVector q_solved(7,0);
  JntArray q_out(7);
  KDL::Rotation R(edMw[0][0],edMw[0][1],edMw[0][2],
                  edMw[1][0],edMw[1][1],edMw[0][2],
                  edMw[2][0],edMw[2][1],edMw[2][2]);
  KDL::Vector v(edMw[0][3],edMw[1][3],edMw[2][3]);
  Frame dest(R, v);
  std::lock_guard<std::mutex> lock(_mutex);
  int ret = _iksolver_JL_->CartToJnt(_q_,dest,q_out);
  switch(ret){
  case KDL::SolverI::E_NOERROR:{
    std::cout << "fakeFCI::SolveIK: E_NOERROR \n" ;
    break;
  }
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED:{
    std::cout << "fakeFCI::SolveIK: E_MAX_ITERATIONS_EXCEEDED \n" ;
    break;
  }
  case KDL::SolverI::E_NOT_IMPLEMENTED:{
    std::cout << "fakeFCI::SolveIK: E_NOT_IMPLEMENTED \n" ;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "fakeFCI::SolveIK: unable to solve ik\n"));
  }

  }

  for(int i=0;i<7;i++){
    q_solved[i] = q_out(i);
  }
  return q_solved;
};

void fakeFCI::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &d_velocity){
  switch(frame) {
  case vpRobot::JOINT_STATE: {
//    std::cout << "fakeFCI::getVelocity(): JOINT_STATE: \n";
	  d_velocity.resize(7);
    std::lock_guard<std::mutex> lock(_mutex);
    d_velocity = _dq;
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
//    std::cout << "fakeFCI::getVelocity(): END_EFFECTOR_FRAME: \n";
	  d_velocity.resize(6);
    vpMatrix eJe(6,7);
    this->get_eJe(eJe);
    std::lock_guard<std::mutex> lock(_mutex);
    d_velocity = eJe*_dq;
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
//    std::cout << "fakeFCI::getVelocity(): REFERENCE_FRAME: \n";
	  d_velocity.resize(6);
    vpMatrix fJe(6,7);
    this->get_fJe(fJe);
    std::lock_guard<std::mutex> lock(_mutex);
    d_velocity = fJe*_dq;
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka position for the specified frame "));
  }
  }

};

void fakeFCI::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel){
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    std::cout <<  "Cannot send a velocity to the robot. "
                           "Use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first. \n";
  }

  switch (frame) {
  case vpRobot::JOINT_STATE: {
    if (vel.size() != 7) {
      std::cout <<  "Joint velocity vector "<< vel.size() <<" is not of size 7 \n";
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _dq_des = vel;
    break;
  }

  case vpRobot::REFERENCE_FRAME: {
    if (vel.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << vel.size() << " is not of size 6 \n";
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++){
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // velocities are expressed in Base frame
    _v_cart_des = vpRobot::saturateVelocities(vel, vel_max, true);

    KDL::Twist v_cart;
    for(int i=0; i<3;i++){
      v_cart.vel.data[i] = _v_cart_des[i];
      v_cart.rot.data[i] = _v_cart_des[i + 3];
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _DiffIkSolver_->CartToJnt(_q_,v_cart,_dq_des_);
    for(int i=0; i<7;i++){
      _dq_des[i] = _dq_des_.data(i);
    }
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME: {
    if (vel.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << vel.size() << " is not of size 6 \n";
    }
    // Apply Cartesian velocity limits according to the specifications
    vpColVector vel_max(6);
    for (unsigned int i = 0; i < 3; i++){
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // Refer End-Effector velocities in Base frame
    vpHomogeneousMatrix fMe = this->get_fMe();
    vpMatrix RR(6,6);
    RR.insert(fMe.getRotationMatrix(),0,0);
    RR.insert(fMe.getRotationMatrix(),3,3);
    _v_cart_des = RR*vpRobot::saturateVelocities(vel, vel_max, true);

    KDL::Twist v_cart;
    for(int i=0; i<3;i++){
      v_cart.vel.data[i] = _v_cart_des[i];
      v_cart.rot.data[i] = _v_cart_des[i + 3];
    }

    std::lock_guard<std::mutex> lock(_mutex);
    _DiffIkSolver_->CartToJnt(_q_,v_cart,_dq_des_);
    for(int i=0; i<7;i++){
      _dq_des[i] = _dq_des_.data(i);
    }

    // Inverse Differential Kinemeatics using the Jacobian Weighted pseudo-inverse
//    vpMatrix Jac(6,7), I7(7,7),B(7,7),JacB(6,7);
//    this->get_eJe(Jac);
//    I7.eye();
//    this->getMass(B);
//    JacB = B.inverseByCholesky()*Jac.t()*(Jac*B.inverseByCholesky()*Jac.t()).pseudoInverse();
//    _mutex.lock();
//    _dq_des = JacB*vel ;//+ *(I7- JacB*Jac)*_dq;
//    _mutex.unlock();

    break;
  }
  case vpRobot::CAMERA_FRAME: {
    if (vel.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << vel.size() << " is not of size 6 \n";
    }
    // Apply Cartesian velocity limits according to the specifications
    vpColVector vel_max(6);
    for (unsigned int i = 0; i < 3; i++){
      vel_max[i] = 1.7;
      vel_max[3 + i] = 2.5;
    }
    // Refer End-Effector velocities in Base frame
    vpHomogeneousMatrix fMe = this->get_fMe();
    vpMatrix RR(6,6);
    RR.insert(fMe.getRotationMatrix(),0,0);
    RR.insert(fMe.getRotationMatrix(),3,3);
    std::lock_guard<std::mutex> lock(_mutex);
    _v_cart_des = vpRobot::saturateVelocities(RR*_eTc*vel, vel_max, true);

    KDL::Twist v_cart;
    for(int i=0; i<3;i++){
      v_cart.vel.data[i] = _v_cart_des[i];
      v_cart.rot.data[i] = _v_cart_des[i + 3];
    }

    _DiffIkSolver_->CartToJnt(_q_,v_cart,_dq_des_);
    for(int i=0; i<7;i++){
      _dq_des[i] = _dq_des_.data(i);
    }

    // Inverse Differential Kinemeatics using the Jacobian Weighted pseudo-inverse
//    vpMatrix Jac(6,7), I7(7,7),B(7,7),JacB(6,7);
//    this->get_eJe(Jac);
//    I7.eye();
//    this->getMass(B);
//    JacB = B.inverseByCholesky()*Jac.t()*(Jac*B.inverseByCholesky()*Jac.t()).pseudoInverse();
//    _mutex.lock();
//    _dq_des = JacB*vel ;//+ *(I7- JacB*Jac)*_dq;
//    _mutex.unlock();

    break;
  }

  }

}

void fakeFCI::getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force){
  switch(frame) {
  case vpRobot::JOINT_STATE: {
    force.resize(7);
    std::lock_guard<std::mutex> lock(_mutex);
    force = _tau_J;
    break;
  }
//  case vpRobot::END_EFFECTOR_FRAME: {//TODO
//    force.resize(6);
//    vpMatrix eJe;
//    this->get_eJe(eJe);
//    _mutex.lock();
//    force = eJe*_dq;
//    _mutex.unlock();
//    break;
//  }
//  case vpRobot::REFERENCE_FRAME: {//TODO
//    force.resize(6);
//    vpMatrix fJe;
//    this->get_fJe(fJe);
//    _mutex.lock();
//    force = fJe*_dq;
//    _mutex.unlock();
//    break;
//  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka position for the specified frame "));
  }
  }

};


void fakeFCI::setForceTorque(const vpRobot::vpControlFrameType frame, const vpColVector &force){

  if (vpRobot::STATE_FORCE_TORQUE_CONTROL != getRobotState()) {
    std::cout <<  "Cannot send a torque command to the robot. "
                  "Use setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL) first. \n";
  }

  switch (frame) {
  // Saturation in joint space
  case vpRobot::JOINT_STATE: {
    if (force.size() != 7) {
      std::cout <<  "Joint velocity vector "<< force.size() <<" is not of size 7 \n";
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _tau_J_des = force;

    break;
  }

  case vpRobot::REFERENCE_FRAME: {
    if (force.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << force.size() << " is not of size 6 \n";
    }
    vpMatrix fJe(6,7);
    this->get_fJe(fJe);
    std::lock_guard<std::mutex> lock(_mutex);
    _tau_J_des = fJe.t()*force;
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    if (force.size() != 6) {
      std::cout <<  "Cartesian velocity vector " << force.size() << " is not of size 6 \n";
    }
    vpMatrix eJe(6,7);
    this->get_eJe(eJe);
    std::lock_guard<std::mutex> lock(_mutex);
    _tau_J_des = eJe.t()*force;
    break;
  }

  }
};

vpHomogeneousMatrix fakeFCI::get_fMe(){
  vpHomogeneousMatrix fMe;
  KDL::Frame cartpos;
  // Calculate forward kinematics
  bool kinematics_status;
  vpRotationMatrix R;
  vpTranslationVector t;
  std::lock_guard<std::mutex> lock(_mutex);
  kinematics_status = _fksolver_->JntToCart(_q_,cartpos);
  if(kinematics_status>=0){
    for(unsigned int i=0;i<3;i++){
      for(unsigned int j=0;j<3;j++){
        R[i][j] = cartpos.M.data[3*i+j];
      }
      t[i] = cartpos.p.data[i];
    }
    fMe.buildFrom(t,R);
  }

  return fMe;
};

vpHomogeneousMatrix fakeFCI::get_fMe(const vpColVector &q){
  KDL::Frame cartpos;
  KDL::JntArray qq(7);
  for(int i=0;i<7;i++){
    qq(i) = q[i];
  }
  // Calculate forward kinematics
  bool kinematics_status;
  vpRotationMatrix R;
  vpTranslationVector t;
  kinematics_status = _fksolver_->JntToCart(qq,cartpos);
  if(kinematics_status>=0){
    for(unsigned int i=0;i<3;i++){
      for(unsigned int j=0;j<3;j++){
        R[i][j] = cartpos.M.data[3*i+j];
      }
      t[i] = cartpos.p.data[i];
    }
  }
  vpHomogeneousMatrix fMe(t,R);
  return fMe;

};


void fakeFCI::getMass(vpMatrix &mass){
  std::lock_guard<std::mutex> lock(_mutex);
  mass = MassMatrix(_q);
};
void fakeFCI::getGravity(vpColVector &gravity){
  std::lock_guard<std::mutex> lock(_mutex);
  gravity = GravityVector(_q);
};

void fakeFCI::getCoriolis(vpColVector &coriolis){
  std::lock_guard<std::mutex> lock(_mutex);
  vpMatrix C(7,7);
  C = CoriolisMatrix(_q,_dq);
  coriolis = C*_dq;
};

