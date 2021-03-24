/*
 * fakeFCI.h
 *
 *  Created on: Apr 16, 2020
 *      Author: oliva
 */
#include <thread>
#include <mutex>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpThetaUVector.h>

#include <franka_model.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/solveri.hpp>
//#include <kdl/chainiksolvervel_pinv_nso.hpp> // virtual solver, not yet implemented
//#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "Iir.h"

class fakeFCI {
  public:
    fakeFCI();
    virtual ~fakeFCI();

    void connect();

    vpRobot::vpRobotStateType getRobotState(void);
    vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

    void get_fJe(vpMatrix &fJe);
    void get_fJe(const vpColVector &q, vpMatrix &fJe);
    void get_eJe(vpMatrix &eJe_);
    void get_eJe(const vpColVector &q, vpMatrix &fJe);
    void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position);
    // this function should control the robot in order to bring it to the desired configuration "position"
    // be aware that by now it directly assigns the given position as current position!
    void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);
    vpHomogeneousMatrix get_fMe(const vpColVector &q);
    vpHomogeneousMatrix get_fMe();
    void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &d_position);
    void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);
    void getForceTorque(const vpRobot::vpControlFrameType frame, vpColVector &force);
    void setForceTorque(const vpRobot::vpControlFrameType frame, const vpColVector &force);

    void getMass(vpMatrix &mass);
    void getGravity(vpColVector &gravity);
    void getCoriolis(vpColVector &coriolis);

    void set_eMc(const vpHomogeneousMatrix &eMc);

    //services
    vpColVector SolveIK(const vpHomogeneousMatrix &edMw);
    vpColVector getVelDes();
    bool is_connected(){return this->_connected;};

  private:
    vpColVector _q;
    vpColVector _dq;
    vpColVector _tau_J;

    bool _connected;
    std::thread _acquisitionThread;
    std::mutex _mutex;
    void readingLoop();
    void JointState_callback(const sensor_msgs::JointState& J_state);

    KDL::JntArray _q_;
    KDL::JntArray _dq_des_;
    KDL::Chain _chain_;
    KDL::JntArray  _q_min_;
    KDL::JntArray  _q_max_;
    KDL::ChainFkSolverPos_recursive* _fksolver_ ;
    KDL::ChainJntToJacSolver* _JacobianSolver_ ;
//    KDL::ChainIkSolverVel_pinv_nso* _DiffIkSolver_;
    KDL::ChainIkSolverVel_pinv* _DiffIkSolver_;
    KDL::ChainIkSolverPos_NR* _iksolver_;
    KDL::ChainIkSolverPos_NR_JL* _iksolver_JL_;
//    ChainIkSolverVel_pinv_givens* _DiffIkSolver;
//    ChainIkSolverVel_pinv_nso* _DiffIkSolver;
//    ChainIkSolverVel_wdls* _DiffIkSolver;

    vpRobot::vpRobotStateType _stateRobot;

    // Velocity controller
    std::thread _velControlThread;
    std::atomic_bool _velControlThreadIsRunning;
    std::atomic_bool _velControlThreadStopAsked;
    vpColVector _dq_des;   // Desired joint velocity
    vpColVector _dq_des_filt;   // Desired joint velocity
    vpColVector _v_cart_des;         // Desired cartesian velocity either in reference, end-effector, camera, or tool frame
    void velocityContolLoop();

    // Force/torque controller
    std::thread _ftControlThread;
    std::atomic_bool _ftControlThreadIsRunning;
    std::atomic_bool _ftControlThreadStopAsked;
    vpColVector _tau_J_des; // Desired joint torques
    vpColVector _tau_J_des_filt; // Desired joint torques filtered
    vpColVector _ft_cart_des;         // Desired cartesian force/torque either in reference, end-effector, camera, or tool frame
    void torqueControlLoop();

    vpHomogeneousMatrix _eMc;
    vpVelocityTwistMatrix _eTc;

};

