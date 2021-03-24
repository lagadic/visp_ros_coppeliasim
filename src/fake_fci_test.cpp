/*
 * fake_fci_test.cpp
 *
 *  Created on: Apr 16, 2020
 *      Author: oliva
 */

#include <visp3/core/vpConfig.h>
#include "fakeFCI.h"

#include <visp3/robot/vpRobotFranka.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/gui/vpPlot.h>

#include <kdl_parser/kdl_parser.hpp>
#include "ros/exception.h"
#include "urdf/model.h"
#include <string>

#include <visp3/core/vpImageConvert.h>

//#include "geometry_msgs/TwistStamped.h"
//#include "geometry_msgs/WrenchStamped.h"
//#include "geometry_msgs/PoseStamped.h"
//#include <visp_ros/vpROSGrabber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

double simTime = 0;
pthread_mutex_t mutex_ros_time;
void simTime_callback(const std_msgs::Float32& msg )
{
  pthread_mutex_lock(&mutex_ros_time);
  simTime = msg.data;
  pthread_mutex_unlock(&mutex_ros_time);
}

bool compute = true;

void stepDone_callback(const std_msgs::Bool &msg){
  pthread_mutex_lock(&mutex_ros_time);
  compute = msg.data;
  pthread_mutex_unlock(&mutex_ros_time);
}

int main(int argc, char **argv)
{
  vpPlot *plotter = nullptr;
  plotter = new vpPlot(2, 500, 500, 800, 10, "Real time curves plotter");
  plotter->setTitle(0, "Visual features error");
  plotter->initGraph(0, 7);
  plotter->setLegend(0, 0, "Vx");
  plotter->setLegend(0, 1, "Vy");
  plotter->setLegend(0, 2, "Vz");
  plotter->setLegend(0, 3, "Wx");
  plotter->setLegend(0, 4, "Wy");
  plotter->setLegend(0, 5, "Wz");
  plotter->setLegend(0, 6, "Wz");

  plotter->setTitle(1, "Visual features error");
  plotter->initGraph(1, 7);
  plotter->setLegend(1, 0, "Vx");
  plotter->setLegend(1, 1, "Vy");
  plotter->setLegend(1, 2, "Vz");
  plotter->setLegend(1, 3, "Wx");
  plotter->setLegend(1, 4, "Wy");
  plotter->setLegend(1, 5, "Wz");
  plotter->setLegend(1, 6, "Wz");


  //    ROS node    //
  ros::init(argc, argv, "fakeFCI");
//  ros::NodeHandle n;
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Rate loop_rate(100);
  ros::spinOnce();

  fakeFCI robot;
  robot.connect();
  vpTime::wait(1000);


  // Franka cartesian info publishers:
  ros::Publisher nextStepTrigger_pub = n->advertise<std_msgs::Bool>("/triggerNextStep", 1);
  ros::Publisher enableSyncMode_pub = n->advertise<std_msgs::Bool>("/enableSyncMode", 1);
  ros::Publisher startSimTrigger_pub = n->advertise<std_msgs::Bool>("/startSimulation", 1);
  ros::Publisher stopSimTrigger_pub = n->advertise<std_msgs::Bool>("/stopSimulation", 1);
  ros::Publisher pauseSimTrigger_pub = n->advertise<std_msgs::Bool>("/pauseSimulation", 1);
  std_msgs::Bool trigger;
  std_msgs::Bool syncMode;
  std_msgs::Bool StartStopSim;

  // subscribers
  ros::Subscriber sub_simulationTime = n->subscribe("/simulationTime", 1, simTime_callback);
  ros::Subscriber sub_new_pose = n->subscribe("/simulationStepDone", 1, stepDone_callback);

  // Create joint array
  vpColVector q(7,0), qd(7,0),dq(7,0), dq_des(7,0),in(7,0), pos(6,0), pos_d(6,0);
  vpMatrix J(6,7);

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
//  robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);
  vpTime::wait(1000);
  vpColVector vel(6,0), vel_d(6,0), tau_d(7,0), C(7,0), g(7,0);
  std::chrono::time_point<std::chrono::system_clock> timeStart, timeCurrent;
  timeStart = std::chrono::system_clock::now();
  timeCurrent = timeStart;
  double delta_t = 0;
  double t = 0;

  vpMatrix B(7,7);


  vpMatrix Kp6(6,6), Kd6(6,6);
  Kp6.diag(3);
  Kd6.diag(0.5);

//  qd = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
//  qd = {0, 0, 0, -M_PI_2, 0, M_PI_2, 0};
  qd[3] = -M_PI_2;
  qd[5] = M_PI_2;

  vpHomogeneousMatrix oMed, oMe;
  oMed.buildFrom(0.3,0.0,0.5,M_PI,0,0);

  vpPoseVector pose_err;


  vpTime::wait(1000);
  bool final_quit = true;
  bool send_velocities  = false;


    StartStopSim.data = true;
    startSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(100);
    stopSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(3000);
    syncMode.data = true;
    enableSyncMode_pub.publish(trigger);
    startSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(1000);
//    trigger.data = true;
//    nextStepTrigger_pub.publish(trigger);

    vpColVector gains(7,0);
    vpMatrix Kp(7,7), Kd(7,7);
    gains = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 10.0};
    Kp.diag(gains/10);
    gains = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 2.0};
    Kd.diag(gains/10);

  std::cout << "entering the main loop.. \n" ;
  while(final_quit){

    if(compute){
//    timeCurrent = std::chrono::system_clock::now();
//    t = 0.001*std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeStart).count();


//    t += delta_t;
//    std::cout << "getting velocity.. \n" ;
    robot.getVelocity(vpRobot::JOINT_STATE,dq);
//    robot.getVelocity(vpRobot::REFERENCE_FRAME,vel);
//    std::cout << vel << "\n---" << std::endl;
//    std::cout << "getting position.. \n" ;
    robot.getPosition(vpRobot::JOINT_STATE,q);
//    robot.getPosition(vpRobot::END_EFFECTOR_FRAME,pos);
    oMe = robot.get_fMe();
//    std::cout << pos << "\n---" << std::endl;
//    robot.getMass(B);
//    robot.getCoriolis(C);
//    robot.getGravity(g);

//    tau_d = (B*(Kp*(qd - q) -Kd*vel) + C);
    dq_des = Kp*(qd - q) -Kd*dq;
    pose_err.buildFrom(oMe.inverse()*oMed);
    pos[0] = pose_err[0];
    pos[1] = pose_err[1];
    pos[2] = pose_err[2];
    pos[3] = pose_err[3];
    pos[4] = pose_err[4];
    pos[5] = pose_err[5];
    vel_d = Kp6*pos - Kd6*vel;

    robot.getCoriolis(C);
    tau_d = Kp*(qd - q) - Kd*dq + C;

//    std::cout << "setting velocity.. \n" ;
    robot.setVelocity(vpRobot::JOINT_STATE,dq_des);
    if(!send_velocities){
      vel_d = 0;
      tau_d = 0;
    }
//    vel_d[0] = tau_d[3];
//    tau_d = 0;
//    tau_d[3] = vel_d[0];
//    robot.setVelocity(vpRobot::END_EFFECTOR_FRAME,vel_d);
//    robot.setVelocity(vpRobot::END_EFFECTOR_FRAME,vel_d.extract(0,6));
//    robot.setForceTorque(vpRobot::JOINT_STATE,tau_d);
//    std::cout << "triggering new step... \n";
    nextStepTrigger_pub.publish(trigger);
    pthread_mutex_lock(&mutex_ros_time);
    compute = false;
    pthread_mutex_unlock(&mutex_ros_time);
//    robot.getForceTorque(vpRobot::JOINT_STATE,vel);
//    double aux = g[1];
//    g = 0; g[1] = aux;
//    aux = vel[1];
//    vel = 0; vel[1] = aux;


    plotter->plot(0, simTime, (q*180)/M_PI);
    plotter->plot(1, simTime, (qd*180)/M_PI);
    }
//    std::cout << t << std::endl;
//    std::cout << " waiting a bit... \n" ;
    vpTime::wait(10);

    vpMouseButton::vpMouseButtonType button;
    if (vpDisplay::getClick(plotter->I, button, false)) {
      switch (button) {
      case vpMouseButton::button1:
        send_velocities = !send_velocities;
        break;

      case vpMouseButton::button3:
        final_quit = false;
        break;

      default:
        break;
      }

    }
  }

  stopSimTrigger_pub.publish(StartStopSim);
  if (plotter != nullptr) {
    delete plotter;
    plotter = nullptr;
  }

//  vpTime::wait(10*1000);
  robot.setRobotState(vpRobot::STATE_STOP);


  return 0;
}
