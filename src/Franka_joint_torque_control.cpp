/*
 * Franka_joint_torque_control.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: oliva
 */

#include "fakeFCI.h"
#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <iostream>
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#undef Bool
#include <visp3/io/vpImageIo.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpMath.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/vs/vpServo.h>
#include <visp3/gui/vpPlot.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Joy.h"

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <visp_ros/vpROSGrabber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>


bool compute = true;
pthread_mutex_t mutex_ros;
void stepDone_callback(const std_msgs::Bool &msg){
  pthread_mutex_lock(&mutex_ros);
  compute = msg.data;
  pthread_mutex_unlock(&mutex_ros);
  return;
}

double simTime = 0;
void simTime_callback(const std_msgs::Float32& msg )
{
  pthread_mutex_lock(&mutex_ros);
  simTime = msg.data;
  pthread_mutex_unlock(&mutex_ros);
}

enum joycon_axe {
  JOY_AXE_LEFT_RIGHT = 4,
  JOY_AXE_UP_DOWN
};
enum joycon_button {
  JOY_BUTTON_DOWN,
  JOY_BUTTON_RIGHT,
  JOY_BUTTON_LEFT,
  JOY_BUTTON_UP,
  JOY_BUTTON_TOP_L,
  JOY_BUTTON_TOP_R,
  JOY_BUTTON_SELECT = 8,
  JOY_BUTTON_PRESS = 10,
  JOY_BUTTON_START = 13,
  JOY_BUTTON_L,
  JOY_BUTTON_ZL
};
vpColVector joy_axes, joy_buttons;
void joycon_callback(const sensor_msgs::Joy& joy )
{
  pthread_mutex_lock(&mutex_ros);
  for(int i = 0; i < joy.axes.size(); i++){
    joy_axes[i] = joy.axes[i];
    joy_buttons[i] = joy.buttons[i];
  }
  pthread_mutex_unlock(&mutex_ros);
}


int main(int argc, char **argv)
{
  double kp = 0.5, kv = 0.1;
  bool opt_plot = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] <<  "[--plot][--help] [-h] \n";
      return EXIT_SUCCESS;
    }
  }
  try {
    //    ROS node    //
    ros::init(argc, argv, "Franka_joint_torque_control");
    //    ros::NodeHandle n;
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Rate loop_rate(1000);
    ros::spinOnce();

    fakeFCI robot;
    robot.connect();
    robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);

    // Controller publishers:
    ros::Publisher nextStepTrigger_pub = n->advertise<std_msgs::Bool>("/triggerNextStep", 1);
    ros::Publisher enableSyncMode_pub = n->advertise<std_msgs::Bool>("/enableSyncMode", 1);
    ros::Publisher startSimTrigger_pub = n->advertise<std_msgs::Bool>("/startSimulation", 1);
    ros::Publisher stopSimTrigger_pub = n->advertise<std_msgs::Bool>("/stopSimulation", 1);
    ros::Publisher verbosity_pub = n->advertise<std_msgs::Bool>("/verbose", 1);
    std_msgs::Bool trigger;
    std_msgs::Bool syncMode;
    std_msgs::Bool StartStopSim;
    std_msgs::Bool verbosity;
    // Controller subscrivers:
    ros::Subscriber sub_simulationStepDone = n->subscribe("/simulationStepDone", 1, stepDone_callback);
    ros::Subscriber sub_simulationTime = n->subscribe("/simulationTime", 1, simTime_callback);
    ros::Subscriber sub_joycon = n->subscribe("/joy", 1, joycon_callback);

    vpPlot *plotter = nullptr;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, 800, 10, "Real time curves plotter");
      plotter->setTitle(0, "Joint error");
      plotter->initGraph(0, 7);
      plotter->setLegend(0, 0, "error_q1");
      plotter->setLegend(0, 1, "error_q2");
      plotter->setLegend(0, 2, "error_q3");
      plotter->setLegend(0, 3, "error_q4");
      plotter->setLegend(0, 4, "error_q5");
      plotter->setLegend(0, 5, "error_q6");
      plotter->setLegend(0, 6, "error_q7");

      plotter->setTitle(1, "Torque Commands");
      plotter->initGraph(1, 7);
      plotter->setLegend(1, 0, "Tau_1");
      plotter->setLegend(1, 1, "Tau_2");
      plotter->setLegend(1, 2, "Tau_3");
      plotter->setLegend(1, 3, "Tau_4");
      plotter->setLegend(1, 4, "Tau_5");
      plotter->setLegend(1, 5, "Tau_6");
      plotter->setLegend(1, 6, "Tau_7");
    }

    vpColVector q(7,0), q_d(7,0), dq(7,0), dq_d(7,0),ddq_d(7,0), dq_d_old(7,0),
                tau(7,0), C(7,0),gains(7,0);
    vpMatrix Kp(7,7), Kd(7,7);
    gains = {300.0, 300.0, 150.0, 150.0, 100.0, 50.0, 2.0};
    Kp.diag(gains);
    gains = {20.0, 20.0, 15.0, 15.0, 10.0, 5.0, 0.5};
    Kd.diag(gains);

//    verbosity.data = true;
//    verbosity_pub.publish(verbosity);
    vpTime::wait(100);
    StartStopSim.data = true;
    syncMode.data = true;
    trigger.data = true;
    startSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(100);
    stopSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(3000);
    enableSyncMode_pub.publish(syncMode);
    startSimTrigger_pub.publish(StartStopSim);
    nextStepTrigger_pub.publish(trigger);
    vpTime::wait(1000);
    nextStepTrigger_pub.publish(trigger);
    vpTime::wait(1000);
    nextStepTrigger_pub.publish(trigger);

    bool send_cmds = false;
    bool servo_started = false;
    bool final_quit = false;
    double delta_t = 0;
    double old_time = 0;
    double current_time = 0;
    double start_time = 0;

    start_time = simTime;
    old_time = start_time;
    while(!final_quit) {
      try {
        if(compute){

          pthread_mutex_lock(&mutex_ros);
          compute = false;
          current_time = simTime;
          pthread_mutex_unlock(&mutex_ros);
          delta_t = (current_time - old_time);
//          std::cout << "current_time: " << current_time << std::endl;

          robot.getCoriolis(C);
          robot.getPosition(vpRobot::JOINT_STATE,q);
          robot.getVelocity(vpRobot::JOINT_STATE,dq);

          // Joint space impedance control
          q_d = {0, -M_PI_4, 0, -3*M_PI_4, 0 , M_PI_2, M_PI_4};
          q_d[0] = q_d[0] + (M_PI/3)*std::sin(2*M_PI*0.1*(current_time - start_time));

          if( delta_t == 0 ){
            ddq_d = 0;
          }else{
            ddq_d = (dq_d - dq_d_old)/delta_t;
          }
          dq_d_old = dq_d;

          tau = ddq_d + Kp*(q_d - q) + Kd*(dq_d - dq) + C;

          if(!send_cmds){
            tau = 0;
          }
          vpColVector aux(7,0);
          for(int i = 0; i < 7; i++){
            aux[i] = vpMath::deg((q_d[i] - q[i]));
          }

          if (opt_plot && (current_time > 0.005)) {
            plotter->plot(0, current_time, aux);
            plotter->plot(1, current_time, tau);
          }

          robot.setForceTorque(vpRobot::JOINT_STATE,tau);
          old_time = current_time;
          nextStepTrigger_pub.publish(trigger);
          vpTime::wait(10);

        }// end if(compute)
        else{
//          std::cout << "not compute \n" ;
          vpTime::wait(10);
        }

        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(plotter->I, button, false)) {
          switch (button) {
          case vpMouseButton::button1:
            send_cmds = !send_cmds;
            break;

          case vpMouseButton::button3:
            final_quit = true;
            break;

          default:
            break;
          }

        }

      } catch (...) {

        trigger.data = true;
        nextStepTrigger_pub.publish(trigger);
        exit(1);
      }

    }
    stopSimTrigger_pub.publish(StartStopSim);
    robot.setRobotState(vpRobot::STATE_STOP);

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Test failed with exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}





