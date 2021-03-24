/*
 * orientation_test.cpp
 *
 *  Created on: Jul 9, 2020
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
#include <visp3/core/vpColVector.h>
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

#include <Eigen/Dense>
#include <Eigen/Geometry>


bool compute = false;
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

vpColVector extract_euler_RPY(const vpMatrix &R)
{
  vpColVector euler(3,0);
  if(R[2][0] < 1){
    if(R[2][0] > -1){
      euler[0] = std::atan2(R[2][1],R[2][2]);
      euler[1] = std::asin(-R[2][0]);
      euler[2] = std::atan2(R[1][0],R[0][0]);
//      euler[2] = std::atan2(R[2][1],R[2][2]);
//      euler[1] = std::atan2(-R[2][0],std::sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2]));
//      euler[0] = std::atan2(R[1][0],R[0][0]);

    }else{ // R[2][0] = -1
      euler[0] = 0;
      euler[1] = M_PI_2;
      euler[2] = -std::atan2(-R[1][2],R[1][1]);
    }

  }else{ // R[2][0] = 1
    euler[0] = 0;
    euler[1] = -M_PI_2;
    euler[2] = std::atan2(-R[1][2],R[1][1]);
  }

  return euler;
}


vpColVector euler_RPY(const vpMatrix &R){
  vpColVector euler(3,0);
  euler[0] = std::atan2(-R[2][1],R[2][2]);
  euler[1] = std::asin(R[2][0]);
  euler[2] = std::atan2(-R[1][0],R[0][0]);

  return euler;
}

vpColVector compute_orient_error_euler_RPY(const vpMatrix &R,const vpMatrix &Rd)
{
  vpColVector Err(3,0), euler(3,0), euler_d(3,0);

  euler   = extract_euler_RPY(R);
  euler_d = extract_euler_RPY(Rd);

  ((euler_d[0] - euler[0]) > M_PI ? Err[0] = (euler_d[0] - euler[0])-2*M_PI : ((euler_d[0] - euler[0]) < -M_PI ? Err[0] = (euler_d[0] - euler[0]) + 2*M_PI : Err[0] = (euler_d[0] - euler[0])));
  ((euler_d[1] - euler[1]) > M_PI ? Err[1] = (euler_d[1] - euler[1])-2*M_PI : ((euler_d[1] - euler[1]) < -M_PI ? Err[1] = (euler_d[1] - euler[1]) + 2*M_PI : Err[1] = (euler_d[1] - euler[1])));
  ((euler_d[2] - euler[2]) > M_PI ? Err[2] = (euler_d[2] - euler[2])-2*M_PI : ((euler_d[2] - euler[2]) < -M_PI ? Err[2] = (euler_d[2] - euler[2]) + 2*M_PI : Err[2] = (euler_d[2] - euler[2])));

  return -Err;
}

vpMatrix T_RPY_ZYX(const vpColVector &rpy)
{
  vpMatrix res(3,3);
  res = {{0, -std::sin(rpy[0]),std::cos(rpy[1])*std::cos(rpy[0])},
         {0,  std::cos(rpy[0]),std::cos(rpy[1])*std::sin(rpy[0])},
         {1,          0       ,          -std::sin(rpy[1])      }};
  return res;
}

vpMatrix T_RPY_ZYX_inv(const vpColVector &rpy)
{
  vpMatrix res(3,3);
  res = {{std::cos(rpy[0])*std::sin(rpy[1])/std::cos(rpy[1]), std::sin(rpy[0])*std::sin(rpy[1])/std::cos(rpy[1]),1},
         {-std::sin(rpy[0]),  std::cos(rpy[0]), 0},
         {std::cos(rpy[0])/std::cos(rpy[1]),std::sin(rpy[0])/std::cos(rpy[1]),   0  }};
  return res;
}

vpMatrix T_RPY_XYZ(const vpColVector &rpy)
{
  vpMatrix res(3,3);
  res = {{1,          0       ,           std::sin(rpy[1])      },
         {0,  std::cos(rpy[0]),-std::cos(rpy[1])*std::sin(rpy[0])},
         {0, -std::sin(rpy[0]), std::cos(rpy[1])*std::cos(rpy[0])},};
  return res;
}

vpMatrix T_RPY_XYZ_inv(const vpColVector &rpy)
{
  vpMatrix res(3,3);
  res = {{1 , std::sin(rpy[0])*std::sin(rpy[1])/std::cos(rpy[1]), -std::cos(rpy[0])*std::sin(rpy[1])/std::cos(rpy[1])},
         {0 , std::cos(rpy[0]),  std::sin(rpy[0])},
         {0 ,-std::sin(rpy[0])/std::cos(rpy[1]),std::cos(rpy[0])/std::cos(rpy[1])}};
  return res;
}

vpColVector extract_euler_RPY_eigen(const vpMatrix &R)
{
  vpColVector c(3,0);
  Eigen::Matrix3d m;
  for(int i=0; i<3;i++){
    for(int j=0; j<3;j++){
      m(i,j) = R[i][j];
    }
  }
  Eigen::Vector3d ea = m.eulerAngles(2, 1, 0);
  c[0] = ea(0);
  c[1] = ea(1);
  c[2] = ea(2);

  return c;
}

int main(int argc, char **argv)
{
  double kp = 0.5, kv = 0.1;
  bool opt_plot = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--kp" ){
      kp = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--kv" ){
      kv = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] <<  "[--plot] [--no-convergence-threshold] [--kp <proportional_gain>] [--kv <derivative_gain>] [--help] [-h]"
          << "\n";
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
    geometry_msgs::TwistStamped camera_twist;
    geometry_msgs::WrenchStamped camera_wrench;
    std_msgs::Bool trigger;
    std_msgs::Bool syncMode;
    std_msgs::Bool StartStopSim;
    // Controller subscrivers:
    ros::Subscriber sub_simulationStepDone = n->subscribe("/simulationStepDone", 1, stepDone_callback);
    ros::Subscriber sub_simulationTime = n->subscribe("/simulationTime", 1, simTime_callback);
    ros::Subscriber sub_joycon = n->subscribe("/joy", 1, joycon_callback);

    vpPlot *plotter = nullptr;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, 800, 10, "Real time curves plotter");
      plotter->setTitle(0, "Cartesian error");
      plotter->initGraph(0, 6);
      plotter->setLegend(0, 0, "error_Px");
      plotter->setLegend(0, 1, "error_Py");
      plotter->setLegend(0, 2, "error_Pz");
      plotter->setLegend(0, 3, "error_ThetaX");
      plotter->setLegend(0, 4, "error_ThetaY");
      plotter->setLegend(0, 5, "error_ThetaZ");

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

    vpColVector p(3,0), p_d(3,0), dx(6,0), dx_d(6,0), tau(7,0), C(7,0),gains(6,0),
                dq(7,0), tau_damped(7,0), r(3,0), err(6,0), dot_err(6,0),Eo(3,0),dot_Eo(3,0);
    vpMatrix Kp(6,6), Kd(6,6), fJe(6,7), old_Ja(6,7), dot_Ja(6,7), M(7,7),
             J_m(6,7), P(7,7), L(3,3), IL(6,6), RRd(6,6),Ta(6,6), Ja(6,7),T_eul(3,3);
    vpRotationMatrix Re, Rd, dRe;
    vpThetaUVector tu;
    vpHomogeneousMatrix fMe;
    double theta = 0;
    gains = {300.0, 300.0, 300.0, 350.0, 350.0, 350.0};
    Kp.diag(gains);
    for(int i=0; i<3; i++){
      gains[i] = 2*std::sqrt(gains[i]);
    }
    for(int i=3; i<6; i++){
      gains[i] = 5*std::sqrt(gains[i]);
    }
    Kd.diag(gains);


    StartStopSim.data = true;
    startSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(100);
    stopSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(3000);
    syncMode.data = true;
    trigger.data = true;
    enableSyncMode_pub.publish(syncMode);
    startSimTrigger_pub.publish(StartStopSim);
    nextStepTrigger_pub.publish(trigger);
    vpTime::wait(1000);
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

    fMe = robot.get_fMe();
//    p_d[0] = fMe[0][3];
//    p_d[1] = fMe[1][3];
//    p_d[2] = fMe[2][3];
    p_d = {0.3, 0.2, 0.7};
    Rd = fMe.getRotationMatrix();
//    Rd.buildFrom(M_PI,0,0);
//    Rd.buildFrom(M_PI,0,0);
//    Rd[0][0] = 0.7071;
//    Rd[0][1] = 0.7071;
//    Rd[0][2] = 0.0;
//    Rd[1][0] = 0.7071;
//    Rd[1][1] = -0.7071;
//    Rd[1][2] = 0.0;
//    Rd[2][0] = 0.0;
//    Rd[2][1] = 0.0;
//    Rd[2][2] = -1.0;


//    std::cout << "Rd: " << Rd << std::endl;
//    std::cout << "Rd*Rd^T: " << Rd*Rd.t() << std::endl;

    unsigned int count = 0;

    start_time = simTime;
    old_time = start_time;
    while(!final_quit) {
      try {
        if(compute && robot.is_connected()){

          pthread_mutex_lock(&mutex_ros);
          compute = false;
          current_time = simTime;
          pthread_mutex_unlock(&mutex_ros);
          delta_t = (current_time - old_time);
//          std::cout << "delta_t: " << delta_t << std::endl;

          robot.getCoriolis(C);
          robot.get_fJe(fJe);
          robot.getMass(M);
          robot.getVelocity(vpRobot::JOINT_STATE,dq);
          fMe = robot.get_fMe();
//          std::cout << "fMe: \n" << fMe << "\n ----------- \n";
          p[0] = fMe[0][3];
          p[1] = fMe[1][3];
          p[2] = fMe[2][3];
          Re = fMe.getRotationMatrix();
//          std::cout << "Re: \n" << Re << std::endl;
//          std::cout << "Re*Re^T: " << Re*Re.t() << std::endl;

//          std::cout << "M: \n" << M << "\n ----------- \n";

          // Cartesian space impedance control
          //  - Compute position error
          err.insert(0,Rd.t()*(p_d-p));
          //  - Compute the orientation error with RPY Euler angles
          dRe = (Rd.t() * Re);
//          vpThetaUVector tu;
//          tu.buildFrom((vpRotationMatrix)dRe);
//          err.insert(3,tu);
//          err[3] = tu[0];
//          err[4] = tu[1];
//          err[5] = tu[2];
//          std::cout << "tu: \n" << err.extract(2,3);
          vpColVector rpy(3,0);
          rpy = euler_RPY(dRe);
//          std::cout << "rpy: " << vpMath::deg(rpy[0]) << " , " << vpMath::deg(rpy[1]) << " , " << vpMath::deg(rpy[2]) << std::endl;
          err[3] = rpy[0];
          err[4] = rpy[1];
          err[5] = rpy[2];
//          std::cout << "\nxyz: \n" << err.extract(2,3);
//          std::cout << "\n--------------- \n";

          Ta.eye();
          T_eul = T_RPY_ZYX_inv(rpy);
          Ta.insert(Rd.t(),0,0);
          Ta.insert(T_eul*Rd.t(),3,3);
//          std::cout << "Ta: \n" << Ta << std::endl;;

          Ja = Ta*fJe;

          dot_err = -Ja*dq;

          // Dynamically consistent redundancy resolution
          J_m = ((Ja*M.inverseByCholesky()*Ja.t()).pseudoInverse())*Ja*M.inverseByCholesky();
          P.eye();
          P = P - Ja.t()*J_m;
          if(delta_t == 0){
            tau_damped = 0;
            dot_Ja = 0;
            count++;
            std::cout << count << std::endl;
          }else{
            tau_damped = - (M*dq)/delta_t;
            dot_Ja = (Ja - old_Ja)/delta_t;
          }
          old_Ja = Ja;

          // Task-Frame impedance control
          tau = M*Ja.pseudoInverse()*( Kp*err + Kd*dot_err -dot_Ja*dq)+ C + 0.5*P*tau_damped;

          if(!send_cmds){
            tau = 0;
          }

//          err[3] = vpMath::deg(rpy[0]);
//          err[4] = vpMath::deg(rpy[1]);
//          err[5] = vpMath::deg(rpy[2]);

          if (opt_plot) {
            plotter->plot(0, current_time, err);
            plotter->plot(1, current_time, tau);
//            plotter->plot(1, current_time, -Kp*err);
          }

          robot.setForceTorque(vpRobot::JOINT_STATE,tau);
          old_time = current_time;
          nextStepTrigger_pub.publish(trigger);
          vpTime::wait(100);

        }// end if(compute)
        else{
          vpTime::wait(2);
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

      } catch (const vpException &e) {
        std::cout << "ViSP exception: " << e.what() << std::endl;
        std::cout << "Stop the robot " << std::endl;
        trigger.data = true;
        nextStepTrigger_pub.publish(trigger);
//        robot.setRobotState(vpRobot::STATE_STOP);
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







