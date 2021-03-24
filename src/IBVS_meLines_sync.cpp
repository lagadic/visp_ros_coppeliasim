/*
 * IBVS_Lines_sync.cpp
 *
 *  Created on: Apr 9, 2020
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
#include <visp3/vs/vpServo.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/detection/vpDetectorAprilTag.h>

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

void stepDone_callback(const std_msgs::Bool &msg){
  compute = true;
  return;
}

vpColVector Vc(6,0), Wc(6,0);
pthread_mutex_t mutex_ros;

void Twist_callback(const geometry_msgs::TwistStamped& cam_twist)
{
  pthread_mutex_lock(&mutex_ros);
  Vc[0]=cam_twist.twist.linear.x;
  Vc[1]=cam_twist.twist.linear.y;
  Vc[2]=cam_twist.twist.linear.z;
  Vc[3]=cam_twist.twist.angular.x;
  Vc[4]=cam_twist.twist.angular.y;
  Vc[5]=cam_twist.twist.angular.z;
  pthread_mutex_unlock(&mutex_ros);
  return;
}

void Wrench_callback(const geometry_msgs::WrenchStamped& sensor_wrench)
{
  pthread_mutex_lock(&mutex_ros);
  Wc[0]=sensor_wrench.wrench.force.x;
  Wc[1]=sensor_wrench.wrench.force.y;
  Wc[2]=sensor_wrench.wrench.force.z;
  Wc[3]=sensor_wrench.wrench.torque.x;
  Wc[4]=sensor_wrench.wrench.torque.y;
  Wc[5]=sensor_wrench.wrench.torque.z;
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
  double convergence_threshold = 0.00005;
  bool display_tag = true;
  int opt_quad_decimate = 2;
  double opt_tagSize = 0.08;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold") {
      convergence_threshold = 0.;
    }else if (std::string(argv[i]) == "--kp" ){
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
    ros::init(argc, argv, "visp_ros_coppeliasim");
//    ros::NodeHandle n;
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Rate loop_rate(1000);
    ros::spinOnce();

    fakeFCI robot;
    robot.connect();
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    // Controller publishers:
    ros::Publisher nextStepTrigger_pub = n->advertise<std_msgs::Bool>("/triggerNextStep", 1);
    ros::Publisher enableSyncMode_pub = n->advertise<std_msgs::Bool>("/enableSyncMode", 1);
    ros::Publisher startSimTrigger_pub = n->advertise<std_msgs::Bool>("/startSimulation", 1);
    ros::Publisher stopSimTrigger_pub = n->advertise<std_msgs::Bool>("/stopSimulation", 1);

    std_msgs::Bool trigger;
    std_msgs::Bool syncMode;
    std_msgs::Bool StartStopSim;
    // Controller subscrivers:
    ros::Subscriber sub_vel_franka = n->subscribe("/vrep/camera/twist", 1, Twist_callback);
    ros::Subscriber sub_wrench_flycam = n->subscribe("/vrep/camera/wrench", 1, Wrench_callback);
    ros::Subscriber sub_pose_flycam = n->subscribe("/simulationStepDone", 1, stepDone_callback);
    ros::Subscriber sub_simulationTime = n->subscribe("/simulationTime", 1, simTime_callback);
    ros::Subscriber sub_joycon = n->subscribe("/joy", 1, joycon_callback);

    unsigned int width = 512, height = 512;
    vpROSGrabber g;
    g.setImageTopic("/vrep/franka_camera/image");
    g.open(argc, argv);

    vpImage<unsigned char> I(height, width);
    vpCameraParameters cam(305, 305, I.getWidth() / 2, I.getHeight() / 2);

    vpDisplayX display(I, 100, 100, "Current image");
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I);

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

    vpServo task;

    int i;
    int nbline = 3;

    vpMeLine line[nbline];

    vpMe me;
    me.setRange(10);
    me.setPointsToTrack(100);
    me.setThreshold(50000);
    me.setSampleStep(10);

//    // Initialize the tracking. Define the four lines to track.
//    for (i = 0; i < nbline; i++) {
//      line[i].setDisplay(vpMeSite::RANGE_RESULT);
//      line[i].setMe(&me);
//
//      line[i].initTracking(I);
//      line[i].track(I);
//    }
//
//    vpFeatureLine p[nbline];
//    for (i = 0; i < nbline; i++)
//      vpFeatureBuilder::create(p[i], cam, line[i]);

    vpLine lined[nbline];
//    lined[0].setWorldCoordinates(1, 0, 0, 0.2, 0, 0, 1, 0);
//    lined[1].setWorldCoordinates(0, 1, 0, 0.2, 0, 0, 1, 0);
////    lined[2].setWorldCoordinates(1, 0, 0, -0.2, 0, 0, 1, 0);
//    lined[2].setWorldCoordinates(0, 1, 0, -0.2, 0, 0, 1, 0);

    lined[0].setWorldCoordinates(0, 1, 0,  0.2, 0, 0, 1, 0);
    lined[1].setWorldCoordinates(1, 0, 0,  0.28, 0, 0, 1, 0);
    lined[2].setWorldCoordinates(1, 0, 0,  -0.28, 0, 0, 1, 0);

    vpHomogeneousMatrix cMo(0, 0, 0.5, 0, 0, vpMath::rad(0));

    lined[0].project(cMo);
    lined[1].project(cMo);
    lined[2].project(cMo);
//    lined[2].project(cMo);

    // Those lines are needed to keep the conventions define in vpMeLine
    // (Those in vpLine are less restrictive)  Another way to have the
    // coordinates of the desired features is to learn them before executing
    // the program.
    lined[0].setRho(-fabs(lined[0].getRho()));
    lined[0].setTheta(M_PI_2);
    lined[1].setRho(-fabs(lined[1].getRho()));
    lined[1].setTheta(M_PI);
    lined[2].setRho(-fabs(lined[2].getRho()));
    lined[2].setTheta(0);
//    lined[2].setRho(-fabs(lined[3].getRho()));
//    lined[2].setTheta(-M_PI / 2);


    task.setServo(vpServo::EYEINHAND_CAMERA);
//    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

//    vpFeatureLine pd[nbline];
//    for (i = 0; i < nbline; i++){
//      vpFeatureBuilder::create(pd[i], lined[i]);
//      task.addFeature(p[i], pd[i]);
//    }
    task.setLambda(1);

    std::cout << "first part done... features configuration... \n";

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
//      plotter->setTitle(1, "Camera velocities");
      plotter->setTitle(1, "Force/torque Measurements");
      plotter->initGraph(0, 6);
      plotter->initGraph(1, 6);
      plotter->setLegend(0, 0, "error_feat_l1_rho");
      plotter->setLegend(0, 1, "error_feat_l1_theta");
      plotter->setLegend(0, 2, "error_feat_l2_rho");
      plotter->setLegend(0, 3, "error_feat_l2_theta");
      plotter->setLegend(0, 4, "error_feat_l3_rho");
      plotter->setLegend(0, 5, "error_feat_l3_theta");
//      plotter->setLegend(0, 6, "error_feat_l4_rho");
//      plotter->setLegend(0, 7, "error_feat_l4_theta");
      plotter->setLegend(1, 0, "F_x");
      plotter->setLegend(1, 1, "F_y");
      plotter->setLegend(1, 2, "F_z");
      plotter->setLegend(1, 3, "Tau_x");
      plotter->setLegend(1, 4, "Tau_y");
      plotter->setLegend(1, 5, "Tau_z");
    }

    vpHomogeneousMatrix eMc;
  //  eMc.buildFrom(0.05,-0.05 ,0,0,0,-M_PI_4);
    eMc.buildFrom(0.0,std::sqrt(2*(0.05*0.05)) ,0,0,0,M_PI_4);

    vpVelocityTwistMatrix eVc;
  //  cVe.buildFrom(eMc.inverse());
    eVc.buildFrom(eMc);

    vpForceTwistMatrix cFe;
    cFe.buildFrom(eMc.inverse());

    unsigned int iter = 0;
    vpColVector v_c;
    vpImage<vpRGBa> Ic;
    double lambda_av = 0.5;
    double alpha = 0.05;
    double beta = 3;
    bool final_quit = false;
    bool send_velocities = false;
    const unsigned int Nfeatures = 2*nbline;

    vpMatrix Ls(Nfeatures,6), Ls_pinv(6,Nfeatures);
    Ls = 0;
    vpMatrix M(6,6), I3(3,3), M_inv(6,6);
    I3.eye();
    M.insert(0.2*I3,0,0);
    M.insert(0.001*I3,3,3);
    M_inv = M.inverseByCholeskyOpenCV();

    vpColVector s(Nfeatures,0), sd(Nfeatures,0), dot_s(Nfeatures,0), dot_sd(Nfeatures,0), dot_dot_sd(Nfeatures,0), es(Nfeatures,0), dot_es(Nfeatures,0), Fs(Nfeatures,0);
    dot_sd = 0;
    dot_dot_sd = 0;
    es = 0;
    dot_es = 0;
    Fs = 0;

    double old_time = simTime;
    double delta_t = 0;
    vpColVector dds_star(Nfeatures,0), ds_star(Nfeatures,0), s_star(Nfeatures,0);
    vpMatrix Ks(Nfeatures,Nfeatures), Ds(Nfeatures,Nfeatures);
    dds_star = 0;
    ds_star = 0;
    s_star = 0;
    vpColVector k = {kp, 8000, kp, 8000, kp, 8000};//, kp, 8000};
    vpColVector d = {kv, 150, kv, 150, kv, 150};//, kv, 150};
    Ks.diag(kp);
    Ds.diag(kv);//2*std::sqrt(kp));

//    for (i = 0; i < nbline; i++){
//      s_star[2*i] = pd[i].getRho();
//      s_star[2*i + 1] = pd[i].getTheta();
//    }
//    sd = s_star;

    vpColor colors[4];
    colors[0] = vpColor::red;
    colors[1] = vpColor::green;
    colors[2] = vpColor::blue;
    colors[3] = vpColor::yellow;
    bool first_time = true;

    vpFeatureLine p[nbline], pd[nbline];

    while(!final_quit) {
      try {
        g.acquire(I);
        vpDisplay::display(I);

        delta_t = (simTime - old_time);

        if(first_time){
          std::vector<vpHomogeneousMatrix> cMo_vec;
          detector.detect(I, opt_tagSize, cam, cMo_vec);
          std::vector<vpImagePoint> corners = detector.getPolygon(0);

          if (cMo_vec.size() == 1) {
//            cMo = cMo_vec[0];
            // Initialize the tracking. Define the four lines to track.
            for (i = 0; i < nbline; i++) {
              line[i].setDisplay(vpMeSite::RANGE_RESULT);
              line[i].setMe(&me);
              if(i == 0){
                line[i].initTracking(I,corners[3],corners[2]);
              }else if(i == 1){
                line[i].initTracking(I,corners[2],corners[1]);
              }else if(i == 2){
                line[i].initTracking(I,corners[3],corners[0]);
              }

              line[i].track(I);

              vpFeatureBuilder::create(p[i], cam, line[i]);
              vpFeatureBuilder::create(pd[i], lined[i]);
              task.addFeature(p[i], pd[i]);
            }

          }
          first_time = false;
        }

//        dot_dot_sd = Ks*(s_star - sd)  + Ds*(ds_star - dot_sd) + dds_star + Ls*M_inv*Wc ; //  Js_tmp*B_tmp.inverseByCholesky()*tau_ext_tmp
//        dot_sd += dot_dot_sd*delta_t;
//        sd += dot_sd*delta_t;

        // Track the lines and update the features
        for (i = 0; i < nbline; i++) {
          line[i].track(I);
          line[i].display(I, colors[i]);

          vpFeatureBuilder::create(p[i], cam, line[i]);
          // update desired feature
//          pd[i].setRhoTheta(sd[2*i],sd[2*i+1]) ;

          p[i].display(cam, I, colors[i]);
          pd[i].display(cam, I, colors[i],3);
        }

        double gain;
        {
          if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon())
            gain = lambda_av;
          else {
            gain = alpha * exp(-beta * (task.getError()).sumSquare()) + lambda_av;
          }
        }
        task.setLambda(3);

        v_c = task.computeControlLaw();
        Ls = task.getInteractionMatrix();
        vpDisplay::flush(I);

        if (opt_plot) {
          plotter->plot(0, simTime, task.getError());
          plotter->plot(1, simTime, Wc);
//              iter_plot++;
        }

        if (iter == 0)
          vpDisplay::getClick(I);
//        if (v_c.sumSquare() > 0.5) {
//          v_c = 0;
//          vpDisplay::getClick(I);
//          std::cout << "Camera velocity is too high!" << std::endl;
//        }
        if(!send_velocities){
          v_c = 0;
        }

        // Send command to the camera
        robot.setVelocity(vpRobot::END_EFFECTOR_FRAME,(eVc*v_c));


//        trigger.data = true;
//        nextStepTrigger_pub.publish(trigger);

        vpMouseButton::vpMouseButtonType button;
        vpTime::wait(2);
        if (vpDisplay::getClick(I, button, false)) {
          switch (button) {
          case vpMouseButton::button1:
            send_velocities = !send_velocities;
            break;

          case vpMouseButton::button3:
            final_quit = true;
            v_c = 0;
//            F_c = 0;
            break;

          default:
            break;
          }

        }

        old_time = simTime;

      } catch (...) {
        v_c = 0;
//        trigger.data = true;
//        nextStepTrigger_pub.publish(trigger);
        exit(1);
      }

      iter++;
    }

//    task.print();
    task.kill();
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Test failed with exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}



