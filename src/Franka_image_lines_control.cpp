/*
 * Franka_image_lines_control.cpp
 *
 *  Created on: Aug 31, 2020
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

#include "Iir.h"

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

vpColVector FT_readings(6,0);
void FT_sensor_callback(const geometry_msgs::WrenchStamped& msg)
{
  pthread_mutex_lock(&mutex_ros);
  FT_readings[0] = msg.wrench.force.x;
  FT_readings[1] = msg.wrench.force.y;
  FT_readings[2] = msg.wrench.force.z;
  FT_readings[3] = msg.wrench.torque.x;
  FT_readings[4] = msg.wrench.torque.y;
  FT_readings[5] = msg.wrench.torque.z;
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

void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
    std::vector<vpImagePoint> *traj_vip)
{
  for (size_t i = 0; i < vip.size(); i++) {
    if (traj_vip[i].size()) {
      // Add the point only if distance with the previous > 1 pixel
      if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
        traj_vip[i].push_back(vip[i]);
      }
    }
    else {
      traj_vip[i].push_back(vip[i]);
    }
  }
  for (size_t i = 0; i < vip.size(); i++) {
    for (size_t j = 1; j < traj_vip[i].size(); j++) {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}

int main(int argc, char **argv)
{
  double kp = 0.5, kd = 0.1;
  bool opt_plot = false;
  bool display_tag = true;
  int opt_quad_decimate = 2;
  bool opt_adaptive_gain = false;
  double opt_tagSize = 0.08;
  double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad(0.5);


  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      opt_tagSize = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--no-convergence-threshold") {
      convergence_threshold_t = 0.;
      convergence_threshold_tu = 0.;
    }else if (std::string(argv[i]) == "--kp" ){
      kp = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--kd" ){
      kd = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--adaptive_gain") {
      opt_adaptive_gain = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] <<  "[--plot] [--no-convergence-threshold] [--kp <proportional_gain>] [--kd <derivative_gain>] [--help] [-h]"
          << "\n";
      return EXIT_SUCCESS;
    }
  }
  try {

    //----------------------Butterworth Filter -------------------------------//
    //------------------------------------------------------------------------//
      std::cout << "Declaring a Butterworth 4th order filter \n" << std::endl;
    const int order = 1; // 4th order (=2 biquads)
    std::array<Iir::Butterworth::LowPass<order>, 6> MultiChannelFilter;

    const float samplingrate = 1000; // Hz
    const float cutoff_frequency = 10; // Hz
    for(int i=0;i<6;i++){
      MultiChannelFilter[i].setup (samplingrate, cutoff_frequency);
    }


    std::array<Iir::Butterworth::LowPass<order>, 7> TorqueFilter;
    for(int i=0;i<7;i++){
      TorqueFilter[i].setup (samplingrate, 10);
    }
    std::array<Iir::Butterworth::LowPass<2>, 6> TorqueFilter2;
    for(int i=0;i<6;i++){
      TorqueFilter2[i].setup (samplingrate, 10);
    }
    //------------------------------------------------------------------------//
    //------------------------------------------------------------------------//
    //    ROS node    //
    ros::init(argc, argv, "Franka_joint_torque_control");
    //    ros::NodeHandle n;
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Rate loop_rate(1000);
    ros::spinOnce();

    fakeFCI robot;
    robot.connect();
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
//    robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);

    // Controller publishers:
    ros::Publisher nextStepTrigger_pub = n->advertise<std_msgs::Bool>("/triggerNextStep", 1);
    ros::Publisher enableSyncMode_pub = n->advertise<std_msgs::Bool>("/enableSyncMode", 1);
    ros::Publisher startSimTrigger_pub = n->advertise<std_msgs::Bool>("/startSimulation", 1);
    ros::Publisher stopSimTrigger_pub = n->advertise<std_msgs::Bool>("/stopSimulation", 1);
    std_msgs::Bool trigger;
    std_msgs::Bool syncMode;
    std_msgs::Bool StartStopSim;
    // Controller subscrivers:
    ros::Subscriber sub_simulationStepDone = n->subscribe("/simulationStepDone", 1, stepDone_callback);
    ros::Subscriber sub_simulationTime = n->subscribe("/simulationTime", 1, simTime_callback);
    ros::Subscriber sub_joycon = n->subscribe("/joy", 1, joycon_callback);
    ros::Subscriber sub_FT = n->subscribe("/FT_sensor", 1, FT_sensor_callback);

    unsigned int width = 512, height = 512;
    vpROSGrabber rs;
    rs.setImageTopic("/vrep/franka_camera/image");
    rs.open(argc, argv);

    vpImage<unsigned char> I(height, width);
    vpDisplayX dc(I, 10, 10, "Color image");
    vpCameraParameters cam(305, 305, I.getWidth() / 2, I.getHeight() / 2);

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

    StartStopSim.data = true;
    startSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(100);
    stopSimTrigger_pub.publish(StartStopSim);
    vpTime::wait(3000);
    syncMode.data = true;
    trigger.data = true;
    enableSyncMode_pub.publish(syncMode);

    // Servo
    vpHomogeneousMatrix cdMc, cMo, oMo;

    // Desired pose to reach
//    vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0, 0.1), // 3 times tag with along camera z axis
//                              vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );

    vpHomogeneousMatrix cdMo(0, 0, 0.2, 0, 0, vpMath::rad(0));

    cdMc = cdMo * cMo.inverse();
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

    lined[0].setWorldCoordinates(0, 1, 0,  opt_tagSize/2, 0, 0, 1, 0);
    lined[1].setWorldCoordinates(1, 0, 0,  opt_tagSize/2, 0, 0, 1, 0);
    lined[2].setWorldCoordinates(1, 0, 0,  -opt_tagSize/2, 0, 0, 1, 0);



    lined[0].project(cdMo);
    lined[1].project(cdMo);
    lined[2].project(cdMo);
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
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.setLambda(1);

    vpPlot *plotter = nullptr;

    if (opt_plot) {
      plotter = new vpPlot(4, static_cast<int>(250 * 4), 500, 800, 10, "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "Joint Torque commands");
      plotter->setTitle(2, "Error Norm tr");
      plotter->setTitle(3, "Error Norm tu");
      plotter->initGraph(0, 7);
      plotter->initGraph(1, 6);
      plotter->initGraph(2, 1);
      plotter->initGraph(3, 1);
      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
//      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "vc_x");
      plotter->setLegend(1, 1, "vc_y");
      plotter->setLegend(1, 2, "vc_z");
      plotter->setLegend(1, 3, "wc_x");
      plotter->setLegend(1, 4, "wc_y");
      plotter->setLegend(1, 5, "wc_z");
//      plotter->setLegend(1, 6, "wc_z");
//      plotter->setLegend(1, 7, "wc_z");

      plotter->setLegend(2, 0, "Error tr");
      plotter->setLegend(3, 0, "Error tu");
    }

    bool final_quit = false;
    double delta_t = 0;
    double old_time = 0;
    double current_time = 0;
    double start_time = 0;

    int nFeat = 6;

    vpColVector q(7,0), q_d(7,0), dq(7,0), dq_d(7,0), tau(7,0), C(7,0), g(7,0),
                gains(6,0), v(6,0),s_old(nFeat,0), ds_old(nFeat,0), tau_J(7,0),
                v_c(6,0), s(nFeat,0), sd(nFeat,0), ds(nFeat,0), ds_n(nFeat,0),
                dsd(nFeat,0), dds(nFeat,0),dds_n(nFeat,0), ddsd(nFeat,0),
                es(nFeat,0), des(nFeat,0), tau_damped(7,0);
    vpMatrix Kp_c(nFeat,nFeat), Kd_c(nFeat,nFeat), eJe(6,7), fJe(6,7),Ls(nFeat,6),
             Ls_old(nFeat,6), d_Ls(nFeat,6),B(7,7),Js(nFeat,7), Js_old(nFeat,7),
             d_Js(nFeat,7), Js_B(7,nFeat), B_inv(7,7), Ps(7,7), Kp(7,7), Kd(7,7),
             Bc_inv(6,6), eJe_old(6,7), d_eJe(6,7);
    gains = {1,1,1, 100,100,100};
    Bc_inv.diag(gains);

    gains.resize(7,true);
    gains = {300.0, 300.0, 150.0, 150.0, 100.0, 50.0, 2.0};
    Kp.diag(kp);
    gains = {20.0, 20.0, 15.0, 15.0, 10.0, 5.0, 0.5};
    Kd.diag(kd);;

    q_d = {0, -M_PI_4, 0, - 3* M_PI_4, 0, M_PI_2, M_PI_4};
    gains.resize(nFeat,true);
    gains = {20.0, 20.0, 20.0, 2.0, 2.0, 2.0,
//             20.0, 20.0
            };
    Kp_c.diag(kp);
    gains = {2*std::sqrt(gains[0]), 2*std::sqrt(gains[1]), 2*std::sqrt(gains[2]),
             2*std::sqrt(gains[3]), 2*std::sqrt(gains[4]), 2*std::sqrt(gains[5]),
//             2*std::sqrt(gains[6]), 2*std::sqrt(gains[7])
    };
    Kd_c.diag(kd);


    startSimTrigger_pub.publish(StartStopSim);
    nextStepTrigger_pub.publish(trigger);
    vpTime::wait(1000);
    nextStepTrigger_pub.publish(trigger);
    vpTime::wait(1000);
    nextStepTrigger_pub.publish(trigger);
    vpTime::wait(1000);
    nextStepTrigger_pub.publish(trigger);

    bool has_converged = false;
    bool send_cmds = false;
    bool servo_started = false;
    double alpha = 0.05;
    double lambda_av = 0.5;
    double beta = 3;
    std::vector<vpImagePoint> *traj_vip = nullptr; // To memorize point trajectory
    std::vector<vpImagePoint> *traj_corners = nullptr;
    vpHomogeneousMatrix eMc;
    eMc.buildFrom(0.05,-0.05 ,0,0,0,M_PI_4);
    robot.set_eMc(eMc);
//    std::cout << "eMc: \n" << eMc << std::endl;

    vpVelocityTwistMatrix eTc(eMc);
    vpVelocityTwistMatrix cTe(eMc.inverse());
    vpForceTwistMatrix cFe(eMc.inverse());
    vpForceTwistMatrix eFc(eMc);

    vpColor colors[4];
    colors[0] = vpColor::red;
    colors[1] = vpColor::green;
    colors[2] = vpColor::blue;
    colors[3] = vpColor::yellow;
    bool first_time = true;

    vpFeatureLine p[nbline], pd[nbline];

    start_time = simTime;
    old_time = start_time;
    while(!final_quit) {
      if(compute){
        pthread_mutex_lock(&mutex_ros);
        compute = false;
        current_time = simTime;
        pthread_mutex_unlock(&mutex_ros);
        delta_t = (current_time - old_time);
//        std::cout << "delta_t: " << delta_t << std::endl;
        rs.acquire(I);
        vpDisplay::display(I);

        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, opt_tagSize, cam, cMo_vec);

        std::stringstream ss;
        ss << "Left click to " << (send_cmds ? "stop the robot" : "servo the robot") << ", right click to quit.";
        vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

        // Get tag corners
        std::vector<vpImagePoint> corners = detector.getPolygon(0);

        static bool first_time = true;
        if (first_time) {
          // Only one tag is detected
          if (cMo_vec.size() == 1) {
            cMo = cMo_vec[0];
            // Introduce security wrt tag positionning in order to avoid PI rotation
            std::vector<vpHomogeneousMatrix> v_oMo(2), v_cdMc(2);
            v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
            for (size_t i = 0; i < 2; i++) {
              v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
            }
            if (std::fabs(v_cdMc[0].getThetaUVector().getTheta()) < std::fabs(v_cdMc[1].getThetaUVector().getTheta())) {
              oMo = v_oMo[0];
            }
            else {
              std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
              oMo = v_oMo[1];   // Introduce PI rotation
            }

            // Initialize the tracking. Define the four lines to track.
            for (i = 0; i < nbline; i++) {
              line[i].setDisplay(vpMeSite::RANGE_RESULT);
              line[i].setMe(&me);
              if(i == 0){
                line[i].initTracking(I,corners[0],corners[1]);
              }else if(i == 1){
                line[i].initTracking(I,corners[0],corners[3]);
              }else if(i == 2){
                line[i].initTracking(I,corners[1],corners[2]);
              }

              line[i].track(I);

              vpFeatureBuilder::create(p[i], cam, line[i]);
              vpFeatureBuilder::create(pd[i], lined[i]);
              task.addFeature(p[i], pd[i]);
            }
          }// end if (cMo_vec.size() == 1)
          first_time = false;
        }

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

//        double gain;
//        {
//          if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon())
//            gain = lambda_av;
//          else {
//            gain = alpha * std::exp(-beta * (task.getError()).sumSquare()) + lambda_av;
//          }
//        }
        task.setLambda(2);

        v_c = task.computeControlLaw();
        Ls = task.getInteractionMatrix();


        robot.getMass(B);
        robot.getCoriolis(C);
        B_inv = B.inverseByCholesky();
        robot.get_eJe(eJe);
        robot.get_fJe(fJe);
        robot.getVelocity(vpRobot::JOINT_STATE,dq);
        robot.getPosition(vpRobot::JOINT_STATE,q);
        robot.getGravity(g);
        robot.getForceTorque(vpRobot::JOINT_STATE,tau_J);

        Js = Ls*cTe*eJe;
        ds = Js*dq;
        Js_B = B_inv*Js.t()*(Js*B_inv*Js.t()).pseudoInverse();

        if(delta_t == 0){
          d_Ls = 0;
          d_Js = 0;
          tau_damped = 0;
          ds_n = 0;
          dds_n = 0;
          d_eJe = 0;
        }else{
          d_Js = (Js - Js_old)/delta_t;
          tau_damped = 0.5*(B*dq)/delta_t;
          ds_n = (s - s_old)/delta_t;
          d_Ls = (Ls - Ls_old)/delta_t;
          d_eJe = (eJe - eJe_old)/delta_t;
        }
        Ls_old = Ls;
        Js_old = Js;
        s_old = s;
        eJe_old = eJe;

        // computing the NULL space projector for dynamically consistent redundancy resolution
        Ps.eye();
        Ps = Ps - Js.t()*((Js*B_inv*Js.t()).pseudoInverse()*Js*B_inv);

        vpColVector hs(6,0),  aux(7,0),  aux2(6,0);
        for(int i = 0; i< 7; i++){
          aux[i] = TorqueFilter[i].filter((float)((Ps*tau_damped)[i]));
        }
        for(int i = 0; i< 6; i++){
          hs[i] = MultiChannelFilter[i].filter((float)(d_Js*dq)[i]);
          ds[i] = TorqueFilter2[i].filter((float)ds[i]);
        }

        //          tau = Js_B*(Kp_c*(sd - s) - Kd_c*ds - d_Js*dq ) + C - Ps*tau_damped;
        tau = B*Js.pseudoInverse()*(-Kp_c*task.getError() - Kd_c*ds - hs + 0*Ls*Bc_inv*cFe*aux2) + C  - 0*eJe.t()*aux2 - Ps*tau_damped;



        if (opt_plot ) {
          plotter->plot(0, current_time, aux);
          //            plotter->plot(1, current_time, ;
          //            plotter->plot(3, current_time, );
        }

        if (!send_cmds) {
          v_c = 0;
          tau = 0;
        }
        // Send to the robot
        robot.setVelocity(vpRobot::CAMERA_FRAME,v_c);
//        robot.setForceTorque(vpRobot::JOINT_STATE,tau);

        nextStepTrigger_pub.publish(trigger);
        old_time = current_time;

        ss.str("");
        ss << "Loop time: " << delta_t*1000 << " ms";
        //          std::cout << "loop time: " << delta_t*1000 << " [ms]\n";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
        vpDisplay::flush(I);

      }// end if(compute)
      else{
//        std::cout << "waiting for computing...\n";
        vpTime::wait(100);
      }

      vpMouseButton::vpMouseButtonType button;
      vpTime::wait(2);
      if (vpDisplay::getClick(I, button, false)) {
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

    }// while(final_quit)
    trigger.data = false;
    enableSyncMode_pub.publish(syncMode);
    vpTime::wait(10);
    stopSimTrigger_pub.publish(StartStopSim);
    robot.setRobotState(vpRobot::STATE_STOP);
  }
  catch (const vpException &e) {
    std::cout << "Test failed with exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}




