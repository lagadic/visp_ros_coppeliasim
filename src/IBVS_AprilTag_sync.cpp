/*
 * IBVS_AprilTag_sync.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: oliva
 */

#include "fakeFCI.h"
#include <iostream>
#include <mutex>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#undef Bool
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
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

#include <boost/thread/thread.hpp>

#include "Iir.h"


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
bool compute = true;
pthread_mutex_t mutex_ros_compute;
void stepDone_callback(const std_msgs::Bool &msg){
  pthread_mutex_lock(&mutex_ros_compute);
	compute = msg.data;
	pthread_mutex_unlock(&mutex_ros_compute);
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
  double opt_tagSize = 0.08;
  std::string opt_eMc_filename = "";
  bool display_tag = true;
  int opt_quad_decimate = 2;
  bool opt_verbose = false;
  bool opt_plot = false;
  bool opt_adaptive_gain = false;
  bool opt_task_sequencing = false;
  double convergence_threshold = 0.00005;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      opt_tagSize = std::stod(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--verbose") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--plot") {
      opt_plot = true;
    }
    else if (std::string(argv[i]) == "--adaptive_gain") {
      opt_adaptive_gain = true;
    }
    else if (std::string(argv[i]) == "--task_sequencing") {
      opt_task_sequencing = true;
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      opt_quad_decimate = std::stoi(argv[i + 1]);
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
      std::cout << argv[0] << "[--tag_size <marker size in meter; default " << opt_tagSize << ">] [--eMc <eMc extrinsic file>] "
                           << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] [--adaptive_gain] [--plot] [--task_sequencing] [--no-convergence-threshold] [--verbose] [--help] [-h]"
                           << "\n";
      return EXIT_SUCCESS;
    }
  }

  std::mutex vs_mutex;

  try {

    //----------------------Butterworth Filter -------------------------------//
    //------------------------------------------------------------------------//
    const int order = 4; // 4th order (=2 biquads)
    std::array<Iir::Butterworth::LowPass<order>, 7> MultiChannelFilter;
    const float samplingrate = 1000; // Hz
    const float cutoff_frequency = 2; // Hz
    for(int i=0;i<7;i++){
      MultiChannelFilter[i].setup(samplingrate, cutoff_frequency);
    }
    //------------------------------------------------------------------------//
    //------------------------------------------------------------------------//
	  //    ROS node    //
	  ros::init(argc, argv, "visp_ros_coppeliasim");
//	  ros::NodeHandle n;
	  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
	  ros::Rate loop_rate(1000);
	  ros::spinOnce();

	  fakeFCI robot;
	  robot.connect();
	  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
//	  robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);

	  // Franka cartesian info publishers:
	  ros::Publisher camera_twist_pub = n->advertise<geometry_msgs::TwistStamped>("/camera/twist", 1);
	  ros::Publisher camera_wrench_pub = n->advertise<geometry_msgs::WrenchStamped>("/camera/wrench", 1);
	  ros::Publisher nextStepTrigger_pub = n->advertise<std_msgs::Bool>("/triggerNextStep", 1);
	  ros::Publisher enableSyncMode_pub = n->advertise<std_msgs::Bool>("/enableSyncMode", 1);
	  ros::Publisher startSimTrigger_pub = n->advertise<std_msgs::Bool>("/startSimulation", 1);
	  ros::Publisher stopSimTrigger_pub = n->advertise<std_msgs::Bool>("/stopSimulation", 1);
	  geometry_msgs::TwistStamped camera_twist;
	  geometry_msgs::WrenchStamped camera_wrench;
	  std_msgs::Bool trigger;
	  std_msgs::Bool syncMode;
	  std_msgs::Bool StartStopSim;

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

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

    // Servo
    vpHomogeneousMatrix cdMc, cMo, oMo,cMft;

//    cMft.buildFrom(0,-0.062,0.016,0,0,0);
    cMft.buildFrom(0,0,0,0,0,0);
    vpForceTwistMatrix cFft;
    cFft.buildFrom(cMft);

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo( vpTranslationVector(0, 0.0, 0.22),
                              vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );


    // Create visual features
    std::vector<vpFeaturePoint> p(4), pd(4); // We use 4 points

    // Define 4 3D points corresponding to the CAD model of the Apriltag
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSize/2., -opt_tagSize/2., 0);
    point[1].setWorldCoordinates( opt_tagSize/2., -opt_tagSize/2., 0);
    point[2].setWorldCoordinates( opt_tagSize/2.,  opt_tagSize/2., 0);
    point[3].setWorldCoordinates(-opt_tagSize/2.,  opt_tagSize/2., 0);

    vpServo task, task_pbvs;
    // Add the 4 visual feature points
    for (size_t i = 0; i < p.size(); i++) {
      task.addFeature(p[i], pd[i]);
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    cdMc = cdMo * cMo.inverse();
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc);
    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);
    task_pbvs.addFeature(t, td);
    task_pbvs.addFeature(tu, tud);
    task_pbvs.setServo(vpServo::EYEINHAND_CAMERA);
    task_pbvs.setInteractionMatrixType(vpServo::CURRENT);

    //----------------------------------------------
//    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
//    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
//    t.buildFrom(cMo*cdMo.inverse());
//    tu.buildFrom(cMo*cdMo.inverse());
//
//    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
//    vpFeatureThetaU tud(vpFeatureThetaU::cdRc);
//
//    vpServo task;
//    task.addFeature(t, td);
//    task.addFeature(tu, tud);
//    task.setServo(vpServo::EYEINHAND_CAMERA);
//    task.setInteractionMatrixType(vpServo::CURRENT);

    //-----------------------

    if (opt_adaptive_gain) {
      vpAdaptiveGain lambda(10, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
      task.setLambda(lambda);
      task_pbvs.setLambda(lambda);
    }
    else {
      task.setLambda(4);
      task_pbvs.setLambda(4);
    }

    vpPlot *plotter = nullptr;
    int iter_plot = 0;

    if (opt_plot) {
      plotter = new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
      plotter->setTitle(0, "Visual features error");
      plotter->setTitle(1, "joint velocities");
//      plotter->setTitle(1, "Force/torque Measurements");
      plotter->initGraph(0, 8);
      plotter->initGraph(1, 7);
      plotter->setLegend(0, 0, "error_feat_p1_x");
      plotter->setLegend(0, 1, "error_feat_p1_y");
      plotter->setLegend(0, 2, "error_feat_p2_x");
      plotter->setLegend(0, 3, "error_feat_p2_y");
      plotter->setLegend(0, 4, "error_feat_p3_x");
      plotter->setLegend(0, 5, "error_feat_p3_y");
      plotter->setLegend(0, 6, "error_feat_p4_x");
      plotter->setLegend(0, 7, "error_feat_p4_y");
      plotter->setLegend(1, 0, "F_x");
      plotter->setLegend(1, 1, "F_y");
      plotter->setLegend(1, 2, "F_z");
      plotter->setLegend(1, 3, "Tau_x");
      plotter->setLegend(1, 4, "Tau_y");
      plotter->setLegend(1, 5, "Tau_z");
      plotter->setLegend(1, 6, "Tau_z");
    }

    bool final_quit = false;
    bool has_converged = false;
    bool send_velocities = false;
    bool servo_started = false;
    std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

    static double t_init_servo = vpTime::measureTimeMs();
    vpColVector v_c(6,0), F_c(7,0), v_c_pbvs(6,0);

    vpMatrix Ls(8,6), Ls_pinv(6,8), Kp(8,8), Kv(8,8), dot_Ls(8,6), Ls_old(8,6);
    Kp.diag(kp);
    Kv.diag(kv);
    Ls = 0;
    vpMatrix M(6,6), I3(3,3), Bs(8,8), Cs(8,8), M_inv(6,6);
    vpColVector s(8,0), sd(8,0), dot_s(8,0), dot_sd(8,0), dot_dot_sd(8,0), es(8,0), dot_es(8,0), Fs(8,0);
    vpColVector dq(7,0);
    dot_sd = 0;
    dot_dot_sd = 0;
    es = 0;
    dot_es = 0;
    Fs = 0;

//	I3.eye();
//	M.insert(5*I3,0,0);
//	M.insert(0.1*I3,3,3);
//	M_inv = M.inverseByCholeskyOpenCV();

	Ls_old = 0;
	dot_Ls = 0;

	StartStopSim.data = true;
  startSimTrigger_pub.publish(StartStopSim);
  vpTime::wait(1000);
	stopSimTrigger_pub.publish(StartStopSim);
	vpTime::wait(1000);
	syncMode.data = true;
  enableSyncMode_pub.publish(trigger);
  startSimTrigger_pub.publish(StartStopSim);
  vpTime::wait(1000);
//  trigger.data = true;
//  nextStepTrigger_pub.publish(trigger);

  double old_time = simTime;
  double current_time = simTime;
  double delta_t = 0;
  double convergence_time = 0;
  vpColVector hq(8,0);
  vpColVector dds_star(8,0), ds_star(8,0), s_star(8,0);
  vpMatrix Ks(8,8), Ds(8,8);
  dds_star = 0;
  ds_star = 0;
  s_star = 0;
  Ks.diag(kp);
  Ds.diag(kv);//2*std::sqrt(kp));

  vpColVector Fc_d(6,0), Fc_tmp(6,0), Fc_int(6,0);
  vpMatrix Kfp(6,6), Kfi(6,6);
  Kfp.diag(5);
  Kfi.diag(12);

  Fc_d[2] = -5*0;

  vpHomogeneousMatrix eMc;
  eMc.buildFrom(0.05,-0.05 ,0,0,0,M_PI_4);
  robot.set_eMc(eMc);

  vpVelocityTwistMatrix cVe;
  cVe.buildFrom(eMc.inverse());
  vpForceTwistMatrix cFe;
  cFe.buildFrom(eMc.inverse());
  vpMatrix eJe(6,7), B(7,7), eJe_old(6,7), dot_eJe(6,7), Js(8,7), Ps(7,7);
  vpColVector C(7,0), tau_damped(7,0);



    while (!final_quit) {
		ros::spinOnce();
//		loop_rate.sleep();
    	if(compute){
        pthread_mutex_lock(&mutex_ros_compute);
        compute = false;
        current_time = simTime;
        pthread_mutex_unlock(&mutex_ros_compute);

    	  double t_start = vpTime::measureTimeMs();
    	  g.acquire(I);
    	  vpDisplay::display(I);

    	  std::vector<vpHomogeneousMatrix> cMo_vec;
    	  detector.detect(I, opt_tagSize, cam, cMo_vec);

    	  {
    	    std::stringstream ss;
    	    ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
    	    vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
    	  }

    	  // Only one tag is detected
    	  if (cMo_vec.size() == 1) {
    	    cMo = cMo_vec[0];

    	    static bool first_time = true;
    	    if (first_time) {
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

    	      // Compute the desired position of the features from the desired pose
    	      std::cout << "Initializing desired pose" << std::endl;
    	      for (size_t i = 0; i < point.size(); i++) {
    	        vpColVector cP, p_;
    	        point[i].changeFrame(cdMo * oMo, cP);
    	        point[i].projection(cP, p_);

    	        pd[i].set_x(p_[0]);
    	        pd[i].set_y(p_[1]);
    	        s_star[2*i] = p_[0];
    	        s_star[2*i + 1] = p_[1];
    	        pd[i].set_Z(cP[2]);
    	      }

    	      sd = s_star;
    	    }// end first_time

          delta_t = (current_time - old_time);
          v_c = task.computeControlLaw();
          v_c_pbvs = task_pbvs.computeControlLaw();
          Ls = task.getInteractionMatrix();
//          std::cout << "delt t: " << delta_t << std::endl;

          Fc_int += (Fc_d - Wc)*delta_t;
          Fc_tmp = cFft*(Fc_d + Kfp*(Fc_d - Wc) + 0*Kfi*Fc_int);

//          std::cout << "Computing admittance law" << std::endl;
//          Wc[0] = Wc[1] = Wc[3] = Wc[4] = Wc[5] = 0;
          robot.get_eJe(eJe);
          robot.getMass(B);
          robot.getCoriolis(C);
          robot.getVelocity(vpRobot::JOINT_STATE,dq);

          Js = Ls*cVe*eJe;
//          dot_dot_sd = dds_star + Ks*(s_star - sd)  + Ds*(ds_star - dot_sd)  - Ls*eVc.inverse()*eJe*B.inverseByCholesky()*eJe.t()*Wc ;
//          dot_dot_sd = dds_star + Ks*(s_star - sd)  + Ds*(ds_star - dot_sd)  - Js*B.inverseByCholesky()*eJe.t()*Fc_tmp;
//          dot_sd += dot_dot_sd*delta_t;
//          sd += dot_sd*delta_t;

            // Get tag corners
            std::vector<vpImagePoint> corners = detector.getPolygon(0);

            // Update visual features
//            std::cout << "updating image reference" << std::endl;
            for (size_t i = 0; i < corners.size(); i++) {
              // Update the desired point feature from the admittance law

              pd[i].set_x(sd[2*i]);
              pd[i].set_y(sd[2*i + 1]);

              // Update the point feature from the tag corners location
              vpFeatureBuilder::create(p[i], cam, corners[i]);
              // Set the feature Z coordinate from the pose
              vpColVector cP;
              point[i].changeFrame(cMo, cP);

              p[i].set_Z(cP[2]);
            }

            cdMc = cdMo * oMo * cMo.inverse();
            t.buildFrom(cdMc);
            tu.buildFrom(cdMc);

              if(delta_t != 0){
                dot_Ls = (Ls_old - Ls)/delta_t;
                dot_eJe = (eJe_old - eJe)/delta_t;
                tau_damped = (B*dq)/delta_t;
              }else{
                dot_Ls = 0;
                dot_eJe = 0;
                tau_damped = 0;
              }
//              std::cout << Ls_old << "\n --- \n" << Ls << std::endl;
//              std::cin.ignore();
              hq = dot_Ls*cVe*eJe*dq + Ls*cVe*dot_eJe*dq;
//              for(int i = 0; i< 8; i++){
//                dotLsVc[i] = MultiChannelFilter[i].filter((float)dotLsVc[i]);
//              }
              Ls_old = Ls;
              eJe_old = eJe;

              s = task.s;
              sd = task.sStar;
              dot_s = Js*dq;
//
              es = sd - s;
              dot_es = dot_sd - dot_s;
              Fs = Kp*es + Kv*dot_es + 0*dot_dot_sd - 0*hq;

              // computing the NULL space projector for dynamically consistent redundancy resolution
              Ps.eye();
              Ps = Ps - Js.t()*((Js*B.inverseByCholesky()*Js.t()).pseudoInverse()*Js*B.inverseByCholesky());

//              F_c = (M*Ls.t()*(Ls*M*Ls.t()).pseudoInverse())*Fs;
//              F_c = (B*Js.t()*(Js*B*Js.t()).pseudoInverse())*Fs;
              F_c = (B.inverseByCholesky()*Js.t()*(Js*B.inverseByCholesky()*Js.t()).pseudoInverse())*Fs + 0*C - 0*Ps*tau_damped;//------------------
//              F_c = M*Ls.pseudoInverse()*Fs + 0*Wc;
//              F_c = B*Js.pseudoInverse()*Fs + C - Ps*tau_daped;
              for(int i=0; i<7;i++){
                F_c[i] = MultiChannelFilter[i].filter((float)F_c[i]);
              }

            // Display the current and desired feature points in the image display
            vpServoDisplay::display(task, cam, I);
            for (size_t i = 0; i < corners.size(); i++) {
              std::stringstream ss;
              ss << i;
              // Display current point indexes
              vpDisplay::displayText(I, corners[i]+vpImagePoint(15, 15), ss.str(), vpColor::red);
              // Display desired point indexes
              vpImagePoint ip;
              vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
              vpDisplay::displayText(I, ip+vpImagePoint(15, 15), ss.str(), vpColor::red);
            }
            if (first_time) {
               traj_corners = new std::vector<vpImagePoint> [corners.size()];
            }
            // Display the trajectory of the points used as features
            display_point_trajectory(I, corners, traj_corners);

            if (opt_plot) {
              plotter->plot(0, current_time, es);
              plotter->plot(1, current_time, v_c);
//              iter_plot++;
            }

            if (opt_verbose) {
              std::cout << "v_c: " << v_c.t() << std::endl;
            }

            double error = task.getError().sumSquare();
            std::stringstream ss;
            ss << "error: " << error;
            vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

            if (opt_verbose)
              std::cout << "error: " << error << std::endl;

            if (!has_converged && error < convergence_threshold) {
              convergence_time = current_time;
              has_converged = true;
              std::cout << "Servo task has converged" << "\n";
              vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
            }

//            if(has_converged){
//              // this can be used to programatically generate force measurements
//              Wc[3] = 0.1*std::sin((simTime - convergence_time)*10);
//            }

            if (first_time) {
            	first_time = false;
            }
          } // end if (cMo_vec.size() == 1)
          else {
            v_c = 0;
            F_c = 0;
          }

          if (!send_velocities) {
            v_c = 0;
            F_c = 0;
          }


          vpColVector aux(6,0);
          aux = cVe.inverse()*v_c;
//          aux = eVc*v_c_pbvs;
//          aux = v_c;
//          aux[0] = 0.5;
//          if (opt_plot) {
//            plotter->plot(1, simTime, Wc);
//            //              iter_plot++;
//          }
//          std::cout << "setting velocities" << std::endl;
          robot.setVelocity(vpRobot::CAMERA_FRAME,v_c);
//          robot.setVelocity(vpRobot::CAMERA_FRAME,v_c);


//          F_c =

//          robot.setForceTorque(vpRobot::JOINT_STATE,F_c);

          trigger.data = true;
//          nextStepTrigger_pub.publish(trigger);


//          std::cout << "updating time in Image" << std::endl;
          std::stringstream ss;
          ss << "Loop time: " << delta_t * 1000<< " ms";
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

          vpDisplay::flush(I);
          old_time = current_time;
          vpTime::wait(50);

    	}// end if(compute)

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
            F_c = 0;
            break;

          default:
            break;
          }

        }

    }//end while


    if (opt_plot && plotter != nullptr) {
      delete plotter;
      plotter = nullptr;
    }
//    syncMode.data = false;
//    enableSyncMode_pub.publish(syncMode);
    stopSimTrigger_pub.publish(StartStopSim);

    task.kill();

    if (!final_quit) {
      while (!final_quit) {
        g.acquire(I);
        vpDisplay::display(I);

        vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
        vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

        if (vpDisplay::getClick(I, false)) {
          final_quit = true;
        }

        vpDisplay::flush(I);
      }
    }
    if (traj_corners) {
      delete [] traj_corners;
    }
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}

