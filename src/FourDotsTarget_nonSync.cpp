/*
 * FourDotsTarget_nonSync.cpp
 *
 *  Created on: 8 feb 2020
 *      Author: oliva
 */

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#include <pthread.h>


#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#undef Bool
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/gui/vpPlot.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpMath.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/vs/vpServo.h>

#include <visp3/sensor/vpRealSense2.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/vision/vpPose.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp3/blob/vpDot.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <visp_ros/vpROSGrabber.h>
#include <std_msgs/Bool.h>
#include <visp3/robot/vpSimulatorCamera.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}

void SortBlobsInTarget(std::list<vpDot2> &list_of_blobs);

void SortBlobsInTarget(std::list<vpDot2> &list_of_blobs){
	std::vector<vpDot2> blob_vect{ std::begin(list_of_blobs), std::end(list_of_blobs) };
	int min_idx = 0, max_idx = 0;
	double Min = 1000, Max = 0;
	for(int i = 0; i < 4 ; i++){
		if(blob_vect[i].getCog().get_i() < Min){
			Min = blob_vect[i].getCog().get_i();
			min_idx = i;
		}
		if(blob_vect[i].getCog().get_i() > Max){
			Max = blob_vect[i].getCog().get_i();
			max_idx = i;
		}
	}
	std::vector<vpDot2> sorted_blobs;
	int middle_idx[2];
	int j=0;
	for(int i = 0; i < 4; i++){
		if(i != min_idx && i != max_idx){
			middle_idx[j] = i;
			j++;
		}
	}
	double i_c=0,j_c=0,d=0,h=0,alpha=0;
	i_c = (blob_vect[max_idx].getCog().get_i() + blob_vect[min_idx].getCog().get_i())/2;
	j_c = (blob_vect[middle_idx[0]].getCog().get_j() + blob_vect[middle_idx[1]].getCog().get_j())/2;

	if(blob_vect[middle_idx[0]].getCog().get_j() > blob_vect[min_idx].getCog().get_j()){
		h = i_c - blob_vect[middle_idx[0]].getCog().get_i();
		alpha = std::atan2(h,d);
		if(alpha <= M_PI_4 && alpha > 0){
			sorted_blobs.push_back(blob_vect[min_idx]);
			sorted_blobs.push_back(blob_vect[middle_idx[0]]);
			sorted_blobs.push_back(blob_vect[max_idx]);
			sorted_blobs.push_back(blob_vect[middle_idx[1]]);
		}else{
			sorted_blobs.push_back(blob_vect[middle_idx[1]]);
			sorted_blobs.push_back(blob_vect[min_idx]);
			sorted_blobs.push_back(blob_vect[middle_idx[0]]);
			sorted_blobs.push_back(blob_vect[max_idx]);
		}
	}else{
		d = std::abs(blob_vect[middle_idx[1]].getCog().get_j() - j_c);
		h = i_c - blob_vect[middle_idx[1]].getCog().get_i();
		alpha = std::atan2(h,d);
		if(alpha <= M_PI_4 && alpha > 0){
			sorted_blobs.push_back(blob_vect[min_idx]);
			sorted_blobs.push_back(blob_vect[middle_idx[1]]);
			sorted_blobs.push_back(blob_vect[max_idx]);
			sorted_blobs.push_back(blob_vect[middle_idx[0]]);
		}else{
			sorted_blobs.push_back(blob_vect[middle_idx[0]]);
			sorted_blobs.push_back(blob_vect[min_idx]);
			sorted_blobs.push_back(blob_vect[middle_idx[1]]);
			sorted_blobs.push_back(blob_vect[max_idx]);
		}
	}

	list_of_blobs.clear();
	std::copy( sorted_blobs.begin(), sorted_blobs.end(), std::back_inserter( list_of_blobs ) );

}
void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot);

void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot)
{
	static std::vector<vpImagePoint> traj[4];
	for (unsigned int i = 0; i < 4; i++) {
		traj[i].push_back(dot[i].getCog());
	}
	for (unsigned int i = 0; i < 4; i++) {
		for (unsigned int j = 1; j < traj[i].size(); j++) {
			vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green, 2);
		}
	}
};

vpColVector Vc(6,0), Pc(6,0);
pthread_mutex_t mutex_ros;
bool compute = true;

void Twist_callback(const geometry_msgs::TwistStamped& cam_twist)
{
  pthread_mutex_lock(&mutex_ros);
  Vc[0]=cam_twist.twist.linear.x;
  Vc[1]=cam_twist.twist.linear.y;
  Vc[2]=cam_twist.twist.linear.z;
  Vc[3]=cam_twist.twist.angular.x;
  Vc[4]=cam_twist.twist.angular.y;
  Vc[5]=cam_twist.twist.angular.z;
  compute = true;
  pthread_mutex_unlock(&mutex_ros);
  return;
}

void Pose_callback(const geometry_msgs::PoseStamped& cam_pose)
{
  pthread_mutex_lock(&mutex_ros);
  Pc[0]=cam_pose.pose.position.x;
  Pc[1]=cam_pose.pose.position.y;
  Pc[2]=cam_pose.pose.position.z;
  Pc[3]=cam_pose.pose.orientation.x;
  Pc[4]=cam_pose.pose.orientation.y;
  Pc[5]=cam_pose.pose.orientation.z;
  pthread_mutex_unlock(&mutex_ros);
  return;
}

int main(int argc, char **argv)
{
	double kp = 0.5, kv = 0.1, a = 1;
	for (int i = 1; i < argc; i++) {
		if (std::string(argv[i]) == "--kp" ){
			kp = std::stod(argv[i + 1]);
		}
		else if (std::string(argv[i]) == "--kv" ){
			kv = std::stod(argv[i + 1]);
		}else if (std::string(argv[i]) == "--a" ){
			a = std::stod(argv[i + 1]);
		}
	}
	try {
		vpSimulatorCamera Flycam;
		Flycam.setSamplingTime(0.10);

		Vc = 0;
		vpColVector Vc_dot(6,0), Vc_old(6,0);

		vpPlot *plotter = NULL;
		int iter_plot = 0;

		plotter = new vpPlot(2, 250 * 2, 500, 0, 550, "Real time curves plotter");
		plotter->setTitle(0, "Commanded force");
		plotter->setTitle(1, "Feature errors");
		plotter->initGraph(0, 6);
		plotter->initGraph(1, 8);
		plotter->setLegend(0, 0, "F_x");
		plotter->setLegend(0, 1, "F_y");
		plotter->setLegend(0, 2, "F_z");
		plotter->setLegend(0, 3, "Tau_x");
		plotter->setLegend(0, 4, "Tau_y");
		plotter->setLegend(0, 5, "Tau_z");
		plotter->setLegend(1, 0, "x1");
		plotter->setLegend(1, 1, "y1");
		plotter->setLegend(1, 2, "x2");
		plotter->setLegend(1, 3, "y2");
		plotter->setLegend(1, 4, "x3");
		plotter->setLegend(1, 5, "y3");
		plotter->setLegend(1, 6, "x4");
		plotter->setLegend(1, 7, "y4");

		//    ROS node    //
		ros::init(argc, argv, "blob_tracker");
		ros::NodeHandle n;
		ros::Rate loop_rate(500);
		ros::spinOnce();

		// Franka cartesian info publishers:
		ros::Publisher camera_twist_pub = n.advertise<geometry_msgs::TwistStamped>("/camera/twist", 1);
		ros::Publisher camera_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("/camera/wrench", 1);
		ros::Publisher nextStepTrigger_pub = n.advertise<std_msgs::Bool>("/triggerNextStep", 1);
		ros::Publisher enableSyncMode_pub = n.advertise<std_msgs::Bool>("/enableSyncMode", 1);
		geometry_msgs::TwistStamped camera_twist;
		geometry_msgs::WrenchStamped camera_wrench;
		std_msgs::Bool trigger;
		std_msgs::Bool syncMode;

		ros::Subscriber sub_vel_franka = n.subscribe("/vrep/camera/twist", 1, Twist_callback);
		ros::Subscriber sub_pose_flycam = n.subscribe("/vrep/camera/pose", 1, Pose_callback);


		vpImage<unsigned char> I(512,512); // Create a gray level image container
		vpROSGrabber g;
		g.setImageTopic("/vrep/franka_camera/image");
		g.open(argc, argv);

		vpDisplayX d(I, 0, 0, "V-Rep view");
		vpCameraParameters cam(305, 305, I.getWidth() / 2, I.getHeight() / 2);

		g.acquire(I);

		vpDisplay::display(I);
		vpDisplay::flush(I);

		vpServo task;
		int i;

		task.setServo(vpServo::EYEINHAND_CAMERA);
		task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

		vpMatrix Ls(8,6), Ls_pinv(6,8), Kp(8,8), Kv(8,8), dot_Ls(8,6), Ls_old(8,6);
		Kp.diag(kp);
		Kv.diag(kv);
//		Kv[0][1]=Kv[1][0]=Kv[2][3]=Kv[3][2]=Kv[4][5]=Kv[5][4]=Kv[6][7]=Kv[7][6] = kv;

		double lambda = 5;
		task.setLambda(lambda);

		unsigned int iter = 0;
		vpColVector v(6), Fc(6);
		vpImage<vpRGBa> Ic;
		double lambda_av = 0.05;
		double alpha = 0.05;
		double beta = 3;
		double old_time = vpTime::measureTimeMicros();

		vpFeaturePoint p[4], pd[4];
		std::vector<vpImagePoint> ip(4);

		vpDot2 blob;
		std::list<vpDot2> blob_list;

		blob.setGraphics(true);
		blob.setGraphicsThickness(2);
		vpImagePoint germ;

		std::vector<vpPoint> point;
		double l = 0.05;   // V-REP target

		point.push_back(vpPoint(-l, -l, 0));
		point.push_back(vpPoint( l, -l, 0));
		point.push_back(vpPoint( l,  l, 0));
		point.push_back(vpPoint(-l,  l, 0));

		vpHomogeneousMatrix cdMo(-0.1,0,0.3,0,0,0);
		vpHomogeneousMatrix cMo, wMo, wMflycam;

		bool final_quit = false;
		bool has_converged = false;
		bool send_velocities = false;
		bool servo_starting = true;
		bool learn = true;
		bool init_done = false;

		double t_start = 0;
		double t_current = 0;
		double delta_t = 0.01;

		vpMatrix M(6,6), I3(3,3), Bs(8,8), Cs(8,8), M_inv(6,6);
		vpColVector s(8), sd(8), dot_s(8), dot_sd(8), dot_dot_sd(8), es(8), dot_es(8), Fs(8);
		dot_sd = 0;
		dot_dot_sd = 0;
		es = 0;
		dot_es = 0;
		Fs = 0;

		I3.eye();
		M.insert(0.2*I3,0,0);
		M.insert(0.00001*I3,3,3);
		M_inv = M.inverseByCholeskyOpenCV();

		Ls_old = 0;
		dot_Ls = 0;

		std::cout << " Starting the loop " << std::endl;
		while (!has_converged && !final_quit) {
			ros::spinOnce();
//			loop_rate.sleep();
			try {
			  if(compute){
//			    delta_t =  0.000001 * (vpTime::measureTimeMicros() - old_time);
//			    std::cout << iter << ": "<<  delta_t <<std::endl;
//			    old_time = vpTime::measureTimeMicros();


			    g.acquire(I);
			    vpDisplay::display(I);

			    if (!init_done ) {
			      vpDisplay::flush(I);
			      if(vpDisplay::getClick(I, germ, false)){
			        if (learn) {
			          std::cout << "Learning the characteristics of the blob to auto detect \n" << std::endl;
			          blob.setGraphics(true);
			          std::cout << "setGraphics done.. " << std::endl;
			          blob.setGraphicsThickness(1);
			          std::cout << "setGraphicsThickness done.. " << std::endl;
			          blob.initTracking(I);
			          std::cout << "initTracking done.. " << std::endl;
			          blob.track(I);
			          std::cout << "track done.. " << std::endl;
			          std::cout << "Blob characteristics: " << std::endl;
			          std::cout << " width : " << blob.getWidth() << std::endl;
			          std::cout << " height: " << blob.getHeight() << std::endl;
#if VISP_VERSION_INT > VP_VERSION_INT(2, 7, 0)
			          std::cout << " area: " << blob.getArea() << std::endl;
#endif
			          std::cout << " gray level min: " << blob.getGrayLevelMin() << std::endl;
			          std::cout << " gray level max: " << blob.getGrayLevelMax() << std::endl;
			          std::cout << " grayLevelPrecision: " << blob.getGrayLevelPrecision() << std::endl;
			          std::cout << " sizePrecision: " << blob.getSizePrecision() << std::endl;
			          std::cout << " ellipsoidShapePrecision: " << blob.getEllipsoidShapePrecision() << std::endl;
			          //          }
			        }else {
			          // Set blob characteristics for the auto detection
			          blob.setWidth(50);
			          blob.setHeight(50);
#if VISP_VERSION_INT > VP_VERSION_INT(2, 7, 0)
			          blob.setArea(1700);
#endif
			          blob.setGrayLevelMin(0);
			          blob.setGrayLevelMax(30);
			          blob.setGrayLevelPrecision(0.8);
			          blob.setSizePrecision(0.65);
			          blob.setEllipsoidShapePrecision(0.65);
			        }

			        // The blob that is tracked by initTracking() is not in the list of auto
			        // detected blobs We add it:
			        blob.searchDotsInArea(I, 0, 0, I.getWidth(), I.getHeight(), blob_list);
			        if (learn) {
			          blob_list.push_back(blob);
			        }
			        //sort the blobs
			        SortBlobsInTarget(blob_list);
			        int i = 0;
			        for (std::list<vpDot2>::iterator it = blob_list.begin(); it != blob_list.end(); ++it){
			          point[i].track(cdMo);
			          vpFeatureBuilder::create(pd[i], point[i]);
			          (*it).setGraphics(true);
			          (*it).setGraphicsThickness(3);
			          vpDisplay::flush(I);
			          vpFeatureBuilder::create(p[i], cam, ip[i]=(*it).getCog());
			          task.addFeature(p[i], pd[i]);
			          i++;
			        }
			        computePose(point, ip, cam, servo_starting, cMo);
			        // initialize the camera position
//			        vpHomogeneousMatrix wMflycam;
//			        vpPoseVector pose;
//			        pose[0] = Pc[0];
//			        pose[1] = Pc[1];
//			        pose[2] = Pc[2];
//			        pose[3] = Pc[3];
//			        pose[4] = Pc[4];
//			        pose[5] = Pc[5];
//			        wMflycam.buildFrom(pose);
//			        Flycam.setPosition(wMflycam);
//			        wMo = wMflycam * cMo;


			        if (blob_list.size() == 4){
			          init_done = true;
			          std::cout << "Number of auto detected blob: " << blob_list.size() << std::endl;
			          std::cout << "Right click to quit..." << std::endl;
			        }
			        t_start = vpTime::measureTimeSecond();
			        task.computeControlLaw();
			        Ls = task.getInteractionMatrix();

			      }

			    } else {
			      //          std::cout << "sono nell'else.... \n";

			      if (servo_starting){
			        servo_starting = false;
//			        syncMode.data = true;
//			        enableSyncMode_pub.publish(syncMode);
			      }
			      computePose(point, ip, cam, servo_starting, cMo);

//			      Flycam.getPosition(wMflycam);
//			      cMo = wMflycam.inverse() * wMo;

			      int i = 0;
			      for (std::list<vpDot2>::iterator it = blob_list.begin(); it != blob_list.end(); ++it) {
			        //            point[i].track(cdMo);
			        //            vpFeatureBuilder::create(pd[i], point[i]);
			        //            pd[i].set_x(pd[i].get_x()+ ds[2*i]) ;
			        //            pd[i].set_y(pd[i].get_y()+ ds[(2*i) + 1]);


			        (*it).track(I);
			        std::stringstream ss;
			        ss << i;
			        vpDisplay::displayText(I, (*it).getCog() + vpImagePoint(20, 20), ss.str(), vpColor::red);
			        vpDisplay::flush(I);
			        vpFeatureBuilder::create(p[i], cam, (*it).getCog());
			        ip[i] = (*it).getCog();

			        // Set the feature Z coordinate from the pose
			        vpColVector cP;
			        point[i].changeFrame(cMo, cP);
			        p[i].set_Z(cP[2]);

			        //            point[i].track(cMo);
			        //            vpFeatureBuilder::create(p[i], point[i]);
			        i++;
			      }
			      //          std::cout << "fatto l'update dei blob.... \n";


			      v = task.computeControlLaw();
//			      Ls = task.getInteractionMatrix();
//			      Ls_pinv = Ls.pseudoInverse();
//
//			      dot_Ls = (Ls_old - Ls)/delta_t;
//			      Ls_old = Ls;
//			      Vc_dot = (Vc_old - Vc)/delta_t;
//			      Vc_old = Vc;
//			      //          std::cout << "calcolato la matrice di interazione.... \n";
//			      s = task.s;
//			      sd = task.sStar;
//			      pthread_mutex_lock(&mutex_ros);
//			      dot_s = Ls*Vc;
//			      pthread_mutex_unlock(&mutex_ros);
//			      es = sd - s;
//			      dot_es = dot_sd - dot_s;

			      //          Bs = Ls.t().pseudoInverse()*M*Ls_pinv;
			      //          Cs = -Bs*dot_Ls*Ls_pinv;
			      //          std::cout << "calcolato un po di roba.... \n";
			      //          Fs = Kp*es + Kv*dot_es;
			      //          Fs = Bs*(dot_dot_sd + a*dot_es)+ Cs*(dot_sd + a*es) + Kp*es + Kv*dot_es;
			      //          std::cout << "calcolato le coppie in image space.... \n";
			      //          Fc = M*Ls_pinv*Fs; // case 1   // params: --kp 5 --kv 15 --a 0.1
			      //          Fc = Ls.t()*Fs; // case 2  // params: --kp 5 --kv 15 --a 0.1
			      //          Fc = (M.inverseByCholesky()*Ls.t()*(Ls*M.inverseByCholesky()*Ls.t()).pseudoInverse())*(Kp*es + Kv*dot_es - dot_Ls*Vc);
//			      Fc = M*Ls_pinv*(Kp*es + Kv*dot_es - dot_Ls*Vc);
			      //          Fc = Kp*es + Kv*dot_es - dot_Ls*Vc;
			      //          Fc[0]=Fc[1]=Fc[2]=0;
			      plotter->plot(0,t_current , v);
			      plotter->plot(1,t_current , task.getError());
			      t_current += delta_t;
//			      iter_plot++;
			      //          std::cout << "calcolato le coppie in camera space.... \n";

			      vpDisplay::displayFrame(I, cdMo, cam, 0.05, vpColor::none, 2);
			      vpDisplay::displayFrame(I, cMo,  cam, 0.05, vpColor::none, 3);
			      vpServoDisplay::display(task, cam, I, vpColor::green, vpColor::red);
			      //          std::cout << "fatto display dei frame.... \n";
			      std::vector<vpDot2> blob_vect{ std::begin(blob_list), std::end(blob_list) };
			      display_trajectory(I,blob_vect);

			      vpDisplay::flush(I);
			      vpMouseButton::vpMouseButtonType button;
			      if (vpDisplay::getClick(I, button, false)) {
			        switch (button) {
			        case vpMouseButton::button1:
			          send_velocities = !send_velocities;
			          break;

			        case vpMouseButton::button3:
			          final_quit = true;
			          v = 0;
			          break;

			        default:
			          break;
			        }
			      }

			      if (!send_velocities) {
			        v = 0;
			      }
			      camera_twist.twist.linear.x = v[0];
			      camera_twist.twist.linear.y = v[1];
			      camera_twist.twist.linear.z = v[2];
			      camera_twist.twist.angular.x = v[3];
			      camera_twist.twist.angular.y = v[4];
			      camera_twist.twist.angular.z = v[5];
			      camera_twist_pub.publish(camera_twist);

			      Flycam.setVelocity(vpRobot::CAMERA_FRAME, v);

//			      camera_wrench.wrench.force.x = Fc[0];
//			      camera_wrench.wrench.force.y = Fc[1];
//			      camera_wrench.wrench.force.z =  Fc[2];
//			      camera_wrench.wrench.torque.x = Fc[3];
//			      camera_wrench.wrench.torque.y = Fc[4];
//			      camera_wrench.wrench.torque.z = Fc[5];
//			      camera_wrench_pub.publish(camera_wrench);
//			      trigger.data = true;
//			      nextStepTrigger_pub.publish(trigger);

			      //          std::cout << "pubblicato i wrench.... \n";


			    }
			    compute = false;
			  }


			}catch (const vpException &e) {
				v = 0;
				delete plotter;
				plotter = NULL;
				exit(1);
			}

			iter++;
		}
		task.kill();
		delete plotter;
		plotter = NULL;
		return EXIT_SUCCESS;

		syncMode.data = false;
		enableSyncMode_pub.publish(syncMode);
	}
	catch (const vpException &e) {
		std::cout << "Test failed with exception: " << e << std::endl;
		return EXIT_FAILURE;
	}
}




