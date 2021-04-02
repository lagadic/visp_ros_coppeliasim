/*
 * Cartesian_impedance.cpp
 *
 *  Created on: Mar 28, 2021
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
	plotter->initGraph(0, 6);
	plotter->setLegend(0, 0, "Vx");
	plotter->setLegend(0, 1, "Vy");
	plotter->setLegend(0, 2, "Vz");
	plotter->setLegend(0, 3, "Wx");
	plotter->setLegend(0, 4, "Wy");
	plotter->setLegend(0, 5, "Wz");
	//  plotter->setLegend(0, 6, "Wz");

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
	// Give some time to the ROS topics to connect with the master
	vpTime::wait(1000);

	StartStopSim.data = true;
	startSimTrigger_pub.publish(StartStopSim);
	syncMode.data = true;
	enableSyncMode_pub.publish(syncMode);
//	vpTime::wait(1000);
//	StartStopSim.data = false;
//	pauseSimTrigger_pub.publish(StartStopSim);
	//  vpTime::wait(1000);
	trigger.data = true;
	//    nextStepTrigger_pub.publish(trigger);

	// -------------------------------------------------------------------------//

	fakeFCI robot;
	robot.connect();

	// Create joint array
	vpColVector q(7,0), qd(7,0),dq(7,0), dq_des(7,0),in(7,0), pos_d(6,0), v_e(6,0), err_old(6,0), d_err(6,0);
	vpMatrix J(6,7), B(7,7), Ja(6,7), dJa(6,7), Ja_old(6,7);

	robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);
	vpTime::wait(1000);

	vpColVector tau_d(7,0), C(7,0);
	std::chrono::time_point<std::chrono::system_clock> timeStart, timeCurrent;
	timeStart = std::chrono::system_clock::now();
	timeCurrent = timeStart;
	double delta_t = 0;
	double t = 0;


	qd = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};


	vpHomogeneousMatrix oMed, oMe;
	oMed.buildFrom(0.3,0.0,0.6,M_PI_2,0,0);

	vpPoseVector pose_err;


	vpTime::wait(1000);
	bool final_quit = true;
	bool send_velocities  = false;


	vpColVector gains(6,0);
	vpMatrix Kp(6,6), Kd(6,6), fJe(6,7), Ta_inv(6,6), Lw(3,3), RR(6,6);
	gains = {500.0, 500.0, 500.0, 700.0, 700.0, 700.0};
	Kp.diag(gains);
	gains = {sqrt(500), sqrt(500), sqrt(500), sqrt(700), sqrt(700),sqrt(700)};
	Kd.diag(2*2*gains);
	Ta_inv.eye();

	double k = 500;
	double d = 4*2*sqrt(500);

	double old_time = simTime;
	unsigned int count = 0.0;
	double dt = 0.0;

	RR.insert((vpMatrix)oMed.getRotationMatrix().t(), 0, 0);
	RR.insert((vpMatrix)oMed.getRotationMatrix().t(), 3, 3);

	std::cout << "entering the main loop.. \n" ;
	while(final_quit){

		if(compute){
			pthread_mutex_lock(&mutex_ros_time);
			compute = false;
			pthread_mutex_unlock(&mutex_ros_time);
			count++;
			dt = simTime - old_time;
			std::cout << count <<  ": time: " << simTime << " [s] ; dt: " <<  dt << " [s] \n";
			old_time = simTime;

			robot.getVelocity(vpRobot::JOINT_STATE,dq);
//			robot.getPosition(vpRobot::JOINT_STATE,q);
			oMe = robot.get_fMe();
			robot.getMass(B);
			robot.getCoriolis(C);
			robot.get_fJe(J);

			pose_err.buildFrom(oMed.inverse()*oMe);
//			pose_err[3] = -pose_err[3];
//			pose_err[4] = -pose_err[4];
//			pose_err[5] = -pose_err[5];
			std::cout << pose_err.t() << std::endl;
			vpThetaUVector tu(pose_err);

			vpColVector u;
			double theta;
			tu.extract(theta, u);
			Lw.eye();
			Lw += 0.5*theta*vpColVector::skew(u);
			Lw += (1-((vpMath::sinc(theta))/(vpMath::sqr(vpMath::sinc(theta*0.5)))))*vpColVector::skew(u)*vpColVector::skew(u);

			Ta_inv.insert(Lw, 3, 3);
//			std::cout << Ta_inv << "\n----\n";

			Ja = Ta_inv*RR*J;
			if(dt != 0.0){
				dJa = (Ja - Ja_old)/dt;
				d_err = ((vpColVector)pose_err - err_old)/dt;
			}else{
				dJa = 0;
				d_err = 0;
			}
			Ja_old = Ja;
			err_old = (vpColVector)pose_err;

//			Js_pinv_B_T = (Ja*B.inverseByCholesky()*Ja.t()).inverseByLU()*Ja*B.inverseByCholesky();

			// computing the NULL space projector for dynamically consistent redundancy resolution
//			tau_damped = -(B*dq)/0.001;
//			Ps.eye();
//			Ps = Ps - Js.t()*((Js*B_inv*Js.t()).pseudoInverse()*Js*B_inv);
//			Ps = I7 - B_inv*Js.t()*(Js*B_inv*Js.t()).inverseByCholesky()*Js;

//			tau_d = B*(k*(qd - q) -d*dq) + C;
//			tau_d = -Ja.t()*20*Ja*dq*0 - Ja.t()*50*(vpColVector)pose_err ;
			tau_d = Ja.t()*(Ja*B.inverseByCholesky()*Ja.t()).pseudoInverse()*(-Kp*(vpColVector)pose_err - Kd*Ja*dq - dJa*dq) + C;


			robot.getPosition(vpRobot::END_EFFECTOR_FRAME,v_e);
			if(simTime > 0.05){
				plotter->plot(0, simTime, (vpColVector)pose_err);
				plotter->plot(1, simTime, tau_d);
			}
			vpTime::wait(10);

			if(!send_velocities){
				tau_d = 0;
			}

			robot.setForceTorque(vpRobot::JOINT_STATE,tau_d);
			nextStepTrigger_pub.publish(trigger);
		}

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
	syncMode.data = false;
	enableSyncMode_pub.publish(syncMode);
	StartStopSim.data = true;
	stopSimTrigger_pub.publish(StartStopSim);

	if (plotter != nullptr) {
		delete plotter;
		plotter = nullptr;
	}

	robot.setRobotState(vpRobot::STATE_STOP);


	return 0;
}



