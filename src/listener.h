#ifndef LISTNER
#define LISTNER

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <stdio.h>
#include <thread>

#include "../src/miRobot.h"


namespace gazebo{
	class MiRobot;
	class Listener{
		private:
			std::unique_ptr<ros::NodeHandle> nodo;
			ros::Subscriber subscriber,pid_values,trajectory_values;
			ros::Publisher publisher, joint_pub;
			ros::CallbackQueue cola;
			ros::CallbackQueue cola2;
			ros::CallbackQueue cola3;
			std::thread threadColas;
			//static int contadorConexiones;
			MiRobot * robot;

		public:
			void init(MiRobot *,const std::string);
			void listener(const std_msgs::String::ConstPtr& msg);
			void pidCallback(const trajectory_msgs::JointTrajectory &msg);
			void trajectoryCallback(const trajectory_msgs::JointTrajectory &msg);
			void conexion();
			void desconexion();
			void thread();
	};
}
#endif
