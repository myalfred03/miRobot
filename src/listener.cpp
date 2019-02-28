#include "../src/listener.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <gazebo/gazebo.hh>

#include "comandos.h"
namespace gazebo{
	void Listener::init(MiRobot * robot, const std::string topic) {
		this->robot=robot;
		//Listener::contadorConexiones=0;
		if(!ros::isInitialized()){
			int argc=0;
			char **argv=NULL;
			ros::init(argc,argv, "gazebo_client", ros::init_options::NoSigintHandler);
		}

		this->nodo.reset(new ros::NodeHandle("gazebo_client"));

		ros::SubscribeOptions so=ros::SubscribeOptions::create<std_msgs::String>(
				topic,
				1,
				boost::bind(&Listener::listener, this, _1),
				ros::VoidPtr(),
				&this->cola
				);
		this->subscriber=this->nodo->subscribe(so);

		ros::AdvertiseOptions ad=ros::AdvertiseOptions::create<std_msgs::String>(
						topic+"_m",
						1,
						boost::bind(&Listener::conexion,this),
								boost::bind(&Listener::desconexion,this),
						ros::VoidPtr(),
						&this->cola2
						);
		ros::AdvertiseOptions ad2=ros::AdvertiseOptions::create<trajectory_msgs::JointTrajectory>(
						"joint_values_gazebo",
						1,
						boost::bind(&Listener::conexion,this),
								boost::bind(&Listener::desconexion,this),
						ros::VoidPtr(),
						&this->cola3
						);

		this->pid_values = this->nodo->subscribe("/pid_value",10, &Listener::pidCallback, this);

		this->trajectory_values = this->nodo->subscribe("/set_joint_trajectory",10, &Listener::trajectoryCallback, this);

	    this->joint_pub =this->nodo->advertise(ad2);
	    //<trajectory_msgs::JointTrajectory>("joint_values_gazebo", 10);

		this->publisher=this->nodo->advertise(ad);

		this->threadColas=std::thread(std::bind(&Listener::thread, this));
	}
	void Listener::listener(const std_msgs::String::ConstPtr& msg){
		std::string m=msg->data.c_str();
		Comandos::ruta = "";
		Comandos::procesar(m, this->robot);
		//gzdbg<<m<<"\r\n";

	}

    void Listener::pidCallback(const trajectory_msgs::JointTrajectory &msg) //Valores de los limtes de los joints
	{
	  std::cout<<"DATAOK"<< std::endl;
	  this->robot->pidupdate(msg);
	//  limit.data.resize(12);

	    // limit = *msglimit;

	    // Q_EMIT setvaluesSubs();
	}

	void Listener::trajectoryCallback(const trajectory_msgs::JointTrajectory &msg) // toma el ultimo valor
    {             trajectory_msgs::JointTrajectory msgS;
                  int it=0;
                  msgS.points.resize(1);
                  msgS.points[0].positions.resize(6);
                  double jointx, joint;
                  std::vector<double> velvalues(6);
                  std::vector<double> jointvalues(6);
                  std::vector<std::string> jointname(6);
                  it = msg.points.size();

                  for (int m=0; m<it; m++){

                  for (int i = 0; i < 6; i++) {
                    jointvalues[i] = msg.points[m].positions[i] + 1;
                    velvalues  [i] = msg.points[m].velocities[i];
                    jointname[i] = msg.joint_names[i];
                    std::cout<< jointname[5] << std::endl;
                     this->robot->mover(jointname[i],  jointvalues[i]);
                     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//                    msgpub.points[0].positions[i] = msg.points[0].positions[i];

                  //  Comandos::wait(velocities(0));
                   
                  }

              } //for points



     }

	void Listener::conexion(){
		ROS_INFO("Me conecto");
		gzdbg<<"Me conecto"<<"\r\n";
		//Listner::contadorConexiones++;
	}
	void Listener::desconexion(){
		ROS_INFO("Me desconecto");
		//Listner::contadorConexiones--;
	}
	void Listener::thread() {
		static const double timeout=0.01;
		while(this->nodo->ok()){
			this->cola.callAvailable(ros::WallDuration(timeout));
			///if(Listner::contadorConexiones>0){
				std_msgs::String m;
				std::stringstream ms;
				trajectory_msgs::JointTrajectory msg;

				//ms<<"Hay conectados \r\n";
				ms<<this->robot->getEstado()<<"\r\n";
				m.data=ms.str();
				msg = this->robot->getmsg();

				this->publisher.publish(m);
				this->joint_pub.publish(msg);

			//}
		}
	}
}
