#ifndef MIROBOT
#define MIROBOT
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <map>
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#include "../src/listener.h"
namespace gazebo{
   struct Union{
	   physics::JointPtr joint;
	   common::PID pid;
	   common::Time tiempoActual;

	   double velocidad;
	   double anguloFinal;

	   bool moviendo=false;

   };
    class Listener;
	class MiRobot: public ModelPlugin{
		Listener *listener;
		physics::ModelPtr modelo;
		sdf::ElementPtr sdf;
		std::map<std::string,Union> uniones;
		event::ConnectionPtr conexionUpdate;

		std::unique_ptr<ros::NodeHandle> nodo1;
		trajectory_msgs::JointTrajectory msg1;



		std::string estado;
		std::map<std::string,physics::BasePtr> buscar(physics::BasePtr contenedor, const physics::Entity::EntityType &);
		public:
		void Load (physics::ModelPtr _model,sdf::ElementPtr _sdf);
		void OnUpdate(const common::UpdateInfo & _info);
		const std::string& getEstado() const {
			return estado;
		}

		const trajectory_msgs::JointTrajectory& getmsg() const {
			return msg1;
		}
		void mover(std::string laUnion, double valor);
		void parametrizar(std::string laUnion,std::string tipo, double valor);
		void pintar(std::string);
		void cargarUniones();
		void pidupdate(const trajectory_msgs::JointTrajectory &msg);
	};
}
#endif
