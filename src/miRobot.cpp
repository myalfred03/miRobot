#include "listener.h"
#include "../src/miRobot.h"

#include "../src/listener.h"

namespace gazebo{
	 void MiRobot::Load (physics::ModelPtr _model,sdf::ElementPtr _sdf){
		 this->modelo=_model;
		 this->sdf=_sdf;
		 cargarUniones();
		 //gzdbg<<"Hola Mundo\r\n";
		 listener=new Listener();
		 this->estado="inciado";
		 listener->init(this,"/"+((_sdf->HasElement("topic"))?_sdf->GetElement("topic")->GetValue()->GetAsString():_model->GetName()));

		 this->conexionUpdate=event::Events::ConnectWorldUpdateBegin(boost::bind(&MiRobot::OnUpdate,this,_1));


	 }

	 void MiRobot::OnUpdate(const common::UpdateInfo & _info){
	 	int i=0;
	 	 msg1.points.resize(2);
         msg1.points[0].positions.resize(10);
         msg1.points[1].positions.resize(10);

		 common::Time actual=modelo->GetWorld()->GetSimTime();
		 this->estado=" Elementos \r\n";
		 //Buscar los elementos de unión
		 for(auto it=uniones.begin();it!=uniones.end();it++){
			 Union *_union=&it->second;
			 this->estado+=it->first+": A:"+std::to_string(_union->joint->GetAngle(0).Radian());
			 this->estado+=" V:"+std::to_string(_union->joint->GetVelocity(0));
			 this->estado+="\r\n";
			 if(_union->moviendo){
				 common::Time ticActual=actual-_union->tiempoActual;
				 double error=_union->joint->GetAngle(0).Radian()-_union->anguloFinal;

				 double absError=(error>0)?error:-1*error;
				 double velocidad=0;
				 if(absError>0.001){
					 velocidad=_union->pid.Update(error, ticActual);

				 }else{
					 _union->moviendo=false;
				 }
				 _union->velocidad=velocidad;
			 }

			 _union->joint->SetVelocity(0,_union->velocidad);
			 msg1.points[0].positions[i] = _union->joint->GetAngle(0).Radian();
 			 msg1.points[1].positions[i] = _union->velocidad;
            // joint_pub.publish(msg1) ;
 			 i++;


		 }
		 i=0;
		// joint_pub.pub 
	 }

	 void MiRobot::pidupdate(const trajectory_msgs::JointTrajectory &msg){
             std::string j = msg.joint_names[0];
         Union * _union = &uniones[j];
	 		 _union->pid.SetDGain(msg.points[0].positions[2]);
	 		 _union->pid.SetIGain(msg.points[0].positions[1]);
	 		 _union->pid.SetPGain(msg.points[0].positions[0]);
	 		 _union->pid.SetIMax(msg.points[0].positions[3]);
	 		 _union->pid.SetIMin(msg.points[0].positions[4]);
	 		 _union->moviendo=false;

	 		 
	 }

	 void MiRobot::mover(std::string laUnion, double valor){
		 Union *_union=&uniones[laUnion];
		 if(_union){
			 _union->anguloFinal=valor;
			 _union->tiempoActual=0;
			 _union->moviendo=true;
		 }
	 }
	 void MiRobot::parametrizar(std::string laUnion,std::string tipo, double valor){
		 Union *_union=&uniones[laUnion];
		 if(_union){
			 if(tipo=="V"){
				 _union->velocidad=valor;
				 _union->moviendo=false;
			 }else if(tipo=="VM"){
				 _union->pid.SetCmdMax(valor);
				 _union->pid.SetCmdMin(-1*valor);
			 }

		 }
	 }

	 // void MiRobot::parametrizar(std::string laUnion,std::string tipo, double valor){

		// 		 _union->pid.SetCmdMax(valor);
		// 		 _union->pid.SetCmdMin(-1*valor);
	 // }
	 void MiRobot::pintar(std::string tipo){
		 if(tipo=="uniones"){
			 this->estado=" Elementos \r\n";
			 //Buscar los elementos de unión
			 std::map<std::string,physics::BasePtr> uniones=buscar(this->modelo,physics::Entity::EntityType::JOINT);
			 for(auto it=uniones.begin();it!=uniones.end();it++){
				 this->estado+=it->first+"\r\n";
			 }
		 }
	 }
	 std::map<std::string,physics::BasePtr> MiRobot::buscar(physics::BasePtr contenedor, const physics::Entity::EntityType & t){
		 unsigned int n=contenedor->GetChildCount();
		 std::map<std::string,physics::BasePtr> resultado;
		 for(int i=0;i<n; i++){
			 if(contenedor->GetChild(i)->HasType(t)){
				 resultado[contenedor->GetChild(i)->GetName()]=contenedor->GetChild(i);
			 }
			 std::map<std::string,physics::BasePtr> resultado2=buscar(contenedor->GetChild(i),t);
			 resultado.insert(resultado2.begin(),resultado2.end());
		 }
		 return resultado;
	 }
	 void MiRobot::cargarUniones(){
		 std::map<std::string,physics::BasePtr> uniones=buscar(this->modelo,physics::Entity::EntityType::JOINT);
		 for(auto it=uniones.begin();it!=uniones.end();it++){
			 Union _union;
			 _union.joint=boost::static_pointer_cast<physics::Joint>(it->second);
			 _union.pid.Init(2, 1, 1, 2, 2, _union.joint->GetVelocityLimit(0), -1*_union.joint->GetVelocityLimit(0));
			 _union.tiempoActual=0;
			 this->uniones[it->first]=_union;
		 }
	 }

	GZ_REGISTER_MODEL_PLUGIN(MiRobot);
}
