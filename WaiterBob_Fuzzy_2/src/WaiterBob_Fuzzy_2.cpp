#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include "FLIE-master/flie.h"
using namespace std;

//Projeto de Robótica Móvel.
//Este programa controla o WaiterBob com um Fuzzy que segmenta os dados do Lidar em 3 partes.


//****************************************************************//
//			          	VARIÁVEIS GLOBAIS	                      //
//****************************************************************//
tf::Pose pose;
double x=0,y=0,theta, x_ant, y_ant, delta, travelled = 0;
double frente, direita, esquerda;
geometry_msgs::Twist msg;	
bool ori_ok = false, pos_ok = false;
float posdesejada[2], oridesejada, erropos=99, erroorie=99, erropos_1, erropos_2;
float tolerance_orie = 0.05, tolerance_pos = 0.1;
float angulo;
bool is_frente = false;
bool is_esquerda = false;
bool is_direita = false;
int i,j,k;

double range_desvio_far = 1;
double range_desvio_close = 0.7;
double range_desvio_very_close = 0.4;
double range_desvio = range_desvio_far;

double lidar_size, lidar_size_received;
double lidar_ignorar_percentage = 0.3;
double lidar_laterais_percentage = 0.95;

double count_lados;
double count_frente;
double count_ign;

double max_lin_free = 1.2;
double max_lin_obs = 0.6;

double seg_esq, seg_frente, seg_dir, seg_ign, seg_ign_esq, seg_ign_dir;

int size_1, size_2,	size_3,	size_4,	size_5, total;

float MESA[23],DELIVERY[6];
float X1,Y1,X2,Y2;



//****************************************************************//
//			          TRATAMENTO DO FEEDBACK                      //
//****************************************************************//

//Callback da Odometria.
void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	x_ant = x;
	y_ant = y;

	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;

	delta = sqrt(pow(x_ant-x,2)+pow(y_ant-y,2));

	travelled = travelled + delta;

	tf::poseMsgToTF(msg->pose.pose, pose);
  	theta = tf::getYaw(pose.getRotation());
}

//Callback do LIDAR.
void subCallback_lidar(const sensor_msgs::LaserScan::ConstPtr& lidar)
{	
	lidar_size_received = lidar->ranges.size();
	lidar_size = floor(lidar_size_received*(1-lidar_ignorar_percentage));

	size_1 = floor((lidar_size_received - lidar_size)/2);
	size_2 = floor(lidar_size * lidar_laterais_percentage)/2;
	size_4 = size_2;
	size_5 = size_1;
	size_3 = lidar_size_received - size_1 - size_2 - size_4 - size_5;

	total = size_1+size_2+size_3+size_4+size_5;

	direita = 1.5;
	for (i=size_1;i<(size_1 + size_2);i++){
		if(lidar->ranges[i] < direita && lidar->ranges[i] > 0.01){
			direita = lidar->ranges[i];
		}
	}
	
	frente = 1.5;
	for (i=i;i<(size_1 + size_2 + size_3);i++){
		if(lidar->ranges[i] < frente && lidar->ranges[i] > 0.01){
			frente = lidar->ranges[i];
		}
	}

	esquerda = 1.5;
	for (i=i;i<(size_1 + size_2 + size_3 + size_4);i++){
		if(lidar->ranges[i] < esquerda && lidar->ranges[i] > 0.01){
			esquerda = lidar->ranges[i];
		}
	}
	
	range_desvio = erropos < range_desvio_close ? range_desvio_close : range_desvio_far;
	range_desvio = erropos < range_desvio_very_close ? range_desvio_very_close : range_desvio;

	//Verifica os objetos próximos e, se dentro do range escolhido, aciona a flag de desvio.
	is_esquerda = esquerda < range_desvio ? true : false;
	is_frente = frente < range_desvio*1.2 ? true : false;
	is_direita = direita < range_desvio ? true : false;
}




int main(int argc, char **argv)
{

//****************************************************************//
//	   					CONFIGURAÇÃO DO ROS 		              //
//****************************************************************//
ros::init(argc, argv, "WaiterBob_Fuzzy_2");

ros::NodeHandle n;
ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
ros::Subscriber sub = n.subscribe("odom", 1000, subCallback_odom);
ros::Subscriber sub_lidar = n.subscribe("scan", 1000, subCallback_lidar);

ros::Rate loop_rate(10);


//****************************************************************//
//	   			DEFINIÇÃO DAS VARIÁVEIS LINGUÍSTICAS              //
//****************************************************************//

fuzzy_set cat_linear[10];

/*** Erro Linear - begin ***/

		linguisticvariable erro_linear;

		cat_linear[0].setname("QZ");
		cat_linear[0].setrange(0,20);
		cat_linear[0].setval(0,0,0.7);

		cat_linear[1].setname("VERYCLOSE");
		cat_linear[1].setrange(0,20);
		cat_linear[1].setval(0.3,0.7,1.1);

		cat_linear[2].setname("CLOSE");
		cat_linear[2].setrange(0,20);
		cat_linear[2].setval(0.7,1.1,1.5);

		cat_linear[3].setname("FAR");
		cat_linear[3].setrange(0,20);
		cat_linear[3].setval(1.1,1.5,2);

		cat_linear[4].setname("VERYFAR");
		cat_linear[4].setrange(0,20);
		cat_linear[4].setval(1.5,2,20,20);

		erro_linear.setname("erro_linear");
		erro_linear.includecategory(&cat_linear[0]);
		erro_linear.includecategory(&cat_linear[1]);
		erro_linear.includecategory(&cat_linear[2]);
		erro_linear.includecategory(&cat_linear[3]);
		erro_linear.includecategory(&cat_linear[4]);

/*** Erro Linear - end ***/


/*** Velocidade Linear - begin ***/ 

		linguisticvariable vel_linear;

		cat_linear[5].setname("QZ");
		cat_linear[5].setrange(0,1);
		cat_linear[5].setval(0,0,0.2);

		cat_linear[6].setname("VERYSLOW");
		cat_linear[6].setrange(0,1);
		cat_linear[6].setval(0,0.2,0.35);

		cat_linear[7].setname("SLOW");
		cat_linear[7].setrange(0,1);
		cat_linear[7].setval(0.20,0.35,0.70);

		cat_linear[8].setname("FAST");
		cat_linear[8].setrange(0,1);
		cat_linear[8].setval(0.35,0.70,0.90);

		cat_linear[9].setname("VERYFAST");
		cat_linear[9].setrange(0,2);
		cat_linear[9].setval(0.90,1,1);

		vel_linear.setname("vel_linear");
		vel_linear.includecategory(&cat_linear[5]);
		vel_linear.includecategory(&cat_linear[6]);
		vel_linear.includecategory(&cat_linear[7]);
		vel_linear.includecategory(&cat_linear[8]);
		vel_linear.includecategory(&cat_linear[9]);
		
/*** Velocidade Linear - end ***/


fuzzy_set cat_angular[10];

/*** Erro Angular - begin ***/

		linguisticvariable erro_angular;

		cat_angular[0].setname("NB");
		cat_angular[0].setrange(-M_PI,0);
		cat_angular[0].setval(-M_PI,-M_PI, -M_PI/3, -M_PI/10);

		cat_angular[1].setname("NS");
		cat_angular[1].setrange(-M_PI,0);
		cat_angular[1].setval(-M_PI/3,-M_PI/10,0);

		cat_angular[2].setname("QZ");
		cat_angular[2].setrange(-M_PI,+M_PI);
		cat_angular[2].setval(-M_PI/10,0,+M_PI/10);

		cat_angular[3].setname("PS");
		cat_angular[3].setrange(0,+M_PI);
		cat_angular[3].setval(0,M_PI/10,M_PI/3);

		cat_angular[4].setname("PB");
		cat_angular[4].setrange(0,+M_PI);
		cat_angular[4].setval(M_PI/10,M_PI/3, M_PI, M_PI);

		erro_angular.setname("erro_angular");
		erro_angular.includecategory(&cat_angular[0]);
		erro_angular.includecategory(&cat_angular[1]);
		erro_angular.includecategory(&cat_angular[2]);
		erro_angular.includecategory(&cat_angular[3]);
		erro_angular.includecategory(&cat_angular[4]);

/*** Erro Angular - end ***/


/*** Velocidade Angular - begin ***/

		linguisticvariable vel_angular;

		cat_angular[5].setname("NB");
		cat_angular[5].setrange(-2*M_PI,0);
		cat_angular[5].setval(-2*M_PI,-2*M_PI,-M_PI/2,-0.3);

		cat_angular[6].setname("NS");
		cat_angular[6].setrange(-2*M_PI,0);
		cat_angular[6].setval(-M_PI/2,-0.3,0);

		cat_angular[7].setname("QZ");
		cat_angular[7].setrange(-2*M_PI,+2*M_PI);
		cat_angular[7].setval(-0.3,0,0.3);

		cat_angular[8].setname("PS");
		cat_angular[8].setrange(0,+2*M_PI);
		cat_angular[8].setval(0,0.3,M_PI/2);

		cat_angular[9].setname("PB");
		cat_angular[9].setrange(0,+2*M_PI);
		cat_angular[9].setval(0.3,M_PI/2,2*M_PI,2*M_PI);

		vel_angular.setname("vel_angular");
		vel_angular.includecategory(&cat_angular[5]);
		vel_angular.includecategory(&cat_angular[6]);
		vel_angular.includecategory(&cat_angular[7]);
		vel_angular.includecategory(&cat_angular[8]);
		vel_angular.includecategory(&cat_angular[9]);
		
/*** Velocidade Angular - end ***/



/*** Proximidade do obstáculo - begin ***/

		fuzzy_set cat_obstaculo[3];

		cat_obstaculo[0].setname("VERYCLOSE");
		cat_obstaculo[0].setrange(0, 2);
		cat_obstaculo[0].setval(0, 0, 0.4, 0.6);

		cat_obstaculo[1].setname("CLOSE");
		cat_obstaculo[1].setrange(0, 2);
		cat_obstaculo[1].setval(0.5, 0.7, 0.9);

		cat_obstaculo[2].setname("FAR");
		cat_obstaculo[2].setrange(0, 2);
		cat_obstaculo[2].setval(0.8, 1, 2, 2);

		linguisticvariable obstaculo_esquerda;
		obstaculo_esquerda.setname("obstaculo_esquerda");
		obstaculo_esquerda.includecategory(&cat_obstaculo[0]);
		obstaculo_esquerda.includecategory(&cat_obstaculo[1]);
		obstaculo_esquerda.includecategory(&cat_obstaculo[2]);

		linguisticvariable obstaculo_frente;
		obstaculo_frente.setname("obstaculo_frente");
		obstaculo_frente.includecategory(&cat_obstaculo[0]);
		obstaculo_frente.includecategory(&cat_obstaculo[1]);
		obstaculo_frente.includecategory(&cat_obstaculo[2]);

		linguisticvariable obstaculo_direita;
		obstaculo_direita.setname("obstaculo_direita");
		obstaculo_direita.includecategory(&cat_obstaculo[0]);
		obstaculo_direita.includecategory(&cat_obstaculo[1]);
		obstaculo_direita.includecategory(&cat_obstaculo[2]);

/*** Proximidade do obstáculo - end ***/


/*** Velocidade Linear para Desvio de Obstáculo - begin ***/

		fuzzy_set cat_vel_lin_obs[4];

		cat_vel_lin_obs[0].setname("QZ");
		cat_vel_lin_obs[0].setrange(0,1);
		cat_vel_lin_obs[0].setval(0,0,0.3);

		cat_vel_lin_obs[1].setname("VERYSLOW");
		cat_vel_lin_obs[1].setrange(0,1);
		cat_vel_lin_obs[1].setval(0,0.3,0.6);

		cat_vel_lin_obs[2].setname("SLOW");
		cat_vel_lin_obs[2].setrange(0,1);
		cat_vel_lin_obs[2].setval(0.3,0.6,1,1);

		linguisticvariable vel_linear_obs;
		vel_linear_obs.setname("vel_linear_obs");
		vel_linear_obs.includecategory(&cat_vel_lin_obs[0]);
		vel_linear_obs.includecategory(&cat_vel_lin_obs[1]);
		vel_linear_obs.includecategory(&cat_vel_lin_obs[2]);

/*** Velocidade Linear para Desvio de Obstáculo - end ***/


/*** Velocidade Angular para Desvio de Obstáculo - begin ***/

		fuzzy_set cat_vel_ang_obs[7];

		cat_vel_ang_obs[0].setname("NVB");
		cat_vel_ang_obs[0].setrange(-3,0);
		cat_vel_ang_obs[0].setval(-3, -3, -2.5, -2);

		cat_vel_ang_obs[1].setname("NB");
		cat_vel_ang_obs[1].setrange(-3,0);
		cat_vel_ang_obs[1].setval(-2.5, -2, -1);

		cat_vel_ang_obs[2].setname("NS");
		cat_vel_ang_obs[2].setrange(-3,0);
		cat_vel_ang_obs[2].setval(-2, -1, -0.4);

		cat_vel_ang_obs[3].setname("QZ");
		cat_vel_ang_obs[3].setrange(-3,3);
		cat_vel_ang_obs[3].setval(-0.7, 0, 0.7);

		cat_vel_ang_obs[4].setname("PS");
		cat_vel_ang_obs[4].setrange(0,3);
		cat_vel_ang_obs[4].setval(0.4, 1, 2);

		cat_vel_ang_obs[5].setname("PB");
		cat_vel_ang_obs[5].setrange(0,3);
		cat_vel_ang_obs[5].setval(1, 2, 2.5);

		cat_vel_ang_obs[6].setname("PVB");
		cat_vel_ang_obs[6].setrange(0,3);
		cat_vel_ang_obs[6].setval(2, 2.5, 3, 3);

		linguisticvariable vel_angular_obs;
		vel_angular_obs.setname("vel_angular");
		vel_angular_obs.includecategory(&cat_vel_ang_obs[0]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[1]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[2]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[3]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[4]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[5]);
		vel_angular_obs.includecategory(&cat_vel_ang_obs[6]);
		
/*** Velocidade Angular para Desvio de Obstáculo - end ***/



//****************************************************************//
//		      REGRAS FUZZY PARA CONTROLE DE POSIÇÃO	              //
//****************************************************************//

/*** Regras de controle de posição linear - begin ***/

	fuzzy_control fc_linear;
	fc_linear.set_defuzz(CENTROID);
	fc_linear.definevars(erro_linear, erro_angular, vel_linear);

	//Para a velocidade linear há uma não-linearidade na lógica.
	//Se o erro angular for grande, a velocidade linear tem que ser pequena.
	//Isso serve para ele não sair disparado na direção errada porque o erro é grande.
	//Logo, é necessário que o erro angular seja uma variável de entrada.
	
	//Se erro linear é "tanto faz" e erro angular é "negative big", então velocidade linear é "quase zero".
	fc_linear.insert_rule("QZ","NB","QZ");
	fc_linear.insert_rule("VERYCLOSE","NB","QZ");
	fc_linear.insert_rule("CLOSE","NB","QZ");
	fc_linear.insert_rule("FAR","NB","QZ");
	fc_linear.insert_rule("VERYFAR","NB","QZ");

	//Se erro linear é "tanto faz" e erro angular é "positive big", então velocidade linear é "quase zero".
	fc_linear.insert_rule("QZ","PB","QZ");
	fc_linear.insert_rule("VERYCLOSE","PB","QZ");
	fc_linear.insert_rule("CLOSE","PB","QZ");
	fc_linear.insert_rule("FAR","PB","QZ");
	fc_linear.insert_rule("VERYFAR","PB","QZ");

	//Se erro angular é "tanto faz" e erro linear é "quase zero", então velocidade linear é "quase zero".
	fc_linear.insert_rule("QZ","NB","QZ");
	fc_linear.insert_rule("QZ","NS","QZ");
	fc_linear.insert_rule("QZ","QZ","QZ");
	fc_linear.insert_rule("QZ","PS","QZ");
	fc_linear.insert_rule("QZ","PB","QZ");

	//Se erro angular é "tanto faz" e erro linear é "very close", então velocidade linear é "very slow".
	fc_linear.insert_rule("VERYCLOSE","NB","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","NS","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","QZ","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","PS","VERYSLOW");
	fc_linear.insert_rule("VERYCLOSE","PB","VERYSLOW");

	//Se erro angular é "tanto faz" e erro linear é "close", então velocidade linear é "slow".
	fc_linear.insert_rule("CLOSE","NB","SLOW");
	fc_linear.insert_rule("CLOSE","NS","SLOW");
	fc_linear.insert_rule("CLOSE","QZ","SLOW");
	fc_linear.insert_rule("CLOSE","PS","SLOW");
	fc_linear.insert_rule("CLOSE","PB","SLOW");

	//Se erro angular é "tanto faz" e erro linear é "far", então velocidade linear é "fast".
	fc_linear.insert_rule("FAR","NB","FAST");
	fc_linear.insert_rule("FAR","NS","FAST");
	fc_linear.insert_rule("FAR","QZ","FAST");
	fc_linear.insert_rule("FAR","PS","FAST");
	fc_linear.insert_rule("FAR","PB","FAST");

	//Se erro angular é "tanto faz" e erro linear é "very far", então velocidade linear é "very fast".
	fc_linear.insert_rule("VERYFAR","NB","VERYFAST");
	fc_linear.insert_rule("VERYFAR","NS","VERYFAST");
	fc_linear.insert_rule("VERYFAR","QZ","VERYFAST");
	fc_linear.insert_rule("VERYFAR","PS","VERYFAST");
	fc_linear.insert_rule("VERYFAR","PB","VERYFAST");

/*** Regras de controle de posição linear - end ***/

/*** Regras de controle de posição angular - begin ***/

	fuzzy_control fc_angular;
	fc_angular.set_defuzz(CENTROID);
	fc_angular.definevars(erro_angular, vel_angular);
	fc_angular.insert_rule("NB","NB");
	fc_angular.insert_rule("NS","NS");
	fc_angular.insert_rule("QZ","QZ");
	fc_angular.insert_rule("PS","PS");
	fc_angular.insert_rule("PB","PB");
	
/*** Regras de controle de posição angular - end ***/



//****************************************************************//
//		      REGRAS FUZZY PARA DESVIO DE OBSTÁCULO	              //
//****************************************************************//

/*** Regras de controle de desvio linear - begin ***/

	fuzzy_control fc_desvio_linear;
	fc_desvio_linear.set_defuzz(CENTROID);
	fc_desvio_linear.definevars(obstaculo_esquerda, obstaculo_frente, obstaculo_direita, vel_linear_obs);

	fc_desvio_linear.insert_rule("VERYCLOSE","VERYCLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","VERYCLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","VERYCLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","CLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","CLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","CLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","FAR","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","FAR","CLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","FAR","FAR","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYCLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYCLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYCLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("CLOSE","CLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","CLOSE","CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("CLOSE","CLOSE","FAR","VERYSLOW");
	fc_desvio_linear.insert_rule("CLOSE","FAR","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","FAR","CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("CLOSE","FAR","FAR","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","VERYCLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","VERYCLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","VERYCLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("FAR","CLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","CLOSE","CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","CLOSE","FAR","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","FAR","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","FAR","CLOSE","VERYSLOW");//slow
	fc_desvio_linear.insert_rule("FAR","FAR","FAR","SLOW");
	
/*** Regras de controle de desvio linear - end ***/


/*** Regras de controle de desvio angular - begin ***/

	fuzzy_control fc_desvio_angular;
	fc_desvio_angular.set_defuzz(CENTROID);
	fc_desvio_angular.definevars(obstaculo_esquerda, obstaculo_frente, obstaculo_direita, vel_angular_obs);

	fc_desvio_angular.insert_rule("VERYCLOSE","VERYCLOSE","VERYCLOSE","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","VERYCLOSE","CLOSE","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","VERYCLOSE","FAR","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","CLOSE","VERYCLOSE","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","CLOSE","CLOSE","NB");
	fc_desvio_angular.insert_rule("VERYCLOSE","CLOSE","FAR","NB");
	fc_desvio_angular.insert_rule("VERYCLOSE","FAR","VERYCLOSE","QZ");
	fc_desvio_angular.insert_rule("VERYCLOSE","FAR","CLOSE","NS");
	fc_desvio_angular.insert_rule("VERYCLOSE","FAR","FAR","NVB");
	fc_desvio_angular.insert_rule("CLOSE","VERYCLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("CLOSE","VERYCLOSE","CLOSE","NVB");
	fc_desvio_angular.insert_rule("CLOSE","VERYCLOSE","FAR","NVB");
	fc_desvio_angular.insert_rule("CLOSE","CLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("CLOSE","CLOSE","CLOSE","NB");
	fc_desvio_angular.insert_rule("CLOSE","CLOSE","FAR","NS");
	fc_desvio_angular.insert_rule("CLOSE","FAR","VERYCLOSE","PB");
	fc_desvio_angular.insert_rule("CLOSE","FAR","CLOSE","NS");
	fc_desvio_angular.insert_rule("CLOSE","FAR","FAR","NS");
	fc_desvio_angular.insert_rule("FAR","VERYCLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","VERYCLOSE","CLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","VERYCLOSE","FAR","NVB");
	fc_desvio_angular.insert_rule("FAR","CLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","CLOSE","CLOSE","PS");
	fc_desvio_angular.insert_rule("FAR","CLOSE","FAR","NS");
	fc_desvio_angular.insert_rule("FAR","FAR","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","FAR","CLOSE","PS");
	fc_desvio_angular.insert_rule("FAR","FAR","FAR","NS");
	
/*** Regras de controle de desvio angular - end ***/


//****************************************************************//
//			 			  PROMPT SETPOINT	 	                  //
//****************************************************************//

	if (ros::ok())
	{
		//Mesa 1
		MESA[0] = 3.0632;
		MESA[1] = 12.283;
		MESA[2] = 3.0632;
		MESA[3] = 12.283;

		//Mesa 2
		MESA[4] = 5.9132;
		MESA[5] = 12.283;
		MESA[6] = 7.9382;
		MESA[7] = 12.283;

		//Mesa 3
		MESA[8] = 10.8630;
		MESA[9] = 12.2820;
		MESA[10] = 10.8630;
		MESA[11] = 12.2820;

		//Mesa 4
		MESA[12] = 3.0630;
		MESA[13] = 7.6827;
		MESA[14] = 3.0630;
		MESA[15] = 5.4827;

		//Mesa 5
		MESA[16] = 10.463;
		MESA[17] = 7.6824;
		MESA[18] = 10.463;
		MESA[19] = 5.4824;

		//Mesa 6
		MESA[20] = 5.7129;
		MESA[21] = 3.4826;
		MESA[22] = 8.0129;
		MESA[23] = 3.4826;

		//mesas que ele vai:
		DELIVERY[0] = 4;
		DELIVERY[1] = 3;
		DELIVERY[2] = 6;
		DELIVERY[3] = 1;
		
		ros::spinOnce();

//****************************************************************//
//			 			CONTROLE DE POSIÇÃO	 	                  //
//****************************************************************//

	for (j=0;j<7;j++){

	  /*j=0 -> corredor ida
		j=1 -> entrega 1
		j=2 -> entrega 2
		j=3 -> entrega 3
		j=4 -> entrega 4
		j=5 -> corredor volta
		j=6 -> origem*/

		switch (j)
		{
			case 0:
				//Se está partindo da origem, antes vai par a saída do corredor.
				X1 = 0.000;
				Y1 = 2.500;
				X2 = X1;
				Y2 = Y1;
				break;

			case 1:
				//Faz a entrega 1.
				k = (DELIVERY[j-1]-1)*4;

				X1 = MESA[k];
				Y1 = MESA[k+1];
				X2 = MESA[k+2];
				Y2 = MESA[k+3];
				break;

			case 2:
				//Faz a entrega 2.
				k = (DELIVERY[j-1]-1)*4;

				X1 = MESA[k];
				Y1 = MESA[k+1];
				X2 = MESA[k+2];
				Y2 = MESA[k+3];
				break;

			case 3:
				//Faz a entrega 3.
				k = (DELIVERY[j-1]-1)*4;

				X1 = MESA[k];
				Y1 = MESA[k+1];
				X2 = MESA[k+2];
				Y2 = MESA[k+3];
				break;

			case 4:
				//Faz a entrega 4.
				k = (DELIVERY[j-1]-1)*4;

				X1 = MESA[k];
				Y1 = MESA[k+1];
				X2 = MESA[k+2];
				Y2 = MESA[k+3];
				break;

			case 5:
				//Se já passou por todas as entregas, vai para a entrada do corredor.
				X1 = 2.85;
				Y1 = -0.275;
				X2 = X1;
				Y2 = Y1;
				break;

			case 6:
				//Se já voltou para o corredor, segue até a origem.
				X1 = 0;
				Y1 = 0;
				X2 = X1;
				Y2 = Y1;
				break;
		}
		
		pos_ok = false;
		ori_ok = false;

		//APENAS PARA O TESTE DOS OBSTÁCULOS:
				//X1 = 5;
				//Y1 = 0;
				//X2 = X1;
				//Y2 = Y1;
		
		while (!pos_ok) {

			//Calcula o erro de posição até cada um dos dois possíveis pontos de entrega.
			erropos_1 = sqrt(pow(X1-x,2)+pow(Y1-y,2));
			erropos_2 = sqrt(pow(X2-x,2)+pow(Y2-y,2));

			//Após verificar qual é o ponto mais próximo, define suas coordenadas como setpoint.
			if(erropos_1 < erropos_2){
				posdesejada[0] = X1;
				posdesejada[1] = Y1;
			}else{
				posdesejada[0] = X2;
				posdesejada[1] = Y2;
			}

			//Verifica se há objetos próximos.
			//Se sim, aciona o controle de desvio.
			//Se não, aciona o controle de posição.
			if(is_frente || is_esquerda || is_direita){

				msg.angular.z = fc_desvio_angular.make_inference(esquerda, frente, direita);
				msg.linear.x = fc_desvio_linear.make_inference(esquerda, frente, direita)*max_lin_obs;

				erropos =  sqrt(pow(posdesejada[0]-x,2)+pow(posdesejada[1]-y,2));

				pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();

				system("clear");
				printf("Controle Ativo: DESVIO DE OBSTÁCULO.\n");
				printf("Distância total percorrida: %.2fm\n", travelled);
				printf("Erro de orientação: -\n");
				printf("Erro de posição: %.2f m\n",erropos);
				printf("Velocidade angular enviada: %.2f rad/s\n",msg.angular.z);
				printf("Velocidade linear enviada: %.2f m/s\n\n",msg.linear.x);

			}else{
			
				//Calcula o setpoint da orientação.
				angulo = atan2(posdesejada[1]-y,posdesejada[0]-x);
		  	
		  		//Calcula a diferença entre os ângulos.
		  		erroorie = (angulo - theta + 2*M_PI);

		  		//Calcula o MOD360.
		  		erroorie = erroorie/(2*M_PI) - 1;
		  		erroorie = 2*M_PI*erroorie;
		  		if (erroorie < 0){
				  	erroorie = erroorie + 2*M_PI;
		  		}

			  	//Define o sinal da diferença para saber pra qual lado vai.
			  	if (erroorie > M_PI){
				  	erroorie = -(2*M_PI-erroorie);
			  	}

				if (abs(erroorie) > tolerance_orie){
					ori_ok = false;
					msg.angular.z = fc_angular.make_inference(erroorie);
				}else{
					ori_ok = true;
				}
			
				//Controle de posição
				erropos =  sqrt(pow(posdesejada[0]-x,2)+pow(posdesejada[1]-y,2));
				if (erropos > tolerance_pos){
					pos_ok = false;
					msg.linear.x = fc_linear.make_inference(erropos, erroorie)*max_lin_free;
				}else{
					pos_ok = true;
					
					msg.linear.x = 0;
					msg.angular.z = 0;
					pub.publish(msg);
					ros::spinOnce();
					loop_rate.sleep();
				}

				pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();

				system("clear");
				printf("Controle Ativo: POSIÇÃO CARTESIANA.\n");
				printf("Distância total percorrida: %.2fm\n", travelled);
				printf("Erro de orientação: %.2f rad\n",erroorie);
				printf("Erro de posição: %.2f m\n",erropos);
				printf("Velocidade angular enviada: %.2f rad/s\n",msg.angular.z);
				printf("Velocidade linear enviada: %.2f m/s\n\n",msg.linear.x);

			}
		}
	}
	printf("Entregas efetuadas!\nGarçom retornou à origem!\n");
	printf("Distância total percorrida: %.2fm\n", travelled);
  	}

  	return 0;
}
