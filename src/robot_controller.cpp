//includo le librerie che mi servono
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

//parametri globali
const double pi = 2*acos(0.0);

class Controller_Class {

public:
  //variabili di classe che aggiorno continuamente
  double X_ref, X_feed, Y_ref, Y_feed, theta, theta_ff, W, W_ff, Vpx_ff, Vpy_ff, Kx, Ky, d, P_dist, R, ictr, num_ref, num_feed;
  ros::Time t0, t1;

  //costruttore --> setto parametri (gains) + inizializzo tempo
  Controller_Class(std::string init, double Kx, double Ky) {
	ROS_INFO_STREAM(init << " with gains: [Kx = " << Kx << " ,Ky = " << Ky << "]" << std::endl);
	this->Kx = Kx;
	this->Ky = Ky;
	this->t0 = ros::Time::now();
	this->t1 = this->t0;
	this->d = 0.16;
	this->P_dist = 0.1;
	this->R = 0.033;
  }
  
  //trasformo quaternion in euler angles [nav_msgs::Odometry]
  double euler_from_quaternion(const nav_msgs::OdometryConstPtr &msg){
        if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 && msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
            return 0;
        else{
            tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
            }
  }
  
  //trasformo quaternion in euler angles [geometry_msgs::PoseStamped]
  double euler_from_quaternion(const geometry_msgs::PoseStampedConstPtr &msg){
        if (msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 && msg->pose.orientation.z == 0 && msg->pose.orientation.w == 0)
            return 0;
        else{
            tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
            }
  }
  
  //metodo che chiamo nel main e lancia callback
  void getData(){
	sub_ref.subscribe(n, "/plan", 1);
        sub_feed.subscribe(n, "/odom", 1);
	sync.reset(new Sync(MySyncPolicy(1), sub_ref, sub_feed));      
	sync->registerCallback(boost::bind(&Controller_Class::callback, this, _1, _2));
  }
  
  //callback eseguito con reference e feedback sincronizzati
  void callback(const geometry_msgs::PoseStampedConstPtr &msg_ref, const nav_msgs::OdometryConstPtr &msg_feed) {
  	//aggiorno i tempi
  	this->t1 = ros::Time::now();
	double dt = std::max((this->t1 - this->t0).toSec(), 1e-1);
	this->t0 = this->t1;
	//calcolo V e W in feed forward derivando posizioni
	this->W_ff = (isnan(msg_ref->pose.orientation.z)) ? this->W_ff : (this->theta_ff - euler_from_quaternion(msg_ref)) / dt;
	this->theta_ff = (isnan(msg_ref->pose.orientation.z)) ? this->theta_ff : euler_from_quaternion(msg_ref);
	this->Vpx_ff = (isnan(msg_ref->pose.position.x)) ? this->Vpx_ff : (X_ref - msg_ref->pose.position.x) / dt - P_dist * sin(this->theta_ff) * this->W_ff;
	this->Vpy_ff = (isnan(msg_ref->pose.position.y)) ? this->Vpy_ff : (Y_ref - msg_ref->pose.position.y) / dt + P_dist * cos(this->theta_ff) * this->W_ff;	
	this->W_ff = (isnan(msg_ref->pose.orientation.z)) ? this->W_ff : (this->theta_ff - euler_from_quaternion(msg_ref)) / dt;
	// parametri dipendenti da reference
	this->X_ref = (isnan(msg_ref->pose.position.x)) ? this->X_ref : msg_ref->pose.position.x;
	this->Y_ref = (isnan(msg_ref->pose.position.y)) ? this->Y_ref : msg_ref->pose.position.y;
	//parametri dipendenti da feedback
	this->X_feed = (std::isnan(msg_feed->pose.pose.position.x)) ? this->X_feed : msg_feed->pose.pose.position.x;
	this->Y_feed = (std::isnan(msg_feed->pose.pose.position.y)) ? this->Y_feed : msg_feed->pose.pose.position.y;
	this->theta = euler_from_quaternion(msg_feed);
	//do cmd_vel solo se chiamato da callback	
	Controller_Class::stepForward();
  }
  
  //pubblico comandi wl e wr a attuatori
  void publishTwist(double Vpx, double Vpy, double W){
  	geometry_msgs::TwistStamped cmdL;
  	geometry_msgs::TwistStamped cmdR;
  	//ricalcolo velocita centro rotazione
  	double Vx = Vpx + P_dist * sin(this->theta) * W;
  	double Vy = Vpy - P_dist * cos(this->theta) * W;
  	double V = Vx * cos(this->theta) + Vy * sin(this->theta);
  	// left wheel
  	cmdL.header.stamp = ros::Time::now();
	cmdL.twist.angular.z = (V - W * this->d / 2) / R;
	// right wheel
  	cmdR.header.stamp = cmdL.header.stamp;
	cmdR.twist.angular.z = (V + W * this->d / 2) / R;
	ROS_INFO_STREAM(std::endl << "CTR:  Wl = " << (V - W * this->d / 2) / R << " / Wr = " << (V + W * this->d / 2) / R << std::endl);
	pubL.publish(cmdL);	
	pubR.publish(cmdR);
  }
  
  //step in avanti (lo chiamo io manualmente)
  void stepForward(){
  	//calcolo posizione punto esterno
	double Xp_ref = this->X_ref + P_dist * cos(this->theta);
	double Xp_feed = this->X_feed + P_dist * cos(this->theta);
	double Yp_ref = this->Y_ref + P_dist * sin(this->theta);
	double Yp_feed = this->Y_feed + P_dist * sin(this->theta);
	//proportional controller + feed_forward (su punto esterno)
	double Vpx = this->Vpx_ff + Kx * (Xp_ref - Xp_feed);
	double Vpy = this->Vpy_ff + Ky * (Yp_ref - Yp_feed);
	//feedback linearization
	this->W = (Vpy * cos(this->theta) - Vpx * sin(this->theta)) / P_dist;
	ROS_INFO_STREAM(std::endl << "CTR_num: " << ictr << " / Vpx = " << Vpx << " / Vpy = " << Vpy << " / W = " << W << " / theta = " << theta << std::endl);
	ictr+=1;
	//pubblico comando velocita
	Controller_Class::publishTwist(Vpx, Vpy, this->W);
  }

private:
  //variabili private
  ros::NodeHandle n;
  ros::Publisher pubL = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel_L", 1);
  ros::Publisher pubR = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel_R", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_ref;
  message_filters::Subscriber<nav_msgs::Odometry> sub_feed;
  //definisco puntatore a sincronizzatore e lo uso per chiamare callback
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
};

int main(int argc, char **argv){
  
  ros::init(argc, argv, "robot_controller");
  Controller_Class CTR("controller initialized", 2, 2);
  
  while(ros::ok){
     CTR.getData();
     ros::spin();  
  }
  
  return 0;
}

