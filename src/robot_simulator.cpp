#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

//definisco pi_greco
const double pi = 2*acos(0.0);

class Simulator_Class {
  
public:
  //variabili di classe che aggiorno continuamente
  double Wl, Wr, W, Vpx, Vpy, X, Y, R, d, theta, counter, P_dist, isim;
  const double Wm_max = 7; //max rotazione motore -> [rad/s] calcolato da vmax datasheet
  ros::Time t0, t1;
  bool LNew, RNew;
  boost::shared_ptr<nav_msgs::Odometry const> ptr;

  //costruttore: inizializzo i timer e condizioni iniziali
  Simulator_Class(std::string init){
     ROS_INFO_STREAM(init << std::endl);
     this->t0 = ros::Time::now();
     this->t1 = ros::Time::now();
     this->R = 0.033; //raggio standard di turtlebot3 burger
     this->d = 0.16; //distanza centro centro ruote
     this->P_dist = 0.1;
     this->LNew = false;
     this->RNew = false;
     this->X = 0;
     this->Y = 0;
     this->theta = 0;
     this->Vpx = 0;
     this->Vpy = 0;
     this->Wr = 0;
     this->Wl = 0;
     this->W = 0;
  }
  
  void setStart(){
     ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/start",n);
     this->X = ptr->pose.pose.position.x;
     this->Y = ptr->pose.pose.position.y;
     this->theta = ptr->pose.pose.orientation.z;
     ROS_INFO_STREAM(std::endl << "CTR recived start position" << std::endl);
  }
  
  //metodo che chiamo nel main e aggiorna  
  void getData(){
     sub_R = n.subscribe("/mot_vel_R", 1, &Simulator_Class::callback_R, this);
     sub_L = n.subscribe("/mot_vel_L", 1, &Simulator_Class::callback_L, this);
  }
  
  //callback eseguito con reference e feedback sincronizzati
  void callback_R(const geometry_msgs::TwistStampedConstPtr &msg_R){
     double tmp = (std::isnan(msg_R->twist.angular.z)) ? this->Wr : msg_R->twist.angular.z;
     this->Wr = tmp <= -Wm_max ? -Wm_max : tmp <= Wm_max ? tmp : Wm_max;
     this->RNew = true;
     ROS_INFO_STREAM("CTR: Wr = " << Wr << " recived" << std::endl);
  }
  
  //callback eseguito con reference e feedback sincronizzati
  void callback_L(const geometry_msgs::TwistStampedConstPtr &msg_L){
     double tmp = (std::isnan(msg_L->twist.angular.z)) ? this->Wl : msg_L->twist.angular.z;
     this->Wl = tmp <= -Wm_max ? -Wm_max : tmp <= Wm_max ? tmp : Wm_max;
     this->LNew = true;
     ROS_INFO_STREAM("CTR: Wl = " << Wl << " recived" << std::endl);
  }
  
  //metodo per avanzare di uno step
  void StepForward(){
     //ricalcolo le velocita
     if (this->RNew && this->LNew){
         ROS_INFO_STREAM(std::endl << "Vel update" << std::endl);
         this->W = this->R * (this->Wr - this->Wl) / this->d;
         double V = this->R * (this->Wr + this->Wl) / 2;        
         this->Vpx = V * cos(this->theta) - P_dist * sin(this->theta) * this->W;
         this->Vpy = V * sin(this->theta) + P_dist * cos(this->theta) * this->W;
     }     
     //aggiorno tempo
     this->t1 = ros::Time::now();
     double dt = std::max((this->t1 - this->t0).toSec(), 1e-5);
     
     ROS_INFO_STREAM(std::endl << "SIM_num: " << isim << " / Vpx = " << Vpx << " / Vpy = " << Vpy << " / W = " << W << " / theta = " << theta << " / dt = " << dt << std::endl);
     isim+=1;
     
     // Rutte-Kutta integration
     this->X = this->X + Vpx * dt;
     this->Y = this->Y + Vpy * dt;
     this->theta = std::fmod(this->theta + W * dt, 2*pi);
     this->t0 = this->t1;
     this->RNew = false;
     this->LNew = false;  
     //pubblico odometry e transformation
     Simulator_Class::publishOdometry();
     Simulator_Class::publishTransform();
  }

  //pubblico l'odometry
  void publishOdometry(){
    nav_msgs::Odometry pos;
    pos.header.stamp = this->t1;
    pos.header.frame_id = "world";
    pos.child_frame_id="robot_feed";
    pos.pose.pose.position.x = this->X;
    pos.pose.pose.position.y = this->Y;
    pos.pose.pose.position.z = 0;
    pos.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->theta);
    pub.publish(pos);
  }

  //pubblico la trasformazione
  void publishTransform() {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->theta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = this->t1;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "robot_feed";
    odom_trans.transform.translation.x = this->X;
    odom_trans.transform.translation.y = this->Y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    br.sendTransform(odom_trans);
  }

private:
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Subscriber sub_L;
    ros::Subscriber sub_R;
    tf::TransformBroadcaster br;
};


int main(int argc, char **argv){
  
  ros::init(argc, argv, "robot_simulator");
  Simulator_Class SIM("simulator initialized");
  
  int lr = 20;
  ros::Rate loop_rate(lr);
    
  while(ros::ok){
     ros::spinOnce();
     SIM.getData();
     SIM.StepForward();
     loop_rate.sleep();
  }
    
  return 0;

}

