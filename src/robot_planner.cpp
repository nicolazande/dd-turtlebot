//includo le librerie che mi servono
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <time.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <iterator>

double pi = 2*acos(0.0);

class Planner_Class{

public:
  double X, Y, theta, R, V, DIR_X, DIR_Y, indx, nxt, Stot, Scurr, a, Tup;
  //devo clippare Vmax(limite coppia motore) e Vmin(non andare negativo) -> lower dovrebbe essere 0 ma lo metto per 
  const double upper = 0.22, lower = -0.22;
  boost::shared_ptr<nav_msgs::Path const> ptr;
  ros::Time t0, t1;
  
  //costruttore
  Planner_Class(double R, double a, double Tup){
    this->R = R;
    this->nxt = 1;
    this->a = a;
    this->Tup = Tup;
  }
  
  //ricevo path da matlab e lo salvo con un puntatore al path
  void getMsg(){  
    nav_msgs::Path edge;
    ptr = ros::topic::waitForMessage<nav_msgs::Path>("/path",n);
    ROS_INFO_STREAM("path recived has: " << ptr->poses.size() << " poses" << std::endl);
    X = ptr->poses[0].pose.position.x;
    Y = ptr->poses[0].pose.position.y;
    theta = ptr->poses[0].pose.orientation.z;
    this->t0 = ros::Time::now();
    this->t1 = ros::Time::now();
    //trovo lunghezza totale del path
    for (int i = 0; i < ptr->poses.size()-1; i += 1){
       this->Stot += sqrt(pow(ptr->poses[i+1].pose.position.x - ptr->poses[i].pose.position.x, 2) + pow(ptr->poses[i+1].pose.position.y - ptr->poses[i].pose.position.y, 2));
    }
  }
  
  void CTRstart(){
    //pubblico posizione iniziale per inizializzare bene il simulatore
    ros::Publisher sim_pub = n.advertise<nav_msgs::Odometry>("/start", 1);
    nav_msgs::Odometry start_pos;
    start_pos.pose.pose.position.x = X;
    start_pos.pose.pose.position.y = Y;
    start_pos.pose.pose.orientation.z = theta;
    sim_pub.publish(start_pos);
  }
  
  //Trapezoidal Velocity Profile: aggiorno V = V(Scurr)
  double TVP (){
    if (Scurr < Stot * Tup){
       //ROS_INFO_STREAM("UP" <<  std::endl);
       return a;
    }
    else if (Scurr > Stot * Tup && Scurr < (1 - Tup) * Stot){
       //ROS_INFO_STREAM("CONST" << std::endl);
       return 0;
    }
    else{
       //ROS_INFO_STREAM("DOWN" << std::endl);
       return -a;
    }
  }
  
  //pubblico pose corrente con interpolazione
  void StepForward(double lr){
    //aggiorno i tempi
    this->t1 = ros::Time::now();
    double dt = std::max((this->t1 - this->t0).toSec(), 1e-4);
    this->t0 = this->t1;
    //fine-inizio path
    if (indx == 0){
       nxt = 1;
       Scurr = 0;
       V=0;
    }
    else if (indx == ptr->poses.size()-1){
       nxt = -1;
       Scurr = 0;
       V = 0;
    }
    
    //aggiorno direzione e velocita
    double norm = sqrt(pow(ptr->poses[indx+nxt].pose.position.x - X, 2) + pow(ptr->poses[indx+nxt].pose.position.y - Y, 2));
    DIR_X = (norm == 0) ? 0 : (ptr->poses[indx+nxt].pose.position.x - X) / norm;
    DIR_Y = (norm == 0) ? 0 : (ptr->poses[indx+nxt].pose.position.y - Y) / norm;
    double tmp = V + Planner_Class::TVP();
    V = tmp <= lower ? lower : tmp <= upper ? tmp : upper;
    //salvo posizione vechcia
    double Xold = X;
    double Yold = Y;
    
    //aggiorno posizione
    if (DIR_X >= 0)
        X = std::min(X + DIR_X * V * dt, ptr->poses[indx+nxt].pose.position.x);
    else
        X = std::max(X + DIR_X * V * dt, ptr->poses[indx+nxt].pose.position.x);
    if (DIR_Y >= 0)
        Y = std::min(Y + DIR_Y * V * dt, ptr->poses[indx+nxt].pose.position.y);
    else
        Y = std::max(Y + DIR_Y * V * dt, ptr->poses[indx+nxt].pose.position.y);
    theta = ptr->poses[indx].pose.orientation.z;
    Scurr += sqrt(pow(X- Xold, 2) + pow(Y - Yold, 2));
    
    //pubblico
    Planner_Class::publishTransform();
    Planner_Class::publishPose();
    //controllo posizione tra pose del path
    Planner_Class::isBetween();
  }
  
  //aggiorno indice se supero pose del path_msg
  void isBetween(){
    double distance = sqrt(pow(ptr->poses[indx+nxt].pose.position.x - X, 2) + pow(ptr->poses[indx+nxt].pose.position.y - Y, 2));
    if (distance < R) {
       this->indx += nxt;
    }
  }

  //funzione per pubblicare pose
  void publishPose(){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "world";
      pose.pose.position.x = X;
      pose.pose.position.y = Y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      pub.publish(pose);
  }
  //pubblico transform
  void publishTransform(){
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "world";
      odom_trans.child_frame_id = "robot_ref";
      odom_trans.transform.translation.x = X;
      odom_trans.transform.translation.y = Y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
      br.sendTransform(odom_trans);
  }
  
private:
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/plan", 1000);
  tf::TransformBroadcaster br;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "robot_planner");
  //Planner_Class PLN(R, acc(const), %Tup)
  Planner_Class PLN(0.005, 0.004, 0.4);
  
  //ricevo path da matlab
  PLN.getMsg();
  //PLN.CTRstart();
    
  int lr = 10;
  ros::Rate loop_rate(lr);
    
  while(ros::ok){
     ros::spinOnce();
     PLN.StepForward(lr);
     loop_rate.sleep();
  }

  return 0;
}
