//includo le librerie che mi servono
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <time.h>
#include <ros/console.h>

// modello servo ruota destra
class Right_Wheel_Class {

public:
  geometry_msgs::TwistStamped cmdR;
  
  //costruttore
  Right_Wheel_Class() {
     ROS_INFO_STREAM("right wheel ready" << std::endl);
  }
  
  //metodo che chiamo nel main e lancia callback
  void getData(){
     sub = n.subscribe("/cmd_vel_R", 1, &Right_Wheel_Class::callback, this);
  }
  
  //callback eseguito con reference e feedback sincronizzati
  void callback(const geometry_msgs::TwistStampedConstPtr &msg) {
     this->cmdR = (std::isnan(msg->twist.angular.z)) ? this->cmdR : *msg;
     pub.publish(this->cmdR);
  }
  
private:
  //variabili private
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/mot_vel_R", 1);
};


int main(int argc, char **argv){
  
  ros::init(argc, argv, "right_wheel");
  Right_Wheel_Class RW;
  
  while(ros::ok){
     RW.getData();
     ros::spin();
  }
  
  return 0;
}


