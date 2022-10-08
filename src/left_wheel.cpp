//includo le librerie che mi servono
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <time.h>
#include <ros/console.h>

// modello servo ruota sinistra
class Left_Wheel_Class {

public:
  geometry_msgs::TwistStamped cmdL;
  
  //costruttore
  Left_Wheel_Class() {
	ROS_INFO_STREAM("left wheel ready" << std::endl);
  }
  
  //metodo che chiamo nel main e lancia callback
  void getData(){
        sub = n.subscribe("/cmd_vel_L", 1, &Left_Wheel_Class::callback, this);
  }
  
  //callback eseguito con reference e feedback sincronizzati
  void callback(const geometry_msgs::TwistStampedConstPtr &msg) {
  	this->cmdL = (std::isnan(msg->twist.angular.z)) ? this->cmdL : *msg;
  	pub.publish(this->cmdL);
  }

private:
  //variabili private
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/mot_vel_L", 1);
};


int main(int argc, char **argv){
  
  ros::init(argc, argv, "left_wheel");
  Left_Wheel_Class LW;
  
  while(ros::ok){
     LW.getData();
     ros::spin();
  }
  
  return 0;
}


