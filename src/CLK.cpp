#include "ros/ros.h"
#include <sstream>
#include "rosgraph_msgs/Clock.h"

//clock publisher -> uso per pubblicare sim_time
int main(int argc, char **argv){

  double counter=0;
  int lr = 100;
  
  ros::init(argc, argv, "CLK");
  ros::NodeHandle n;
  ros::Publisher pub_clock = n.advertise<rosgraph_msgs::Clock>("/clock", 1);
  rosgraph_msgs::Clock clock_msg;
  
  ros::Rate loop_rate(lr);  
  while(ros::ok){
     ros::spinOnce();

     clock_msg.clock.sec = std::floor(counter/lr); //secondi
     clock_msg.clock.nsec = (counter/lr - std::floor(counter/lr)) * 1e9; //nanosecondi
     
     pub_clock.publish(clock_msg);
     
     loop_rate.sleep();
     counter++;
  }

  return 0;
}

