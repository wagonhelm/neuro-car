#include <ros/ros.h>
#include <neuro_car/sigint_handler.h>

int main(int argc, char** argv) {
  handleInterrupts(argc, argv, "car", true);
  ros::NodeHandle priv_nh("~");

  while(!exitFlag)
  {
    
    ros::spinOnce();
  }
}