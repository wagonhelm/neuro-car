#include <ros/ros.h>
#include <thundersvm/model/svc.h>
#include <neuro_car/sigint_handler.h>

int main(int argc, char** argv) {
  handleInterrupts(argc, argv, "svm", true);
  ros::NodeHandle priv_nh("~");

  // load params
  int fps = 0;
  priv_nh.getParam("fps", fps);
  if (!fps) {
    ROS_FATAL("FPS parameter could not be parsed or was not given.");
    return 1;
  }

  // load SVM model
  ROS_INFO("Running at FPS: %d", fps);
  ros::Rate rate(fps);
  while(!exitFlag)
  {
    
    rate.sleep();
  }
}