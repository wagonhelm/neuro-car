#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <thundersvm/model/svc.h>
#include <neuro_car/sigint_handler.h>
#include <vector>
#include <string>

ros::Publisher detectionPub;
ros::Subscriber vectorSub;
SVC svm;

std::vector<DataSet::node> row;
std::string mode;

void detectionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // fill sparse row
  if(mode == "eyes_alpha")
  {
    for(int i = 0; i < 13; ++i)
    {
      row.push_back(DataSet::node(i, msg->data[i]));
    }
  }
  else
  {
    for(int i = 0; i < 120; ++i)
    {
      row.push_back(DataSet::node(i, msg->data[i]));
    }
  }

  // make one row matrix
  DataSet::node2d matrix;
  matrix.push_back(row);
  row.clear();

  // predict classes
  std::vector<double> estimatedClass = svm.predict(matrix, 1);

  // publish
  std_msgs::Int32 e;
  e.data = estimatedClass[0];
  detectionPub.publish(e);
}

int main(int argc, char** argv) {
  handleInterrupts(argc, argv, "svm", true);
  ros::NodeHandle priv_nh("~");

  // load params
  int fps = 0;
  if (!priv_nh.getParam("fps", fps)) {
    ROS_FATAL("FPS parameter could not be parsed or was not given.");
    return 1;
  }
  std::string modelPath;
  if (!priv_nh.getParam("model", modelPath)) {
    ROS_FATAL("Model path parameter could not be parsed or was not given.");
    return 1;
  }
  if(!priv_nh.getParam("mode", mode)) {
    ROS_FATAL("Mode parameter could not be parsed or was not given.");
    return 1;
  }

  // log
  ROS_INFO("Running at FPS: %d", fps);
  ROS_INFO("Loading model at: %s", modelPath.c_str());

  // load SVM model
  svm.load_from_file(modelPath.c_str());

  // publisher and subscriber
  detectionPub = priv_nh.advertise<std_msgs::Int32>("detection", 0);
  vectorSub = priv_nh.subscribe<std_msgs::Float64MultiArray>("/muse_filtered_data", 0, detectionCallback);

  // loop
  ros::Rate rate(fps);
  while(!exitFlag && ros::ok())
  {
    // wait
    ros::spinOnce();
    rate.sleep();
  }
}