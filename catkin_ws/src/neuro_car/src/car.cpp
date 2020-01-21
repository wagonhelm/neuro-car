#include <ros/ros.h>
#include <neuro_car/sigint_handler.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <list>

// note that ROS handles the controlCallback in series, so mutexes are unnecessary

enum EYES { CLOSED, OPEN };
enum FATIGUE { AWAKE, MEDIUM, TIRED };
enum ATTENTION { UNFOCUSED, FOCUSED };

int maxBufferLen = 10;
std::list<int> eyesBuffer, fatigueBuffer, attentionBuffer;
ros::Subscriber svmEyesSub, svmFatigueSub, svmAttentionSub;
ros::Publisher eyesPub, fatiguePub, attentionPub;

void eyesCallback(const std_msgs::Int32::ConstPtr& msg)
{
  eyesBuffer.push_back(static_cast<int>(msg->data));
  if(eyesBuffer.size() > maxBufferLen)
  {
    eyesBuffer.pop_front();
  }

  double avg = 0;
  for(auto iter : eyesBuffer)
  {
    avg += iter;
  }
  avg /= static_cast<double>(eyesBuffer.size());

  std_msgs::Int32 e;
  e.data = static_cast<int>(std::round(avg));
  eyesPub.publish(e);
}

void fatigueCallback(const std_msgs::Int32::ConstPtr& msg)
{
  fatigueBuffer.push_back(static_cast<int>(msg->data));
  if(fatigueBuffer.size() > maxBufferLen)
  {
    fatigueBuffer.pop_front();
  }

  double avg = 0;
  for(auto iter : fatigueBuffer)
  {
    avg += iter;
  }
  avg /= static_cast<double>(fatigueBuffer.size());

  std_msgs::Int32 e;
  e.data = static_cast<int>(std::round(avg));
  fatiguePub.publish(e);
}

void attentionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  attentionBuffer.push_back(static_cast<int>(msg->data));
  if(attentionBuffer.size() > maxBufferLen)
  {
    attentionBuffer.pop_front();
  }

  double avg = 0;
  for(auto iter : attentionBuffer)
  {
    avg += iter;
  }
  avg /= static_cast<double>(attentionBuffer.size());

  std_msgs::Int32 e;
  e.data = static_cast<int>(std::round(avg));
  attentionPub.publish(e);
}

int main(int argc, char** argv) {
  handleInterrupts(argc, argv, "car", true);
  ros::NodeHandle priv_nh("~");

  // load params
  int fps = 0;
  if (!priv_nh.getParam("fps", fps)) {
    ROS_FATAL("FPS parameter could not be parsed or was not given.");
    return 1;
  }

  // log
  ROS_INFO("Running at FPS: %d", fps);

  // publisher
  eyesPub = priv_nh.advertise<std_msgs::Int32>("eyes", 0);
  fatiguePub = priv_nh.advertise<std_msgs::Int32>("fatigue", 0);
  attentionPub = priv_nh.advertise<std_msgs::Int32>("attention", 0);

  // subscriber
  svmEyesSub = priv_nh.subscribe<std_msgs::Int32>("/svm_eyes/detection", 0, eyesCallback);
  svmFatigueSub = priv_nh.subscribe<std_msgs::Int32>("/svm_fatigue/detection", 0, fatigueCallback);
  svmAttentionSub = priv_nh.subscribe<std_msgs::Int32>("/svm_attention/detection", 0, attentionCallback);

  // loop
  ros::Rate rate(fps);
  while(!exitFlag && ros::ok())
  {
    // process control callbacks
    ros::spinOnce();
    rate.sleep();
  }
}