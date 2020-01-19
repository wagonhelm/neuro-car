#include <ros/ros.h>
#include <neuro_car/sigint_handler.h>
#include <std_msgs/Int32.h>
#include <list>

// note that ROS handles the controlCallback in series, so mutexes are unnecessary
enum ACTIONS { STOP, FORWARD };
ACTIONS lastAction = ACTIONS::STOP;
int actionRepeats = 0;

const int MAX_QUEUED_ACTIONS = 2;
const int MIN_ACTION_REPEAT = 5;

std::list<ACTIONS> actionQueue;
ros::Subscriber controlSub;

void controlCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ACTIONS action = static_cast<ACTIONS>(msg->data);

  // check for repeated action (only execute if classification is stable)
  if(action == lastAction)
  {
    actionRepeats++;
    // only execute action if:
    // - action is repeated at least MIN_ACTION_REPEAT times
    // - last action in queue is not the new one (cannot repeat actions in queue)
    // - there are not already too many actions queued
    if(actionRepeats == MIN_ACTION_REPEAT && actionQueue.size() < MAX_QUEUED_ACTIONS && actionQueue.back() != action)
    {
      actionQueue.push_back(action);
      lastAction = action;
      actionRepeats = 0;
    }
  }
  else
  {
    lastAction = action;
    actionRepeats = 0;
  }
}

int main(int argc, char** argv) {
  handleInterrupts(argc, argv, "car", true);
  ros::NodeHandle priv_nh("~");

  // subscriber
  controlSub = priv_nh.subscribe<std_msgs::Int32>("/svm/detection", 0, controlCallback);

  while(!exitFlag && ros::ok())
  {
    // do actions
    if(actionQueue.size())
    {
      // pop action
      auto action = actionQueue.back();
      actionQueue.pop_back();

      // execute
      switch(action)
      {
      case ACTIONS::STOP:
        //DEBUG
        ROS_INFO("STOP");
        break;
      case ACTIONS::FORWARD:
        //DEBUG
        ROS_INFO("FORWARD");
        break;
      }
    }

    // process control callbacks
    ros::spinOnce();
  }
}