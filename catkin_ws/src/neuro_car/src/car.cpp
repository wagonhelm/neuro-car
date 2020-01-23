#include <ros/ros.h>
#include <neuro_car/sigint_handler.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <list>
#include <neuro_car/gpio.h>

// note that ROS handles the controlCallback in series, so mutexes are unnecessary

enum EYES { CLOSED, OPEN };
enum FATIGUE { AWAKE, MEDIUM, TIRED };
enum ATTENTION { UNFOCUSED, FOCUSED };

int maxBufferLen = 10;
int maxClosedLen = 20;
std::list<int> eyesBuffer, fatigueBuffer, attentionBuffer;
std::list<int> closedBuffer={1,1,1,1,1,1,1,1,1,1};
ros::Subscriber svmEyesSub, svmFatigueSub, svmAttentionSub;
ros::Publisher eyesPub, fatiguePub, attentionPub;

int forwardPin = PIN_7;
int pullOverPin = PIN_11;

void eyesCallback(const std_msgs::Int32::ConstPtr& msg)
{
    std::cout << "received " << static_cast<int>(msg->data) << std::endl;
    eyesBuffer.push_back(static_cast<int>(msg->data));
    closedBuffer.push_back(static_cast<int>(msg->data));
    if(eyesBuffer.size() > maxBufferLen)
    {
        eyesBuffer.pop_front();
    }
    if(closedBuffer.size() > maxClosedLen)
    {
        closedBuffer.pop_front();
    }

    double avg = 0;
    double closed_avg = 0;

    for(auto iter : eyesBuffer)
    {
        avg += iter;
    }

    for(auto iter : closedBuffer)
    {
        closed_avg += iter;
    }

    avg /= static_cast<double>(eyesBuffer.size());
    closed_avg /= static_cast<double>(closedBuffer.size());
    std::cout << "closed average " << closed_avg << std::endl;

    std_msgs::Int32 e;
    e.data = static_cast<int>(std::round(avg));
    eyesPub.publish(e);
    std::cout << "publishing " << e.data << std::endl;

    // Maintain buffer of eyes closed

    if (closed_avg <= 0.1) {
        std::cout << "pull over" << std::endl;
        gpio_write(forwardPin, LOW);
        gpio_write(pullOverPin, HIGH);
        ros::Duration(2.0).sleep();
        gpio_write(pullOverPin, LOW);
        closedBuffer.pop_front();
        closedBuffer.pop_front();
        closedBuffer.pop_front();
        closedBuffer.pop_front();
        closedBuffer.pop_front();
        closedBuffer.push_back(1);
        closedBuffer.push_back(1);
        closedBuffer.push_back(1);
        closedBuffer.push_back(1);
        closedBuffer.push_back(1);
    }

    if (e.data == 1) {
        std::cout << "forward" << std::endl;
        gpio_write(forwardPin, HIGH);
        gpio_write(pullOverPin, LOW);
    }

    else {
        std::cout << "stop" << std::endl;
        gpio_write(forwardPin, LOW);
        gpio_write(pullOverPin, LOW);
    }
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

    // Initialize Pins
    gpio_export(forwardPin);
    gpio_export(pullOverPin);
    gpio_set_dir(forwardPin, OUTPUT);
    gpio_set_dir(pullOverPin, OUTPUT);
    gpio_write(forwardPin, LOW);
    gpio_write(pullOverPin, LOW);

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