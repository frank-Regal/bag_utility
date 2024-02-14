#include <string>
#include <chrono>

#include "rosbag/bag.h"
#include <ros/ros.h>
#include "audio_common_msgs/AudioData.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"

class SaveBags {

public:
    // constructors
    SaveBags():
        file_timestamp("new"),
        post_fix_("bag"){}
    
    SaveBags(std::string& PostFix):
        file_timestamp("new"),
        post_fix_(PostFix) {}

    // destructor
    ~SaveBags(){};
    
    // start utility function
    void StartRecordingBag(const std_msgs::Empty::ConstPtr& Msg)
    {
        GetTimeStamp(file_timestamp);
        bag_.open(std::string(file_timestamp + post_fix_ + ".bag"), rosbag::bagmode::Write);
        ROS_INFO_STREAM("Bag opened for recording.");

    }
    
    // stop utility function 
    void StopRecordingBag(const std_msgs::Empty::ConstPtr& Msg)
    {
        bag_.close();
        ROS_INFO_STREAM("Bag closed");
    }

    // write utility function
    void WriteImageMsg(const sensor_msgs::Image::ConstPtr& Msg, const std::string& TopicName)
    {
        try 
        {
            bag_.write(TopicName, Msg->header.stamp, *Msg);
            ROS_INFO_STREAM("writing new message ...");
        } 
        catch (const std::exception& e) 
        {
            ROS_ERROR_STREAM("Error in StartRecordingBag: " << e.what());
        }
        
    }

    void callback(const ros::MessageEvent<sensor_msgs::Image const>& event)
    {
      const std::string& publisher_name = event.getPublisherName();
      ROS_INFO_STREAM(publisher_name);
      const ros::M_string& header = event.getConnectionHeader();
      std::string topic = header.at("topic");
      ROS_INFO_STREAM(topic);
      ros::Time receipt_time = event.getReceiptTime();
    
      const sensor_msgs::ImageConstPtr& msg = event.getMessage();
    }

    // // write utility function
    // void WriteAudioDataMsg(audio_common_msgs::AudioData& Msg)
    // {

    // }


private:
    // class variables
    std::string file_timestamp;
    std::string post_fix_;
    rosbag::Bag bag_;

    // timestamp function
    void GetTimeStamp(std::string& TimeStamp)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S_");
        TimeStamp = ss.str();
    }

};

// ROS Node
int main(int argc, char** argv) {

    // init node
    ros::init(argc, argv, "save_bags");
    ros::NodeHandle nh;

    // init class
    SaveBags save_bags;

    // listen to start topic
    ros::Subscriber start_topic = nh.subscribe<std_msgs::Empty>("/hololens/natural_input_capture/start", 0, &SaveBags::StartRecordingBag, &save_bags);
    ros::Subscriber stop_topic = nh.subscribe<std_msgs::Empty>("/hololens/natural_input_capture/stop", 0, &SaveBags::StopRecordingBag, &save_bags);

    std::string topic_name = "/hololens/LEFT_FRONT/image";
    ros::Subscriber image_topic = nh.subscribe<sensor_msgs::Image>(topic_name, 100, boost::bind(&SaveBags::WriteImageMsg, &save_bags, _1, topic_name));

    ros::spin();
    return 0;
}
