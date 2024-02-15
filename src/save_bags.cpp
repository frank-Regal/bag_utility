#include <string>
#include <chrono>
#include <vector>
#include <functional>

#include "rosbag/bag.h"
#include <ros/ros.h>
#include <ros/time.h>
#include "audio_common_msgs/AudioData.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"

class SaveBags {

public:
    // constructors
    SaveBags(ros::NodeHandle& Nh):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_("bag"),
        output_directory_("./")
    {
        if(nh_.getParam("output_directory", output_directory_)) {
            ROS_INFO("Bags will be saved to: %s", output_directory_.c_str()); 
        } else {
            ROS_ERROR("Faild to load 'output_directory' param.");
            ROS_INFO("Bags will be saved to: %s", output_directory_.c_str()); 
        }

        if(nh_.getParam("postfix", post_fix_)) {
            ROS_INFO("Bags post fixed with: %s", post_fix_.c_str()); 
        } else {
            ROS_ERROR("Faild to load 'postfix' param.");
            ROS_INFO("Bags post fixed with: %s", post_fix_.c_str()); 
        }
    }

    // destructor
    ~SaveBags()
    {
        bag_.close();
    };

    // Subscriber function
    template<typename T>
    bool SubscribeToTopic(
        const std::string& TopicParam,
        const int& QueueSize, 
        void(SaveBags::*CallbackFunc)(const typename T::ConstPtr&, const std::string&))
    {   
        // init param options
        std::string topic_name;
        std::vector<std::string> topic_name_list;

        // based on param type, subscribe
        if(nh_.getParam(TopicParam, topic_name))
        {
            CreateSubscriber<T>(topic_name, QueueSize, CallbackFunc);
            return true;
        }
        else if(nh_.getParam(TopicParam, topic_name_list))
        {
            CreateSubscriber<T>(topic_name_list, QueueSize, CallbackFunc);
            return true;
        }
        else
        {
            ROS_ERROR("Faild to load '%s' param.", TopicParam.c_str());
            return false;
        }
    }
    
    // write topics to bag
    template<typename T>
    void WriteToBag(
        const typename T::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        try 
        {
            bag_.write(TopicName, ros::Time::now(), *Msg);
            //ROS_INFO("Bagged msg from: %s", TopicName.c_str());
        } 
        catch (const std::exception& e) 
        {
            ROS_ERROR_STREAM("Error in writing to bag: " << e.what());
        }
    }

    // write topics to bag
    template<typename T>
    void WriteToBagStamped(
        const typename T::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        try 
        {
            bag_.write(TopicName, Msg->header.stamp, *Msg);
            //ROS_INFO("Bagged msg from: %s", TopicName.c_str());
        } 
        catch (const std::exception& e) 
        {
            ROS_ERROR_STREAM("Error in writing to bag: " << e.what());
        }
    }

    // start utility function
    void StartRecordingBag(
        const std_msgs::Empty::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        GetTimeStamp(file_timestamp_);
        std::string bag_name = std::string(file_timestamp_ + post_fix_ + ".bag");

        std::string out_path = output_directory_ + "/" + bag_name;
        ROS_INFO("Saving to: %s", out_path.c_str());

        bag_.open(out_path, rosbag::bagmode::Write);
        ROS_INFO_STREAM("Bag opened for recording.");

        WriteToBag<std_msgs::Empty>(Msg, TopicName);
    }
    
    // stop utility function 
    void StopRecordingBag(
        const std_msgs::Empty::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        WriteToBag<std_msgs::Empty>(Msg, TopicName);
        bag_.close();
        ROS_INFO_STREAM("Bag closed");
    }

private:
    // class variables
    std::string file_timestamp_;
    std::string output_directory_;
    std::string post_fix_;
    rosbag::Bag bag_;
    std::vector<ros::Subscriber> subscribers_;
    ros::NodeHandle nh_;

    // timestamp function
    void GetTimeStamp(std::string& TimeStamp)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S_");
        TimeStamp = ss.str();
    }

    // subscribe to specific topic
    template<typename T>
    void CreateSubscriber(
        const std::string& TopicName,
        const int& QueueSize, 
        void(SaveBags::*CallbackFunc)(const typename T::ConstPtr&, const std::string&))
    {
        ros::Subscriber sub = nh_.subscribe<T>(TopicName, QueueSize, boost::bind(CallbackFunc, this, _1, TopicName));
        ROS_INFO("Subscribed to: %s", TopicName.c_str());
        subscribers_.push_back(sub);
    }

    // subscribe to a list of topics
    template<typename T>
    void CreateSubscriber(
        const std::vector<std::string>& TopicNameList,
        const int& QueueSize, 
        void(SaveBags::*CallbackFunc)(const typename T::ConstPtr&, const std::string&))
    {
        for(const std::string& topic_name : TopicNameList)
        {
            ros::Subscriber sub = nh_.subscribe<T>(topic_name, QueueSize, boost::bind(CallbackFunc, this, _1, topic_name));
            ROS_INFO("Subscribed to: %s", topic_name.c_str());
            subscribers_.push_back(sub);
        }
    }

};

// ROS Node
int main(int argc, char** argv) {

    // init node
    ros::init(argc, argv, "save_bags");
    ros::NodeHandle nh;

    // init class
    SaveBags save_bags(nh);

    // start listening to configured topics
    save_bags.SubscribeToTopic<std_msgs::Empty>("start_capture_topicname_", 0, &SaveBags::StartRecordingBag);
    save_bags.SubscribeToTopic<std_msgs::Empty>("stop_capture_topicname_", 0, &SaveBags::StopRecordingBag);
    save_bags.SubscribeToTopic<std_msgs::Empty>("image_stop_topicnames_", 0, &SaveBags::WriteToBag<std_msgs::Empty>);
    save_bags.SubscribeToTopic<sensor_msgs::Image>("image_topicnames_", 100, &SaveBags::WriteToBagStamped<sensor_msgs::Image>);
    save_bags.SubscribeToTopic<audio_common_msgs::AudioData>("audio_data_topicname_", 100, &SaveBags::WriteToBag<audio_common_msgs::AudioData>);

    // spin
    ros::spin();

    return 0;
}
