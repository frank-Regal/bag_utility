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
    /**
     * @brief Construct a new Save Bags object
     * 
     * @param Nh - ROS node handle
     */
    SaveBags(
        ros::NodeHandle& Nh):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_("data"),
        output_directory_("~/"),
        is_recording_(false) {
    }

    /**
     * @brief Construct a new Save Bags object
     * 
     * @param Nh               - ROS node handle
     * @param OutputDirectory  - output directory path bags should be saved to
     * @param PostFix          - a string that would be post fixed to the end of bag file name (after the cur timestamp)
     */
    SaveBags(
        ros::NodeHandle& Nh,
        std::string& OutputDirectory,
        std::string& PostFix):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_(PostFix),
        output_directory_(OutputDirectory),
        is_recording_(false) {
    }

    /**
     * @brief Destroy the Save Bags object
     * 
     */
    ~SaveBags()
    {
        bag_.close();
    };

    /**
     * @brief Template subscriber function used to subscribe to specific topic types
     * 
     * @tparam T           - message topic type (i.e. std_msgs/Empty)
     * @param TopicParam   - parameter server param that holds the topic name. this could be a list or a single variable
     * @param QueueSize    - message queue size
     * @param CallbackFunc - funtion you want to call when a message is received
     * @return true        - if a subscriber was successfully created
     * @return false       - if a subscriber could not be created because param on server was not a recognized type
     */
    template<typename T>
    bool SubscribeToTopic(
        const std::string& TopicParam,
        const int& QueueSize, 
        void(SaveBags::*CallbackFunc)(const typename T::ConstPtr&, const std::string&))
    {   
        // init param options
        std::string topic_name;
        std::vector<std::string> topic_name_list;

        // subscribe based on param type
        if(nh_.getParam(TopicParam, topic_name)) {
            CreateSubscriber<T>(topic_name, QueueSize, CallbackFunc);
            return true;
        } 
        else if (nh_.getParam(TopicParam, topic_name_list)) {
            CreateSubscriber<T>(topic_name_list, QueueSize, CallbackFunc);
            return true;
        } 
        else {
            ROS_ERROR("Faild to load '%s' param.", TopicParam.c_str());
            return false;
        }
    }
    
    /**
     * @brief Overloaded template callback function for msgs W/O headers.
     * 
     * This writes a message to the bag open for writing when called from a topic subscriber.
     * 
     * @tparam T         - message topic type (i.e. std_msgs/Empty)
     * @param Msg        - msg heard
     * @param TopicName  - topic name the msg was published on
     */
    template<typename T>
    void WriteToBag(
        const typename T::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        try {
            bag_.write(TopicName, ros::Time::now(), *Msg);
            //ROS_INFO("Bagged msg from: %s", TopicName.c_str());
        } 
        catch (const std::exception& e) {
            ROS_WARN_STREAM("Error in writing [" << TopicName << "] msg to bag: " << e.what());
        }
    }

    /**
     * @brief Overloaded template callback function for msgs WITH headers.
     * 
     * @tparam T         - message topic type (i.e. std_msgs/Empty)
     * @param Msg        - msg heard
     * @param TopicName  - topic name the msg was published on
     */
    template<typename T>
    void WriteToBagStamped(
        const typename T::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        try {
            bag_.write(TopicName, Msg->header.stamp, *Msg);
        } 
        catch (const std::exception& e) {
            ROS_WARN_STREAM("Error in writing [" << TopicName << "] msg to bag: " << e.what());
        }
    }

    /**
     * @brief Start recording to bag trigger function
     * 
     * 1) sets a new bag filename and path, 
     * 2) opens the bag for writing, 
     * 3) writes the message that called this callback to the bag.
     * 
     * @param Msg        - std_msgs/Empty msg heard
     * @param TopicName  - topic name the msg was published on
     */
    void StartRecordingBag(
        const std_msgs::Empty::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        if(is_recording_){ForceCloseBag();}

        GetTimeStamp(file_timestamp_);
        bag_name_ = file_timestamp_ + post_fix_ + ".bag";
        std::string out_path = output_directory_ + "/" + bag_name_;
        bag_.open(out_path, rosbag::bagmode::Write);
        is_recording_ = true;
        
        ROS_INFO("'%s' opened.\n\nWriting ...\n", bag_name_.c_str());

        WriteToBag<std_msgs::Empty>(Msg, TopicName);
    }
    
    void Reset()
    {
        bag_.close();
        is_recording_ = false;
        ROS_INFO("'%s' closed", bag_name_.c_str());
    }


    /**
     * @brief Stop recording to bag trigger function
     * 
     * Should be setup to be called with a std_msgs/Empty msg.
     * 1) writes the message to the bag
     * 2) closes the bag that was open for writing
     * 
     * @param Msg       - std_msgs/Empty msg heard
     * @param TopicName - topic name the msg was published on
     */
    void StopRecordingBag(
        const std_msgs::Empty::ConstPtr& Msg, 
        const std::string& TopicName)
    {
        WriteToBag<std_msgs::Empty>(Msg, TopicName);
        Reset();
    }

    /**
     * @brief Handle calls to start recording new bag, but other bag isn't closed.
     * 
     */
    void ForceCloseBag()
    {
        ROS_ERROR("'start' msg heard but did not hear a 'stop' msg to close bag. Closing bag ...");
        std_msgs::Empty stop_msg;
        std::string stop_topic;
        if(nh_.getParam("stop_capture_topicname_", stop_topic)){
            bag_.write(stop_topic, ros::Time::now(), stop_msg);
            Reset();
        }
        else {
            ROS_ERROR("Faild to load '%s' param.", stop_topic.c_str());
        }
    }


private:
    // init
    std::vector<ros::Subscriber> subscribers_;
    std::string file_timestamp_;
    std::string output_directory_;
    std::string post_fix_;
    std::string bag_name_;
    ros::NodeHandle nh_;
    rosbag::Bag bag_;
    bool is_recording_;

    /**
     * @brief Get current timestamp
     * 
     * @param TimeStamp - outputs the current ROS time
     */
    void GetTimeStamp(std::string& TimeStamp)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S_");
        TimeStamp = ss.str();
    }

    /**
     * @brief Overloaded template function to create a subscriber
     * 
     * This is called when there is just one topic name associated with
     * the param server.
     * 
     * @tparam T           - message topic type (i.e. std_msgs/Empty)
     * @param TopicParam   - parameter server param that holds the topic name. this could be a list or a single variable
     * @param QueueSize    - message queue size
     * @param CallbackFunc - funtion you want to call when a message is received
     */
    template<typename T>
    void CreateSubscriber(
        const std::string& TopicName,
        const int& QueueSize, 
        void(SaveBags::*CallbackFunc)(const typename T::ConstPtr&, const std::string&))
    {
        ros::Subscriber sub = nh_.subscribe<T>(TopicName, QueueSize, boost::bind(CallbackFunc, this, _1, TopicName));
        ROS_INFO("listening to: %s", TopicName.c_str());
        subscribers_.push_back(sub);
    }

    /**
     * @brief Overloaded template function to create a subscriber
     * 
     * This is called when there is a list of topic names
     * associated with the param server.
     * 
     * @tparam T           - message topic type (i.e. std_msgs/Empty)
     * @param TopicParam   - parameter server param that holds the topic name. this could be a list or a single variable
     * @param QueueSize    - message queue size
     * @param CallbackFunc - funtion you want to call when a message is received
     */
    template<typename T>
    void CreateSubscriber(
        const std::vector<std::string>& TopicNameList,
        const int& QueueSize, 
        void(SaveBags::*CallbackFunc)(const typename T::ConstPtr&, const std::string&))
    {
        for(const std::string& topic_name : TopicNameList)
        {
            ros::Subscriber sub = nh_.subscribe<T>(topic_name, QueueSize, boost::bind(CallbackFunc, this, _1, topic_name));
            ROS_INFO("listening to: %s", topic_name.c_str());
            subscribers_.push_back(sub);
        }
    }

};

// ROS Node
int main(int argc, char** argv) {

    // init node
    ros::init(argc, argv, "save_bags");
    ros::NodeHandle nh;

    // load file output directory and postfix
    std::string output_directory, post_fix;
    if(nh.getParam("output_directory", output_directory) && nh.getParam("postfix", post_fix)) {
        ROS_INFO("\n\tSaving bags to: '%s'\n\tFiles with be post fixed with: '%s'", output_directory.c_str(), post_fix.c_str()); 
    } else {
        ROS_ERROR("Faild to load 'output_directory' and 'post_fixed' params.");
    }

    // init class
    SaveBags save_bags(nh, output_directory, post_fix);

    // init topic subscribers
    save_bags.SubscribeToTopic<std_msgs::Empty>("start_capture_topicname_", 0, &SaveBags::StartRecordingBag); // topic starts bag recording
    save_bags.SubscribeToTopic<std_msgs::Empty>("stop_capture_topicname_", 0, &SaveBags::StopRecordingBag);   // topic ends bag recording

    // bag audio, image, and image stop topics
    save_bags.SubscribeToTopic<audio_common_msgs::AudioData>("audio_data_topicname_", 100, &SaveBags::WriteToBag<audio_common_msgs::AudioData>);
    save_bags.SubscribeToTopic<sensor_msgs::Image>("image_topicnames_", 100, &SaveBags::WriteToBagStamped<sensor_msgs::Image>);
    save_bags.SubscribeToTopic<std_msgs::Empty>("image_stop_topicnames_", 0, &SaveBags::WriteToBag<std_msgs::Empty>);

    // ros std
    ros::spin();
    return 0;
}
