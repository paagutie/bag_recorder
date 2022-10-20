/**
 * @example    topic_subscriber.hpp
 *
 * @author     Pablo Gutiérrez F. (paagutie)
 * @date       03/08/2022
 */

#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/writer.hpp>

using std::placeholders::_1;

namespace rosbag_recorder {

template <typename ROS_MSG>
class TopicSubscriber {

private:

    typename rclcpp::Node::SharedPtr node;
    std::string ros_topic;
    std::string ros_topic_type;
    std::shared_ptr<rosbag2_cpp::Writer> writer_;
    std::shared_ptr<bool> saveROSBag;
    typename rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr ros_sub;


    void record_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        if(*saveROSBag)
        {
            rclcpp::Time time_stamp = node->now();
            writer_->write(*msg, ros_topic, ros_topic_type, time_stamp);
        }
    }

public:

    TopicSubscriber(rclcpp::Node::SharedPtr ros_node,
                    const std::string& ros_topic,
                    const std::string& ros_topic_type,
                    bool sensorQoS,
                    std::shared_ptr<rosbag2_cpp::Writer> writer,
                    std::shared_ptr<bool> active)
    : node(ros_node),
    ros_topic(ros_topic),
    ros_topic_type(ros_topic_type),
    writer_(writer),
    saveROSBag(active)
    {
        RCLCPP_INFO(ros_node->get_logger(), "New subscriber - topic_name: %s - topic_type: %s", ros_topic.c_str(), ros_topic_type.c_str());
        // Create a callback group
        rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //Suscribers and publishers
        rclcpp::QoS qos(rclcpp::KeepLast(10));

        // Setup subscriber options
        auto rosSubOptions = rclcpp::SubscriptionOptions();
        rosSubOptions.callback_group = cb_group;

        RCLCPP_DEBUG(ros_node->get_logger(), "writer ref count: %ld", writer_.use_count());

        if(sensorQoS)
            ros_sub = ros_node->create_subscription<ROS_MSG>(ros_topic, rclcpp::SensorDataQoS(), std::bind(&TopicSubscriber::record_callback, this, std::placeholders::_1));
        else
            ros_sub = ros_node->create_subscription<ROS_MSG>(ros_topic, qos, std::bind(&TopicSubscriber::record_callback, this, std::placeholders::_1));
    }

    ~TopicSubscriber(){
        //writer_.reset();
        //saveROSBag.reset();
    }

};

} // namespace rosbag_recorder

#endif // ROSBAG_RECORDER_H
