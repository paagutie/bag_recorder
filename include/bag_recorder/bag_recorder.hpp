/**
 * @example    bag_recorder.cpp
 *
 * @author     Pablo Gutiérrez F. (paagutie)
 * @date       11/10/2022
 */

#ifndef BAG_RECORDER_HPP
#define BAG_RECORDER_HPP

#include <chrono>
#include <memory>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "recorder.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BagRecorder
{
    private:

        rosbag2_storage::StorageOptions storage_options;
        std::shared_ptr<rosbag2_cpp::Writer> writer_;
        rclcpp::Node::SharedPtr node;

        std::shared_ptr<bool> bagActive;
        std::vector<std::string> topic_name;
        std::vector<std::string> topic_type;
        std::vector<bool> sensor_qos;

        bool saveROSbagFlag = false;
        bool rosbag_record = false;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_sub;

        //ROS2 Subscribe methods
        void rosbag_control(const std_msgs::msg::Bool::SharedPtr msg);

        std::unique_ptr<Recorder> record;
        void setup();

    public:

        BagRecorder(rclcpp::Node::SharedPtr node_);
        ~BagRecorder();
        


};




#endif // BAG_RECORDER_HPP
