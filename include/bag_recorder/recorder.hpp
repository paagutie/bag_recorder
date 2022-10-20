/**
 * @example    types.h
 *
 * @author     Pablo Gutiérrez F. (paagutie)
 * @date       11/10/2022
 */

#ifndef ROS_TYPES_H
#define ROS_TYPES_H

//Add here your ros message type
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "uuv_msgs/msg/control.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "uuv_msgs/msg/barometer.hpp"
#include "uuv_msgs/msg/battery.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "uuv_msgs/msg/rov_actuators.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "topic_subscriber.hpp"
#include <any>

enum ROSTypes{
      uuv_msgs_msg_Control,
      uuv_msgs_msg_Barometer,
      uuv_msgs_msg_Battery,
      uuv_msgs_msg_RovActuators,
      dvl_msgs_msg_DVL,
      dvl_msgs_msg_DVLDR,
      std_msgs_msg_String,
      std_msgs_msg_Bool,
      sensor_msgs_msg_Imu,
      sensor_msgs_msg_Joy,
      geometry_msgs_msg_Vector3Stamped,
      Invalid
};

ROSTypes resolveOption(std::string input) {
    if( input == "uuv_msgs/msg/Control" ) return uuv_msgs_msg_Control;
    if( input == "uuv_msgs/msg/Barometer" ) return uuv_msgs_msg_Barometer;
    if( input == "uuv_msgs/msg/Battery" ) return uuv_msgs_msg_Battery;
    if( input == "uuv_msgs/msg/RovActuators" ) return uuv_msgs_msg_RovActuators;
    if( input == "dvl_msgs/msg/DVL" ) return dvl_msgs_msg_DVL;
    if( input == "dvl_msgs/msg/DVLDR" ) return dvl_msgs_msg_DVLDR;
    if( input == "std_msgs/msg/String" ) return std_msgs_msg_String;
    if( input == "std_msgs/msg/Bool" ) return std_msgs_msg_Bool;
    if( input == "sensor_msgs/msg/Imu" ) return sensor_msgs_msg_Imu;
    if( input == "sensor_msgs/msg/Joy" ) return sensor_msgs_msg_Joy;
    if( input == "geometry_msgs/msg/Vector3Stamped" ) return geometry_msgs_msg_Vector3Stamped;

    return Invalid;
 }

class Recorder{

    private:
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rosbag2_cpp::Writer> writer;
        std::shared_ptr<bool> bagActive;
        

        template <typename ROS_MSG>
        void create_subscriber(const std::string &topic_name, const std::string &topic_type, bool sensorQoS)
        {
            //std::shared_ptr<rosbag_recorder::TopicSubscriber<ROS_MSG>> obj;
            auto obj =  std::make_shared<rosbag_recorder::TopicSubscriber<ROS_MSG>>(node, topic_name, topic_type, sensorQoS, writer, bagActive);
            //obj->writer_ = writer;
            //obj->saveROSBag = bagActive;
            sub.push_back(obj);
        }

    public:
        
        std::vector<std::any> sub;

        Recorder(rclcpp::Node::SharedPtr node_, std::shared_ptr<rosbag2_cpp::Writer> writer_, std::shared_ptr<bool> bagActive_):
        node(node_),
        writer(writer_),
        bagActive(bagActive_)
        {

        }

        ~Recorder()
        {
            sub.clear();
        }

        void add_subscriber(const std::string &topic_name, const std::string &topic_type, bool sensorQoS)
        {
            switch( resolveOption(topic_type) )
            {
                case uuv_msgs_msg_Control: {
                    this->create_subscriber<uuv_msgs::msg::Control>(topic_name, topic_type, sensorQoS);
                    break;
                }
                case uuv_msgs_msg_Barometer: {
                    this->create_subscriber<uuv_msgs::msg::Barometer>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case uuv_msgs_msg_Battery: {
                    this->create_subscriber<uuv_msgs::msg::Battery>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case uuv_msgs_msg_RovActuators: {
                    this->create_subscriber<uuv_msgs::msg::RovActuators>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case dvl_msgs_msg_DVL: {
                    this->create_subscriber<dvl_msgs::msg::DVL>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case dvl_msgs_msg_DVLDR: {
                    this->create_subscriber<dvl_msgs::msg::DVLDR>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case std_msgs_msg_String: {
                    this->create_subscriber<std_msgs::msg::String>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case std_msgs_msg_Bool: {
                    this->create_subscriber<std_msgs::msg::Bool>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case sensor_msgs_msg_Imu: {
                    this->create_subscriber<sensor_msgs::msg::Imu>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case sensor_msgs_msg_Joy: {
                    this->create_subscriber<sensor_msgs::msg::Joy>(topic_name, topic_type, sensorQoS);
                    break;
                }

                case geometry_msgs_msg_Vector3Stamped: {
                    this->create_subscriber<geometry_msgs::msg::Vector3Stamped>(topic_name, topic_type, sensorQoS);
                    break;
                }
                // handles Option_Invalid and any other missing/unmapped cases
                default: {
                    RCLCPP_ERROR(node->get_logger(), "Invalid Option. Please add the new message type!");
                    break;
                }
            }
        }


};

#endif // ROS_TYPES_H
