/**
 * @example    bag_recorder.cpp
 *
 * @author     Pablo Gutiérrez F. (paagutie)
 * @date       11/10/2022
 * 
 * Comment:
 */

#include "bag_recorder/bag_recorder.hpp"

BagRecorder::BagRecorder(rclcpp::Node::SharedPtr node_) : node(node_)
{

  //ROS parameters
  node->declare_parameter("topic_name", std::vector<std::string>{});
  node->declare_parameter("topic_type", std::vector<std::string>{});
  node->declare_parameter("sensor_qos", std::vector<bool>{});

  topic_name = node->get_parameter("topic_name").as_string_array();
  topic_type = node->get_parameter("topic_type").as_string_array();
  sensor_qos = node->get_parameter("sensor_qos").as_bool_array();

  storage_options = rosbag2_storage::StorageOptions();
  storage_options.storage_id = "sqlite3";

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  control_sub = node->create_subscription<std_msgs::msg::Bool>("rosbag/status",
                                                                qos,
                                                                std::bind(&BagRecorder::rosbag_control, this, _1));

    
  //std::any_cast<std::shared_ptr<rosbag_recorder::TopicSubscriber<std_msgs::msg::String>>>(sub[0])->writer_ = writer_;
  //std::any_cast<std::shared_ptr<rosbag_recorder::TopicSubscriber<std_msgs::msg::String>>>(sub[0])->saveROSBag = true;
}

void BagRecorder::setup()
{
  RCLCPP_INFO(node->get_logger(), "Creating subscribers...");
  for (uint8_t i = 0; i < topic_name.size(); i++)
    record->add_subscriber(topic_name[i], topic_type[i], sensor_qos[i]);
  RCLCPP_INFO(node->get_logger(), "Successfully created!");

  RCLCPP_DEBUG(node->get_logger(), "writer1 ref count: %ld", writer_.use_count());
}

BagRecorder::~BagRecorder()
{
    topic_name.clear();
    topic_type.clear();
    sensor_qos.clear();
}


void BagRecorder::rosbag_control(const std_msgs::msg::Bool::SharedPtr msg)
{
  rosbag_record = msg->data;

  if(rosbag_record && !saveROSbagFlag)
  {
    RCLCPP_INFO(node->get_logger(), "The rosbag was started");
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream rosbag_path;
    rosbag_path << "ros2bags/rosbag_" << std::put_time(std::localtime(&now), "%d_%m_%Y_%H_%M_%S");
    storage_options.uri = rosbag_path.str();

    writer_ = std::make_shared<rosbag2_cpp::Writer>();
    bagActive = std::make_shared<bool>(false);
    record = std::make_unique<Recorder>(node, writer_, bagActive);

    writer_->open(storage_options, {"rmw_format", "rmw_format"});
    /*
    writer_->create_topic(
                          {"rosbag/status",
                           "std_msgs/msg/Bool",
                           rmw_get_serialization_format(),
                           ""});
    */

    this->setup();
    saveROSbagFlag = true;
    *bagActive = true;
    
  }
  else if(!rosbag_record && saveROSbagFlag)
  {
    RCLCPP_INFO(node->get_logger(), "The rosbag was stopped");
    *bagActive = false;
    writer_.reset();
    bagActive.reset();
    record->sub.clear();
    record.reset();
    RCLCPP_DEBUG(node->get_logger(), "writer2 ref count: %ld", writer_.use_count());
    saveROSbagFlag = false;

  }

  /*
  //---------- Save this topic in the ROSbag -------------------------
  if(saveROSbagFlag)
  {
    writer_->write(*msg, "rosbag/status", node->now());
  }
  */

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = rclcpp::Node::make_shared("bag_recorder_node");
  BagRecorder bagrecorder(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
