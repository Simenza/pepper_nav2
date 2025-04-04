#ifndef ANIMATED_SPEECH_SUBSCRIBER_HPP
#define ANIMATED_SPEECH_SUBSCRIBER_HPP

/*
* LOCAL includes
*/
#include "subscriber_base.hpp"

/*
* ROS includes
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace naoqi
{
namespace subscriber
{

class AnimatedSpeechSubscriber: public BaseSubscriber<AnimatedSpeechSubscriber>
{
public:
  AnimatedSpeechSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session );
  ~AnimatedSpeechSubscriber(){}

  void reset( rclcpp::Node* node );
  void speech_callback( const std_msgs::msg::String::SharedPtr msg );

private:

  std::string topic_;

  qi::AnyObject p_animated_speech_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_animated_speech_;



}; // class Speech

} // subscriber
}// naoqi
#endif
 