#include "animated_speech.hpp"

namespace naoqi
{
namespace subscriber
{

AnimatedSpeechSubscriber::AnimatedSpeechSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session ):
  topic_(topic),
  BaseSubscriber( name, topic, session ),
  p_animated_speech_( session->service("ALAnimatedSpeech").value() )
{}

void AnimatedSpeechSubscriber::reset( rclcpp::Node* node )
{
  sub_animated_speech_ = node->create_subscription<std_msgs::msg::String>(
    topic_,
    10,
    std::bind(&AnimatedSpeechSubscriber::speech_callback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void AnimatedSpeechSubscriber::speech_callback( const std_msgs::msg::String::SharedPtr string_msg )
{
  std::map<std::string, std::string> configuration;
  configuration["bodyLanguageMode"] = "contextual";
  p_animated_speech_.async<void>("say", string_msg->data, configuration);
}

} //subscriber
} //naoqi