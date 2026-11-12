#pragma once
#include <string>
#include <type_traits>

namespace uav_research
{
template<typename T, typename = void> struct MessageVersion
{
    static constexpr unsigned value = 0;
};
template<typename T> struct MessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>>
{
    static constexpr unsigned value = T::MESSAGE_VERSION;
};
// PX4 >=1.16 appends _vN for versioned message types. Older px4_msgs do not
// define MESSAGE_VERSION. ROS remapping remains available for custom bridges.
template<typename T> std::string px4_topic(const std::string &base)
{
    return MessageVersion<T>::value == 0 ? base : base+"_v"+std::to_string(MessageVersion<T>::value);
}
} // namespace uav_research
