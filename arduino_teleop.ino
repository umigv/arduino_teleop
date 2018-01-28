#include <ros.h>
#include <Sabertooth.h>

namespace std {

template <bool v, typename T = void>
struct enable_if { };

template <typename T>
struct enable_if<true, T> {
    using type = T;
};

template <bool v, typename T = void>
using enable_if_t = typename enable_if<v, T>::type;

} // namespace std

template <typename To, typename From,
          typename = std::enable_if_t<sizeof(To) == sizeof(From)
                                      and alignof(To) == alignof(From)>>
constexpr To& pun_cast(From &from) noexcept {
    return *reinterpret_cast<To*>(&from);
}

template <typename To, typename From,
          typename = std::enable_if_t<sizeof(To) == sizeof(From)
                                      and alignof(To) == alignof(From)>>
constexpr const To& pun_cast(const From &from) noexcept {
    return *reinterpret_cast<const To*>(&from);
}

ros::NodeHandle nh;

void setup() {
    while (not nh.connected()) { }

    nh.loginfo("setup");
}

void loop() {
    nh.logdebug("loop");
    nh.spinOnce();
    delayMicroseconds(1);
}
