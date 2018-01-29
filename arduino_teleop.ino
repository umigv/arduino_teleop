#include <ros.h>
#include <std_msgs/UInt16.h>
#include <AltSoftSerial.h>
#include <Sabertooth.h>

class CallbackHandler;

using SubscriberT = ros::Subscriber<std_msgs::UInt16, CallbackHandler>;

class CallbackHandler {
public:
    CallbackHandler(Sabertooth &controller);

    void callback(const std_msgs::UInt16 &message);

private:
    template <unsigned long long N = 8>
    int sign_extend(int to_extend);

    Sabertooth *controller_ptr_;
};

constexpr byte operator""_b(const unsigned long long literal) {
    return static_cast<byte>(literal);
}

constexpr auto BAUD_RATE = 9600l;
constexpr auto ADDRESS = 128_b;
constexpr auto RAMPING_VALUE = 14_b;
constexpr auto TIMEOUT_VALUE_MS = 500;

constexpr auto LEFT_MOTOR = 1_b;
constexpr auto RIGHT_MOTOR = 2_b;

auto serial = AltSoftSerial{ };
auto controller = Sabertooth{ ADDRESS, serial };

auto handler = CallbackHandler{ controller };
ros::NodeHandle handle;
auto subscriber =
    SubscriberT{ "sabertooth", &CallbackHandler::callback, &handler };


void setup() {
    handle.initNode();
    handle.subscribe(subscriber);

    serial.begin(BAUD_RATE);
    controller.autobaud();
    controller.setRamping(RAMPING_VALUE);
    controller.setTimeout(TIMEOUT_VALUE_MS);
}

void loop() {
    handle.spinOnce();
}

CallbackHandler::CallbackHandler(Sabertooth &controller)
    : controller_ptr_{ &controller }
{ }

void CallbackHandler::callback(const std_msgs::UInt16 &message) {
    const auto left = sign_extend(message.data >> 8); // get bits in [8, 16)
    const auto right = sign_extend(message.data & 0xff); // get bits in [0, 8)

    controller_ptr_->motor(LEFT_MOTOR, left);
    controller_ptr_->motor(RIGHT_MOTOR, right);
}

template <unsigned long long N>
int CallbackHandler::sign_extend(const int to_extend) {
    struct Extender {
        int data : N;
    };

    const auto extended = Extender{ to_extend };

    return extended.data;
}
