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

constexpr long SABERTOOTH_BAUD_RATE = 9600;
constexpr byte SABERTOOTH_ADDRESS = 128;

auto serial = AltSoftSerial{ };
auto controller = Sabertooth{ SABERTOOTH_ADDRESS, serial };

auto handler = CallbackHandler{ controller };
ros::NodeHandle handle;
auto subscriber =
    SubscriberT{ "sabertooth", &CallbackHandler::callback, &handler };


void setup() {
    handle.initNode();
    handle.subscribe(subscriber);

    serial.begin(SABERTOOTH_BAUD_RATE);
    controller.autobaud();
    controller.setRamping(27);
    controller.setTimeout(500);
}

void loop() {
    handle.spinOnce();
}

CallbackHandler::CallbackHandler(Sabertooth &controller)
    : controller_ptr_{ &controller }
{ }

void CallbackHandler::callback(const std_msgs::UInt16 &message) {
    const auto right = sign_extend(message.data >> 8); // get bits in [8, 16)
    const auto left = sign_extend(message.data & 0xff); // get bits in [0, 8)

    controller_ptr_->motor(2, -left);
    controller_ptr_->motor(1, -right);
}

template <unsigned long long N>
int CallbackHandler::sign_extend(const int to_extend) {
    struct Extender {
        int data : N;
    };

    const auto extended = Extender{ to_extend };

    return extended.data;
}
