#include <ros/ros.h>

#include <s8_msgs/IRDistances.h>
#include <s8_common_node/Node.h>
#include <ras_arduino_msgs/ADConverter.h>

#define NODE_NAME           "s8_ir_sensors_node"

#define TOPIC_ADC           "/arduino/adc"
#define TOPIC_IR_DISTANCES  "ir_distances"

// Double check but those are likely defaults
#define PARAM_SHORT_TRESHOLD_NEAR_NAME          "short_treshold_near"
#define PARAM_SHORT_TRESHOLD_NEAR_DEFAULT       550
#define PARAM_SHORT_TRESHOLD_FAR_NAME           "short_treshold_far"
#define PARAM_SHORT_TRESHOLD_FAR_DEFAULT        80
#define PARAM_LONG_TRESHOLD_NEAR_NAME           "long_treshold_near"
#define PARAM_LONG_TRESHOLD_NEAR_DEFAULT        500
#define PARAM_LONG_TRESHOLD_FAR_NAME            "long_treshold_far"
#define PARAM_LONG_TRESHOLD_FAR_DEFAULT         90
#define PARAM_TRESHOLD_VALUE_NAME               "treshold_value"
#define PARAM_TRESHOLD_VALUE_DEFAULT            -1.0

class IRSensors : public s8::Node {
    int short_treshold_near;
    int short_treshold_far;
    int long_treshold_near;
    int long_treshold_far;
    double treshold_value;
    ros::Subscriber adc_subscriber;
    ros::Publisher ir_distances_publisher;

public:
    IRSensors() {
        init_params();
        print_params();
        adc_subscriber = nh.subscribe<ras_arduino_msgs::ADConverter>(TOPIC_ADC, 1000, &IRSensors::adc_callback, this);
        ir_distances_publisher = nh.advertise<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1000);
    }

private:
    void adc_callback(const ras_arduino_msgs::ADConverter::ConstPtr & adc) {
        s8_msgs::IRDistances distances;

        distances.front_left = transform_short(adc->ch1);
        distances.front_right = transform_short(adc->ch2);
        distances.front_middle = transform_long(adc->ch3);
        distances.left_front = transform_short(adc->ch4);
        distances.left_back = transform_short(adc->ch5);
        distances.right_front = transform_short(adc->ch6);
        distances.right_back = transform_short(adc->ch7);
        distances.back_middle = transform_long(adc->ch8);

        ir_distances_publisher.publish(distances);
    }

    double transform_short(int adc) {
        if (adc <= short_treshold_far || adc >= short_treshold_near)
            return treshold_value;
        return (1782*pow(adc,-0.9461))/100;
    }

    double transform_long(int adc) {
        if (adc >= long_treshold_far || adc <= long_treshold_near)
            return treshold_value;
        return (23070*pow(adc,-1.295))/100;
    }

    void init_params() {
        add_param(PARAM_SHORT_TRESHOLD_NEAR_NAME, short_treshold_near, PARAM_SHORT_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_SHORT_TRESHOLD_FAR_NAME, short_treshold_far, PARAM_SHORT_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_LONG_TRESHOLD_NEAR_NAME, long_treshold_near, PARAM_LONG_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_LONG_TRESHOLD_FAR_NAME, long_treshold_far, PARAM_LONG_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_TRESHOLD_VALUE_NAME, treshold_value, PARAM_TRESHOLD_VALUE_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    IRSensors ir_sensors;
    ros::spin();

    return 0;
}
