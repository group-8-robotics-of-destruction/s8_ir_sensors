#include <ros/ros.h>

#include <s8_msgs/IRDistances.h>
#include <s8_common_node/Node.h>
#include <ras_arduino_msgs/ADConverter.h>

#define NODE_NAME           "s8_ir_sensors_node"

#define TOPIC_ADC           "/arduino/adc"
#define TOPIC_IR_DISTANCES  "ir_distances"

//TODO: Change!!
#define PARAM_SHORT_TRESHOLD_NEAR_NAME          "short_treshold_near"
#define PARAM_SHORT_TRESHOLD_NEAR_DEFAULT       600
#define PARAM_SHORT_TRESHOLD_FAR_NAME           "short_treshold_far"
#define PARAM_SHORT_TRESHOLD_FAR_DEFAULT        100
#define PARAM_LONG_TRESHOLD_NEAR_NAME           "long_treshold_near"
#define PARAM_LONG_TRESHOLD_NEAR_DEFAULT        0
#define PARAM_LONG_TRESHOLD_FAR_NAME            "long_treshold_far"
#define PARAM_LONG_TRESHOLD_FAR_DEFAULT         10000
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
        distances.left_front = transform_short(adc->ch3);
        distances.left_back = transform_short(adc->ch4);
        distances.right_front = transform_short(adc->ch5);
        distances.right_back = transform_short(adc->ch6);
        distances.front_middle = transform_long(adc->ch7);
        distances.back_middle = transform_long(adc->ch8);

        ir_distances_publisher.publish(distances);
    }

    double transform_short(int adc) {
        return transform(adc, short_treshold_near, short_treshold_far, 0.04, 0.4);
    }

    double transform_long(int adc) {
        return transform(adc, long_treshold_near, long_treshold_far, 0.1, 0.8);
    }

    double transform(int adc, double treshold_near, double treshold_far, double min_distance, double max_distance) {
        if(adc <= short_treshold_far || adc >= short_treshold_near) {
            return treshold_value;
        }

        //TODO: Change this transform.
        return (1.0 - ((double)adc / (treshold_near - treshold_far))) * (max_distance - min_distance) + min_distance;
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