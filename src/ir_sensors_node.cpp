#include <functional>
#include <vector>

#include <ros/ros.h>

#include <s8_msgs/IRDistances.h>
#include <s8_common_node/Node.h>
#include <ras_arduino_msgs/ADConverter.h>

#define NODE_NAME           "s8_ir_sensors_node"

#define TOPIC_ADC           "/arduino/adc"
#define TOPIC_IR_DISTANCES  "ir_distances"

// Double check but those are likely defaults
#define PARAM_SHORT_TRESHOLD_NEAR_NAME          "short_treshold_near"
#define PARAM_SHORT_TRESHOLD_NEAR_DEFAULT       650
#define PARAM_SHORT_TRESHOLD_FAR_NAME           "short_treshold_far"
#define PARAM_SHORT_TRESHOLD_FAR_DEFAULT        60
#define PARAM_LONG_TRESHOLD_NEAR_NAME           "long_treshold_near"
#define PARAM_LONG_TRESHOLD_NEAR_DEFAULT        500
#define PARAM_LONG_TRESHOLD_FAR_NAME            "long_treshold_far"
#define PARAM_LONG_TRESHOLD_FAR_DEFAULT         90
#define PARAM_TRESHOLD_VALUE_NAME               "treshold_value"
#define PARAM_TRESHOLD_VALUE_DEFAULT            -1.0
#define PARAM_NUM_VALUES_FOR_AVERAGE_NAME       "num_values_for_average"
#define PARAM_NUM_VALUES_FOR_AVERAGE_DEFAULT    5

class IRSensors : public s8::Node {
    class SlidingAverage {
        std::vector<double> values;
        int index;
        int size;
        bool filled;

    public:
        SlidingAverage() : size(0) {
            reset();
        }

        SlidingAverage(int size) : size(size) {
            reset();
        }

        void add(double value) {
            values[index] = value;
            index++;

            if(index == size) {
                index = 0;
                filled = true;
            }
        }

        double average() {
            if(!filled) {
                //ROS_WARN("Average called but vector is not full.");
                return -2.0;
            }

            double average = 0.0;

            for(auto v : values) {
                average += v;
            }

            return average / size;
        }

        bool is_full() {
            return filled;
        }

        void reset() {
            values = std::vector<double>(size, 0.0);
            index = 0;
            filled = false;
        }
    };

    int short_treshold_near;
    int short_treshold_far;
    int long_treshold_near;
    int long_treshold_far;
    double treshold_value;
    ros::Subscriber adc_subscriber;
    ros::Publisher ir_distances_publisher;

    int num_values_for_average;
    SlidingAverage front_left_avg;
    SlidingAverage front_right_avg;
    SlidingAverage front_middle_avg;
    SlidingAverage left_front_avg;
    SlidingAverage left_back_avg;
    SlidingAverage right_front_avg;
    SlidingAverage right_back_avg;
    SlidingAverage back_middle_avg;

public:
    IRSensors() {
        init_params();
        print_params();
        adc_subscriber = nh.subscribe<ras_arduino_msgs::ADConverter>(TOPIC_ADC, 1000, &IRSensors::adc_callback, this);
        ir_distances_publisher = nh.advertise<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1000);

        front_left_avg = SlidingAverage(num_values_for_average);
        front_right_avg = SlidingAverage(num_values_for_average);
        front_middle_avg = SlidingAverage(num_values_for_average);
        left_front_avg = SlidingAverage(num_values_for_average);
        left_back_avg = SlidingAverage(num_values_for_average);
        right_front_avg = SlidingAverage(num_values_for_average);
        right_back_avg = SlidingAverage(num_values_for_average);
        back_middle_avg = SlidingAverage(num_values_for_average);
    }

private:
    void adc_callback(const ras_arduino_msgs::ADConverter::ConstPtr & adc) {
        s8_msgs::IRDistances distances;

        distances.front_left = compute_short(front_left_avg, adc->ch1);
        distances.front_right = compute_short(front_right_avg, adc->ch2);
        distances.front_middle = compute_long(front_middle_avg, adc->ch3);
        distances.left_front = compute_short(left_front_avg, adc->ch4);
        distances.left_back = compute_short(left_back_avg, adc->ch5);
        distances.right_front = compute_short(right_front_avg, adc->ch6);
        distances.right_back = compute_short(right_back_avg, adc->ch7);
        distances.back_middle = compute_long(back_middle_avg, adc->ch8);

        ir_distances_publisher.publish(distances);
    }

    double compute_short(SlidingAverage & average, int adc) {
        return compute(average, adc, short_treshold_near, short_treshold_far, &IRSensors::transform_short);
    }

    double compute_long(SlidingAverage & average, int adc) {
        return compute(average, adc, long_treshold_near, long_treshold_far, &IRSensors::transform_long);
    }

    double compute(SlidingAverage & average, int adc, int treshold_near, int treshold_far, std::function<double (int)> transform) {
        if(adc <= treshold_far || adc >= treshold_near) {
            //Out of range.
            return treshold_value;
        }

        average.add(transform(adc));
        return average.average();
    }

    static double transform_short(int adc) {
        return 39*pow(adc,-1.078);
    }

    static double transform_long(int adc) {
        return (23070*pow(adc,-1.295))/100;
    }

    void init_params() {
        add_param(PARAM_SHORT_TRESHOLD_NEAR_NAME, short_treshold_near, PARAM_SHORT_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_SHORT_TRESHOLD_FAR_NAME, short_treshold_far, PARAM_SHORT_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_LONG_TRESHOLD_NEAR_NAME, long_treshold_near, PARAM_LONG_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_LONG_TRESHOLD_FAR_NAME, long_treshold_far, PARAM_LONG_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_TRESHOLD_VALUE_NAME, treshold_value, PARAM_TRESHOLD_VALUE_DEFAULT);
        add_param(PARAM_NUM_VALUES_FOR_AVERAGE_NAME, num_values_for_average, PARAM_NUM_VALUES_FOR_AVERAGE_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    IRSensors ir_sensors;
    ros::spin();

    return 0;
}
