#include <ros/ros.h>
#include <functional>
#include <vector>

#include <s8_ir_sensors/ir_sensors_node.h>

#include <s8_msgs/IRDistances.h>
#include <s8_common_node/Node.h>
#include <ras_arduino_msgs/ADConverter.h>

// Double check but those are likely defaults
#define PARAM_SHORT_TRESHOLD_NEAR_NAME          "short_treshold_near"
#define PARAM_SHORT_TRESHOLD_NEAR_DEFAULT       650
#define PARAM_SHORT_TRESHOLD_FAR_NAME           "short_treshold_far"
#define PARAM_SHORT_TRESHOLD_FAR_DEFAULT        60
#define PARAM_LONG_TRESHOLD_NEAR_NAME           "long_treshold_near"
#define PARAM_LONG_TRESHOLD_NEAR_DEFAULT        500
#define PARAM_LONG_TRESHOLD_FAR_NAME            "long_treshold_far"
#define PARAM_LONG_TRESHOLD_FAR_DEFAULT         80
#define PARAM_NUM_VALUES_FOR_AVERAGE_NAME       "num_values_for_average"
#define PARAM_NUM_VALUES_FOR_AVERAGE_DEFAULT    5

using namespace s8;
using namespace s8::ir_sensors_node;

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

        double median() {
            if(!filled) {
                return -2.0;
            }

            double median;

            sort(values.begin(), values.end());

            if (size  % 2 == 0)
            {
                median = (values[size / 2 - 1] + values[size / 2]) / 2;
            }
            else 
            {
                median = values[size / 2];
            }

            return median;
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
    IRSensors() : treshold_value(TRESHOLD_VALUE) {
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
        // NB left and back middle not used
        distances.front_left = compute_long(front_left_avg, adc->ch1);
        distances.front_right = compute_long(front_right_avg, adc->ch2);
        distances.front_middle = compute_short(front_middle_avg, adc->ch3);
        distances.left_front = compute_ch4(left_front_avg, adc->ch4);
        distances.left_back = compute_short(left_back_avg, adc->ch5);
        distances.right_front = compute_short(right_front_avg, adc->ch6);
        distances.right_back = compute_ch7(right_back_avg, adc->ch7);
        distances.back_middle = compute_short(back_middle_avg, adc->ch8);

	ROS_INFO("CH4 Old: %lf, CH4 New: %lf", transform_short(adc->ch4), transform_ch4(adc->ch4));
        ROS_INFO("CH6 Old: %lf, CH6 New: %lf", transform_short(adc->ch6), transform_ch6(adc->ch6));
        ROS_INFO("CH7 Old: %lf, CH7 New: %lf", transform_short(adc->ch7), transform_ch7(adc->ch7));

        ir_distances_publisher.publish(distances);
    }

    double compute_short(SlidingAverage & average, int adc) {
        return compute(average, adc, short_treshold_near, short_treshold_far, &IRSensors::transform_short);
    }

    double compute_ch4(SlidingAverage & average, int adc) {
        return compute(average, adc, short_treshold_near, short_treshold_far, &IRSensors::transform_ch4);
    }

    double compute_ch6(SlidingAverage & average, int adc) {
	// TO CHANGE
	return compute(average, adc, short_treshold_near, short_treshold_far, &IRSensors::transform_ch6);
    }


    double compute_ch7(SlidingAverage & average, int adc) {
	return compute(average, adc, short_treshold_near, short_treshold_far, &IRSensors::transform_ch7);
    }


    double compute_long(SlidingAverage & average, int adc) {
        return compute(average, adc, long_treshold_near, long_treshold_far, &IRSensors::transform_long);
    }

    double compute(SlidingAverage & average, int adc, int treshold_near, int treshold_far, std::function<double (int)> transform) {
        // if(adc <= treshold_far || adc >= treshold_near) {
        //     //Out of range.
        //     average.reset(); //Should we wait 5 out of range to reset?
        //     return treshold_value;
        // }

        average.add(transform(adc));
        return average.median();
    }

    static double transform_short(int adc) {
        //return 17.8*pow(adc,-0.9461);
       return 38.9*pow(adc,-1.0785);
    }

    static double transform_long(int adc) {
        //return (23070*pow(adc,-1.295))/100;
       return 152.2*pow(adc,-1.1767);
    }

    static double transform_ch4(int adc){
        return 46.9*pow(adc, -1.0785) - 0.01; 
    }

    static double transform_ch6(int adc) {
	return 15.8 * pow(adc, -0.9114);
	//return 9.391 * pow(adc, -0.7653) - 0.035;
    }
    
    static double transform_ch7(int adc) {
	return 4619 * pow(adc, -2.014) + 0.035;
	//return 16210 * pow(adc, -2.275) + 0.043;
    } 

    void init_params() {
        add_param(PARAM_SHORT_TRESHOLD_NEAR_NAME, short_treshold_near, PARAM_SHORT_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_SHORT_TRESHOLD_FAR_NAME, short_treshold_far, PARAM_SHORT_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_LONG_TRESHOLD_NEAR_NAME, long_treshold_near, PARAM_LONG_TRESHOLD_NEAR_DEFAULT);
        add_param(PARAM_LONG_TRESHOLD_FAR_NAME, long_treshold_far, PARAM_LONG_TRESHOLD_FAR_DEFAULT);
        add_param(PARAM_NUM_VALUES_FOR_AVERAGE_NAME, num_values_for_average, PARAM_NUM_VALUES_FOR_AVERAGE_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    IRSensors ir_sensors;
    ros::spin();

    return 0;
}
