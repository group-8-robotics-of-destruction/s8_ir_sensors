#ifndef __IR_SENSORS_NODE_H
#define __IR_SENSORS_NODE_H

#include <string>

namespace s8 {
	namespace ir_sensors_node {
		const std::string NODE_NAME =           "s8_ir_sensors_node";

		const std::string TOPIC_ADC =           "/arduino/adc";
		const std::string TOPIC_IR_DISTANCES =  "/s8/ir_distances";

		const int AVERAGE_NOT_READY_VALUE = 	-2;
		const int TRESHOLD_VALUE = 				-1;

		bool is_valid_ir_value(double value) {
			return value > 0;
		}
	}
}

#endif
