#include <iostream>
#include "odrive_cpp_ros/odrive_cpp_ros.h"

#define ODRIVE_SERIAL "35722175526216"
int main(int argc, const char* argv[]) {
    std::string ser_nums[1] = {ODRIVE_SERIAL};
    std::string ser_motor_map[2] = {ODRIVE_SERIAL, ODRIVE_SERIAL};

    uint8_t motor_indexes[2] = {0, 1};

    odrive::OdriveDriver odrive_cpp_ros(
            ser_nums,
            1,
            ser_motor_map,
            motor_indexes,
            2
    );

    int result = odrive_cpp_ros.init();
    std::cout << "Init result: " << result << std::endl;

    int current_pos[2] = {0,0};
    result = odrive_cpp_ros.readCurrentMotorPositions(current_pos);
    std::cout << "Read enc result: " << result << " value: " << current_pos[0] << " , " << current_pos[1] << std::endl;

}