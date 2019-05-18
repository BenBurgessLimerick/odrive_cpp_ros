
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "controller_manager/controller_manager.h"

#include "odrive_cpp_ros/odrive_cpp_ros.h"
#include "ros/callback_queue.h"


#define RIGHT_ODRIVE_SERIAL "35722175526216" 
#define LEFT_ODRIVE_SERIAL  "35584669134923"
#define ENCODER_CPR 90
#define PI 3.14159265358979
#define TWO_PI (2 * PI)
#define RAD_PER_CPR (TWO_PI / ENCODER_CPR)

class TugBot : public hardware_interface::RobotHW {
    public:
    TugBot() {
        hardware_interface::JointStateHandle state_handle_front_left("front_left", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle state_handle_back_left("back_left", &pos[1], &vel[1], &eff[1]);
        hardware_interface::JointStateHandle state_handle_front_right("front_right", &pos[2], &vel[2], &eff[2]);
        hardware_interface::JointStateHandle state_handle_back_right("back_right", &pos[3], &vel[3], &eff[3]);
        
        jnt_state_interface.registerHandle(state_handle_front_left);
        jnt_state_interface.registerHandle(state_handle_back_left);
        jnt_state_interface.registerHandle(state_handle_front_right);
        jnt_state_interface.registerHandle(state_handle_back_right);

        registerInterface(&jnt_state_interface);

        hardware_interface::JointHandle vel_handle_front_left(jnt_state_interface.getHandle("front_left"), &cmd[0]);
        hardware_interface::JointHandle vel_handle_back_left(jnt_state_interface.getHandle("back_left"), &cmd[1]);
        hardware_interface::JointHandle vel_handle_front_right(jnt_state_interface.getHandle("front_right"), &cmd[2]);
        hardware_interface::JointHandle vel_handle_back_right(jnt_state_interface.getHandle("back_right"), &cmd[3]);
        
        for (int i = 0; i < 4; ++i) {
            cmd[i] = 0.0;
        }

        jnt_vel_interface.registerHandle(vel_handle_front_left);
        jnt_vel_interface.registerHandle(vel_handle_back_left);
        jnt_vel_interface.registerHandle(vel_handle_front_right);
        jnt_vel_interface.registerHandle(vel_handle_back_right);

        registerInterface(&jnt_vel_interface);

        std::string ser_nums[2] = {LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
        std::string set_motor_map[4] = {LEFT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
        uint8_t motor_indexes[4] = {1, 0, 0, 1};
        motor_driver = new odrive::ODriveDriver(ser_nums, 2, set_motor_map, motor_indexes, 4);
        int result = motor_driver->init();
        if (result != 0) {
            std::cout << "Could not connect to odrives!"<< std::endl;
            motors_enabled = false;
        } else {
            motors_enabled = true;
        }
    }

    void updateJointsFromHardware() {
        std::cout << "Updating joints" << std::endl;
        float speed;

        float pos_cpr;
        float pos_delta;
        double angle_delta;
        for (int i = 0; i < 4; ++i) {
            motor_driver->getMotorSpeed(i, speed);
            vel[i] = ((speed * direction_multipliers[i]) / ENCODER_CPR) * 2 * 3.141592;
            
            motor_driver->getPosCPR(i, pos_cpr);
            last_pos_cpr[i] = pos_cpr;
            std::cout << "pos: " << pos_cpr < std::endl;
            if (last_cpr_populated) {
                pos_delta = pos_cpr - last_pos_cpr[i];
                std::cout << "pos delta : " << pos_delta < std::endl;
                
                if (pos_delta < - ENCODER_CPR / 2.0) {
                    //overflow will go from eg 80 to 10. Delta = -70
                    // want to add on ENCODER_CPR to change to delta = 20
                    pos_delta += ENCODER_CPR;
                } else if (pos_delta > ENCODER_CPR / 2.0) {
                    //underflow will go from eg 10 to 80, Delta = 70
                    pos_delta -= ENCODER_CPR;
                }

                angle_delta = pos_delta * RAD_PER_CPR; // angle in radians
                
                pos[i] = pos[i] + angle_delta;

                std::cout << "Motor " << i << " pos " << pos[i] << " delta: " << angle_delta << std::endl;

            }              
            // int motor_pos;
            // motor_driver->readCurrentMotorPosition(i, motor_pos);
            // std::cout << "sad: " << i << " : " << motor_pos << std::endl;
            // motor_driver->getMotorPosition(i, motor_pos);
            //motor_driver->getBusVoltage(i, speed);
        }
        last_cpr_populated = true;
    }   

    void writeCommandsToHardware() {
 
        float target_speeds[4];
        for (int i = 0; i < 4; ++i) {
            if (cmd[i] < -200 || cmd[i] > 200) {
                std::cout << "Motor speed request: " << cmd[i] << " ignored." << std::endl;
                target_speeds[i] = 0.0;
            } else {
                target_speeds[i] = cmd[i] / (2 * 3.141592) * ENCODER_CPR * direction_multipliers[i];
            }
            //std::cout << target_speeds[i] << " ";
        }
        // std::cout << std::endl;
        
        if (motors_enabled) {
            motor_driver->setMotorSpeeds(target_speeds);
        }
    }

    private:
        odrive::ODriveDriver *motor_driver;
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        double cmd[4] = {0, 0, 0, 0};
        double pos[4] = {0, 0, 0, 0};
        double vel[4] = {0, 0, 0, 0};
        double eff[4] = {0, 0, 0, 0};

        float last_pos_cpr[4];
        bool last_cpr_populated = false;

        int direction_multipliers[4] = {-1, -1, 1, 1};
        bool motors_enabled;


};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "tugbot_odrive_interface");
    TugBot robot;
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0);
    while(ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        robot.updateJointsFromHardware();
        cm.update(time, period);
        robot.writeCommandsToHardware();
        rate.sleep();
    }
    return 0;
}
