
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "controller_manager/controller_manager.h"

#include "odrive_cpp_ros/odrive_cpp_ros.h"
#include "ros/callback_queue.h"


#define RIGHT_ODRIVE_SERIAL "35722175526216" 
#define LEFT_ODRIVE_SERIAL  "35584669134923"


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
        
        jnt_vel_interface.registerHandle(vel_handle_front_left);
        jnt_vel_interface.registerHandle(vel_handle_back_left);
        jnt_vel_interface.registerHandle(vel_handle_front_right);
        jnt_vel_interface.registerHandle(vel_handle_back_right);

        registerInterface(&jnt_vel_interface);

    }

    void updateJointsFromHardware() {
        std::cout << "Updating joints" << std::endl;
    }   

    void writeCommandsToHardware() {
        std::cout << "Writing to hardware" << std::endl;
    }

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        double cmd[4];
        double pos[4];
        double vel[4];
        double eff[4];


};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "tugbot_odrive_interface");
    TugBot robot;
    controller_manager::ControllerManager cm(&robot);

    while(true) {
        robot.updateJointsFromHardware();
        cm.update(ros::Time::now(), ros::Duration(0.1));
        robot.writeCommandsToHardware();
        sleep(1);
    }
    return 0;
}