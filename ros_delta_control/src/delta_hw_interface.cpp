#include <ros_delta_control/delta_hw_interface.h>


DeltaHwInterface :: DeltaHwInterface(){
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("joint_a", &pos[0], &vel[0], &eff[0]);
    joint_state_interface.registerHandle(state_handle_a);

    // hardware_interface::JointStateHandle state_handle_b("joint_b", &pos[1], &vel[1], &eff[1]);
    // joint_state_interface.registerHandle(state_handle_b);

    // hardware_interface::JointStateHandle state_handle_c("joint_c", &pos[2], &vel[2], &eff[2]);
    // joint_state_interface.registerHandle(state_handle_c);

    registerInterface(&joint_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(joint_state_interface.getHandle("joint_a"), &cmd[0]);
    position_joint_interface.registerHandle(pos_handle_a);

    // hardware_interface::JointHandle pos_handle_b(joint_state_interface.getHandle("joint_b"), &cmd[1]);
    // position_joint_interface.registerHandle(pos_handle_b);

    // hardware_interface::JointHandle pos_handle_c(joint_state_interface.getHandle("joint_c"), &cmd[2]);
    // position_joint_interface.registerHandle(pos_handle_c);

    registerInterface(&position_joint_interface);
}

void DeltaHwInterface::read()
{
}

void DeltaHwInterface::write(ros::Duration elapsed_time)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_hw_interface_node");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2);
    DeltaHwInterface delta_hw_interface(nh);
    delta_hw_interface.init();

    ros::spin();
    return 0;
}