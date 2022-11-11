#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class DeltaHwInterface : public hardware_interface::RobotHW
{
	public:
		DeltaHwInterface(ros::NodeHandle &nh);
		DeltaHwInterface();
		void init();
		void update(const ros::TimerEvent& e);
		void read();
		void write(ros::Duration elapsed_time);
	private:
		hardware_interface::JointStateInterface joint_state_interface;
		hardware_interface::PositionJointInterface position_joint_interface;
		double cmd[3];
		double pos[3];
		double vel[3];
		double eff[3];
};