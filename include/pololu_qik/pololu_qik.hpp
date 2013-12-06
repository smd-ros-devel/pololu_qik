#ifndef _pololu_qik_hpp
#define _pololu_qik_hpp

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace pololu_qik
{

class pololu_qik
{
public:
	pololu_qik( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv );

	bool open( );
	void close( );
	bool start( );
	void stop( );
private:
	bool is_open( ) const;
	void JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg );
	bool set_ch1( double speed );
	bool set_ch2( double speed );

	std::string port;
	std::string ch1_joint_name;
	std::string ch2_joint_name;
	int fd;

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Subscriber joint_traj_sub;
};

}

#endif /* _pololu_qik_hpp */

