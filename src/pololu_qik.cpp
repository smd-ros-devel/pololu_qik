#include "pololu_qik/pololu_qik.hpp"

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>

namespace pololu_qik
{

pololu_qik::pololu_qik( ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv ) :
	port( "" ),
	ch1_joint_name( "1" ),
	ch2_joint_name( "2" ),
	fd( -1 ),
	nh( _nh ),
	nh_priv( _nh_priv )
{
	ROS_INFO( "Initializing" );
	nh_priv.param( "port", port, (std::string)"/dev/ttyACM0" );
	nh_priv.param( "ch1_joint_name", ch1_joint_name, (std::string)"1" );
	nh_priv.param( "ch2_joint_name", ch2_joint_name, (std::string)"2" );
}

bool pololu_qik::open( )
{
	struct termios fd_options;
	unsigned char baud_autodetect = 0xAA;

	if( is_open( ) )
	{
		ROS_INFO( "Port is already open - Closing to re-open" );
		close( );
	}

	fd = ::open( port.c_str( ), O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
	{
		ROS_FATAL( "Failed to open port: %s", strerror( errno ) );
		return false;
	}

	if( 0 > fcntl( fd, F_SETFL, 0 ) )
	{
		ROS_FATAL( "Failed to set port descriptor: %s", strerror( errno ) );
		return false;
	}

	if( 0 > tcgetattr( fd, &fd_options ) )
	{
		ROS_FATAL( "Failed to fetch port attributes: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetispeed( &fd_options, B9600 ) )
	{
		ROS_FATAL( "Failed to set input baud: %s", strerror( errno ) );
		return false;
	}
	if( 0 > cfsetospeed( &fd_options, B9600 ) )
	{
		ROS_FATAL( "Failed to set output baud: %s", strerror( errno ) );
		return false;
	}
	if( 0 > tcsetattr( fd, TCSANOW, &fd_options ) )
	{
		ROS_FATAL( "Failed to set port attributes: %s", strerror( errno ) );
		return false;
	}

	if( 0 > write( fd, &baud_autodetect, 1 ) )
	{
		ROS_FATAL( "Failed to initialize device: %s", strerror( errno ) );
		return false;
	}

	return true;
}

void pololu_qik::close( )
{
	ROS_INFO( "Closing Port" );

	::close( fd );
}

bool pololu_qik::start( )
{
	if( !is_open( ) && !open( ) )
		return false;

	ROS_INFO( "Starting" );

	if( !joint_traj_sub )
		joint_traj_sub = nh.subscribe( "joint_trajectory", 1, &pololu_qik::JointTrajCB, this );

	return true;
}

void pololu_qik::stop( )
{
	ROS_INFO( "Stopping" );

	if( joint_traj_sub )
		joint_traj_sub.shutdown( );

	close( );
}

bool pololu_qik::is_open( ) const
{
	return ( fd >= 0 );
}

void pololu_qik::JointTrajCB( const trajectory_msgs::JointTrajectoryPtr &msg )
{
	int ch1_idx = -1;
	int ch2_idx = -1;

	for( size_t i = 0; i < msg->joint_names.size( ); i++ )
	{
		if( msg->joint_names[i] == ch1_joint_name )
			ch1_idx = i;
		if( msg->joint_names[i] == ch2_joint_name )
			ch2_idx = i;
	}

	if( 0 > ch1_idx && 0 > ch2_idx )
	{
		ROS_WARN( "Got a JointTrajectory message with no valid joints" );
		return;
	}

	if( 1 > msg->points.size( ) )
	{
		ROS_WARN( "Got a JointTrajectory message with no valid points" );
		return;
	}

	if( msg->joint_names.size( ) != msg->points[0].velocities.size( ) )
	{
		ROS_WARN( "Got a JointTrajectory message whose points have no velocities" );
		return;
	}

	set_ch1( msg->points[0].velocities[ch1_idx] );
	set_ch2( msg->points[0].velocities[ch2_idx] );
}

bool pololu_qik::set_ch1( double speed )
{
	static bool last_fw = false;
	char temp[2] = { 0 };

	if( !is_open( ) && !open( ) )
		return false;

	speed *= 127;

	//Construct
	if ( speed > 0 || ( !last_fw && speed == 0 ) )
	{
		temp[0] = 0x88;
		last_fw = true;
	}
	else
	{
		temp[0] = 0x8A;
		last_fw = false;
	}

	// Make positive
	if( speed < 0 )
		speed *= -1;

	// Normalize 
	if( speed > 127 )
		speed = 127;

	temp[1] = speed;

	//Send
	if( 0 > write( fd, temp, 2 ) )
	{
		ROS_ERROR( "Failed to update channel 1: %s", strerror( errno ) );
		close( );
		return false;
	}

	return true;
}

bool pololu_qik::set_ch2( double speed )
{
	static bool last_fw = false;
	char temp[2] = { 0 };

	if( !is_open( ) && !open( ) )
		return false;

	speed *= 127;

	//Construct
	if ( speed > 0 || ( !last_fw && speed == 0 ) )
	{
		temp[0] = 0x8C;
		last_fw = true;
	}
	else
	{
		temp[0] = 0x8E;
		last_fw = false;
	}

	// Make positive
	if( speed < 0 )
		speed *= -1;

	// Normalize 
	if( speed > 127 )
		speed = 127;

	temp[1] = speed;

	//Send
	if( 0 > write( fd, temp, 2 ) )
	{
		ROS_ERROR( "Failed to update channel 2: %s", strerror( errno ) );
		close( );
		return false;
	}

	return true;
}

}
