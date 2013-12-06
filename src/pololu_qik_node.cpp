#include <pololu_qik/pololu_qik.hpp>

int main( int argc, char *argv[] )
{
	ros::NodeHandle *nh = NULL;
	ros::NodeHandle *nh_priv = NULL;
	pololu_qik::pololu_qik *qik = NULL;

	ros::init( argc, argv, "pololu_qik_node" );

	nh = new ros::NodeHandle( );
	if( !nh )
	{
		ROS_FATAL( "Failed to initialize NodeHanlde" );
		ros::shutdown( );
		return -1;
	}
	nh_priv = new ros::NodeHandle( "~" );
	if( !nh_priv )
	{
		ROS_FATAL( "Failed to initialize private NodeHanlde" );
		delete nh;
		ros::shutdown( );
		return -2;
	}
	qik = new pololu_qik::pololu_qik( *nh, *nh_priv );
	if( !qik )
	{
		ROS_FATAL( "Failed to initialize driver" );
		delete nh_priv;
		delete nh;
		ros::shutdown( );
		return -3;
	}
	if( !qik->start( ) )
		ROS_ERROR( "Failed to start the driver" );

	ros::spin( );

	delete qik;
	delete nh_priv;
	delete nh;

	return 0;
}
