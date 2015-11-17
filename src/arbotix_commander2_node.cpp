#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <boost/asio.hpp>
#include <stdexcept>
#include <sstream>
#include <cstdlib>

class ArbotixCommander2
{
	ros::Publisher _joy_pub;
	boost::asio::io_service _ioservice;
	boost::asio::serial_port _port;

public:
	ArbotixCommander2(ros::Publisher joy_pub, std::string commander_serial_port, unsigned int baud_rate)
		: _joy_pub(joy_pub), _port(_ioservice)
	{
		boost::system::error_code ec;
		_port.open(commander_serial_port, ec);
		if (ec)
		{
			std::stringstream ss;
			ss << "Could not open \"" << commander_serial_port << "\".";
			throw( std::runtime_error(ss.str()) );
		}

		_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
		_port.set_option(boost::asio::serial_port_base::character_size(8));
		_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
	}

	~ArbotixCommander2()
	{
		_port.close();
	}

	void run()
	{
		unsigned char frame[8];
		boost::system::error_code ec;
		while(ros::ok())
		{
			// Fill the frame
			boost::asio::read(_port, boost::asio::buffer(frame, 8), ec);
			if (ec)
			{
				throw std::runtime_error("Error getting serial data.");
			}
			// Allign the header (0xff)
			if (! (frame[0] == 0xff)) {
				for (int i = 1; i < 8; ++i)
				{
					frame[i-1] = frame[i];
				}
				boost::asio::read(_port, boost::asio::buffer(&frame[8], 1), ec);
				if (ec)
				{
					throw std::runtime_error("Error getting serial data.");
				}
			}
			std::stringstream ss;
			ss << (int)frame[0] << " " << (int)frame[1] << " " << (int)frame[2] << " " << (int)frame[3] << " " << (int)frame[4] << " " << (int)frame[5] << " " << (int)frame[6] << " " << (int)frame[7];
			std::cout << ss.str() << std::endl;
		}

	}
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arbotix_commander2_node");

	// Advertize the joystick mesages.
	ros::NodeHandle n;
	ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("joy", 1);

	// Get the serial port that the commander is connected to. We require
	// that there is a private parameter "serial_port" where the commander
	// is connected.
	ros::NodeHandle private_n("~");
	std::string serial_port;
	if (private_n.getParam("serial_port", serial_port)) {
		ROS_INFO_STREAM("Connecting to Arbotix Commander on " << serial_port);
	} else {
		ROS_ERROR_STREAM(
		    "No serial port was specified for Arbotix Commander. " <<
		    "Please set the private parameter \"serial_port\" for node \"" <<
		    ros::this_node::getName() <<
		    "\", e.g. \"/dev/ttyUSB0\".");
	}

	std::string baud_rate_str;
	unsigned int baud_rate;
	if (private_n.getParam("baud_rate", baud_rate_str)) {
		baud_rate = atoi(baud_rate_str.c_str());
		ROS_INFO_STREAM("Baud rate set to " << baud_rate);
	} else {
		baud_rate = 38400;
		ROS_INFO_STREAM("No baud rate supplied, defaulting to " << baud_rate);
	}

	// Start the driver
	try {
		ArbotixCommander2 joystick(joy_pub, serial_port, baud_rate);
		joystick.run();
	} catch (std::runtime_error re) {
		ROS_ERROR_STREAM( re.what() );
		return 1;
	}

	return 0;
}