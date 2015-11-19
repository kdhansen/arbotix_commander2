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
	unsigned int _sequence_num;

public:
	ArbotixCommander2(ros::Publisher joy_pub, std::string commander_serial_port, unsigned int baud_rate)
		: _joy_pub(joy_pub), _port(_ioservice), _sequence_num(0)
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
		while(ros::ok())
		{
			fill_frame(frame);
			align_frame(frame);
			while (! check_checksum(frame) )
			{
				progress_frame(frame);
				align_frame(frame);
			}
			sensor_msgs::Joy joy_msg = interpret_frame(frame);
			joy_msg.header.seq = _sequence_num++;
			_joy_pub.publish(joy_msg);
		}
	}

private:

	void print_frame(const unsigned char frame[])
	{
		std::stringstream ss;
		for (int i = 0; i < 8; ++i)
		{
			ss << (unsigned int)frame[i] << " ";
		}
		std::cout << ss.str() << std::endl;
	}

	void fill_frame(unsigned char frame[])
	{
		boost::system::error_code ec;
		boost::asio::read(_port, boost::asio::buffer(frame, 8), ec);
		if (ec)
		{
			throw std::runtime_error("Error getting serial data.");
		}
	}

	void align_frame(unsigned char frame[])
	{
		// First byte must be 0xff, next to last byte must be 0x00. Furthermore,
		// the second byte cannot be 255, this happens when the checksum is 255,
		// then the alignment may fix the checksum in the first position is no
		// buttons are pressed.
		while (frame[0] != 0xff || frame[1] == 0xff || frame[6] != 0x00)
		{
			progress_frame(frame);
		}
	}

	void progress_frame(unsigned char frame[])
	{
		// Progress the buffer one entry,
		for (int i = 1; i < 8; ++i)
		{
			frame[i-1] = frame[i];
		}
		// and fill a byte into the end.
		boost::system::error_code ec;
		size_t n;
		n = boost::asio::read(_port, boost::asio::buffer(&frame[7], 1), ec);
		if (ec)
		{
			throw std::runtime_error("Error getting serial data.");
		}
	}

	bool check_checksum(const unsigned char frame[])
	{
		unsigned int sum = 0;
		for (int i = 0; i < 8; ++i)
		{
			sum += frame[i];
		}
		if (sum%256 == 254)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	sensor_msgs::Joy interpret_frame(unsigned char frame[])
	{
		sensor_msgs::Joy joy_msg;
		joy_msg.axes.reserve(4);
		for (int i = 1; i < 5; ++i)
		{
			double val = (frame[i] - 128.0) / 128.0;
			joy_msg.axes.push_back(val);
		}

		joy_msg.buttons.reserve(8);
		for (int i = 0; i < 8; ++i)
		{
			int val = (frame[5] >> i) & 0x01;
			joy_msg.buttons.push_back(val);
		}

		return joy_msg;
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