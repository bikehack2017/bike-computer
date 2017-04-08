#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <mraa/gpio.h>

#include "Sensor.hpp"

#define SENSOR_PIN	2
#define WHEEL_DIAMETER	29.f

#define UART_NAME		"/dev/ttyMFD1"
#define UART_BAUD		(9600)

#define HEADER	(0xAA)

void sendUpdate(boost::asio::serial_port &comPort, float dist, float vel);

int main() {
	using namespace std::chrono_literals;

	Sensor sensor(SENSOR_PIN, WHEEL_DIAMETER);

	boost::asio::io_service ioService;
	boost::asio::serial_port comPort(ioService);


	comPort.open(UART_NAME);
	if(!comPort.is_open()) {
		std::cout << "[Error] Failed to open " << UART_NAME << std::endl;
		return 1;
	}

	comPort.set_option(boost::asio::serial_port_base::baud_rate(UART_BAUD));
	comPort.set_option(boost::asio::serial_port_base::flow_control(
		boost::asio::serial_port_base::flow_control::none));
	comPort.set_option(boost::asio::serial_port_base::parity(
		boost::asio::serial_port_base::parity::none));
	comPort.set_option(boost::asio::serial_port_base::stop_bits(
		boost::asio::serial_port_base::stop_bits::one));
	comPort.set_option(boost::asio::serial_port_base::character_size(8));

	float dist = -1, vel = -1;
	while(1) {
		float tempDist, tempVel;

		tempDist = sensor.getDistance();
		tempVel = sensor.getVelocity();

		if( (tempDist != dist) || (tempVel != vel) ) {
			dist = tempDist;
			vel = tempVel;
			
			sendUpdate(comPort, dist, vel);

			std::cout << vel << "mph\t\t" << dist << "m" << std::endl;
		}

		std::this_thread::sleep_for(1ms);
	}

	return 0;
}

void sendUpdate(boost::asio::serial_port &comPort, float dist, float vel) {
	std::vector<uint8_t> packet(9);

	packet[0] = HEADER;

	std::memcpy(packet.data() + 1, &dist, 4);
	std::memcpy(packet.data() + 5, &vel, 4);

	comPort.write_some(boost::asio::buffer(packet));
}
