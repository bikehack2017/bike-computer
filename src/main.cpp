#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <mraa.h>
#include <mraa/gpio.h>
#include <signal.h>

#include "Sensor.hpp"

#define SENSOR_PIN	2
#define WHEEL_DIAMETER	29.f

#define UART_NAME		"/dev/ttyMFD1"
#define UART_BAUD		(9600)

#define HEADER	(0xAA)

void sendUpdate(boost::asio::serial_port &comPort, float dist, float vel);
void sigHandler(int sig);

mraa_uart_context uart;
bool run;

int main() {
	using namespace std::chrono_literals;

	signal(SIGINT, &sigHandler);

	Sensor sensor(SENSOR_PIN, WHEEL_DIAMETER);

	boost::asio::io_service ioService;
	boost::asio::serial_port comPort(ioService);

/*
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
*/
	
	//uart = mraa_uart_init_raw("/dev/ttyMFD1");
	uart = mraa_uart_init(0);
	 
	if (uart == NULL) {
					fprintf(stderr, "UART failed to setup\n");
					return EXIT_FAILURE;
	}
	printf("Device path: %s\n", mraa_uart_get_dev_path(uart));
	 
	mraa_uart_set_baudrate(uart, UART_BAUD);
	mraa_uart_set_mode(uart, 8, MRAA_UART_PARITY_NONE, 1);
	mraa_uart_set_flowcontrol(uart, 0, 0);
	mraa_uart_set_timeout(uart, 0, 0, 0);

	std::ofstream csv("/var/www/html/ride.csv");

	csv << "#Real Ride\nTime, Speed, Elevation, Distance\n";

	run = true;

	float dist = -1, vel = -1;
	double time = 0.;
	while(run) {
		float tempDist, tempVel;

		tempDist = sensor.getDistance();
		tempVel = sensor.getVelocity();

		if( (tempDist != dist) || (tempVel != vel) ) {
			dist = tempDist;
			vel = tempVel;
			
			sendUpdate(comPort, dist, vel);

			std::cout << vel << "mph\t\t" << dist << "m" << std::endl;

			csv << time << ", " << vel << ", " << 0 << ", " << dist << "\n";
			csv.flush();
		}

		std::this_thread::sleep_for(1ms);
		time += 0.001;
	}

	mraa_uart_stop(uart);
	mraa_deinit();

	return 0;
}

void sendUpdate(boost::asio::serial_port &comPort, float dist, float vel) {
	uint8_t velChar = vel + 0.5f;

	mraa_uart_write(uart, (const char*)&velChar, 1);
	mraa_uart_flush(uart);
}

void sigHandler(int sig) {
	run = false;
}
