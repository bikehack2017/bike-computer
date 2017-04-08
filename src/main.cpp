#include <iostream>
#include <thread>
#include <chrono>

#include <mraa/gpio.h>

#include "Sensor.hpp"

#define SENSOR_PIN	2
#define WHEEL_DIAMETER	29.f

int main() {
	using namespace std::chrono_literals;

	Sensor sensor(SENSOR_PIN, WHEEL_DIAMETER);

	double dist = -1, vel = -1;
	while(1) {
		double tempDist, tempVel;

		tempDist = sensor.getDistance();
		tempVel = sensor.getVelocity();

		if( (tempDist != dist) || (tempVel != vel) ) {
			dist = tempDist;
			vel = tempVel;

			std::cout << vel << "mph\t\t" << dist << "m" << std::endl;
		}

		std::this_thread::sleep_for(1ms);
	}

	return 0;
}
