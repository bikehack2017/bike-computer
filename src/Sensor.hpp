#pragma once

#include <chrono>
#include <cstdint>
#include <mraa/gpio.h>

class Sensor
{
public:
	Sensor(uint8_t pin, float wheelDiameter);

	float getVelocity() const;
	float getDistance() const;

private:
	static void interrupt(void *instance);
	void update();

	float circum;
	float velocity;
	volatile unsigned int count;
	mutable std::chrono::steady_clock::time_point lastTime;
	mraa_gpio_context pin;
};
