#include "Sensor.hpp"

#include <thread>


Sensor::Sensor(uint8_t _pin, float _wheelDiameter)
	:	circum{3.141592654f * _wheelDiameter / 12.f / 5280.f}
	,	velocity{0.f}
	,	count{0}
	,	lastTime{std::chrono::steady_clock::now()}
	,	pin{mraa_gpio_init(_pin)} {
	
	mraa_gpio_dir(pin, MRAA_GPIO_IN);
	mraa_gpio_isr(pin, MRAA_GPIO_EDGE_RISING, &Sensor::interrupt, this);

}

float Sensor::getDistance() const {
	return count * circum;
}

float Sensor::getVelocity() const {
	return velocity * circum;
}

void Sensor::interrupt(void *instance) {
	reinterpret_cast<Sensor*>(instance)->update();
}

void Sensor::update() {
	auto now = std::chrono::steady_clock::now();

	double dt = std::chrono::duration<float, std::milli>(now - lastTime).count() / (60000.f * 60.f);
	velocity = 1. / dt;
	count++;

	lastTime = now;
}
