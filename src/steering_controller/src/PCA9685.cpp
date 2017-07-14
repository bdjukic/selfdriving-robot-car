/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Name        : PCA9685.cpp
 * Author      : Georgi Todorov
 * Version     :
 * Created on  : Dec 9, 2012
 *
 * Copyright © 2012 Georgi Todorov  <terahz@geodar.com>
 */

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <syslog.h>		/* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>

#include "PCA9685.h"

//! Constructor takes bus and address arguments
/*!
 \param bus the bus to use in /dev/i2c-%d.
 \param address the device address on bus
 */
PCA9685::PCA9685(int bus, int address) {
	i2c = new I2C(bus,address);
	reset();
	setPWMFreq(1000);
}

PCA9685::~PCA9685() {
	delete i2c;
}
//! Sets PCA9685 mode to 00
void PCA9685::reset() {

		i2c->write_byte(MODE1, 0x00); //Normal mode
		i2c->write_byte(MODE2, 0x04); //totem pole

}
//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685::setPWMFreq(int freq) {

		uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
		i2c->write_byte(MODE1, 0x10); //sleep
		i2c->write_byte(PRE_SCALE, prescale_val); // multiplyer for PWM frequency
		i2c->write_byte(MODE1, 0x80); //restart
		i2c->write_byte(MODE2, 0x04); //totem pole (default)
}

//! PWM a single channel
/*!
 \param led channel (1-16) to set PWM value for
 \param value 0-4095 value for PWM
 */
void PCA9685::setPWM(uint8_t led, int value) {
	setPWM(led, 0, value);
}
//! PWM a single channel with custom on time
/*!
 \param led channel (1-16) to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setPWM(uint8_t led, int on_value, int off_value) {
		i2c->write_byte(LED0_ON_L + LED_MULTIPLYER * (led - 1), on_value & 0xFF);
		i2c->write_byte(LED0_ON_H + LED_MULTIPLYER * (led - 1), on_value >> 8);
		i2c->write_byte(LED0_OFF_L + LED_MULTIPLYER * (led - 1), off_value & 0xFF);
		i2c->write_byte(LED0_OFF_H + LED_MULTIPLYER * (led - 1), off_value >> 8);
}

//! Get current PWM value
/*!
 \param led channel (1-16) to get PWM value from
 */
int PCA9685::getPWM(uint8_t led){
	int ledval = 0;
	ledval = i2c->read_byte(LED0_OFF_H + LED_MULTIPLYER * (led-1));
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += i2c->read_byte(LED0_OFF_L + LED_MULTIPLYER * (led-1));
	return ledval;
}

