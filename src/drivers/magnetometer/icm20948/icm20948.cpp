/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file icm20948.cpp
 *
 * Driver for ICM-20948.
 * Currently only the AK09916 magnetometer submodule and connection through I2C on address 0x69
 * (as found on the Here! GPS) are supported.
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

#include <drivers/magnetometer/ak09916/ak09916.h>
#include <drivers/magnetometer/ak09916/ak09916_i2c.h>

#include "icm20948.h"




/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int icm20948_main(int argc, char *argv[]);


ICM20948::ICM20948(device::Device *interface, const char *path_mag, enum Rotation rotation) :
    _interface(interface),
	_ak09916_mag(nullptr),
    _path_mag(path_mag),
	_sample_perf(perf_alloc(PC_ELAPSED, "ICM20948_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ICM20948_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "ICM20948_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "ICM20948_conf_err")),
	_rotation(rotation)
{
}

ICM20948::~ICM20948()
{
    /* Delete child AK09116 */
    _ak09916_mag->stop();
    delete _ak09916_mag;

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
ICM20948::init()
{
    int ret = PX4_ERROR;

    /* reset the ICM20948 configuration */
    ret = reset();
    if (ret != OK) {
        PX4_WARN("ICM20948 reset failed");
        return ret;
    }

    /* Now the AK09916 module should be active on the I2C bus. Spawning a driver for it. */
    device::Device *mag_interface = AK09916_I2C_interface(_interface->get_device_bus());
    ret = mag_interface->init();
    if (ret != OK) {
        delete mag_interface;
        warnx("ICM20948 subdevice AK09916 not found on bus %u (type: %u)", (unsigned)_interface->get_device_bus(), (unsigned)_interface->get_device_bus_type());
        return ret;
    }

    _ak09916_mag = new AK09916(mag_interface, _path_mag, _rotation);
    if (_ak09916_mag == nullptr) {
        delete mag_interface;
        return PX4_ERROR;
    }

    ret=_ak09916_mag->init();
    if (ret != OK) {
        delete _ak09916_mag;
        _ak09916_mag = nullptr;
        delete mag_interface;
        warnx("Failed to intialize ICM20948 magnetometer");
        return ret;
    }

    int fd = open(_path_mag, O_RDONLY);
    if (fd < 0) {
        return PX4_ERROR;
    }

    if (::ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        close(fd);
        errx(1, "Failed to setup poll rate");
        return PX4_ERROR;
    }

    close(fd);

    return PX4_OK;
}


void
ICM20948::start()
{
    _ak09916_mag->start();
}

void
ICM20948::stop()
{
    _ak09916_mag->stop();
}

/*
 * Reset and configure ICM20948
 */
int
ICM20948::reset()
{
    int ret = PX4_ERROR;

    ret = write_reg(ADDR_PWR_MGMT_1, REG_PWR_MGMT_1_RESET);
    if (ret != OK) {
        PX4_WARN("ICM20948 reset failed");
        return ret;
    }

    usleep(200);

    /* configure device */
    ret = configure();
    if (ret != OK) {
        PX4_WARN("ICM20948 config failed");
        return ret;
    }

    return ret;
}

/*
 * perform the selftest routines
 */
int ICM20948::selftest() {

	/* only magnetometer selftest by now */
	return _ak09916_mag->selftest();
}

/*
 * Reset and configure AK09916
 */
int
ICM20948::ak09916_reset()
{
    int ret = PX4_ERROR;

    ret=_ak09916_mag->reset();
    if (ret != OK) {
        PX4_DEBUG("Failed to reset ICM20948 magnetometer (AK09916)");
        return ret;
    }

    int fd = open(_path_mag, O_RDONLY);
    if (fd < 0) {
        PX4_DEBUG("Failed to open device file %s", _path_mag);
        return PX4_ERROR;
    }

    if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        close(fd);
        PX4_WARN("Failed to setup poll rate");
        return PX4_ERROR;
    }

    return ret;
}

int
ICM20948::configure()
{
    int ret = PX4_ERROR;

    /* enable device */
    for(uint8_t retries=0; retries < 3; retries++) {
        ret = write_reg(ADDR_PWR_MGMT_1, REG_PWR_MGMT_1_ENABLE);
        if (OK != ret) perf_count(_comms_errors);
        else break;
    }

    usleep(200);

    /* configure I2C passthrough */
    if (ret == OK) {
        for(uint8_t retries=0; retries < 3; retries++) {
           ret = write_reg(ADDR_INT_PIN_CFG, REG_INT_PIN_CFG_BYPASS);
           if (OK != ret) perf_count(_comms_errors);
           else break;
       }
    }

    usleep(200);

    return ret;
}

int
ICM20948::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
ICM20948::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
ICM20948::print_info()
{
    printf("ICM20948 info:\n");
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

    printf("Magnetometer info:\n");
    _ak09916_mag->print_info();

    _ak09916_mag->selftest();
}

