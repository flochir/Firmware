/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * Driver for the Invensense mpu9250 connected via I2C or SPI.
 *
 * @authors Andrew Tridgell
 *          Robert Dickenson
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <px4_getopt.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mpu9250.h"

/** driver 'main' command */
extern "C" { __EXPORT int mpu9250_main(int argc, char *argv[]); }


enum MPU9250_BUS {
	MPU9250_BUS_ALL = 0,
	MPU9250_BUS_I2C_INTERNAL,
	MPU9250_BUS_I2C_EXTERNAL,
	MPU9250_BUS_SPI_INTERNAL,
	MPU9250_BUS_SPI_INTERNAL2,
	MPU9250_BUS_SPI_EXTERNAL
};




/**
 * Local functions in support of the shell command.
 */
namespace mpu9250
{

char accelpath_generated[40];
char gyropath_generated[40];
char magpath_generated[40];

enum MPU9250_GENIDX_DEVTYPE {
    MPU9250_GENIDX_DEVTYPE_MPU9250 = 0,
    MPU9250_GENIDX_DEVTYPE_MPU6500,
    MPU9250_GENIDX_DEVTYPE_ICM20948
};

enum MPU9250_GENIDX_BUSTYPE {
    MPU9250_GENIDX_BUSTYPE_SPI = 0,
    MPU9250_GENIDX_BUSTYPE_I2C
};

enum MPU9250_GENIDX_BUSEXT {
    MPU9250_GENIDX_BUSEXT_INT = 0,
    MPU9250_GENIDX_BUSEXT_EXT
};

const char devpath_base[6] = "/dev/";
const char* const devpath_devtype[] = {"mpu9250_", "mpu6500_", "icm20948_"};
const char* const devpath_bustype[] = {"spi_", "i2c_"};
const char* const devpath_busext[] = {"int_", "ext_"};

/*
  list of supported bus configurations
 */

struct mpu9250_bus_option {
	enum MPU9250_BUS busid;
	MPU_DEVICE_TYPE device_type;
	MPU9250_constructor interface_constructor;
	bool magpassthrough;
	uint8_t busnum;
	uint32_t address;
	MPU9250	*dev;
} bus_options[] = {
#if defined (USE_I2C)
#  if defined(PX4_I2C_BUS_ONBOARD) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_INTERNAL, MPU_DEVICE_TYPE_MPU9250, &MPU9250_I2C_interface, false, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_MPU9250, NULL },
	{ MPU9250_BUS_I2C_INTERNAL, MPU_DEVICE_TYPE_MPU6500, &MPU9250_I2C_interface, false, PX4_I2C_BUS_ONBOARD, PX4_I2C_OBDEV_MPU9250, NULL },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION)
#  if defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU9250, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_MPU9250, NULL },
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU6500, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_OBDEV_MPU9250, NULL },
#  endif
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_ICM20948, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION, PX4_I2C_EXT_ICM20948_1, NULL },
#endif
#  if defined(PX4_I2C_BUS_EXPANSION1) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU9250, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION1, PX4_I2C_OBDEV_MPU9250, NULL },
#  endif
#  if defined(PX4_I2C_BUS_EXPANSION2) && defined(PX4_I2C_OBDEV_MPU9250)
	{ MPU9250_BUS_I2C_EXTERNAL, MPU_DEVICE_TYPE_MPU9250, &MPU9250_I2C_interface, false, PX4_I2C_BUS_EXPANSION2, PX4_I2C_OBDEV_MPU9250, NULL },
#  endif
#endif
#ifdef PX4_SPIDEV_MPU
	{ MPU9250_BUS_SPI_INTERNAL, MPU_DEVICE_TYPE_MPU9250, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU, NULL },
	{ MPU9250_BUS_SPI_INTERNAL, MPU_DEVICE_TYPE_MPU6500, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU, NULL },
#endif
#ifdef PX4_SPIDEV_MPU2
	{ MPU9250_BUS_SPI_INTERNAL2, MPU_DEVICE_TYPE_MPU9250, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU2, NULL },
	{ MPU9250_BUS_SPI_INTERNAL2, MPU_DEVICE_TYPE_MPU6500, &MPU9250_SPI_interface, true, PX4_SPI_BUS_SENSORS, PX4_SPIDEV_MPU2, NULL },
#endif
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU)
	{ MPU9250_BUS_SPI_EXTERNAL, MPU_DEVICE_TYPE_MPU9250, &MPU9250_SPI_interface, true, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, NULL },
	{ MPU9250_BUS_SPI_EXTERNAL, MPU_DEVICE_TYPE_MPU6500, &MPU9250_SPI_interface, true, PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, NULL },
#endif
};

#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))


int     appendit(char *target, const char *source, uint8_t limit);
int     generate_path(char *target, uint8_t target_size, const char *base, const char *devtype,const char *bustype, const char *busext, uint8_t busnum, const char* sensor_type);

void	start(enum MPU9250_BUS busid, enum Rotation rotation, bool external_bus, bool magnetometer_only);
bool	start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external_bus, bool magnetometer_only);
struct mpu9250_bus_option &find_bus(enum MPU9250_BUS busid);
void	stop(enum MPU9250_BUS busid);
void	test(enum MPU9250_BUS busid);
void	reset(enum MPU9250_BUS busid);
void	info(enum MPU9250_BUS busid);
void	regdump(enum MPU9250_BUS busid);
void	testerror(enum MPU9250_BUS busid);
void	usage();


int appendit(char *target, const char *source, uint8_t limit) {
    uint8_t cpcount=0;

    while(cpcount < limit && source[cpcount] != 0) {
        target[cpcount] = source[cpcount];
        cpcount++;
    }

    if(cpcount >= limit && source[cpcount] != 0) {
        return -1;
    }
    else {
        return cpcount;
    }
}

int generate_path(char *target, uint8_t target_size, const char *base, const char *devtype,const char *bustype, const char *busext, uint8_t busnum, const char* sensor_type) {
    uint8_t next_pos=0;
    int cpcount=0;
    int error=0;
    uint8_t limit=target_size-1; // char length without null terminator
    char busnum_str[3];
    const char* partlist[6] = { base, devtype, bustype, busext, busnum_str, sensor_type };

    busnum_str[2]=0;
    busnum_str[1]='_';
    if(busnum>9) {
        return -1;
    }
    else {
        busnum_str[0]=(char)(busnum+48);
    }

    for (uint8_t i=0; i<6; i++) {
        cpcount = appendit(&target[next_pos], partlist[i], limit-next_pos);
        if(cpcount < 0) {
            error=-1;
            break;
        }
        else {
            next_pos += cpcount;
        }
    }
    return error;
}



/**
 * find a bus structure for a busid
 */
struct mpu9250_bus_option &find_bus(enum MPU9250_BUS busid)
{
	for (uint8_t i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MPU9250_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct mpu9250_bus_option &bus, enum Rotation rotation, bool external, bool magnetometer_only)
{
	int fd = -1;

    uint8_t devtype_idx=0;
    uint8_t bustype_idx=0;
    uint8_t busext_idx;
    uint8_t busnum;
    const char sensor_type_accel[] = "accel";
    const char sensor_type_gyro[] = "gyro";
    const char sensor_type_mag[] = "mag";

	PX4_INFO("Bus probed: %d", bus.busid);

	if (bus.dev != nullptr) {
		warnx("%s SPI not available", external ? "External" : "Internal");
		return false;
	}

	device::Device *interface = bus.interface_constructor(bus.busnum, bus.device_type, bus.address, external);

	if (interface == nullptr) {
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	device::Device *mag_interface = nullptr;

#ifdef USE_I2C
	/* For i2c interfaces, connect to the magnetomer directly */
	bool is_i2c = bus.busid == MPU9250_BUS_I2C_INTERNAL || bus.busid == MPU9250_BUS_I2C_EXTERNAL;

	if (is_i2c) {
		mag_interface = AK8963_I2C_interface(bus.busnum, external);
	}

#endif


    switch(bus.device_type) {
    case MPU_DEVICE_TYPE_MPU9250:
        devtype_idx=MPU9250_GENIDX_DEVTYPE_MPU9250;
        break;
    case MPU_DEVICE_TYPE_MPU6500:
        devtype_idx=MPU9250_GENIDX_DEVTYPE_MPU6500;
        break;
    case MPU_DEVICE_TYPE_ICM20948:
        devtype_idx=MPU9250_GENIDX_DEVTYPE_ICM20948;
        break;
    default:
        goto fail;
    }

    switch(bus.busid) {
    case MPU9250_BUS_I2C_INTERNAL:
        bustype_idx=MPU9250_GENIDX_BUSTYPE_I2C;
        busext_idx=MPU9250_GENIDX_BUSEXT_INT;
        busnum=1;
        break;
    case MPU9250_BUS_I2C_EXTERNAL:
        bustype_idx=MPU9250_GENIDX_BUSTYPE_I2C;
        busext_idx=MPU9250_GENIDX_BUSEXT_EXT;
        busnum=1;
        break;
    case MPU9250_BUS_SPI_INTERNAL:
        bustype_idx=MPU9250_GENIDX_BUSTYPE_SPI;
        busext_idx=MPU9250_GENIDX_BUSEXT_INT;
        busnum=1;
        break;
    case MPU9250_BUS_SPI_INTERNAL2:
        bustype_idx=MPU9250_GENIDX_BUSTYPE_SPI;
        busext_idx=MPU9250_GENIDX_BUSEXT_INT;
        busnum=2;
        break;
    case MPU9250_BUS_SPI_EXTERNAL:
        bustype_idx=MPU9250_GENIDX_BUSTYPE_SPI;
        busext_idx=MPU9250_GENIDX_BUSEXT_EXT;
        busnum=1;
        break;
    default:
        goto fail;
    }

    generate_path(accelpath_generated, sizeof(accelpath_generated), devpath_base, devpath_devtype[devtype_idx], devpath_bustype[bustype_idx], devpath_busext[busext_idx], busnum, sensor_type_accel);
    generate_path(gyropath_generated, sizeof(accelpath_generated), devpath_base, devpath_devtype[devtype_idx], devpath_bustype[bustype_idx], devpath_busext[busext_idx], busnum, sensor_type_gyro);
    generate_path(magpath_generated, sizeof(accelpath_generated), devpath_base, devpath_devtype[devtype_idx], devpath_bustype[bustype_idx], devpath_busext[busext_idx], busnum, sensor_type_mag);

    PX4_WARN("Starting with path %s\n", accelpath_generated);

	bus.dev = new MPU9250(interface, mag_interface, accelpath_generated, gyropath_generated, magpath_generated, rotation, bus.device_type,
			      magnetometer_only);

	if (bus.dev == nullptr) {
		delete interface;

		if (mag_interface != nullptr) {
			delete mag_interface;
		}

		return false;
	}

	if (OK != bus.dev->init()) {
		goto fail;
	}

	/*_options[i].busid) {
        case MPU9250_BUS_I2C_EXTERNAL:
            bustype_idx=MPU9250_GENIDX_BUSTYPE_I2C;
            busnum=2;
            break;
	 * Set the poll rate to default, starts automatic data collection.
	 * Doing this through the mag device for the time being - it's always there, even in magnetometer only mode.
	 * Using accel device for MPU6500.
	 */
	if (bus.device_type == MPU_DEVICE_TYPE_MPU6500) {
		fd = open(accelpath_generated, O_RDONLY);

	} else {
		fd = open(magpath_generated, O_RDONLY);
	}

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}


	close(fd);

	return true;

fail:

	if (fd >= 0) {
		close(fd);
	}

	if (bus.dev != nullptr) {
		delete (bus.dev);
		bus.dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(enum MPU9250_BUS busid, enum Rotation rotation, bool external, bool magnetometer_only)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MPU9250_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MPU9250_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}

		if (bus_options[i].device_type == MPU_DEVICE_TYPE_MPU6500 && magnetometer_only) {
			// prevent starting MPU6500 in magnetometer only mode
			continue;
		}

		started |= start_bus(bus_options[i], rotation, external, magnetometer_only);

		if (started) { break; }
	}

	exit(started ? 0 : 1);

}

void
stop(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev != nullptr) {
		delete bus.dev;
		bus.dev = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);
	accel_report a_report;
	gyro_report g_report;
	mag_report m_report;
	ssize_t sz;

	/* get the driver */
	int fd = open(bus.dev->_accelpath, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'm start')", bus.dev->_accelpath);
	}

	/* get the driver */
	int fd_gyro = open(bus.dev->_gyropath, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", bus.dev->_gyropath);
	}

	/* get the driver */
	int fd_mag = open(bus.dev->_magpath, O_RDONLY);

	if (fd_mag < 0) {
		err(1, "%s open failed", bus.dev->_magpath);
	}

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	print_message(a_report);

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	/* do a simple demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(m_report));
		err(1, "immediate mag read failed");
	}

	print_message(m_report);

	/* reset to default polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd);
	close(fd_gyro);
	close(fd_mag);

	/* XXX add poll-rate tests here too */

	reset(busid);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);
	int fd = open(bus.dev->_accelpath, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", bus.dev);
	bus.dev->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("regdump @ %p\n", bus.dev);
	bus.dev->print_registers();

	exit(0);
}

/**
 * deliberately produce an error to test recovery
 */
void
testerror(enum MPU9250_BUS busid)
{
	struct mpu9250_bus_option &bus = find_bus(busid);


	if (bus.dev == nullptr) {
		errx(1, "driver not running");
	}

	bus.dev->test_error();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (i2c external bus)");
	PX4_INFO("    -I    (i2c internal bus)");
	PX4_INFO("    -s    (spi internal bus)");
	PX4_INFO("    -S    (spi external bus)");
	PX4_INFO("    -t    (spi internal bus, 2nd instance)");
	PX4_INFO("    -R rotation");
	PX4_INFO("    -M only enable magnetometer, accel/gyro disabled - not av. on MPU6500");

}

} // namespace

int
mpu9250_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	enum MPU9250_BUS busid = MPU9250_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
	bool magnetometer_only = false;

	while ((ch = px4_getopt(argc, argv, "XISstMR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			busid = MPU9250_BUS_I2C_EXTERNAL;
			break;

		case 'I':
			busid = MPU9250_BUS_I2C_INTERNAL;
			break;

		case 'S':
			busid = MPU9250_BUS_SPI_EXTERNAL;
			break;

		case 's':
			busid = MPU9250_BUS_SPI_INTERNAL;
			break;

		case 't':
			busid = MPU9250_BUS_SPI_INTERNAL2;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'M':
			magnetometer_only = true;
			break;

		default:
			mpu9250::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		mpu9250::usage();
		return -1;
	}

	bool external = busid == MPU9250_BUS_I2C_EXTERNAL || busid == MPU9250_BUS_SPI_EXTERNAL;
	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		mpu9250::start(busid, rotation, external, magnetometer_only);
	}

	if (!strcmp(verb, "stop")) {
		mpu9250::stop(busid);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		mpu9250::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpu9250::reset(busid);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		mpu9250::info(busid);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		mpu9250::regdump(busid);
	}

	if (!strcmp(verb, "testerror")) {
		mpu9250::testerror(busid);
	}

	mpu9250::usage();
	return 0;
}
