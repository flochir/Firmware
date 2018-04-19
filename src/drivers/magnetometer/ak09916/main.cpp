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
 * Driver for the ICM-20948 magnetometer connected via I2C.
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

#include "ak09916.h"

/*
 * ICM20948 internal constants and data structures.
 */

#define ADDR_PWR_MGMT_1         0x06
#define PWR_MGMT_1_REG_ENABLE   0x00

#define ADDR_INT_PIN_CFG        0x0F
#define INT_IN_CFG_REG_BYPASS   0x02



enum ICM20948_BUS {
    ICM20948_BUS_ALL = 0,
    ICM20948_BUS_I2C_INTERNAL,
    ICM20948_BUS_I2C_EXTERNAL
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif



/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ak09916_main(int argc, char *argv[]);



/**
 * Local functions in support of the shell command.
 */
namespace ak09916
{

/*
  list of supported bus configurations
 */
struct ak09916_bus_option {
    enum AK09916_BUS busid;
    const char *devpath;
    AK09916_constructor interface_constructor;
    uint8_t busnum;
    AK09916 *dev;
} bus_options[] = {
    { AK09916_BUS_I2C_EXTERNAL, "/dev/ak09916_ext", &AK09916_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_EXPANSION1
    { AK09916_BUS_I2C_EXTERNAL, "/dev/ak09916_ext1", &AK09916_I2C_interface, PX4_I2C_BUS_EXPANSION1, NULL },
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
    { AK09916_BUS_I2C_EXTERNAL, "/dev/ak09916_ext2", &AK09916_I2C_interface, PX4_I2C_BUS_EXPANSION2, NULL },
#endif
#ifdef PX4_I2C_BUS_ONBOARD
    { AK09916_BUS_I2C_INTERNAL, "/dev/ak09916_int", &AK09916_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void    start(enum AK09916_BUS busid, enum Rotation rotation);
int     stop();
bool    start_bus(struct ak09916_bus_option &bus, enum Rotation rotation);
struct 	ak09916_bus_option &find_bus(enum AK09916_BUS busid);
void    reset(enum AK09916_BUS busid);
int 	info(enum AK09916_BUS busid);
void    usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct ak09916_bus_option &bus, enum Rotation rotation)
{
    if (bus.dev != nullptr) {
        errx(1, "bus option already started");
    }

    device::Device *interface = bus.interface_constructor(bus.busnum);

    if (interface->init() != OK) {
        delete interface;
        PX4_WARN("no device on bus %u (type: %u)", (unsigned)bus.busnum, (unsigned)bus.busid);
        return false;
    }

    bus.dev = new AK09916(interface, bus.devpath, rotation);

    if (bus.dev != nullptr && OK != bus.dev->init()) {
        delete bus.dev;
        bus.dev = NULL;
        return false;
    }

    int fd = open(bus.devpath, O_RDONLY);

    if (fd < 0) {
        return false;
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        close(fd);
        errx(1, "Failed to setup poll rate");
    }

    close(fd);

    return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum AK09916_BUS busid, enum Rotation rotation)
{
    bool started = false;

    for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
        if (busid == AK09916_BUS_ALL && bus_options[i].dev != NULL) {
            // this device is already started
            continue;
        }

        if (busid != AK09916_BUS_ALL && bus_options[i].busid != busid) {
            // not the one that is asked for
            continue;
        }

        started |= start_bus(bus_options[i], rotation);
    }

    if (!started) {
        exit(1);
    }
}

int
stop()
{
    bool stopped = false;

    for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
        if (bus_options[i].dev != nullptr) {
            bus_options[i].dev->stop();
            delete bus_options[i].dev;
            bus_options[i].dev = nullptr;
            stopped = true;
        }
    }

    return !stopped;
}

/**
 * find a bus structure for a busid
 */
struct ak09916_bus_option &find_bus(enum AK09916_BUS busid)
{
    for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
        if ((busid == AK09916_BUS_ALL ||
             busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
            return bus_options[i];
        }
    }

    errx(1, "bus %u not started", (unsigned)busid);
}





/**
 * Reset the driver.
 */
void
reset(enum AK09916_BUS busid)
{
    struct ak09916_bus_option &bus = find_bus(busid);
    const char *path = bus.devpath;

    int fd = open(path, O_RDONLY);

    if (fd < 0) {
        err(1, "failed ");
    }

    if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
        err(1, "driver reset failed");
    }

    if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
        err(1, "driver poll restart failed");
    }

    exit(0);
}



/**
 * Print a little info about the driver.
 */
int
info(enum AK09916_BUS busid)
{
    struct ak09916_bus_option &bus = find_bus(busid);

    PX4_WARN("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
    bus.dev->print_info();
    exit(0);
}

void
usage()
{
    PX4_INFO("missing command: try 'start', 'info', 'test', 'reset', 'info'");
    PX4_INFO("options:");
    PX4_INFO("    -R rotation");
    PX4_INFO("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)
    PX4_INFO("    -I only internal bus");
#endif
}

} // namespace

int
ak09916_main(int argc, char *argv[])
{
    int ch;
    enum AK09916_BUS busid = AK09916_BUS_ALL;
    enum Rotation rotation = ROTATION_NONE;

    if (argc < 2) {
        ak09916::usage();
        exit(0);
    }

    while ((ch = getopt(argc, argv, "XIR:CT")) != EOF) {
        switch (ch) {
        case 'R':
            rotation = (enum Rotation)atoi(optarg);
            break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_HMC)

        case 'I':
            busid = AK09916_BUS_I2C_INTERNAL;
            break;
#endif

        case 'X':
            busid = AK09916_BUS_I2C_EXTERNAL;
            break;

        default:
            ak09916::usage();
            exit(0);
        }
    }

    const char *verb = argv[optind];

    /*
     * Start/load the driver.
     */
    if (!strcmp(verb, "start")) {
        ak09916::start(busid, rotation);

        exit(0);
    }

    /*
     * Stop the driver
     */
    if (!strcmp(verb, "stop")) {
        return ak09916::stop();
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(verb, "reset")) {
        ak09916::reset(busid);
    }

    /*
     * Print driver information.
     */
    if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
        ak09916::info(busid);
    }

    errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
