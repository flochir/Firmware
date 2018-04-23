/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file ak09916.h
 *
 * Shared defines for the ak09916 driver.
 */

#ifndef _AK09916_H
#define _AK09916_H

#include <stdint.h>

#include <systemlib/perf_counter.h>
#include <systemlib/conversions.h>

#include <nuttx/wqueue.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>



#pragma once

/*
 * AK09916 -> AK09916 internal constants and data structures
 */



#define ADDR_ID_A           0x00
#define ADDR_ID_B           0x01

#define ID_A_WHO_AM_I           0x48
#define ID_B_WHO_AM_I           0x09

/*
 * Single measurement mode conversion time is typically 7.2, maximal 8.2ms according to the AK09916 datasheet.
 * Using 8.2 ms (~120Hz)  to be safe.
 */

#define AK09916_CONVERSION_INTERVAL (1000000 / 100) /* microseconds */

#define ADDR_HXL        0x11
#define ADDR_HXH        0x12
#define ADDR_HYL        0x13
#define ADDR_HYH        0x14
#define ADDR_HZL        0x15
#define ADDR_HZH        0x16

#define ADDR_ST1        0x10
#define ADDR_ST2        0x18

#define ADDR_CNTL2          0x31
#define ADDR_CNTL3          0x32

#define CNTL2_REG_POWERDOWN_MODE            0x00
#define CNTL2_REG_SINGLE_MODE               0x01 /* default */
#define CNTL2_REG_CONTINOUS_MODE_10HZ       0x02
#define CNTL2_REG_CONTINOUS_MODE_20HZ       0x04
#define CNTL2_REG_CONTINOUS_MODE_50HZ       0x06
#define CNTL2_REG_CONTINOUS_MODE_100HZ      0x08
#define CNTL2_REG_SELFTEST_MODE             0x10

#define CNTL3_REG_SRST                      0x01

#define ST1_REG_DRDY                        0x01
#define ST1_REG_DOR                         0x02

enum AK09916_BUS {
    AK09916_BUS_ALL = 0,
    AK09916_BUS_I2C_INTERNAL,
    AK09916_BUS_I2C_EXTERNAL
};

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


/* interface factories */
extern device::Device *AK09916_I2C_interface(int bus);
typedef device::Device *(*AK09916_constructor)(int);



class AK09916 : public device::CDev
{
public:
    AK09916(device::Device *interface, const char *path, enum Rotation rotation);
    virtual ~AK09916();

    virtual int     init();

    const char  *my_path;


    virtual ssize_t     read(struct file *filp, char *buffer, size_t buflen);
    virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
     * Initialise the automatic measurement state machine and start it.
     *
     * @note This function is called at open and error time.  It might make sense
     *       to make it more aggressive about resetting the bus in case of errors.
     */
    void            start();

    void            test();

    /**
     * Stop the automatic measurement state machine.
     */
    void            stop();

    /**
     * Reset the device
     */
    int         reset();

    /**
     * check the sensor range.
     *
     * checks that the range of the sensor is correctly set, to
     * cope with communication errors causing the range to change
     */
    void            check_range(void);

    /**
     * perform the AK09916C's selftest feature
     */
    int            selftest(void);

    /**
     * Diagnostics - print some basic information about the driver.
     */
    void            print_info();

protected:
    Device          *_interface;

private:
    work_s          _work{};
    unsigned        _measure_ticks;

    ringbuffer::RingBuffer  *_reports;
    struct mag_calibration_s    _scale;
    float           _range_scale;
    float           _range_ga;
    bool            _collect_phase;
    int         _class_instance;
    int         _orb_class_instance;

    orb_advert_t        _mag_topic;

    perf_counter_t      _sample_perf;
    perf_counter_t      _comms_errors;
    perf_counter_t      _range_errors;
    perf_counter_t      _conf_errors;

    /* status reporting */
    bool            _sensor_ok;     /**< sensor was found and reports ok */

    enum Rotation       _rotation;

    struct mag_report   _last_report {};         /**< used for info() */

    uint8_t         _range_bits;
    uint8_t         _control_reg;
    uint8_t         _temperature_counter;
    uint8_t         _temperature_error_count;

    /**
     * Perform a poll cycle; collect from the previous measurement
     * and start a new one.
     *
     * This is the heart of the measurement state machine.  This function
     * alternately starts a measurement, or collects the data from the
     * previous measurement.
     *
     * When the interval between measurements is greater than the minimum
     * measurement interval, a gap is inserted between collection
     * and measurement to provide the most recent measurement possible
     * at the next interval.
    AK09916_BUS_SPI
     */
    void            cycle();

    /**
     * Static trampoline from the workq context; because we don't have a
     * generic workq wrapper yet.
     *
     * @param arg       Instance pointer for the driver that is polling.
     */
    static void     cycle_trampoline(void *arg);

    /**
     * Write a register.
     *
     * @param reg       The register to write.
     * @param val       The value to write.
     * @return      OK on write success.
     */
    int         write_reg(uint8_t reg, uint8_t val);

    /**
     * Read a register.
     *
     * @param reg       The register to read.
     * @param val       The value read.
     * @return      OK on read success.
     */
    int         read_reg(uint8_t reg, uint8_t &val);

    /**
     * Issue a measurement command.
     *
     * @return      OK if the measurement command was successful.
     */
    int         measure();

    /**
     * Collect the result of the most recent measurement.
     */
    int         collect();

    /* this class has pointer data members, do not allow copying it */
    AK09916(const AK09916 &);
    AK09916 operator=(const AK09916 &);
};

#endif
