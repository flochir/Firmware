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
 * @file icm20948.h
 *
 * Shared defines for the icm20948 driver.
 */

#ifndef _ICM20948_H
#define _ICM20948_H

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

#include <drivers/magnetometer/ak09916/ak09916.h>



#pragma once



/*
 * ICM20948 internal constants and data structures.
 */

#define ADDR_ID_ICM         0x00
#define ID_ICM_WHO_AM_I     0xEA

#define ADDR_PWR_MGMT_1                 0x06
#define REG_PWR_MGMT_1_RESET            0x80
#define REG_PWR_MGMT_1_ENABLE           0x00

#define ADDR_INT_PIN_CFG                0x0F
#define REG_INT_PIN_CFG_BYPASS          0x02

#define ADDR_USER_CTRL                  0x03
#define REG_USER_CTRL_I2C_MST_DISABLE   0x00



#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif





/* interface factories */
extern device::Device *ICM20948_I2C_interface(int bus);
typedef device::Device *(*ICM20948_constructor)(int);



/*
 * Can be extended to inherit device::CDev to provide accel and gyro. For now only configures the partent
 * device to pass through the magnetometer.
 */
class ICM20948
{
public:
    ICM20948(device::Device *interface, const char *path_mag, enum Rotation rotation);
    virtual ~ICM20948();

    virtual int     init();

    /**
     * Diagnostics - print some basic information about the driver.
     */
    void            print_info();

    /*
     * Stop the device
     */
    void         stop();

    /*
     * Start the device
     */
    void         start();

    /**
     * Reset the device
     */
    int         reset();

    /*
     * perform the selftest routines
     */
    int			selftest();

protected:
    device::Device          *_interface;

private:

    AK09916     *_ak09916_mag;
    const char  *_path_mag;

    perf_counter_t      _sample_perf;
    perf_counter_t      _comms_errors;
    perf_counter_t      _range_errors;
    perf_counter_t      _conf_errors;

    enum Rotation       _rotation;

    /**
     * Reset the magnetometer submodule
     */
    int         ak09916_reset();

    /*
     * Configure device, enable I2C passthrough to AK09916
     */
    int         configure();

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

    /* this class has pointer data members, do not allow copying it */
    ICM20948(const ICM20948 &);
    ICM20948 operator=(const ICM20948 &);
};

#endif
