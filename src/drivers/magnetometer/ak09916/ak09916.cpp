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
 * @file ak09916.cpp
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
#include "ak09916_i2c.h"







AK09916::AK09916(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("AK09916", path),
	my_path(path),
	_interface(interface),
	_measure_ticks(0),
	_reports(nullptr),
	_scale{},
	_range_scale(0.15f), /* default range scale from counts to gauss - 0.15uT/LSB fixed (AK09916 datasheet) */
	_range_ga(49.12f), /* Taken from AK09916 datasheet (Typ. +-4912uT) */
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "AK09916_read")),
	_comms_errors(perf_alloc(PC_COUNT, "AK09916_com_err")),
	_range_errors(perf_alloc(PC_COUNT, "AK09916_rng_err")),
	_conf_errors(perf_alloc(PC_COUNT, "AK09916_conf_err")),
	_sensor_ok(false),
	_rotation(rotation),
	_range_bits(0),
	_control_reg(0),
	_temperature_counter(0),
	_temperature_error_count(0)
{
	// set the device type from the interface
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_AK09916;

	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

AK09916::~AK09916()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
AK09916::init()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		PX4_WARN("CDev init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		return ret;
	}

	/* reset the device configuration */
	ret = reset();
	if (ret != OK) {
		PX4_WARN("AK09916 reset failed");
		return ret;
	}

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	start();
	return OK;
}



/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
AK09916::test()
{
//    struct ak09916_bus_option &bus = find_bus(busid);
    struct mag_report report;
    ssize_t sz;
    int ret;

    int fd = ::open(my_path, O_RDONLY);

    if (fd < 0) {
        err(1, "%s open failed (try 'ak09916 start')", my_path);
    }

    /* do a simple demand read */
    sz = ::read(fd, &report, sizeof(report));

    if (sz != sizeof(report)) {
        err(1, "immediate read failed");
    }

    print_message(report);

    /* check if mag is onboard or external */
    if ((ret = ::ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
        errx(1, "failed to get if mag is onboard or external");
    }

    PX4_WARN("device active: %s", ret ? "external" : "onboard");

    /* set the queue depth to 5 */
    if (OK != ::ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
        errx(1, "failed to set queue depth");
    }

    /* start the sensor polling at 2Hz */
    if (OK != ::ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
        errx(1, "failed to set 2Hz poll rate");
    }

    /* read the sensor 5x and report each value */
    for (unsigned i = 0; i < 5; i++) {
        struct pollfd fds;

        /* wait for data to be ready */
        fds.fd = fd;
        fds.events = POLLIN;
        ret = ::poll(&fds, 1, 2000);

        if (ret != 1) {
            errx(1, "timed out waiting for sensor data");
        }

        /* now go get it */
        sz = ::read(fd, &report, sizeof(report));

        if (sz != sizeof(report)) {
            err(1, "periodic read failed");
        }

        print_message(report);
    }

    errx(0, "PASS");
}


/**
   Perform the AK09916C self test
 */
int AK09916::selftest(void)
{
    int ret;
//    uint8_t read_val = 0;

    struct PACKED {
        uint8_t st1;
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t tmps;
        uint8_t st2;
    } report;

	report.x=0;
	report.y=0;
	report.z=0;

    ret = write_reg(ADDR_CNTL2, CNTL2_REG_POWERDOWN_MODE);
    if (OK != ret) {
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    ret = write_reg(ADDR_CNTL2, CNTL2_REG_SELFTEST_MODE);
    if (OK != ret) {
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    /*
     * Wait a bit for the selftest result. No mention in the datasheet, but this might take
     * longer than a normal measurement.
     */
    for(uint8_t i=0; i<5; i++) {
        usleep(AK09916_CONVERSION_INTERVAL);
        ret = _interface->read(ADDR_ST1, (uint8_t *)&report, sizeof(report));
        if (OK != ret) {
            perf_count(_comms_errors);
            PX4_ERR("READ ERROR");
            return PX4_ERROR;
        }
        if (report.st1 & ST1_REG_DRDY) break;
    }

    if ( (report.st1 & ST1_REG_DRDY) == 0 ) {
        PX4_WARN("AK09916 selftest timed out");
        return PX4_ERROR;
    }


    /*
     * The selftest function as specified in the AK09916 datasheet failed,
     * sometimes with large margins, on two Here GPS modules in a row
     * during development, but they seem to be working fine nevertheless.
     * Maybe the module needs more time after initialization for this to work.
     * Will also try again in another environment and with another Here GPS
     * as soon as possible. Always returning OK for now.
     */
    PX4_INFO("Selftest values: X %d, Y %d, Z %d", report.x, report.y, report.z);
    if(report.x > -200 && report.x < 200 &&
       report.y > -200 && report.y < 200 &&
       report.z > -1000 && report.z < 200)
    {
        PX4_INFO("AK09916 selftest OK");
        return OK;
    }
    else {
        PX4_INFO("AK09916 selftest FAIL");
//        return PX4_ERROR;
        return OK;
    }
}

ssize_t
AK09916::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(AK09916_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int
AK09916::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	unsigned dummy = arg;

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(AK09916_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(AK09916_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / TICK2USEC(_measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCGRANGE:
		return _range_ga;

    case MAGIOCSSCALE:
        // Scales can't be changed. Need this ioctl for the calibration routing.
        return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((struct mag_calibration_s *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCSELFTEST:
		return selftest();

	case MAGIOCGEXTERNAL:
		PX4_WARN("MAGIOCGEXTERNAL in main driver");
		return _interface->ioctl(cmd, dummy);

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
AK09916::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&AK09916::cycle_trampoline, this, 1);
}

void
AK09916::stop()
{
	if (_measure_ticks > 0) {
		/* ensure no new items are queued while we cancel this one */
		_measure_ticks = 0;
		work_cancel(HPWORK, &_work);
	}
}

int
AK09916::reset()
{
    int ret = PX4_ERROR;

    ret = write_reg(ADDR_CNTL3, CNTL3_REG_SRST);

	return ret;
}

void
AK09916::cycle_trampoline(void *arg)
{
	AK09916 *dev = (AK09916 *)arg;

	dev->cycle();
}

void
AK09916::cycle()
{
	if (_measure_ticks == 0) {
		return;
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_WARN("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(AK09916_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&AK09916::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(AK09916_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_WARN("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	if (_measure_ticks > 0) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&AK09916::cycle_trampoline,
			   this,
			   USEC2TICK(AK09916_CONVERSION_INTERVAL));
	}
}

int
AK09916::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_CNTL2, CNTL2_REG_SINGLE_MODE);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
AK09916::collect()
{
    struct PACKED {
        uint8_t st1;
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t tmps;
        uint8_t st2;
    } report;

    report.x=0;
    report.y=0;
    report.z=0;

//	uint8_t st2_buffer;

	int	ret;

	perf_begin(_sample_perf);
	struct mag_report new_report;
	bool sensor_is_onboard = false;

	float xraw_f;
	float yraw_f;
	float zraw_f;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
	new_report.error_count = perf_event_count(_comms_errors);
	new_report.range_ga = _range_ga;
	new_report.scaling = _range_scale;
	new_report.device_id = _device_id.devid;

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	ret = _interface->read(ADDR_ST1, (uint8_t *)&report, sizeof(report));
	if (ret != OK) {
		perf_count(_comms_errors);
		goto out;
	}
	else if ( (report.st1 & ST1_REG_DRDY) == 0 ) {
	    /*
	     * Data was not ready - warn and skip sample. A way to count these
	     * events might be good.
	     */
        PX4_WARN("data/status read error");
        goto out;
	}

	/*
	 * RAW outputs
	 *
	 * Flip axes so they give the same result as the Pixhawk 2.1 onboard sensors.
	 * TODO: Double check this
	 */
	new_report.x_raw = -1 * report.y;
	new_report.y_raw = -1 * report.x;
	new_report.z_raw = -1 * report.z;

    xraw_f = new_report.x_raw;
    yraw_f = new_report.y_raw;
    zraw_f = new_report.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
	new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

	if (!(_pub_blocked)) {

		if (_mag_topic != nullptr) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);

		} else {
			_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
							 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

			if (_mag_topic == nullptr) {
				PX4_WARN("ADVERT FAIL");
			}
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	_reports->force(&new_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int
AK09916::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
AK09916::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

void
AK09916::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	print_message(_last_report);
	_reports->print_info("report queue");
}


