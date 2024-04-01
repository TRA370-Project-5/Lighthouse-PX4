/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#ifndef PX4_RDDRONE_H
#define PX4_RDDRONE_H

#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <perf/perf_counter.h>
#include <lib/conversion/rotation.h>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/sensor_lighthouse.h>
#include <uORB/topics/parameter_update.h>

extern "C" {
#include "pulse_processor.h"
#include "pulse_processor_v1.h"
#include "pulse_processor_v2.h"

#include "lighthouse_position_est.h"
}
#define LIGHTHOUSE_DEFAULT_PORT "/dev/ttyS1"

using namespace time_literals;

typedef enum {
  statusNotReceiving = 0,
  statusMissingData = 1,
  statusToEstimator = 2,
} lhSystemStatus_t;

typedef struct {
  bool isSyncFrame;
  pulseProcessorFrame_t data;
} lighthouseUartFrame_t;

typedef struct {
  int sampleCount;
  int hitCount;
} lighthouseBsIdentificationData_t;

static void deckHealthCheck(pulseProcessor_t *appState, const lighthouseUartFrame_t* frame, const uint32_t now_ms);
static void lighthouseUpdateSystemType(void);
static void processFrame(pulseProcessor_t *appState, pulseProcessorResult_t* angles, const lighthouseUartFrame_t* frame, const uint32_t now_ms);
static void usePulseResult(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation, int sweepId, const uint32_t now_ms);
static void useCalibrationData(pulseProcessor_t *appState);
static void updateSystemStatus(const uint32_t now_ms);

void lighthouseCoreSetCalibrationData(const uint8_t baseStation, const lighthouseCalibration_t* calibration);
static void convertV2AnglesToV1Angles(pulseProcessorResult_t* angles);
bool throttleLh2Samples(const uint32_t nowMs);
static void usePulseResultSweeps(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation);

class LH_BC : public ModuleBase<LH_BC>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LH_BC(const char *port);
	~LH_BC();

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	bool init();

	void start();

	void stop();

private:

	void parameters_update();

	void Run() override;
	bool getUartFrameRaw(lighthouseUartFrame_t *data);
	void waitForUartSynchFrame();

	// Publications
	uORB::Publication<sensor_lighthouse_s> _sensor_lighthouse_pub{ORB_ID(sensor_lighthouse)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_lighthouse_sub{this, ORB_ID(sensor_lighthouse)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LH_PORT_CFG>) 			_lighthouse_port_cfg,
		(ParamFloat<px4::params::LH_INIT_OFF_X>) 		_offset_x,
		(ParamFloat<px4::params::LH_INIT_OFF_Y>) 		_offset_y,
		(ParamFloat<px4::params::LH_INIT_OFF_Z>) 		_offset_z,
		(ParamInt<px4::params::LH_SENS_ROT>) 			_sensor_rot
	)
	// Performance (perf) counters
	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;

	sensor_lighthouse_s _sensor_lighthouse{};

	char _port[20] {};
	hrt_abstime param_timestamp{0};

	int _uart{-1};
	fd_set _uart_set;
	struct timeval _uart_timeout {};
	

	unsigned _consecutive_fail_count;
	int _interval{100000};
};
#endif //PX4_RDDRONE_H
