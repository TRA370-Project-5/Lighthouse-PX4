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


/* This is a driver for the NXP SR150 UWB Chip on MK UWB Shield 2
 *	This Driver handles the Communication to the UWB Board.
 *	For Information about HW and SW contact Mobile Knowledge:
 *	https://www.themobileknowledge.com
 * */

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>

#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include <string.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "lighthouse_bitcraze.h"


static const uint32_t MAX_WAIT_TIME_FOR_HEALTH_MS = 4000;

// https://www.bitcraze.io/documentation/repository/lighthouse-bootloader/master/
// https://github.com/bitcraze/lighthouse-fpga

// The default baudrate of the lighthouse_bitcraze module before configuration
#define DEFAULT_BAUD B230400

#define UART_FRAME_LENGTH 12

#define ONE_SECOND 1000
#define HALF_SECOND 500
#define FIFTH_SECOND 200

#define DECK_LIGHTHOUSE_MAX_N_BS 4

#define BYTE_TIMEOUT_US 5000

// Amount of time to wait for a new message. If more time than this passes between messages, then this
// driver assumes that the UWB_SR150 module is disconnected.
// (Right now it does not do anything about this)
#define MESSAGE_TIMEOUT_S 10  //wait 10 seconds.
#define MESSAGE_TIMEOUT_US 1

static bool uartSynchronized = false;
static bool previousWasSyncFrame = false;

static uint8_t estimationMethod = 1;

static pulseProcessorResult_t angle;

static const uint32_t evaluationIntervalMs = 100;
static uint16_t maxRate = 50;  // Samples / second
static float discardProbability = 0.0f;

static uint16_t baseStationAvailabledMapWs;
static uint16_t baseStationAvailabledMap;

// A bitmap that indicates which base staions that are received
static uint16_t baseStationReceivedMapWs;
static uint16_t baseStationReceivedMap;

// A bitmap that indicates which base staions that are actively used in the estimation process, that is receiving sweeps
// as well a has valid geo and calib data
static uint16_t baseStationActiveMapWs;
static uint16_t baseStationActiveMap;

// A bitmap that indicates which base stations that have received calibration data over the air
static uint16_t baseStationCalibConfirmedMap;

// A bitmap that indicates which base stations that have received calibration data that was different to what was stored in memory
static uint16_t baseStationCalibUpdatedMap;

static uint8_t calibStatusReset;

// An overall system status indicating if data is sent to the estimator
static lhSystemStatus_t systemStatus;
static lhSystemStatus_t systemStatusWs;

static const uint32_t SYSTEM_STATUS_UPDATE_INTERVAL = FIFTH_SECOND;
static uint32_t nextUpdateTimeOfSystemStatus = 0;

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];
pulseProcessor_t lighthouseCoreState;

static lighthouseBaseStationType_t systemType = lighthouseBsTypeV2;
static lighthouseBaseStationType_t previousSystemType = lighthouseBsTypeV2;
static pulseProcessorProcessPulse_t pulseProcessorProcessPulse = pulseProcessorV2ProcessPulse;


static void modifyBit(uint16_t *bitmap, const int index, const bool value) {
	const uint16_t mask = (1 << index);

	if (value) {
		*bitmap |= mask;
	} else {
		*bitmap &= ~mask;
	}
}

extern "C" __EXPORT int lighthouse_bitcraze_main(int argc, char *argv[]);

LH_BC::LH_BC(const char *port):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_read_count_perf(perf_alloc(PC_COUNT, "lighthouse_bitcraze_count")),
	_read_err_perf(perf_alloc(PC_COUNT, "lighthouse_bitcraze_err")) {

	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

LH_BC::~LH_BC(){
	printf("LIGHTHOUSE: Ranging Stopped\t\n");

	// stop{}; will be implemented when this is changed to a scheduled work task
	perf_free(_read_err_perf);
	perf_free(_read_count_perf);

	close(_uart);
}

bool LH_BC::init(){
	// execute Run() on every sensor_accel publication
	if (!_sensor_lighthouse_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void LH_BC::start(){
	/* schedule a cycle to start things */
	ScheduleNow();
}

void LH_BC::stop(){
	ScheduleClear();
}

void LH_BC::Run(){
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if UART is closed for some reason
	if (_uart < 0) {
		/* open fd */
		_uart = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_uart < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_uart, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		//TODO: should I keep this?
		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		unsigned speed = DEFAULT_BAUD;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
	}

	if(!uartSynchronized){
		waitForUartSynchFrame();
		uartSynchronized = true;
	}

	lighthouseUartFrame_t frame;

	bool isUartFrameValid = getUartFrameRaw(&frame);

	if(!isUartFrameValid){
		uartSynchronized = false;
		return;
	}

	const uint32_t now_ms = hrt_absolute_time();

	// If a sync frame is getting through, we are only receiving sync frames. So nothing else. Reset state
	if(frame.isSyncFrame && previousWasSyncFrame){
		pulseProcessorAllClear(&angle);
	}

	else if (!frame.isSyncFrame){
		deckHealthCheck(&lighthouseCoreState, &frame, now_ms);
		lighthouseUpdateSystemType();

		if (pulseProcessorProcessPulse) {
        	processFrame(&lighthouseCoreState, &angle, &frame, now_ms);
        }
	}

	previousWasSyncFrame = frame.isSyncFrame;

	updateSystemStatus(now_ms);
}

int LH_BC::custom_command(int argc, char *argv[]){
	return print_usage("Unrecognized command.");
}

int LH_BC::print_usage(const char *reason){
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("lighthouse", "driver");
	PRINT_MODULE_DESCRIPTION(R"DESC_STR(
### Description

Driver for NXP UWB_SR150 UWB positioning system. This driver publishes a `lighthouse_distance` message
whenever the UWB_SR150 has a position measurement available.

### Example

Start the driver with a given device:

$ lighthouse start -d /dev/ttyS2
	)DESC_STR");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Name of device for serial communication with LIGHTHOUSE", false);
	PRINT_MODULE_USAGE_PARAM_STRING('b', nullptr, "<int>", "Baudrate for serial communication", false);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

int LH_BC::task_spawn(int argc, char *argv[]){
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = LIGHTHOUSE_DEFAULT_PORT;
	int baudrate = 0;

	while ((ch = px4_getopt(argc, argv, "d:b", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = option_arg;
			break;

		case 'b':
			px4_get_parameter_value(option_arg, baudrate);
			break;

		default:
			PX4_WARN("Unrecognized flag: %c", ch);
			break;
		}
	}

	LH_BC *instance = new LH_BC(device_name);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleOnInterval(5000_us);

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}



void LH_BC::parameters_update(){
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}


int lighthouse_bitcraze_main(int argc, char *argv[]){
	return LH_BC::main(argc, argv);
}

bool LH_BC::getUartFrameRaw(lighthouseUartFrame_t *frame) {
	static char buffer[UART_FRAME_LENGTH];
	int syncCounter = 0;

	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);
	_uart_timeout.tv_sec = MESSAGE_TIMEOUT_S ;
	_uart_timeout.tv_usec = MESSAGE_TIMEOUT_US;

	size_t buffer_location = 0;

	while (buffer_location < sizeof(buffer) && select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout) > 0) {

		int bytes_read = read(_uart, &buffer[buffer_location], UART_FRAME_LENGTH - buffer_location);

		if (bytes_read > 0) {
			buffer_location += bytes_read;

		} else {
			break;
		}

		FD_ZERO(&_uart_set);
		FD_SET(_uart, &_uart_set);
		_uart_timeout.tv_sec = 0;
		// Setting this timeout too high (> 37ms) will cause problems because the next message will start
		//  coming in, and overlap with the current message.
		// Setting this timeout too low (< 1ms) will cause problems because there is some delay between
		//  the individual bytes of a message, and a too-short timeout will cause the message to be truncated.
		// The current value of 5ms was found experimentally to never cut off a message prematurely.
		// Strictly speaking, there are no downsides to setting this timeout as high as possible (Just under 37ms),
		// because if this process is waiting, it means that the last message was incomplete, so there is no current
		// data waiting to be published. But we would rather set this timeout lower in case the UWB_SR150 board is
		// updated to publish data faster.
		_uart_timeout.tv_usec = BYTE_TIMEOUT_US;
	}

	// Check if sync frame
	for(int i = 0; i < UART_FRAME_LENGTH; i++) {
		if ((unsigned char)buffer[i] == 0xff) {
			syncCounter += 1;
		}
	}

	memset(frame, 0, sizeof(*frame));

	frame->isSyncFrame = (syncCounter == UART_FRAME_LENGTH);

	frame->data.sensor = buffer[0] & 0x03;
	frame->data.channelFound = (buffer[0] & 0x80) == 0;
	frame->data.channel = (buffer[0] >> 3) & 0x0f;
	frame->data.slowBit = (buffer[0] >> 2) & 0x01;
	memcpy(&frame->data.width, &buffer[1], 2);
	memcpy(&frame->data.offset, &buffer[3], 3);
	memcpy(&frame->data.beamData, &buffer[6], 3);
	memcpy(&frame->data.timestamp, &buffer[9], 3);

	// Offset is expressed in a 6 MHz clock, convert to the 24 MHz that is used for timestamps
	frame->data.offset *= 4;

	bool isPaddingZero = (((buffer[5] | buffer[8]) & 0xfe) == 0);
	bool isFrameValid = (isPaddingZero || frame->isSyncFrame);

	return isFrameValid;
}

static void lighthouseUpdateSystemType() {
	// Switch to new pulse processor
	switch (systemType) {
		case lighthouseBsTypeV1:
			pulseProcessorProcessPulse = pulseProcessorV1ProcessPulse;
			baseStationAvailabledMapWs = 3;

			break;
		case lighthouseBsTypeV2:
			pulseProcessorProcessPulse = pulseProcessorV2ProcessPulse;
			for (int i = 0; i < DECK_LIGHTHOUSE_MAX_N_BS; i++){
				modifyBit(&baseStationAvailabledMapWs, i, true);
			}
			break;
		default:
			// Do nothing if the type is not in range, stay on the previous processor
			return;
	}

	if (previousSystemType != systemType) {
		previousSystemType = systemType;

		// Clear state
		memset(&lighthouseCoreState, 0, sizeof(lighthouseCoreState));
	}
}

void lighthouseCoreSetSystemType(const lighthouseBaseStationType_t type) {
	systemType = type;
	previousSystemType = type;
	lighthouseUpdateSystemType();
}

static void deckHealthCheck(pulseProcessor_t *appState, const lighthouseUartFrame_t* frame, const uint32_t now_ms) {
	if (!appState->healthDetermined) {
		if (0 == appState->healthFirstSensorTs) {
			appState->healthFirstSensorTs = now_ms;
		}

		if (0x0f == appState->healthSensorBitField) {
			appState->healthDetermined = true;
		} else {
			appState->healthSensorBitField |= (0x01 << frame->data.sensor);

			if ((now_ms - appState->healthFirstSensorTs) > MAX_WAIT_TIME_FOR_HEALTH_MS) {
				appState->healthDetermined = true;
			}
		}
	}
}

static void processFrame(pulseProcessor_t *appState, pulseProcessorResult_t* angles, const lighthouseUartFrame_t* frame, const uint32_t now_ms) {
    int baseStation;
    int sweepId;
    bool calibDataIsDecoded = false;

    pulseWidth[frame->data.sensor] = frame->data.width;

    if (pulseProcessorProcessPulse(appState, &frame->data, angles, &baseStation, &sweepId, &calibDataIsDecoded)) {
        usePulseResult(appState, angles, baseStation, sweepId, now_ms);
    }

    if (calibDataIsDecoded) {
		useCalibrationData(appState);
    }
}

static void usePulseResult(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation, int sweepId, const uint32_t now_ms) {
	const uint16_t baseStationBitMap = (1 << baseStation);
	baseStationReceivedMapWs |= baseStationBitMap;

	if (sweepId == sweepIdSecond) {
		const bool hasCalibrationData = pulseProcessorApplyCalibration(appState, angles, baseStation);
		if (hasCalibrationData) {
			if (lighthouseBsTypeV2 == angles->measurementType) {
				// Emulate V1 base stations, convert to V1 angles
				convertV2AnglesToV1Angles(angles);
			}

			const bool hasGeoData = appState->bsGeometry[baseStation].valid;
			if (hasGeoData) {
				baseStationActiveMapWs |= baseStationBitMap;

				bool useSample = true;
				if (lighthouseBsTypeV2 == angles->measurementType) {
					useSample = throttleLh2Samples(now_ms);
				}

				if (useSample) {
					switch(estimationMethod) {
						case 0:
							//usePulseResultCrossingBeams(appState, angles, baseStation);
							break;
						case 1:
							usePulseResultSweeps(appState, angles, baseStation);
						break;
							default:
						break;
					}
				}
			}
		}

		if (baseStationActiveMapWs != 0) {
			systemStatusWs = statusToEstimator;
		} else {
			systemStatusWs = statusMissingData;
		}
	}
}

static void convertV2AnglesToV1Angles(pulseProcessorResult_t* angles) {
	for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
		for (int bs = 0; bs < DECK_LIGHTHOUSE_MAX_N_BS; bs++) {
			pulseProcessorSensorMeasurement_t* from = &angles->baseStationMeasurementsLh2[bs].sensorMeasurements[sensor];
			pulseProcessorSensorMeasurement_t* to = &angles->baseStationMeasurementsLh1[bs].sensorMeasurements[sensor];

			if (2 == from->validCount) {
				pulseProcessorV2ConvertToV1Angles(from->correctedAngles[0], from->correctedAngles[1], to->correctedAngles);
				to->validCount = from->validCount;
			} else {
				to->validCount = 0;
			}
		}
	}
}


static void usePulseResultSweeps(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int baseStation) {
	pulseProcessorClearOutdated(appState, angles, baseStation);

	lighthousePositionEstimatePoseSweeps(appState, angles, baseStation);

	pulseProcessorProcessed(angles, baseStation);
}


bool throttleLh2Samples(const uint32_t nowMs) {
    static uint32_t previousEvaluationTime = 0;
    static uint32_t nextEvaluationTime = 0;
    static uint32_t eventCounter = 0;
    static int discardThreshold = 0;

    eventCounter++;

    if (nowMs > nextEvaluationTime) {
        const float currentRate = 1000.0f * (float)eventCounter / (float)(nowMs - previousEvaluationTime);
        if (currentRate < (float)maxRate) {
            discardProbability = 0.0;
        } else {
            discardProbability = 1.0f - (float)maxRate / currentRate;
        }
        discardThreshold = RAND_MAX * discardProbability;

        previousEvaluationTime = nowMs;
        eventCounter = 0;
        nextEvaluationTime = nowMs + evaluationIntervalMs;
    }

    return (rand() > discardThreshold);
}

void LH_BC::waitForUartSynchFrame() {
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  while (!synchronized) {
    read(_uart, &c, 1);
    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
    } else {
      syncCounter = 0;
    }
    synchronized = (syncCounter == UART_FRAME_LENGTH);
  }
}

static void useCalibrationData(pulseProcessor_t *appState) {
	for (int baseStation = 0; baseStation < DECK_LIGHTHOUSE_MAX_N_BS; baseStation++) {
		if (appState->ootxDecoder[baseStation].isFullyDecoded) {
			lighthouseCalibration_t newData;
			lighthouseCalibrationInitFromFrame(&newData, &appState->ootxDecoder[baseStation].frame);

			modifyBit(&baseStationCalibConfirmedMap, baseStation, true);

			lighthouseCalibration_t* currentCalibData = &appState->bsCalibration[baseStation];
			const bool currentCalibDataValid = currentCalibData->valid;
			const bool isDataDifferent = ((newData.uid != currentCalibData->uid) || (newData.valid != currentCalibDataValid));
			if (isDataDifferent) {
				lighthouseCoreSetCalibrationData(baseStation, &newData);

				modifyBit(&baseStationCalibUpdatedMap, baseStation, currentCalibDataValid);
			}
		}
	}
}

void lighthouseCoreSetCalibrationData(const uint8_t baseStation, const lighthouseCalibration_t* calibration) {
  if (baseStation < DECK_LIGHTHOUSE_MAX_N_BS) {
    lighthouseCoreState.bsCalibration[baseStation] = *calibration;
    lighthousePositionCalibrationDataWritten(baseStation);
  }
}

static void updateSystemStatus(const uint32_t now_ms) {
  if (now_ms > nextUpdateTimeOfSystemStatus) {
    baseStationAvailabledMap = baseStationAvailabledMapWs;

    baseStationReceivedMap = baseStationReceivedMapWs;
    baseStationReceivedMapWs = 0;

    baseStationActiveMap = baseStationActiveMapWs;
    baseStationActiveMapWs = 0;

    systemStatus = systemStatusWs;
    systemStatusWs = statusNotReceiving;

    if (calibStatusReset) {
      calibStatusReset = 0;
      baseStationCalibUpdatedMap = 0;
    }

    nextUpdateTimeOfSystemStatus = now_ms + SYSTEM_STATUS_UPDATE_INTERVAL;
  }
}