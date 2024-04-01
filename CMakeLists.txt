############################################################################
#
#   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(CMSIS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_5)
set(CMSIS_DSP ${CMSIS_ROOT}/CMSIS/DSP)

if(${PX4_PLATFORM} MATCHES "NuttX")
	add_compile_options(-DARM_MATH_DSP)
endif()

# Disable 32-bit assembly warnings on apple silicon. Triggered by unused code only.
if(${PX4_PLATFORM} MATCHES "posix" AND APPLE AND ${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm64")
	add_compile_options(-Wno-asm-operand-widths)
endif()

add_compile_options($<$<COMPILE_LANGUAGE:C>:-Wno-nested-externs>)

px4_add_module(
		MODULE drivers__lighthouse_bitcraze
		MAIN lighthouse_bitcraze

		INCLUDES
			${CMSIS_ROOT}/CMSIS/Core/Include
			${CMSIS_DSP}/Include

		SRCS
			lighthouse_bitcraze.cpp
			cfassert.c
			crc32.c
			lighthouse_calibration.c
			lighthouse_geometry.c
			lighthouse_position_est.c
			module.yaml
			ootx_decoder.c
			pulse_processor_v1.c
			pulse_processor_v2.c
			pulse_processor.c

			cf_math.h
			cfassert.h
			crc32.h
			lighthouse_bitcraze.h
			lighthouse_calibration.h
			lighthouse_geometry.h
			lighthouse_position_est.h
			lighthouse_types.h
			ootx_decoder.h
			lighthouse_state.h
			stabilizer_types.h
			pulse_processor.h
			pulse_processor_v2.h
			pulse_processor_v1.h
			physicalConstants.h


			${CMSIS_DSP}/Source/
			${CMSIS_DSP}/Source/
			${CMSIS_DSP}/Source/
			${CMSIS_DSP}/Source/
			${CMSIS_DSP}/Source/

		MODULE_CONFIG
			module.yaml
		DEPENDS
			px4_work_queue
)
