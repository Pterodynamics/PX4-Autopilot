/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file float16_util.hpp
 *
 * Shared float32 <-> float16 bit-pattern conversion for the UAVCAN actuator
 * bridges (esc.cpp, servo.cpp) that pack telemetry into MAVLink TUNNEL
 * payloads. Previously duplicated as a local static function in both files;
 * consolidated here so the two stay in sync.
 */

#pragma once

#include <cstdint>
#include <cstring>

/**
 * Converts a native float32 to its IEEE-754 binary16 (half-precision) bit
 * pattern via ARM's __fp16 storage type. __fp16 is ARM GCC's long-standing
 * half-precision extension (predating the newer _Float16 standard spelling,
 * which some toolchain versions don't recognize) -- it's storage-only by
 * design: any arithmetic on a __fp16 value is promoted to float first, which
 * is exactly the role it plays here (conversion target + bit-pattern source,
 * no arithmetic performed on it). On targets without a hardware FP16 unit
 * (e.g. Cortex-M7), the conversion itself still lowers to a libgcc soft-float
 * routine that correctly round-to-nearest-evens, handles subnormals, and
 * propagates Inf/NaN.
 *
 * marked inline (not static) since this header may be included by more than
 * one translation unit -- inline avoids an ODR violation that a plain
 * non-static free function definition in a header would cause.
 */
inline uint16_t float_to_float16_bits(float value)
{
	__fp16 half = value; // narrowing conversion, promoted/demoted via libgcc soft-float routine

	uint16_t bits;
	static_assert(sizeof(bits) == sizeof(half), "__fp16 must be 16 bits on this target.");
	memcpy(&bits, &half, sizeof(bits));

	return bits;
}

inline size_t pack_float16_le(uint8_t *payload, size_t offset, float value)
{
	uint16_t bits = float_to_float16_bits(value);

	payload[offset++] = static_cast<uint8_t>(bits & 0xFF);
	payload[offset++] = static_cast<uint8_t>((bits >> 8) & 0xFF);

	return offset;
}
