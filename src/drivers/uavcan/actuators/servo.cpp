/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_array_cmd(node),
	_uavcan_sub_status(node)
{
	_uavcan_pub_array_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	reset_payload_to_unknown();
}

int
UavcanServoController::init()
{
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanServoController::actuator_status_sub_cb));

	if (res < 0) {
		PX4_ERR("Actuator status sub failed %i", res);
		return res;
	}

	_actuator_status_pub.advertise();

	_initialized = true;

	return res;
}

static uint16_t float_to_float16_bits(float value)
{
	__fp16 half = value;

	uint16_t bits;
	static_assert(sizeof(bits) == sizeof(half), "__fp16 must be 16 bits on this target.");
	memcpy(&bits, &half, sizeof(bits));

	return bits;
}

static size_t pack_float16_le(uint8_t *payload, size_t offset, float value)
{
	uint16_t bits = float_to_float16_bits(value);

	payload[offset++] = static_cast<uint8_t>(bits & 0xFF);
	payload[offset++] = static_cast<uint8_t>((bits >> 8) & 0xFF);

	return offset;
}

void
UavcanServoController::reset_payload_to_unknown()
{
	for (uint8_t id = 0; id < MAX_ACTUATORS; id++) {
		size_t offset = static_cast<size_t>(id) * ACTUATOR_STATUS_BYTES_PER_ACTUATOR;
		uint8_t *payload = _actuator_status.payload;

		// NOTE: writing the slot index here means an unpopulated record still shows
		// a "plausible" actuator_id -- it's the NaN position/force/speed and the
		// POWER_RATING_PCT_UNKNOWN sentinel below that actually signal "never
		// received," not this byte. Don't infer "this actuator ID is present on the
		// bus" from actuator_id alone; check for NaN in the float fields too.
		payload[offset++] = id;

		float nan_value = __builtin_nanf("");

		offset = pack_float16_le(payload, offset, nan_value); // position
		offset = pack_float16_le(payload, offset, nan_value); // force
		offset = pack_float16_le(payload, offset, nan_value); // speed

		// Reuses the DSDL's own sentinel for "unknown"
		payload[offset++] = uavcan::equipment::actuator::Status::POWER_RATING_PCT_UNKNOWN;
	}
}

void
UavcanServoController::update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
{
	uavcan::equipment::actuator::ArrayCommand msg;

	for (unsigned i = 0; i < num_outputs; ++i) {
		uavcan::equipment::actuator::Command cmd;
		cmd.actuator_id = i;
		cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;
		cmd.command_value = (float)outputs[i] / 500.f - 1.f; // [-1, 1]

		msg.commands.push_back(cmd);
	}

	_uavcan_pub_array_cmd.broadcast(msg);
}

void
UavcanServoController::actuator_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg)
{
	uint8_t actuator_id = msg.actuator_id;

	if (actuator_id < MAX_ACTUATORS) {
		size_t offset = static_cast<size_t>(actuator_id) * ACTUATOR_STATUS_BYTES_PER_ACTUATOR;
		uint8_t *payload = _actuator_status.payload;

		payload[offset++] = actuator_id;

		// uint16_t position = float_to_float16_bits(msg.position);
		// uint16_t force = float_to_float16_bits(msg.force);
		// uint16_t speed = float_to_float16_bits(msg.speed);

		offset = pack_float16_le(payload, offset, msg.position);
		offset = pack_float16_le(payload, offset, msg.force);
		offset = pack_float16_le(payload, offset, msg.speed);

		// payload[offset++] = static_cast<uint8_t>(position & 0xFF);
		// payload[offset++] = static_cast<uint8_t>((position >> 8) & 0xFF);
		// payload[offset++] = static_cast<uint8_t>(force & 0xFF);
		// payload[offset++] = static_cast<uint8_t>((force >> 8) & 0xFF);
		// payload[offset++] = static_cast<uint8_t>(speed & 0xFF);
		// payload[offset++] = static_cast<uint8_t>((speed >> 8) & 0xFF);

		payload[offset++] = static_cast<uint8_t>(msg.power_rating_pct);

		_actuator_status.timestamp = hrt_absolute_time();
		_actuator_status.payload_type = ACTUATOR_STATUS_PAYLOAD_TYPE;
		_actuator_status.target_system = 0; // Broadcast to everyone on the link -- no specific target system.
		_actuator_status.target_component = 0; // Broadcast to everyone on the link
		_actuator_status.payload_length = ACTUATOR_STATUS_PAYLOAD_LENGTH;

		_actuator_status_pub.publish(_actuator_status);
	}
}
