/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <lib/atmosphere/atmosphere.h>

#define MOTOR_BIT(x) (1<<(x))

using namespace time_literals;

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	_uavcan_pub_raw_cmd(node),
	_uavcan_pub_rpm_cmd(node),
	_uavcan_sub_status(node),
	_uavcan_ext_sub_status(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::NumericallyMin); // Highest priority
	reset_esc_status_extended_payload();
}

int
UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC status sub failed %i", res);
		return res;
	}

	_esc_status_pub.advertise();

	res = _uavcan_ext_sub_status.start(StatusExtendedCbBinder(this, &UavcanEscController::esc_status_extended_sub_cb));

	if (res < 0) {
		PX4_ERR("ESC StatusExtended sub failed %i", res);
		return res;
	}

	_esc_status_extended_pub.advertise();

	int32_t iface_mask{0xFF};

	if (param_get(param_find("UAVCAN_ESC_IFACE"), &iface_mask) == OK) {
		_uavcan_pub_raw_cmd.getTransferSender().setIfaceMask(iface_mask);
	}

	char param_name[17];

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		snprintf(param_name, sizeof(param_name), "UAVCAN_EC_FUNC%d", i + 1);
		_param_handles[i] = param_find(param_name);
	}

	_initialized = true;

	return res;
}

void
UavcanEscController::update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned total_outputs)
{
	// TODO: configurable rate limit
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	uavcan::equipment::esc::RawCommand msg = {};

	// directly output values from mixer
	for (unsigned i = 0; i < total_outputs; i++) {
		msg.cmd.push_back(static_cast<int>(outputs[i]));
	}

	// but only output as many channels as are configured
	uint8_t min_size = 0;

	for (int i = 0; i < MAX_ACTUATORS; i++) {
		int32_t val = 0;

		if (param_get(_param_handles[i], &val) == 0) {
			if (val > 0) {
				min_size = i + 1;
			}
		}
	}

	msg.cmd.resize(min_size);

	_uavcan_pub_raw_cmd.broadcast(msg);
}

// void
// UavcanEscController::send_rpm_command(const int32_t* rpm_values)
// {
// 	uint8_t num_escs = static_cast<uint8_t>(rpm_values.size());
// 	if (num_escs > MAX_ACTUATORS) {
// 		PX4_ERR("send_rpm_command: num_escs (%u) exceeds MAX_ACTUATORS (%d)", num_escs, MAX_ACTUATORS);
// 		return;
// 	}

// 	uavcan::equipment::esc::RPMCommand msg = {};

// 	// DroneCAN RPMCommand.rpm is a flat array of rpm values (array position == esc index).
// 	// Positions before esc_index have their RPM set to zero.

// 	for (uint8_t i = 0; i <= num_escs; i++) {
// 		msg.rpm.push_back(rpm_values[i]); // msg.rpm can be any length up to 20.
// 	}

// 	_uavcan_pub_rpm_cmd.broadcast(msg);
// }

void
UavcanEscController::set_rpm_command(uint8_t esc_index, int32_t rpm_value)
{
	if (esc_index >= MAX_ACTUATORS) {
		PX4_ERR("send_rpm_command: esc_index (%u) exceeds MAX_ACTUATORS (%d)", esc_index, MAX_ACTUATORS);
		return;
	}

	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	_last_rpm_command[esc_index] = rpm_value;

	uavcan::equipment::esc::RPMCommand msg = {};
	msg.rpm.resize(MAX_ACTUATORS);

	for (uint8_t i = 0; i < MAX_ACTUATORS; i++) {
		msg.rpm[i] = _last_rpm_command[i]; // msg.rpm can be any length up to 20.
	}

	_uavcan_pub_rpm_cmd.broadcast(msg);
}

void
UavcanEscController::set_rotor_count(uint8_t count)
{
	_rotor_count = count;
}

void
UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature + atmosphere::kAbsoluteNullCelsius; // Kelvin to Celsius
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;

		_esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}

void
UavcanEscController::reset_esc_status_extended_payload()
{
	for (uint8_t id = 0; id < MAX_ACTUATORS; id++) {
		size_t offset = static_cast<size_t>(id) * STATUS_EXTENDED_BYTES_PER_ESC;
		uint8_t *payload = _esc_status_extended.payload;

		payload[offset++] = ESC_STATUS_EXTENDED_UNKNOWN_PCT; // input_pct
		payload[offset++] = ESC_STATUS_EXTENDED_UNKNOWN_PCT; // output_pct

		payload[offset++] = static_cast<uint8_t>(ESC_STATUS_EXTENDED_UNKNOWN_TEMPERATURE & 0xFF); // motor temp
		payload[offset++] = static_cast<uint8_t>((ESC_STATUS_EXTENDED_UNKNOWN_TEMPERATURE >> 8) & 0xFF);

		payload[offset++] = static_cast<uint8_t>(ESC_STATUS_EXTENDED_UNKNOWN_ANGLE & 0xFF); // motor angle
		payload[offset++] = static_cast<uint8_t>((ESC_STATUS_EXTENDED_UNKNOWN_ANGLE >> 8) & 0xFF);

		payload[offset++] = static_cast<uint8_t>(ESC_STATUS_EXTENDED_UNKNOWN_STATUS_FLAGS & 0xFF); // Status flags. Could conflict with manufacturer status flags...
		payload[offset++] = static_cast<uint8_t>((ESC_STATUS_EXTENDED_UNKNOWN_STATUS_FLAGS >> 8) & 0xFF);
		payload[offset++] = static_cast<uint8_t>((ESC_STATUS_EXTENDED_UNKNOWN_STATUS_FLAGS >> 16) & 0xFF);

		payload[offset++] = id;
	}
}

void
UavcanEscController::esc_status_extended_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended> &msg)
{
	uint8_t esc_index = msg.esc_index;
	if (esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status_extended;

		size_t offset = esc_index * STATUS_EXTENDED_BYTES_PER_ESC;
		uint8_t *payload = _esc_status_extended.payload;

		payload[offset++] = static_cast<uint8_t>(msg.input_pct);
		payload[offset++] = static_cast<uint8_t>(msg.output_pct);

		int16_t motor_temp = static_cast<int16_t>(msg.motor_temperature_degC);
		payload[offset++] = static_cast<uint8_t>(motor_temp & 0xFF);
		payload[offset++] = static_cast<uint8_t>((motor_temp >> 8) & 0xFF);

		uint16_t motor_angle = static_cast<uint16_t>(msg.motor_angle);
		payload[offset++] = static_cast<uint8_t>(motor_angle & 0xFF);
		payload[offset++] = static_cast<uint8_t>((motor_angle >> 8) & 0xFF);

		uint32_t status_flags = static_cast<uint32_t>(msg.status_flags);
		payload[offset++] = static_cast<uint8_t>(status_flags & 0xFF);
		payload[offset++] = static_cast<uint8_t>((status_flags >> 8) & 0xFF);
		payload[offset++] = static_cast<uint8_t>((status_flags >> 16) & 0xFF);

		payload[offset++] = esc_index;

		_esc_status_extended.timestamp	    = hrt_absolute_time();
		_esc_status_extended.payload_type    = ESC_STATUS_EXTENDED_PAYLOAD_TYPE;
		_esc_status_extended.target_system   = 0; // Broadcast to everyone on the link -- no specific target system.
		_esc_status_extended.target_component= 0; // Broadcast to everyone on the link
		_esc_status_extended.payload_length  = STATUS_EXTENDED_BYTES_PER_ESC * _rotor_count;

		_esc_status_extended_pub.publish(ref);
	}
}

uint8_t
UavcanEscController::check_escs_status()
{
	int esc_status_flags = 0;
	const hrt_abstime now = hrt_absolute_time();

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (_esc_status.esc[index].timestamp > 0 && now - _esc_status.esc[index].timestamp < 1200_ms) {
			esc_status_flags |= (1 << index);
		}

	}

	return esc_status_flags;
}
