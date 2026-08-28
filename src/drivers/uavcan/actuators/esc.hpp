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
 * @file esc.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/equipment/esc/StatusExtended.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/mavlink_tunnel.h>
#include <uORB/SubscriptionMultiArray.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <parameters/param.h>

class UavcanEscController
{
public:
	static constexpr int MAX_ACTUATORS = esc_status_s::CONNECTED_ESC_MAX;
	static constexpr unsigned MAX_RATE_HZ = 400;

	static_assert(uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize >= MAX_ACTUATORS, "Too many actuators");



	UavcanEscController(uavcan::INode &node);
	~UavcanEscController() = default;

	int init();

	bool initialized() { return _initialized; };

	void update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned total_outputs);

	// void send_rpm_command(int32_t* rpm_values);

	/**
	 * RPM TUNNEL commands received through this.
	 */
	void poll_rpm_tunnel_command();

	/**
	 * Set the number of rotors and enable timer
	 */
	void set_rotor_count(uint8_t count);

	static int max_output_value() { return uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max(); }

	esc_status_s &esc_status() { return _esc_status; }

	// TUNNEL message to transmit ESC_STATUS equivalent for unmapped motors. Used for thrust stand data collection.
	mavlink_tunnel_s &esc_status_basic() {return _esc_status_basic; }

	// TUNNEL message for transmitting esc.StatusExtended messages to companion computer.
	mavlink_tunnel_s &esc_status_extended() { return _esc_status_extended; }

	void reset_esc_status_basic_payload();

	void reset_esc_status_extended_payload();

private:
	// Send RPM command FOR THRUST STAND TESTING ONLY. Will only work on unmapped motors (no UAVCAN_EC_FUNCn assigned)
	void set_rpm_command(uint8_t esc_index, int32_t rpm_value);

	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg);

	/**
	 * ESC StatusExtended message reception reported via this callback.
	 */
	void esc_status_extended_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended> &msg);

	/**
	 * Checks all the ESCs freshness based on timestamp, if an ESC exceeds the timeout then is flagged offline.
	 */
	uint8_t check_escs_status();

	/**
	 * Checks if an ESC is mapped (UAVCAN_EC_FUNCn is not "disabled")
	 */
	bool is_esc_mapped(uint8_t count);


	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>&)> StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::StatusExtended>&)> StatusExtendedCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::TimerEvent &)> TimerCbBinder;

	bool _initialized{};

	esc_status_s	_esc_status{};

	mavlink_tunnel_s	_esc_status_basic{};
	mavlink_tunnel_s	_esc_status_extended{};

	static constexpr uint16_t ESC_STATUS_EXTENDED_PAYLOAD_TYPE = 32805;
	static constexpr uint16_t ESC_STATUS_BASIC_PAYLOAD_TYPE = 32804;
	static constexpr uint16_t RPM_TUNNEL_PAYLOAD_TYPE = 32807;
	static constexpr uint8_t RPM_TUNNEL_RECORD_LENGTH = 3;

	// In the TUNNEL payload, each value will be stored byte-aligned. If link budget is constrained or someone is sufficiently motivated,
	// this could be condensed to 7 bytes through bit packing similar to DroneCAN.
	static constexpr uint8_t STATUS_EXTENDED_BYTES_PER_ESC = 10; // uint7, uint7, int9, uint9, uint19, uint5

	static constexpr uint8_t  ESC_STATUS_EXTENDED_UNKNOWN_PCT          = 0xFF;      // valid range 0-100
	static constexpr int16_t  ESC_STATUS_EXTENDED_UNKNOWN_TEMPERATURE  = 0x7FFF;    // valid range -256..255
	static constexpr uint16_t ESC_STATUS_EXTENDED_UNKNOWN_ANGLE        = 0xFFFF;    // valid range 0..360
	static constexpr uint32_t ESC_STATUS_EXTENDED_UNKNOWN_STATUS_FLAGS = 0xFFFFFFu; // May actually conflict with manufacturer status flags...this byte alone does not indicate a lack of reception.

	static constexpr uint8_t ESC_STATUS_BASIC_BYTES_PER_ESC = 15;

	static constexpr int32_t ESC_STATUS_BASIC_UNKNOWN_RPM = 0x7FFFFF;
	static constexpr uint32_t ESC_STATUS_BASIC_UNKNOWN_ERRORCOUNT = 0xFFFFFFFF;
	static constexpr uint8_t ESC_STATUS_BASIC_UNKNOWN_POWER_RATING_PCT = 0x7F;

	uORB::PublicationMulti<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	uORB::PublicationMulti<mavlink_tunnel_s> _esc_status_basic_pub{ORB_ID(mavlink_tunnel)};
	uORB::PublicationMulti<mavlink_tunnel_s> _esc_status_extended_pub{ORB_ID(mavlink_tunnel)};

	uORB::SubscriptionMultiArray<mavlink_tunnel_s> _rpm_tunnel_subs{ORB_ID::mavlink_tunnel};

	static_assert(MAX_ACTUATORS * STATUS_EXTENDED_BYTES_PER_ESC <= sizeof(_esc_status_extended.payload), "StatusExtended tunnel payload larger than expected.");

	uint8_t		_rotor_count{0};

	int32_t _last_rpm_command[MAX_ACTUATORS] {}; // Persists the last set of RPM values commanded to each ESC, so that commanding one ESC doesn't reset others to 0.

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime									_prev_cmd_pub;   ///< rate limiting
	uavcan::INode										&_node;
	uavcan::Publisher<uavcan::equipment::esc::RawCommand>					_uavcan_pub_raw_cmd;
	uavcan::Publisher<uavcan::equipment::esc::RPMCommand>					_uavcan_pub_rpm_cmd;
	uavcan::Subscriber<uavcan::equipment::esc::Status, StatusCbBinder>			_uavcan_sub_status;
	uavcan::Subscriber<uavcan::equipment::esc::StatusExtended, StatusExtendedCbBinder>	_uavcan_ext_sub_status;


	param_t _param_handles[MAX_ACTUATORS] {PARAM_INVALID};
};
