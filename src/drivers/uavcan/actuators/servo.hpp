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

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/mavlink_tunnel.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

class UavcanServoController
{
public:
	static constexpr int MAX_ACTUATORS = 8;
	static constexpr unsigned MAX_RATE_HZ = 50;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 6;	///< 0..31, inclusive, 0 - highest, 31 - lowest

	UavcanServoController(uavcan::INode &node);
	~UavcanServoController() = default;

	int init();

	bool initialized() { return _initialized; }

	void update_outputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs);

	mavlink_tunnel_s &actuator_status() { return _actuator_status; }

private:
	void actuator_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status> &msg);

	// Fills ever actuator slot's position/force/speed with float16 NaN and power_rating_pct with POWER_RATING_PCT_UNKNOWN from DSDL spec.
	void reset_payload_to_unknown();

	typedef uavcan::MethodBinder<UavcanServoController *,
		void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>&)> StatusCbBinder;

	bool _initialized{};

	mavlink_tunnel_s _actuator_status{};

	// mavlink_tunnel_s _actuator_status[MAX_ACTUATORS]{};

	static constexpr uint16_t ACTUATOR_STATUS_PAYLOAD_TYPE = 32806;
	static constexpr uint8_t ACTUATOR_STATUS_BYTES_PER_ACTUATOR = 8;
	static constexpr uint8_t ACTUATOR_STATUS_PAYLOAD_LENGTH = ACTUATOR_STATUS_BYTES_PER_ACTUATOR * MAX_ACTUATORS;

	static_assert(ACTUATOR_STATUS_PAYLOAD_LENGTH <= sizeof(_actuator_status.payload),
			"Too many actuators to fit status telemetry into a single TUNNEL message.");

	uavcan::INode								&_node;
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> _uavcan_pub_array_cmd;
	uORB::PublicationMulti<mavlink_tunnel_s> _actuator_status_pub{ORB_ID(mavlink_tunnel)};
	uavcan::Subscriber<uavcan::equipment::actuator::Status, StatusCbBinder> _uavcan_sub_status;

	// typedef uavcan::MethodBinder<UavcanServoController *,
	// 	void (UavcanServoController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>&)> StatusServoBinder;
};
