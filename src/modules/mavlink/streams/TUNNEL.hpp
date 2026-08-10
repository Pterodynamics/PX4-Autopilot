/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file TUNNEL.hpp
 *
 * Forwards every instance of the mavlink_tunnel uORB topic to a MAVLink
 * TUNNEL message. Multiple independent publishers (e.g. esc.cpp's
 * StatusExtended bridge, actuator.cpp's Status bridge, or any future tunnel
 * producer) each land on their own uORB instance automatically via
 * PublicationMulti -- this stream doesn't need to know how many producers
 * exist or what they represent, since mavlink_tunnel_s already carries
 * payload_type/target_system/target_component/payload_length verbatim from
 * whatever populated it. Interpreting payload_type is left entirely to
 * whatever parses the TUNNEL message on the receiving end.
 */

#ifndef TUNNEL_HPP
#define TUNNEL_HPP

#include <uORB/topics/mavlink_tunnel.h>

class MavlinkStreamTunnel : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTunnel(mavlink); }

	static constexpr const char *get_name_static() { return "TUNNEL"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TUNNEL; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _tunnel_subs.advertised_count() * (MAVLINK_MSG_ID_TUNNEL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	explicit MavlinkStreamTunnel(Mavlink *mavlink) : MavlinkStream(mavlink) {};

	uORB::SubscriptionMultiArray<mavlink_tunnel_s> _tunnel_subs{ORB_ID::mavlink_tunnel};

	bool send() override
	{
		bool updated = false;

		for (int i = 0; i < _tunnel_subs.size(); i++) {
			mavlink_tunnel_s tunnel;

			if (_tunnel_subs[i].update(&tunnel)) {
				mavlink_tunnel_t msg{};

				msg.payload_type = tunnel.payload_type;
				msg.target_system = tunnel.target_system;
				msg.target_component = tunnel.target_component;
				msg.payload_length = tunnel.payload_length;

				static_assert(sizeof(msg.payload) == sizeof(tunnel.payload),
					      "mavlink_tunnel_t and mavlink_tunnel_s payload sizes must match");
				memcpy(msg.payload, tunnel.payload, sizeof(msg.payload));

				mavlink_msg_tunnel_send_struct(_mavlink->get_channel(), &msg);
				updated = true;
			}
		}

		return updated;
	}
};

#endif // TUNNEL_HPP
