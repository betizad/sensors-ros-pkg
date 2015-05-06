/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *  Author: Gyula Nagy
 *  Created: 14.11.2013.
 *********************************************************************/
#ifndef NOVATEL_NOVATEL_MESSAGES_H
#define NOVATEL_NOVATEL_MESSAGES_H
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <string>
#include <stdint.h>

typedef float vec3f[3];
typedef double vec3d[3];
PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(navigation),Header,
		(std::string, header)
		(std::string, port)
		(std::string, data1)
		(std::string, data2)
		(std::string, steering)
		(std::string, data3)
		(std::string, data4)
		(std::string, data5)
		(std::string, data6)
		(std::string, data7))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(navigation),BestPos,
		(Header, header)
		(std::string, sol_type)
		(std::string, pos_type)
		(vec3d, position)
		(float, undulation)
		(std::string, datum_id)
		(vec3f, position_stddev)
		(std::string, base_id)
		(float, diff_age)
		(float, sol_age)
		(uint8_t, num_tracked_sat)
		(uint8_t, num_sol_sat)
		(uint8_t, num_gg_l1)
		(uint8_t, num_gg_l2)
		(uint8_t, reserved1)
		(std::string, ext_sol_sat)
		(std::string, reserved2)
		(std::string, sigmask)
		(std::string, checksum))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(navigation),BestVel,
		(Header, header)
		(std::string, sol_type)
		(std::string, vel_type)
		(float, latency)
		(float, age)
		(double, hor_spd)
		(double, trk_gnd)
		(double, vert_spd)
		(float, reserved)
		(std::string, checksum))

//NOVATEL_NOVATEL_MESSAGES_H
#endif

