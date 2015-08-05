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
 *********************************************************************/
#ifndef VECTOROVERRIDE_HPP_
#define VECTOROVERRIDE_HPP_
#include <boost/serialization/vector.hpp>

namespace boost
{
	namespace serialization{
		template<class Archive, class Allocator>
		inline void save(
				Archive & ar,
				const std::vector<uint8_t, Allocator> &t,
				const unsigned int /* file_version */
		){
			// record number of elements
			uint8_t count (t.size());
			ar << BOOST_SERIALIZATION_NVP(count);
			std::vector<uint8_t>::const_iterator it = t.begin();
			while(count-- > 0){
				uint8_t tb = *it++;
				ar << boost::serialization::make_nvp("item", tb);
			}
		};

		template<class Archive, class Allocator>
		inline void load(
				Archive & ar,
				std::vector<uint8_t, Allocator> &t,
				const unsigned int /* file_version */
		){
			// retrieve number of elements
			uint8_t count;
			ar >> BOOST_SERIALIZATION_NVP(count);
			t.clear();
			while(count-- > 0){
				uint8_t i;
				ar >> boost::serialization::make_nvp("item", i);
				t.push_back(i);
			}
		};
		template<class Archive, class Allocator>
		inline void save(
				Archive & ar,
				const std::vector<int16_t, Allocator> &t,
				const unsigned int /* file_version */
		){
			// record number of elements
			uint8_t count (t.size());
			ar << BOOST_SERIALIZATION_NVP(count);
			std::vector<int16_t>::const_iterator it = t.begin();
			while(count-- > 0){
				uint8_t tb = *it++;
				ar << boost::serialization::make_nvp("item", tb);
			}
		};

		template<class Archive, class Allocator>
		inline void load(
				Archive & ar,
				std::vector<int16_t, Allocator> &t,
				const unsigned int /* file_version */
		){
			// retrieve number of elements
			uint8_t count;
			ar >> BOOST_SERIALIZATION_NVP(count);
			t.clear();
			while(count-- > 0){
				int16_t i;
				ar >> boost::serialization::make_nvp("item", i);
				t.push_back(i);
			}
		};
	}
};

/* VECTOROVERRIDE_HPP_ */
#endif
