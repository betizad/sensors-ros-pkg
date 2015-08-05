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
 *
 *  Created on: 15.11.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef ASCIIHEX_ISERIALIZATOR_HPP_
#define ASCIIHEX_ISERIALIZATOR_HPP_
#include <cstddef>
#include <sstream>
#include <boost/archive/detail/common_iarchive.hpp>

namespace labust
{
	namespace archive
	{
		/////////////////////////////////////////////////////////////////////////
		// class complete_iarchive
		class asciihex_iarchive :
				public boost::archive::detail::common_iarchive<asciihex_iarchive>
		{
			// permit serialization system privileged access to permit
			// implementation of inline templates for maximum speed.
			friend class boost::archive::load_access;

			// member template for saving primitive types.
			// Specialize for any types/templates that special treatment
			template<class T>
			void load(T & t)
			{
				if (in.fail() && !in.eof())
				{
					boost::serialization::throw_exception(
							boost::archive::archive_exception(
									boost::archive::archive_exception::input_stream_error
							)
					);
				}

				in>>std::hex>>t;
			};

			void load(boost::archive::class_name_type & t){}

			std::istream& in;

		public:
			//////////////////////////////////////////////////////////
			// public interface used by programs that use the
			// serialization library

			// archives are expected to support this function
			void load_binary(void *address, std::size_t count);

			asciihex_iarchive(std::istream& in):
				in(in){};
		};
	}
}

/* ASCIIHEX_IARCHIVE_HPP_ */
#endif
