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
#ifndef SEATRAC_MEDIATOR_H
#define SEATRAC_MEDIATOR_H
#include <labust/seatrac/seatrac_messages.h>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace labust
{
	namespace seatrac
	{
		///Mediator type class to ease dynamic casting between pointers.
		template <class CastType>
		class Mediator
		{
			typedef boost::function<void(CastType)> CallbackFunc;
			typedef boost::shared_ptr<CastType const> CastPtr;
			typedef boost::function<void(const SeatracMessage::ConstPtr&)> TopCallback;

		public:
			Mediator(const CallbackFunc& cl):funct(cl){};

			inline void operator()(const SeatracMessage::ConstPtr& msg)
			{
				CastPtr ptr = boost::shared_dynamic_cast<CastType const>(msg);
				if (ptr != 0)
				{
					//Possibly do something here before callback call
					funct(*ptr);
				}
			}

			TopCallback
			static inline makeCallback(const CallbackFunc& cl)
			{
				return boost::bind(&Mediator<CastType>::operator(),
						Mediator<CastType>(cl),_1);
			}

		private:
			CallbackFunc funct;
		};

		template <class Type>
		boost::function<void(const SeatracMessage::ConstPtr&)>
		getcallback(const Type& med)
		{
			return boost::bind(&Type::operator(),med,_1);
		}
	}
}

/* SEATRAC_MEDIATOR_H */
#endif
