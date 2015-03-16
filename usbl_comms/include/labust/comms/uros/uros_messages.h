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
 *  Author: Dula Nad
 *  Created: 05.03.2015.
 *********************************************************************/
#ifndef USBL_COMMS_UROS_MESSAGES_H
#define USBL_COMMS_UROS_MESSAGES_H
#include <labust/tools/latlon_encoder.h>

namespace labust
{
	namespace comms
	{
		namespace uros
		{
			struct ACData
			{
				virtual ~ACData(){};
				virtual uint64_t encode()=0;
				virtual void decode(uint64_t msg) = 0;
			};

			struct CompressionInfo
			{
				double min;
				double max;
				int bits;
				double quant;
			};

			struct RhodamineReport : public virtual ACData
			{
				enum {ID = 0};
				//Enforce byte len <=8 since uint64_t is used for now
				enum {BYTE_LEN = 6};
				//Bit length to be used for nesting
				enum {BIT_LEN = 48};
				uint16_t rhodamine_adc;
				uint8_t adc_gain;
				double temperature;
				double latitude;
				double longitude;

				//This will be defined in the XML
				const double rhodamine_adc_min = 0;
				const double rhodamine_adc_max = 1024;
				const uint64_t rhodamine_adc_bits = 10;
				const double rhodamine_adc_precission = 1;
				//This will be defined in the XML
				const double latitude_min = -10;
				const double latitude_max = 50;
				const uint64_t latitude_bits = 22;
				const double latitude_precission = 0.0000001;

				uint64_t encode()
				{
					uint64_t retVal;
					uint64_t bitmask;
					uint64_t temp;

					//Example for standard encoding
					bitmask = (1 << rhodamine_adc_bits) -1;
					retVal |= rhodamine_adc & bitmask;
					///Take next in line here for movement (to make room)
					retVal <<= latitude_bits;
					//Example for non-standard trimming and encoding
					//Position bit precission should be the same number
					bitmask = (1 << latitude_bits) -1;
					pcoder.convert(latitude, longitude, latitude_bits);
					temp = pcoder.lat;
					std::cout<<"Encoded lat:"<<temp<<std::endl;
					retVal |= temp & bitmask;
					//The last one does not need to be shifted

					return retVal;
				}

				void decode(uint64_t msg)
				{
					uint64_t bitmask;
					uint64_t temp;

					//Example for standard decoding
					//Start from the last value
					bitmask = (uint64_t(1) << latitude_bits) -1;
					temp = msg & bitmask;
					std::cout<<"Decoded lat:"<<temp<<std::endl;
					pdecoder.convert(temp, 0, latitude_bits);
					latitude = pdecoder.latitude;
					longitude = pdecoder.longitude;
					msg >>= latitude_bits;

					bitmask = (1 << rhodamine_adc_bits) -1;
					rhodamine_adc = uint16_t(msg & bitmask);
				}

			private:
				labust::tools::Bits2LatLon pdecoder;
				labust::tools::LatLon2Bits pcoder;
			};
		}
	}
}
/* USBL_COMMS_UROS_MESSAGES_H */
#endif



