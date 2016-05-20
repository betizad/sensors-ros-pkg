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
 *  Created: 20.05.2016.
 *  Author: Ivor Rendulic
 *********************************************************************/
#ifndef DSPUTILS_HPP_
#define DSPUTILS_HPP_

namespace labust
{
	namespace sensors
	{

    /**
     * Filter function like the one in Matlab.
     * Takes vectors of filter coefficients, and iterators to newest input
     * sample x and previous filtered sample y.
     * It is up to the user to ensure the correct length of x and y with 
     * respect to b and a.
     */
    template<class forward_iterator_x, class forward_iterator_y>
    double filter(const std::vector<double>& b,
        const std::vector<double>& a,
        forward_iterator_x x,
        forward_iterator_y y) {
      double res = 0;
      for (int i=0; i<b.size(); ++i) {
        res += *x * b[i];
        ++x;
      }
      for (int j=1; j<a.size(); ++j) {
        res -= *y * a[j];
        ++y;
      }
      return res / a[0];
    }

	}
}

/* DSPUTILS_HPP_ */
#endif
