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
 *  Created: 23.05.2016.
 *  Author: Ivor Rendulic
 *********************************************************************/
#ifndef ELLIPSOIDFIT_HPP_
#define ELLIPSOIDFIT_HPP_

#include <Eigen/Dense>

namespace labust
{
	namespace sensors
	{

    /**
     * Ellipsoid fit, based on Yury Petrov's Matlab script.
     * http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
     */
    void ellipsoidFit(const Eigen::MatrixXd& data) {
      Eigen::MatrixXd D = Eigen::MatrixXd::Zero(9, data.cols());
      Eigen::MatrixXd x = data.row(0);
      Eigen::MatrixXd y = data.row(1);
      Eigen::MatrixXd z = data.row(2);
      Eigen::MatrixXd xx = x.cwiseProduct(x);
      Eigen::MatrixXd yy = y.cwiseProduct(y);
      Eigen::MatrixXd zz = z.cwiseProduct(z);

      D.row(0) = xx + yy - 2 * zz;
      D.row(1) = xx + zz - 2 * yy;
      D.row(2) = 2 * x.cwiseProduct(y);
      D.row(3) = 2 * x.cwiseProduct(z);
      D.row(4) = 2 * y.cwiseProduct(z);
      D.row(5) = 2 * x;
      D.row(6) = 2 * y;
      D.row(7) = 2 * z;
      D.row(8) = Eigen::MatrixXd::Constant(1, x.cols(), 1.0);

      Eigen::MatrixXd d2 = xx + yy + zz;
      Eigen::MatrixXd u = (D.transpose() * D).colPivHouseholderQr().solve(D.transpose() * d2);
    }

	}
}

/* ELLIPSOIDFIT_HPP_ */
#endif
