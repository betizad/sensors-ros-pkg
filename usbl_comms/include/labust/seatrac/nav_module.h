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
#ifndef USBL_COMMS_NAV_MODULE_H
#define USBL_COMMS_NAV_MODULE_H
#include <labust/comms/caddy/caddy_messages.h>
#include <labust/math/NumberManipulation.hpp>

#include <auv_msgs/NavSts.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

using labust::comms::caddy::BuddyReport;
using labust::comms::caddy::SurfaceReport;
using labust::comms::caddy::DiverReport;

namespace labust
{
namespace seatrac
{
/**
 * The class implements the navigation data module. The module handles
 * acquisition of navigation data for CADDY agents.
 */
class NavModule
{
  enum
  {
    n = 0,
    e,
    d
  };
  typedef boost::function<void(void)> TriggerFunctor;

public:
  /// Main constructor
  NavModule() : has_diver(false){};
  /// Default destructor
  ~NavModule(){};

  /// Listener configuration.
  bool configure(ros::NodeHandle& nh, ros::NodeHandle& ph)
  {
    nav_sub = nh.subscribe("agent_position_out", 1, &NavModule::onEstimatedPos,
                           this);
    diver_sub =
        nh.subscribe("diver_position_out", 1, &NavModule::onDiverPos, this);
    return true;
  }

  /// Pull the newest data in the report message the offset is the acoustic
  /// frame location
  template <class ReportMessage>
  void updateReport(ReportMessage& message, const Eigen::Vector3d& offset);

  /// Register trigger callback
  void registerTrigger(TriggerFunctor callback)
  {
    this->callback = callback;
  }

protected:
  /// Handle the estimated position.
  void onEstimatedPos(const auv_msgs::NavSts::ConstPtr& msg)
  {
    boost::mutex::scoped_lock l(data_mux);
    navdata = *msg;
    l.unlock();

    if (!callback.empty())
      callback();
  }

  /// Handle the estimated position.
  void onDiverPos(const auv_msgs::NavSts::ConstPtr& msg)
  {
    boost::mutex::scoped_lock l(data_mux);
    has_diver = true;
    diverdata = *msg;
  }

  template <class ReportMessage>
  void getSpeedCourse(ReportMessage& message)
  {
    double u = navdata.gbody_velocity.x;
    double v = navdata.gbody_velocity.y;
    double heading =
        labust::math::wrapRad(navdata.orientation.yaw) * 180 / M_PI;
    double course = (navdata.orientation.yaw + atan2(v, u)) * 180 / M_PI;
    double U = sqrt(u * u + v * v);
    // For overactuated
    const double min_speed(0.1);
    message.speed = U;
    message.course = (message.speed < min_speed ? heading : course);
  }

  /// Position estimate subscriber.
  ros::Subscriber nav_sub;
  /// Diver estimate subscriber.
  ros::Subscriber diver_sub;
  /// Save the navigation status
  auv_msgs::NavSts navdata;
  /// Save the diver status
  auv_msgs::NavSts diverdata;
  /// The diver flag
  bool has_diver;
  /// The trigger callback for async operations
  TriggerFunctor callback;
  /// Data protection
  boost::mutex data_mux;
};

/// Buddy specialization
template <>
void NavModule::updateReport<BuddyReport>(BuddyReport& message,
                                          const Eigen::Vector3d& offset);
/// Surface specialization
template <>
void NavModule::updateReport<SurfaceReport>(SurfaceReport& message,
                                            const Eigen::Vector3d& offset);
/// Diver specialization
template <>
void NavModule::updateReport<DiverReport>(DiverReport& message,
                                          const Eigen::Vector3d& offset);
}
}

/* USBL_COMMS_NAV_MODULE_H */
#endif
