//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/crash_course/mMainSimulation.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-26
 *
 * \brief Contains mMainSimulation
 *
 * \b mMainSimulation
 *
 * This module simulates a differential-driven robot and two distance sensors
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__crash_course__mMainSimulation_h__
#define __projects__crash_course__mMainSimulation_h__

#include "plugins/structure/tSenseControlModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tPose2D.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace crash_course
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Main simulation module
/*!
 * This module simulates a differential-driven robot and two distance sensors
 */
class mMainSimulation : public structure::tSenseControlModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*! Desired velocity (in m/s) */
  tControllerInput<double> velocity;

  /*! Desired angular velocity */
  tControllerInput<double> angular_velocity;

  /*! Position of our robot in the world coordinate system */
  tSensorOutput<rrlib::math::tPose2D> pose;

  /*! Simulated distance sensor values to the front and to the rear */
  tSensorOutput<double> ir_distance_front, ir_distance_rear;

  /*! Pose of last collision */
  tSensorOutput<rrlib::math::tPose2D> last_collision_pose;

  /*! Counts the number of spawned robots */
  tSensorOutput<int> robot_counter;

  /*! Maximum acceleration of robot - in m/s² */
  tParameter<double> max_acceleration;

  /*! If the robot hits the wall with more than this speed, it is destroyed */
  tParameter<double> destructive_collision_speed;

  /*! Maximum range of IR sensors */
  tParameter<double> max_ir_sensor_distance;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mMainSimulation(core::tFrameworkElement *parent, const std::string &name = "MainSimulation");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Robot's current speed */
  double current_speed;

  /*! Robot's current position and orientation */
  rrlib::math::tPose2D current_pose;

  /*! Counts the number of spawned robots (internal) */
  uint robot_counter_internal;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mMainSimulation();

  virtual void Sense();

  virtual void Control();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
