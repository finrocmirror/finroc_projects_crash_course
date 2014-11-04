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
#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/canvas/tCanvas2D.h"

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

  /*! Desired velocity */
  tControllerInput<rrlib::si_units::tVelocity<>> velocity;

  /*! Desired angular velocity */
  tControllerInput<rrlib::si_units::tAngularVelocity<>> angular_velocity;

  /*! Position of our robot in the world coordinate system */
  tSensorOutput<rrlib::localization::tPose2D<>> pose;

  /*! Simulated distance sensor values to the front and to the rear */
  tSensorOutput<rrlib::si_units::tLength<>> ir_distance_front, ir_distance_rear;

  /*! Maximum acceleration of robot */
  tParameter<rrlib::si_units::tAcceleration<>> max_acceleration;

  /*! If the robot hits the wall with more than this speed, it is destroyed */
  tParameter<rrlib::si_units::tVelocity<>> destructive_collision_speed;

  /*! Maximum range of IR sensors */
  tParameter<rrlib::si_units::tLength<>> max_ir_sensor_distance;


  /*! Visualization output */
  tVisualizationOutput<rrlib::canvas::tCanvas2D, tLevelOfDetail::ALL> visualization;

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
  rrlib::si_units::tVelocity<> current_speed;

  /*! Robot's current position and orientation */
  rrlib::localization::tPose2D<> current_pose;

  /*! Counts the number of spawned robots (internal) */
  uint robot_counter;

  /*! Info about last collision */
  rrlib::localization::tPose2D<> last_collision_pose;
  rrlib::time::tTimestamp last_collision_timestamp;
  bool last_collision_destructive;

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
