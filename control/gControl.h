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
/*!\file    projects/crash_course/control/gControl.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-27
 *
 * \brief Contains gControl
 *
 * \b gControl
 *
 * Contains robot control system for the 'Crash Course' tutorial.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__crash_course__control__gControl_h__
#define __projects__crash_course__control__gControl_h__

#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"

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
namespace control
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Robot control
/*!
 * Contains robot control system for the 'Crash Course' tutorial.
 */
class gControl : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*! Desired velocity */
  tControllerOutput<rrlib::si_units::tVelocity<>> velocity;

  /*! Desired angular velocity */
  tControllerOutput<rrlib::si_units::tAngularVelocity<>> angular_velocity;

  /*! Position of our robot in the world coordinate system */
  tSensorInput<rrlib::localization::tPose2D<>> pose;

  /*! Simulated distance sensor values to the front and to the rear */
  tSensorInput<rrlib::si_units::tLength<>> ir_distance_front, ir_distance_rear;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gControl(core::tFrameworkElement *parent, const std::string &name = "Control",
           const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~gControl();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
