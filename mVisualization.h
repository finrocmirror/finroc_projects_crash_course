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
/*!\file    projects/crash_course/mVisualization.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-27
 *
 * \brief Contains mVisualization
 *
 * \b mVisualization
 *
 * Visualizes the simulated scene using a tCanvas2D.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__crash_course__mVisualization_h__
#define __projects__crash_course__mVisualization_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tPose2D.h"
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
//! Simulation visualization
/*!
 * Visualizes the simulated scene using a tCanvas2D.
 */
class mVisualization : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*! Position and orientation of robot in the world coordinate system */
  tInput<rrlib::math::tPose2D> pose;

  /*! Pose of last collision */
  tInput<rrlib::math::tPose2D> last_collision_pose;

  /*! Counts the number of spawned robots */
  tInput<int> robot_counter;

  /*! Visualization of simulation */
  tOutput<rrlib::canvas::tCanvas2D> visualization;

  /*! Simulated distance sensor values to the front and to the rear */
  tInput<double> ir_distance_front, ir_distance_rear;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mVisualization(core::tFrameworkElement *parent, const std::string &name = "Visualization");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Was last collision destructive? */
  bool last_collision_destructive;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mVisualization();

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
