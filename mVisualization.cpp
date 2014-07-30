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
/*!\file    projects/crash_course/mVisualization.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-27
 *
 */
//----------------------------------------------------------------------
#include "projects/crash_course/mVisualization.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mVisualization> cCREATE_ACTION_FOR_M_VISUALIZATION("Visualization");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mVisualization constructor
//----------------------------------------------------------------------
mVisualization::mVisualization(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false),
  last_collision_destructive(false)
{}

//----------------------------------------------------------------------
// mVisualization destructor
//----------------------------------------------------------------------
mVisualization::~mVisualization()
{}

//----------------------------------------------------------------------
// mVisualization Update
//----------------------------------------------------------------------
void mVisualization::Update()
{
  if (this->InputChanged())
  {
    // visualize using tCanvas2D
    // obtain buffer
    data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> canvas = visualization.GetUnusedBuffer();
    canvas->Clear();

    // Draw wall
    canvas->SetFill(true);
    canvas->SetColor(128, 64, 0); // brown
    canvas->DrawBox(2, -1000, 2000, 2000);

    // Draw Robot Text
    rrlib::math::tPose2D current_pose = pose.Get();
    canvas->Translate(current_pose.X(), current_pose.Y());
    canvas->SetColor(255, 255, 255);
    char robot_text[128];
    snprintf(robot_text, 128, "Tutorial Robot #%d", robot_counter.Get() + 1);
    canvas->DrawText(0.0, 0.28, robot_text);
    canvas->ResetTransformation();

    // Draw Robot
    canvas->Transform(current_pose);
    canvas->SetEdgeColor(0, 0, 0);
    canvas->SetFillColor(200, 200, 255);
    canvas->DrawEllipsoid(0.1, 0.0, 0.4, 0.4);
    canvas->DrawBox(-0.2, -0.2, 0.3, 0.4);

    // Exercise 2
    canvas->DrawArrow(0.0, 0.0, ir_distance_front.Get(), 0.0, false);
    canvas->DrawArrow(0.0, 0.0, -ir_distance_rear.Get(), 0.0, false);

    canvas->ResetTransformation();

    // Indicate Collision
    if (last_collision_pose.HasChanged())
    {
      last_collision_destructive = (current_pose == rrlib::math::tPose2D::Zero());
    }
    rrlib::time::tTimestamp last_collision_timestamp;
    rrlib::math::tPose2D last_collision = last_collision_pose.Get(last_collision_timestamp);
    rrlib::time::tDuration since_last_collision = rrlib::time::Now() - last_collision_timestamp;
    if (since_last_collision < std::chrono::milliseconds(2000))
    {
      canvas->Translate(last_collision.X(), last_collision.Y());
      double time_factor = 1.0 - (std::chrono::duration_cast<std::chrono::milliseconds>(since_last_collision).count() / 2000.0);
      canvas->SetAlpha(static_cast<uint8_t>(255.0 * time_factor * time_factor));
      canvas->SetColor(last_collision_destructive ? 255 : 50, 50, last_collision_destructive ? 50 : 255);
      canvas->DrawText(0.0, 0.0, last_collision_destructive ? "KABOOM!" : "Plonk");
    }

    visualization.Publish(canvas);
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
