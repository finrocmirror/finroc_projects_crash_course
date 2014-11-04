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
/*!\file    projects/crash_course/mMainSimulation.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-26
 *
 */
//----------------------------------------------------------------------
#include "projects/crash_course/mMainSimulation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/scheduling/tThreadContainerThread.h"

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
using namespace rrlib::si_units;

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
static runtime_construction::tStandardCreateModuleAction<mMainSimulation> cCREATE_ACTION_FOR_M_MAINSIMULATION("MainSimulation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mMainSimulation constructor
//----------------------------------------------------------------------
mMainSimulation::mMainSimulation(finroc::core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name),
  max_acceleration(0.3),
  destructive_collision_speed(0.9),
  max_ir_sensor_distance(2),
  current_speed(0),
  current_pose(),
  robot_counter(0),
  last_collision_pose(),
  last_collision_timestamp(rrlib::time::cNO_TIME),
  last_collision_destructive(false)
{}

//----------------------------------------------------------------------
// mMainSimulation destructor
//----------------------------------------------------------------------
mMainSimulation::~mMainSimulation()
{}

//----------------------------------------------------------------------
// mMainSimulation Sense
//----------------------------------------------------------------------
void mMainSimulation::Sense()
{
  tTime<> delta_t = scheduling::tThreadContainerThread::CurrentThread()->GetCycleTime();
  rrlib::time::tTimestamp now = rrlib::time::Now();

  // Calculate new speed
  const tVelocity<> cZERO_SPEED = 0;
  tVelocity<> desired_speed = velocity.Get();
  tVelocity<> new_speed = 0;
  if (desired_speed >= cZERO_SPEED)
  {
    new_speed = current_speed < cZERO_SPEED ? cZERO_SPEED : ((desired_speed < current_speed) ? desired_speed :
                std::min(velocity.Get(), current_speed + max_acceleration.Get() * delta_t));
  }
  else
  {
    new_speed = current_speed > cZERO_SPEED ? cZERO_SPEED : ((desired_speed > current_speed) ? desired_speed :
                std::max(velocity.Get(), current_speed - max_acceleration.Get() * delta_t));
  }
  tVelocity<> avg_speed = (current_speed + new_speed) / 2;
  current_speed = new_speed;

  // Calculate new orientation
  auto new_direction = current_pose.Yaw() + angular_velocity.Get() * delta_t;
  auto avg_direction = (current_pose.Yaw() + new_direction) / 2;

  // Calculate new coordinates
  tLength<> s = avg_speed * delta_t;
  current_pose.Set(current_pose.X() + s * avg_direction.Value().Cosine(), current_pose.Y() + s * avg_direction.Value().Sine(), new_direction);

  // Did we collide with the wall?
  typedef rrlib::localization::tPose2D<> tPose2D;
  tPose2D back_left = current_pose;
  back_left.ApplyRelativePoseTransformation(tPose2D(-0.2, 0.2));
  tPose2D back_right = current_pose;
  back_right.ApplyRelativePoseTransformation(tPose2D(-0.2, -0.2));
  tPose2D front_center = current_pose;
  front_center.ApplyRelativePoseTransformation(tPose2D(0.1, 0));
  tLength<> max_x = std::max(back_left.X(), std::max(back_right.X(), front_center.X() + tLength<>(0.2)));
  const tLength<> cWALL_X = 2;
  if (max_x > cWALL_X)
  {
    last_collision_pose = current_pose;
    last_collision_destructive = avg_speed > destructive_collision_speed.Get() || (-avg_speed) > destructive_collision_speed.Get();
    last_collision_timestamp = now;
    if (!last_collision_destructive)
    {
      current_pose.SetPosition(current_pose.X() - (max_x - cWALL_X), current_pose.Y());
      current_speed = 0;
      FINROC_LOG_PRINT(WARNING, "Robot collided with wall at a speed of ", avg_speed);
    }
    else
    {
      robot_counter++;
      current_pose.Reset();
      current_speed = 0;
      FINROC_LOG_PRINT(ERROR, "Robot crashed at a speed of ", avg_speed, " and was destroyed. Respawning. Destroyed Robots: ", robot_counter, ".");
    }
  }
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "New robot position: ", current_pose);

  // Calculate sensor values
  tPose2D robot_in_2m = current_pose.Translated(rrlib::math::tVec2d(2, 0).Rotated(current_pose.Yaw().Value()));
  tPose2D robot_2m_back = current_pose.Translated(rrlib::math::tVec2d(-2, 0).Rotated(current_pose.Yaw().Value()));
  tLength<> front_distance = max_ir_sensor_distance.Get();
  tLength<> rear_distance = max_ir_sensor_distance.Get();
  double dx = fabs((robot_in_2m.X() - current_pose.X()).Value());
  if (robot_in_2m.X() > cWALL_X)
  {
    front_distance = (2 * (cWALL_X - current_pose.X()).Value() / dx);
  }
  if (robot_2m_back.X() > cWALL_X)
  {
    rear_distance = (2 * (cWALL_X - current_pose.X()).Value() / dx);
  }

  // publish updated values
  pose.Publish(current_pose, now);
  ir_distance_front.Publish(front_distance, now);
  ir_distance_rear.Publish(rear_distance, now);


  // Create visualization
  if (visualization.IsConnected())
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
    canvas->Translate(current_pose.X().Value(), current_pose.Y().Value());
    canvas->SetColor(255, 255, 255);
    char robot_text[128];
    snprintf(robot_text, 128, "Tutorial Robot #%d", robot_counter + 1);
    canvas->DrawText(0.0, 0.28, robot_text);
    canvas->ResetTransformation();

    // Draw Robot
    canvas->Transform(current_pose);
    canvas->SetEdgeColor(0, 0, 0);
    canvas->SetFillColor(200, 200, 255);
    canvas->DrawEllipsoid(0.1, 0.0, 0.4, 0.4);
    canvas->DrawBox(-0.2, -0.2, 0.3, 0.4);
    canvas->ResetTransformation();

    // Indicate Collision
    rrlib::time::tDuration since_last_collision = rrlib::time::Now() - last_collision_timestamp;
    if (since_last_collision < std::chrono::milliseconds(2000))
    {
      canvas->Translate(last_collision_pose.X().Value(), last_collision_pose.Y().Value());
      double time_factor = 1.0 - (std::chrono::duration_cast<std::chrono::milliseconds>(since_last_collision).count() / 2000.0);
      canvas->SetAlpha(static_cast<uint8_t>(255.0 * time_factor * time_factor));
      canvas->SetColor(last_collision_destructive ? 255 : 50, 50, last_collision_destructive ? 50 : 255);
      canvas->DrawText(0.0, 0.0, last_collision_destructive ? "KABOOM!" : "Plonk");
    }

    visualization.Publish(canvas);
  }
}

//----------------------------------------------------------------------
// mMainSimulation Control
//----------------------------------------------------------------------
void mMainSimulation::Control()
{
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
