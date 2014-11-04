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
/*!\file    projects/crash_course/control/mNoiseFilter.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-27
 *
 */
//----------------------------------------------------------------------
#include "projects/crash_course/control/mNoiseFilter.h"

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
namespace control
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mNoiseFilter> cCREATE_ACTION_FOR_M_NOISEFILTER("NoiseFilter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mNoiseFilter constructor
//----------------------------------------------------------------------
mNoiseFilter::mNoiseFilter(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false),
  input(data_ports::tQueueSettings(true)), // this avoids that any values get lost if input signals e.g. come from a different thread
  last_values(),
  last_element_index(0)
{}

//----------------------------------------------------------------------
// mNoiseFilter Update
//----------------------------------------------------------------------
void mNoiseFilter::Update()
{
  // Add values from input queue to array
  data_ports::tPortBuffers<rrlib::si_units::tLength<>> incoming_values = input.DequeueAll();
  while (!incoming_values.Empty())
  {
    last_element_index++;
    last_element_index = last_element_index % cFILTER_VALUES;
    last_values[last_element_index] = incoming_values.PopFront();
  }

  // Calculate mean
  rrlib::si_units::tLength<> result = 0;
  for (rrlib::si_units::tLength<> value : last_values)
  {
    result += value;
  }
  output.Publish(result / static_cast<double>(cFILTER_VALUES));
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
