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
/*!\file    projects/crash_course/control/mNoiseFilter.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-27
 *
 * \brief Contains mNoiseFilter
 *
 * \b mNoiseFilter
 *
 * Filters an input signal by calculating the arithmetic mean of the last 5 input values
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__crash_course__control__mNoiseFilter_h__
#define __projects__crash_course__control__mNoiseFilter_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
//! Simple noise filter
/*!
 * Filters an input signal by calculating the arithmetic mean of the last 5 input values
 */
class mNoiseFilter : public structure::tModule
{
  /*! Number of values to use for filtering */
  static const size_t cFILTER_VALUES = 5;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*! Input value */
  tInput<double> input;

  /*! Output value (= input value with added noise) */
  tOutput<double> output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mNoiseFilter(core::tFrameworkElement *parent, const std::string &name = "NoiseFilter");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Array with the last values */
  std::array<double, cFILTER_VALUES> last_values;

  /*! Index of last element written to array */
  size_t last_element_index;

  virtual void Update();
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
