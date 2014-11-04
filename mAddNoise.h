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
/*!\file    projects/crash_course/mAddNoise.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-01-26
 *
 * \brief Contains mAddNoise
 *
 * \b mAddNoise
 *
 * Adds noise (a gauss-distributed random value) to the numeric input.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__crash_course__mAddNoise_h__
#define __projects__crash_course__mAddNoise_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <random>
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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Adds noise to the input
/*!
 * Adds noise (a gauss-distributed random value) to the numeric input.
 */
class mAddNoise : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*! Input value */
  tInput<rrlib::si_units::tLength<>> input;

  /*! Output value (= input value with added noise) */
  tOutput<rrlib::si_units::tLength<>> output;

  /*! Standard deviation for added noise */
  tParameter<rrlib::si_units::tLength<>> standard_deviation;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mAddNoise(core::tFrameworkElement *parent, const std::string &name = "AddNoise");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  std::normal_distribution<double> normal_distribution;
  std::mt19937 eng;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mAddNoise();

  virtual void OnParameterChange();

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
