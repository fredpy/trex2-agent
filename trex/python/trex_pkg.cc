/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, MBARI.
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
 *   * Neither the name of the TREX Project nor the names of its
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
 */
#include <boost/python.hpp>

#include <trex/utils/TREXversion.hh>


void export_utils();
void export_domain();

using namespace boost::python;

BOOST_PYTHON_MODULE(trex)
{
  // trex.version class
  // A class with only static read only properties
  //   trex.version.major : major version number
  //   trex.version.minor : minor version number
  //   trex.version.release : release number
  //   trex.version.is_rc : indicates if it is a release candidate
  //   trex.version.rc : release candidate number (or 0 if is_rc is False)
  //   trex.version.str : A string value for this version of TREX
  class_<TREX::version>("version", "Version information about trex", no_init)
  .add_static_property("major", &TREX::version::major)
  .add_static_property("minor", &TREX::version::minor)
  .add_static_property("release", &TREX::version::release)
  .add_static_property("is_rc", &TREX::version::is_release_candidate)
  .add_static_property("rc", &TREX::version::rc_number)
  .add_static_property("str", &TREX::version::str)
  ;

  object pkg = scope();
  pkg.attr("__path__") = "trex";

  export_utils();
  export_domain();
} // BOOST_PYTHON_MODULE(trex)
