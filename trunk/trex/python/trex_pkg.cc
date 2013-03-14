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

#include <trex/utils/TREXversion.hh>


void export_utils();
void export_domain();
void export_transactions();
void export_agent();


namespace {
  
  /*
   * need to encapsulate these two calls like this to avoid a bug on g++ 4.6.3
   * which otherwise complains that major and minor are not declared in 
   * TREX::version
   */
  unsigned long version_major() {
    return TREX::version::major();
  }
  
  unsigned long version_minor() {
    return TREX::version::minor();
  }
}

#include <boost/python.hpp>

using namespace boost::python;

BOOST_PYTHON_MODULE(trex)
{
  
  object pkg = scope();
  pkg.attr("__doc__") = "Python API for trex\n"
  "Provides basic core classes in order to use and extend trex through python."
  "This API attempt to reproduce trex core libraries structure with the 4 "
  "following submodules:\n"
  "  - utils: present basic utilities classes that are used by trex\n"
  "  - domains: basic classes to represent flaxible domains and variables\n"
  "  - transaction: core classes for reactor implementation\n"
  "  - agent: classes used to construct and execute an agent as a collection"
  " of interconnected reactors\n"
  ;
  pkg.attr("__path__") = "trex";
  
  // trex.version class
  // A class with only static read only properties
  //   trex.version.major : major version number
  //   trex.version.minor : minor version number
  //   trex.version.release : release number
  //   trex.version.is_rc : indicates if it is a release candidate
  //   trex.version.rc : release candidate number (or 0 if is_rc is False)
  //   trex.version.str : A string value for this version of TREX
  class_<TREX::version>("version", "Version information about trex.\n"
                        "All its properties are static and indicate version "
                        "informations about the current trex version", no_init)
  // I  do not know how to documment static_properties
  .add_static_property("major", &version_major)
  .add_static_property("minor", &version_minor)
  .add_static_property("release", &TREX::version::release)
  .add_static_property("is_rc", &TREX::version::is_release_candidate)
  .add_static_property("rc", &TREX::version::rc_number)
  .add_static_property("str", &TREX::version::str)
  ;


  export_utils();
  export_domain();
  export_transactions();
  export_agent();
} // BOOST_PYTHON_MODULE(trex)
