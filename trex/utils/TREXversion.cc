/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
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
#include "TREXversion.hh"
#include <trex/version.hh>

#include <sstream>

# include "bits/git_version.hh"

using namespace TREX;

namespace  {
  TREX::version _version;
}

unsigned short version::major_number() {
  return TREX_MAJOR;
}

unsigned short version::minor_number() {
  return TREX_MINOR;
}

unsigned short version::release_number() {
  return TREX_PATCH;
}

bool version::is_release_candidate() {
  return rc_number()>0;
}

unsigned version::rc_number() {
  return TREX_RC;
}


unsigned long version::number() {
  unsigned long version = major_number();
  version = 100*version + minor_number();
  version = 100*version + release_number();
  return version;
}

std::string version::str() {
  return TREX_VERSION;
}

bool version::svn_info() {
  return false;
}

std::string version::svn_root() {
  return std::string();
}

std::string version::svn_revision() {
  return std::string();
}

bool version::git_info() {
#ifdef GIT_PROJECT
  return true;
#else
  return false;
#endif
}

std::string version::git_branch() {
  return GIT_BRANCH;
}

std::string version::git_revision() {
  return GIT_REV;
}

std::string version::full_str() {
  if( git_info() ) 
    return str()+" (git:"+git_branch()+"["+git_revision()+"])";
  else
    return str();
}

