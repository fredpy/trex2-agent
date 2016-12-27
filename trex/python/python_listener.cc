/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Frederic Py.
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
#include <trex/transaction/TeleoReactor.hh>
#include "python_listener.hh"

using namespace TREX::python;
using TREX::transaction::graph;
using TREX::transaction::details::timeline;
using TREX::transaction::Relation;
using TREX::utils::Symbol;

namespace bp=boost::python;

/*
 * class TREX::python::py_tl_listener
 */

// structors

py_tl_listener::py_tl_listener(graph &g):graph::timelines_listener(g) {
  initialize();
}

py_tl_listener::~py_tl_listener() {
}

// python handlers

void py_tl_listener::py_declared(Symbol const &name) {
  bp::override f = this->get_override("declared");
  if( f )
    f(name);
}

void py_tl_listener::py_undeclared(Symbol const &name) {
  bp::override f = this->get_override("undeclared");
  if( f )
    f(name);
}

void py_tl_listener::py_used(Symbol const &name) {
  bp::override f = this->get_override("used");
  if( f )
    f(name);
}

void py_tl_listener::py_unused(Symbol const &name) {
  bp::override f = this->get_override("unused");
  if( f )
    f(name);
}


// internal handlers

void py_tl_listener::declared(timeline const &tl) {
  try {
    py_declared(tl.name());
  } catch(...) {
    // silently ignore any exception from python
  }
}

void py_tl_listener::undeclared(timeline const &tl) {
  try {
    py_undeclared(tl.name());
  } catch(...) {
    // silently ignore any exception from python
  }
}

void py_tl_listener::connected(Relation const &r) {
  try {
    py_used(r.name());
  } catch(...) {
    // silently ignore any exception from python
  }
}

void py_tl_listener::disconnected(Relation const &r) {
  try {
    py_unused(r.name());
  } catch(...) {
    // silently ignore any exception from python
  }
}
