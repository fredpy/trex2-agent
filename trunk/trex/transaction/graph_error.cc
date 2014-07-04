/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#include "graph_error.hh"
#include <string>

using namespace TREX::transaction;

namespace {
  
  class graph_category_impl :public ERROR_CATEGORY {
    char const *name() const throw() {
      return "trex graph";
    }
    std::string message(int ev) const throw();
  }; // ::graph_category_impl
  
  std::string graph_category_impl::message(int ev) const throw() {
    switch (ev) {
      case graph_error::ok:
        return "OK";
      case graph_error::cycle_detected:
        return "Cycle detected";
      case graph_error::already_owned:
        return "Requested timeline is already owned";
      case graph_error::invalid_request_object:
        return "Token requested on a non external timeline";
      case graph_error::invalid_post_object:
        return "Token posted on a non internal timeline";
      case graph_error::invalid_timeline:
        return "Invalid timeline identifier";
      case graph_error::invalid_graph:
        return "Graph is not valid";
      default:
        return "Unknown error";
    }
  }
  
} // ::

ERROR_CODE TREX::transaction::graph_error_code
(graph_error::graph_error_t e) {
  return ERROR_CODE(static_cast<int>(e), graph_category_impl());
}
