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
#include "domain_errors.hh"
#include <string>

using namespace TREX::transaction;

namespace {
  
  class domain_category_impl:public ERROR_CATEGORY {
    char const *name() const throw() {
      return "trex_domain";
    }
    
    std::string message(int ev) const throw();
  }; // ::domain_category_impl

  
  std::string domain_category_impl::message(int ev) const throw() {
    switch (ev) {
      case domain_error::ok:
        return "OK";
      case domain_error::empty_domain:
        return "Empty domain";
      case domain_error::not_a_singleton:
        return "Not a singleton domain";
      case domain_error::incompatible_types:
        return "Incompatible domain types";
      case domain_error::domain_access:
        return "Cannot access domain";
      case domain_error::not_the_same_var:
        return "Variables names are not identical";
      case domain_error::unnamed_var:
        return "Invalid var name";
      case domain_error::incompatible_tokens:
        return "Tokens object or predicate are not identical";
      default:
        return "Unknown error";
    }
  }

}

ERROR_CODE TREX::transaction::domain_error_code(domain_error::domain_error_t e) {
  return ERROR_CODE(static_cast<int>(e), domain_category_impl());
}

