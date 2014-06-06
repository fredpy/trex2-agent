/** @file DomainBase.cc
 * @brief Basic domain utilities implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
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
#include <sstream>
#include "domain_visitor.hh"

using namespace TREX::transaction;

namespace {
  class domain_category_impl :public ERROR_CATEGORY {
    char const *name() const {
      return "trex domain";
    }
    std::string message(int ev) const;
  };

  std::string domain_category_impl::message(int ev) const {
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
      case domain_error::unnamed_var:
        return "Invalid var name";
      default:
        return "Unknown error";
    }
  }
}

ERROR_CODE TREX::transaction::domain_error::make_error(domain_error::domain_error_t e) {
  return ERROR_CODE(static_cast<int>(e), domain_category_impl());
}


/*
 * class TREX::transaction::abstract_domain
 */

// observers

boost::any abstract_domain::get_singleton() const {
  if( !is_singleton() )
    throw SYSTEM_ERROR(make_error(domain_error::not_a_singleton));
  return singleton();
}

std::string abstract_domain::get_singleton_as_string() const {
  if( !is_singleton() )
    throw SYSTEM_ERROR(make_error(domain_error::not_a_singleton));
  return string_singleton();
}

boost::property_tree::ptree abstract_domain::as_tree() const {
  boost::property_tree::ptree ret;
  ret.push_back(boost::property_tree::ptree::value_type(type_name().str(), build_tree()));
  return ret;
}

// manipulators

void abstract_domain::accept(domain_visitor &visitor) const {
  visitor.visit(this, true);
}



