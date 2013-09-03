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
#include "DomainVisitor.hh"

using namespace TREX::transaction;

std::string DomainExcept::build_message(DomainBase const &d, 
				       std::string const &msg) throw() {
  std::ostringstream oss;
  oss<<"Domain "<<d.getTypeName()<<" "<<d<<" : "<<msg;
  return oss.str();
}

void DomainBase::accept(DomainVisitor &visitor) const {
  visitor.visit(this, true);
}

boost::any DomainBase::getSingleton() const {
  if( !isSingleton() )
    throw DomainAccess(*this, ": not a singleton");
  return singleton();
}

std::string DomainBase::getStringSingleton() const {
  if( !isSingleton() )
    throw DomainAccess(*this, ": not a singleton");
  return stringSingleton();
}

boost::property_tree::ptree DomainBase::as_tree() const {
  boost::property_tree::ptree ret;
  ret.push_back(boost::property_tree::ptree::value_type(getTypeName().str(), build_tree()));
  return ret;
}

// friends, etc

namespace TREX {
  namespace transaction {

    std::ostream &operator<<(std::ostream &out, DomainBase const &d) {
      if( d.isFull() )
        return out<<'*';
      else
        return d.print_domain(out);
    }
  }
}


