/* -*- C++ -*- */
/** @file "FloatDomain.hh"
 * @brief Float domain representation
 *
 * This files defines a float based domain
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
#ifndef H_FloatDomain
# define H_FloatDomain

# include "interval_domain.hh"

namespace trex {
  namespace transaction {
    
    double round(double d, size_t places);
    double floor(double d, size_t places);
    double ceil(double d, size_t places);
    
    /** @brief Float domain
     *
     * This class implements a domain used to describe floats.
     * The symbolic type name for this class is "float" and it fully
     * support XML serialization.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class float_domain :public interval_domain<double, false> {
    public:
      using typename interval_domain<double, false>::bound;
      
      /** @brief Symbolic type name for this domain
       *
       * This static attibute gives an access to the symbolic name
       * used for FloatDomain. As stated in the main class description
       * its value is "float" but programmers who want to use this name
       * would better use this variable in case it changes in the future
       */
      static utils::symbol const type_str;
      
      /** @brief Constructor
       *
       * Create a full float domain
       */
      float_domain()
      :interval_domain<double, false>(type_str) {}
      /** @brief Constructor
       *
       * @param node A XML node
       *
       * Create a integer domain by parsing @e node.
       *
       * @throw TREX::utils::bad_string_cast One of the attributes is
       * not correctly formatted
       * @throw EmptyDomain the resulting interval is empty
       */
      float_domain(boost::property_tree::ptree::value_type &node)
      :interval_domain<double, false>(node) {}
      /** @brief Constructor
       * @param lb A lower bound
       * @param ub An upper bound
       *
       * Create an integer domain represented by the interval [lb, ub]
       * @pre [lb, ub] is a valid interval (ie not empty)
       * @throw EmptyDomain the resulting domain is empty
       */
      float_domain(bound const &lb, bound const ub)
      :interval_domain<double, false>(type_str, lb, ub) {}
      /** @brief Constructor
       * @param val A value
       *
       * Create a new integer domain with only the value
       * @e val. in other terms the domain is represented by the interval
       * [val, val]
       */
      float_domain(double val)
      :interval_domain<double, false>(type_str, val) {}
      /** @brief Destructor */
      ~float_domain() {}
      
      abstract_domain *copy() const {
        return new float_domain(*this);
      }
      
    }; // FloatDomain
    
  } // TREX::utils
} // TREX

#endif // H_FloatDomain
