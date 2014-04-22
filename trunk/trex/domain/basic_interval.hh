/* -*- C++ -*- */
/** @file "BasicInterval.hh"
 * @brief Abstract interface for interval domains
 *
 * This file defines the bases class for implementing interval
 * domains in TREX. It offers multiple utilities to have unified
 * access/input/output for this kind of domain.
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
#ifndef H_BasicInterval
# define H_BasicInterval

# include "abstract_domain.hh"

namespace TREX {
  namespace transaction {

    /** @brief Interval Domain base class
     *
     * This class provide a unified abstract interface for implementing
     * interval domains. It especially provide a way to ease and unify
     * the access methods and input/output for all the domain of this kind.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class basic_interval :public abstract_domain {
    public:
      /** @brief Destructor */
      virtual ~basic_interval() {}

      bool is_interval() const {
	return true;
      }
      bool is_enumerated() const {
	return false;
      }

      virtual bool has_lower()  const =0;
      virtual bool has_upper()  const =0;

      bool is_full() const {
	return !( has_lower() || has_upper() );
      }
      
      virtual boost::any get_lower() const =0;
      virtual boost::any get_upper() const =0;

      template<class Ty, bool Safe>
      Ty get_typed_lower() const {
        boost::any val = get_lower();
	if( Safe ) {
          Ty *ret = boost::any_cast<Ty>(&val);
          if( NULL!=ret )
            return *ret;
          else
            return boost::lexical_cast<Ty>(lower_as_string());
	} else 
	  return boost::any_cast<Ty>(val);
      }
           
      template<class Ty, bool Safe>
      Ty get_typed_upper() const {
        boost::any val = get_upper();
	if( Safe ) { 
          Ty *ret = boost::any_cast<Ty>(&val);
          if( NULL!=ret )
            return *ret;
          else
            return boost::lexical_cast<Ty>(upper_as_string());
	} else 
	  return boost::any_cast<Ty>(val);
      }
           
      virtual std::string lower_as_string() const;
      virtual std::string upper_as_string() const;
    protected:
      explicit basic_interval(TREX::utils::symbol const &type)
	:abstract_domain(type) {}
      explicit basic_interval(boost::property_tree::ptree::value_type &node)
	:abstract_domain(node) {}

      void complete_parsing(boost::property_tree::ptree::value_type &node);

      
      virtual void parse_lower(std::string const &val) =0;
      virtual void parse_upper(std::string const &val) =0;
      virtual void parse_singleton(std::string const &val) =0;
      std::ostream &print_singleton(std::ostream &out) const {
	return print_lower(out);
      }
      virtual std::ostream &print_lower(std::ostream &out) const =0;
      virtual std::ostream &print_upper(std::ostream &out) const =0;
      
    private:
      boost::property_tree::ptree build_tree() const;

      
      void accept(domain_visitor &visitor) const;
      std::ostream &print_domain(std::ostream &out) const;
      boost::any singleton() const {
	return get_lower();
      }
      std::string string_singleton() const {
	return lower_as_string();
      }
   }; // TREX::transaction::basic_interval


  } // TREX::transaction
} // TREX 

#endif // H_BasicInterval
