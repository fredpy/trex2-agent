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

# include "DomainBase.hh"

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
    class BasicInterval :public DomainBase {
    public:
      /** @brief Destructor */
      virtual ~BasicInterval() {}

      bool isInterval() const {
	return true;
      }
      bool isEnumerated() const {
	return false;
      }

      virtual bool hasLower()  const =0;
      virtual bool hasUpper()  const =0;

      bool isFull() const {
	return !( hasLower() || hasUpper() );
      }
      
      virtual boost::any getLower() const =0;
      virtual boost::any getUpper() const =0;

      template<class Ty, bool Safe>
      Ty getTypedLower() const {
        boost::any val = getLower();
	if( Safe ) {
          Ty *ret = boost::any_cast<Ty>(&val);
          if( NULL!=ret )
            return *ret;
          else
            return TREX::utils::string_cast<Ty>(getStringLower());
	} else 
	  return boost::any_cast<Ty>(val);
      }
           
      template<class Ty, bool Safe>
      Ty getTypedUpper() const {
        boost::any val = getUpper();
	if( Safe ) { 
          Ty *ret = boost::any_cast<Ty>(&val);
          if( NULL!=ret )
            return *ret;
          else
            return TREX::utils::string_cast<Ty>(getStringUpper());
	} else 
	  return boost::any_cast<Ty>(val);
      }
           
      virtual std::string getStringLower() const;
      virtual std::string getStringUpper() const;
    protected:
      explicit BasicInterval(TREX::utils::Symbol const &type) 
	:DomainBase(type) {}
      explicit BasicInterval(boost::property_tree::ptree::value_type &node)
	:DomainBase(node) {}

      void completeParsing(boost::property_tree::ptree::value_type &node);

      
      virtual void parseLower(std::string const &val) =0;
      virtual void parseUpper(std::string const &val) =0;
      virtual void parseSingleton(std::string const &val) =0;
      std::ostream &print_singleton(std::ostream &out) const {
	return print_lower(out);
      }
      virtual std::ostream &print_lower(std::ostream &out) const =0;
      virtual std::ostream &print_upper(std::ostream &out) const =0;
      
    private:
      boost::property_tree::ptree build_tree() const;

      
      void accept(DomainVisitor &visitor) const;
      std::ostream &print_domain(std::ostream &out) const;
      boost::any singleton() const {
	return getLower();
      }
      std::string stringSingleton() const {
	return getStringLower();
      }
   }; // TREX::transaction::BasicInterval


  } // TREX::transaction
} // TREX 

#endif // H_BasicInterval
