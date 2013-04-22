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
#ifndef H_trex_europa_EuropaDomain
# define H_trex_europa_EuropaDomain

# include <trex/domain/DomainBase.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Domain.hh>
# include <trex/europa/bits/system_header.hh>


namespace TREX {
  namespace europa {
    namespace details {

      /** @brief Proxy between TREX and Europa domains
       * 
       * This class acts as a proxy between Europa domain represention
       * and the TREX one.
       *
       * It allows to quickly access to europa domain information while
       * postponning the conversion into a TREX domain until needed.
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      class EuropaDomain :public TREX::transaction::DomainBase {
      public:
	static TREX::utils::Symbol const type_name;

	explicit EuropaDomain(EUROPA::Domain const &dom);
	EuropaDomain(EuropaDomain const &other);
	~EuropaDomain();
	
	TREX::transaction::DomainBase *copy() const;
	// std::ostream &toXml(std::ostream &out, size_t tabs) const;
      
	
	bool isInterval() const {
	  return m_dom->isInterval();
	}
	bool isEnumerated() const {
	  return m_dom->isEnumerated();
	}
	bool isFull() const;
	bool isSingleton() const {
	  return m_dom->isSingleton();
	}
	bool intersect(TREX::transaction::DomainBase const &other) const;
	bool equals(TREX::transaction::DomainBase const &other) const;

	TREX::transaction::DomainBase &restrictWith(TREX::transaction::DomainBase const &other);
	EUROPA::Domain const &europaDomain() const {
	  return *m_dom;
	}
	
      private:
        boost::property_tree::ptree build_tree() const;
        
	EUROPA::Domain *m_dom;
	
	static EUROPA::Domain *safe_copy(EUROPA::Domain *dom);

	std::ostream &print_domain(std::ostream &out) const;
	boost::any singleton() const;
	std::string stringSingleton() const;

      }; // TREX::europa::details::EuropaDomain

    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_trex_europa_EuropaDomain
