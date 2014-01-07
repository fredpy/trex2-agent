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
#ifndef H_trex_EuropaEntity
# define H_trex_EuropaEntity

# include <trex/domain/EnumeratedDomain.hh>

namespace TREX {
  namespace europa {
    
    /** @brief Europa entity domain proxy
     *
     * This class is used to represent europa entities such as objects. It 
     * represent the set of possible values as an enumeration of the name of all 
     * the possible instances.  
     *
     * @author Frederic Py
     * @ingroup europa
     */
    class EuropaEntity 
    :public TREX::transaction::EnumeratedDomain<TREX::utils::symbol> {
      typedef TREX::transaction::EnumeratedDomain<TREX::utils::symbol> base_class;
    public:
      static TREX::utils::symbol const type_name;
      
      /** @brief Constructor
       *
       * Creates a full domain
       */
      EuropaEntity()
	:base_class(type_name) {}
      /** @brief Constructor
       *
       * @tparam Iter An iterator type
       * @param[in] from Intial iterator
       * @param[in] to End iterator
       *
       * Create a new domain that contains all the values pointed by the 
       * [@p from, @p to) iterator interval
       */
      template<class Iter>
      EuropaEntity(Iter from, Iter to)
	:base_class(type_name, from, to) {}
      /** @brief singleton constructor
       *
       * @param[in] val A value
       *
       * Create the domain with the single value @p val
       */
      explicit EuropaEntity(TREX::utils::symbol const &val) 
	:base_class(type_name, val) {}
      /** @brief XML constructor 
       * @param[in] node XML decsciption of the domain
       *
       * Creat a new domain based on the XML description in @p node. The XML 
       * format for this domain is :
       * @code
       * <europa_object>
       *   <elem value="<val1>" />
       *   <elem value="<val2>" />
       *   [...]
       * </europa_object>
       * @endcode
       */
      explicit EuropaEntity(boost::property_tree::ptree::value_type &node)
	:base_class(node) {}
      /** @brief Destructor */
      ~EuropaEntity() {}

       
      TREX::transaction::DomainBase *copy() const {
	return new EuropaEntity(*this);
      }      
    }; // TREX::europa::EuropaEntity
      
  } // TREX::europa
} // TREX

#endif // H_trex_EuropaEntity
