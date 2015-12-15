/** @file "TeleoReactor.hh"
 * @brief Declares the main interface for a Teleo-Reactor
 *
 * Defines the basis to implement a Teleo-reactor
 *
 * @author Frederic Py <fpy@mbari.org> (initial version from Conor McGann)
 * @ingroup transaction
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
#ifndef H_trex_transaction_reactor
# define H_trex_transaction_reactor

# include "bits/reactor_policies.hh"

# include "Tick.hh"

# include <trex/domain/token.hh>

namespace trex {
  namespace transaction {
    
    class reactor:boost::noncopyable, public ENABLE_SHARED_FROM_THIS<reactor> {
    public:
      typedef details::reactor_factory factory;
      typedef details::reactor_desc    xml_arg_type;
    
      /** @brief Declare a reactor to the factory
       * @tparam Ty the type of the reactor
       *
       * This class is used to gloablly declare a reactor type @p Ty
       * to the reactor factory used by T_REX to parse xml agent 
       * definition.
       */
      template<class Ty>
      class declare;
      
      /** @brief Destructor 
       */
      virtual ~reactor();
      
      utils::symbol name() const;
      TICK latency() const;
      TICK lookahead() const;
      
      bool internal(utils::symbol const &name) const;
      bool external(utils::symbol const &name) const;

      SHARED_PTR<details::node_impl> impl() const {
        return m_impl;
      }
      
    protected:
      virtual void init_complete() {}
      virtual void new_tick(TICK date) =0;
      virtual boost::optional<TICK> synchronize(TICK next_synch,
                                                TICK max_synch,
                                                ERROR_CODE &ec) =0;
      
      
      void provide(utils::symbol const &name, bool g = true, bool p = false);
      token_ref  create_obs(utils::symbol const &tl,
                            utils::symbol const &pred);
      void post(token_ref obs);
      virtual void disowned(utils::symbol tl);
      
      
      void use(utils::symbol const &name, bool g = true, bool p = false);
      virtual void unsubscribed(utils::symbol tl);
      virtual void notify(token_id obs) {}

      
      /** @brief Constructor
       *
       * @param[in] cfg A reactor factory argument
       *
       * Create a new instance based on the information embedded in @p arg
       * @p arg embeds notably a xml structure which is as follow:
       * @code
       * <ReactorType name="reactor_name" lookahead="int_value" latency="int_value">
       * </reactorType>
       * @endcode
       */
      explicit reactor(xml_arg_type &cfg);
      
      /** @brief Extract XML tree
       *
       * @param[in] arg A reactor factory argument
       *
       * Extract the xml tree from @p arg 
       *
       * @return The xml tree encapsulated in @p arg
       */
      static boost::property_tree::ptree::value_type &xml(xml_arg_type const &arg);
      utils::log::stream syslog(utils::symbol const &kind) const;
      
      void isolate();

    private:
      SHARED_PTR<details::node_impl> m_impl;
      
      // Graph managed stuff
      void attached();
      
      reactor() DELETED;
      
      friend class details::graph_impl;
      friend class details::node_impl;
    }; // class TREX::transaction::reactor
    
# define IN_H_trex_transaction_reactor
#  include "bits/reactor_declare.tcc"
# undef IN_H_trex_transaction_reactor
    
  } // TREX::transaction
} // TREX


#endif
