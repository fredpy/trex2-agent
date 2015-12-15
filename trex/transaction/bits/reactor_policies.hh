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
#ifndef H_trex_transaction_bits_reactor_policies
# define H_trex_transaction_bits_reactor_policies

# include "transaction_fwd.hh"

# include <trex/config/memory.hh>
# include <trex/utils/priority_strand.hh>
# include <trex/utils/xml_factory.hh>

# include <boost/thread/once.hpp>

namespace trex {
  namespace transaction {

    class reactor;
    
    namespace details {
      
      typedef SHARED_PTR<utils::priority_strand> exec_ref;

      
      typedef utils::xml_factory< reactor,
                                  SHARED_PTR<reactor>,
                                  SHARED_PTR<details::graph_impl> > reactor_factory;
      
      typedef reactor_factory::argument_type base_arg;
      
      typedef boost::tuple< base_arg::first_type,
                            base_arg::second_type,
                            exec_ref > reactor_desc;

    } // TREX::transaction::details
    
    
    /** @brief Policy of one thread per reactor instance
     *
     * This is the defaut ploicy used wher each reactor created 
     * will execute on its own thread. The advantage of such policy
     * is that it allows all the reactor to run concurrently, the 
     * counterpart of it is that 2 reactors that access to a shared 
     * resource will have to handle the access control explicitely
     */
    struct instance_scope_exec {
      static details::exec_ref init_exec(boost::asio::io_service &io);
    }; // TREX::transaction::instance_scope_exec
    
    /** @brief Policy of one thread shared by a type
     *
     * This policy impose all the reatcors of type Ty to execute on a
     * single thread. By doing so we perohibit concurtrent execution 
     * of these reactors callbacks which simplifies access control of 
     * a resource shared between all reatcors of the type Ty
     */
    template<class Ty>
    struct class_scope_exec {
    public:
      static details::exec_ref init_exec(boost::asio::io_service &io);
      
    private:
      static boost::once_flag                s_once;
      static SHARED_PTR<boost::asio::strand> s_strand;
      
      static void global_init(boost::asio::io_service &io);
    }; // TREX::transaction::class_scope_exec<>
    
    template<class Ty>
    struct exec_policy :public instance_scope_exec {};

# define IN_H_trex_transaction_bits_reactor_policies
# include "reactor_policies.tcc"
# undef  IN_H_trex_transaction_bits_reactor_policies
    
  } // TREX::transaction
} // TREX



#endif // H_trex_transaction_bits_reactor_policies