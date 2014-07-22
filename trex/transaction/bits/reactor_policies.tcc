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
#ifndef IN_H_trex_transaction_bits_reactor_policies
# error "tcc files cannot be included outside of their corresponding header"
#else // IN_H_trex_transaction_bits_reactor_policies

/*
 * class TREX::transaction::class_scope_exec<>
 */

// statics

template<class Ty>
boost::once_flag class_scope_exec<Ty>::s_once = BOOST_ONCE_INIT;

template<class Ty>
SHARED_PTR<boost::asio::strand> class_scope_exec<Ty>::s_strand;


template<class Ty>
void class_scope_exec<Ty>::global_init(boost::asio::io_service &io) {
  if( !s_strand )
    s_strand = MAKE_SHARED<boost::asio::strand>(boost::ref(io));
}

template<class Ty>
details::exec_ref class_scope_exec<Ty>::init_exec(boost::asio::io_service &io) {
  boost::call_once(s_once, boost::bind(&class_scope_exec::global_init,
                                       boost::ref(io)));
  return MAKE_SHARED<utils::priority_strand>(s_strand, false);
}

#endif // IN_H_trex_transaction_bits_reactor_policies