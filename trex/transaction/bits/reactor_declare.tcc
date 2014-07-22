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
#ifndef  IN_H_trex_transaction_reactor
# error "tcc files cannot be included outside of their corresponding header"
#else //  IN_H_trex_transaction_reactor

template<class Ty>
class reactor::declare :public reactor::factory::factory_type::producer {
  // ensure that only reactor derived classes can instantiate this
  BOOST_STATIC_ASSERT(boost::is_base_of<reactor, Ty>::value);
  
  using reactor::factory::factory_type::producer::argument_type;
  using reactor::factory::factory_type::producer::result_type;
  using reactor::factory::factory_type::producer::id_param;
public:
  declare(id_param id);
  ~declare() {}
  
private:
  typedef exec_policy<Ty> policy;
  
  result_type produce(argument_type arg) const;
}; // class TREX::transaction::reactor::declare<>

template<class Ty>
reactor::declare<Ty>::declare(typename reactor::declare<Ty>::id_param id)
:reactor::factory::factory_type::producer(id) {
  reactor::factory::factory_type::producer::notify();
}

template<class Ty>
typename reactor::declare<Ty>::result_type
reactor::declare<Ty>::produce(typename reactor::declare<Ty>::argument_type arg) const {
  // Idenitfy the execution policy for this reactor type
  details::exec_ref exec = policy::init_exec(details::service_of(arg.second));
  // inject this policy in the argument
  xml_arg_type rarg(arg.first, arg.second, exec);
  
  // create the new reactor
  return MAKE_SHARED<Ty>(boost::ref(rarg));
}

#endif //  IN_H_trex_transaction_reactor

