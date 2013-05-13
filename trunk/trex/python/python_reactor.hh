/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, MBARI.
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
#ifndef H_trex_python_python_reactor
# define H_trex_python_python_reactor

# include <trex/transaction/TeleoReactor.hh>

# include <boost/python.hpp>

namespace TREX {
  namespace python {
    void log_error(boost::python::error_already_set const &e);
    
    struct python_reactor
    :transaction::TeleoReactor,
    boost::python::wrapper<transaction::TeleoReactor> {
      explicit python_reactor(transaction::TeleoReactor::xml_arg_type &arg)
      :transaction::TeleoReactor(arg) {}
      virtual ~python_reactor() {}
      
      // logging
      void log_msg(utils::Symbol const &type, std::string const &msg);
      void info(std::string const &msg) {
        log_msg(utils::log::info, msg);
      }
      void warn(std::string const &msg) {
        log_msg(utils::log::warn, msg);
      }
      void error(std::string const &msg) {
        log_msg(utils::log::error, msg);
      }
      
      // timeline management
      void ext_use(utils::Symbol const &tl, bool control);
      bool ext_check(utils::Symbol const &tl) const;
      bool ext_unuse(utils::Symbol const &tl);

      void post_request(transaction::goal_id const &g);
      bool cancel_request(transaction::goal_id const &g);
    
      void int_decl(utils::Symbol const &tl, bool control);
      bool int_check(utils::Symbol const &tl) const;
      bool int_undecl(utils::Symbol const &tl);

      void post_obs(transaction::Observation const &obs, bool verb);
      
      // transaction callbacks
      void notify(transaction::Observation const &o);
      void handleRequest(transaction::goal_id const &g);
      void handleRecall(transaction::goal_id const &g);
      
      // exec callbacks
      void handleInit();
      void handleTickStart();
      bool synchronize();
      bool hasWork();
      void resume();
    
    }; // TREX::python::python_reactor
    
    class producer:public transaction::TeleoReactor::xml_factory::factory_type::producer {
    public:
      explicit producer(utils::Symbol const &name);
      ~producer() {}
      
    private:
      result_type produce(argument_type arg) const;
    };
    
  } // TREX::python
} // TREX

#endif // H_trex_python_python_reactor
