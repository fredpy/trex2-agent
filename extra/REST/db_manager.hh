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
#ifndef H_trex_rest_db_manager
# define H_trex_rest_db_manager 

# include <trex/transaction/Tick.hh>

# include <Wt/Dbo/SqlConnection>
# include <Wt/Dbo/Session>

namespace TREX {
  namespace REST {
    namespace helpers {
      
      class db_manager {
      public:
        static std::string const db_ext;
        
        class exception : public std::runtime_error {
        public:
          ~exception() throw() {}
          
        protected:
          exception(std::string const &msg) throw():std::runtime_error(msg) {}
          
          friend class db_manager;
        }; // TREX::REST::helpers::db_manager::exception
        
        db_manager();
        ~db_manager();
        
        void initialize(std::string const &file_name);
        
        Wt::Dbo::Session &session();
        Wt::Dbo::Session &operator* () {
          return session();
        }
        Wt::Dbo::Session *operator->() {
          return &session();
        }
        
        void add_timeline(std::string const &name);
        void add_token(transaction::TICK start, transaction::TICK end,
                       std::string const &tl, std::string const &json);
        
        typedef transaction::int_domain::bound bound;
        
        size_t get_tokens(std::string const &tl, bound &min, bound const &max,
                          std::ostream &out, size_t max_count=100);
        
        unsigned long long count(std::string const &name, bound const &min, bound const &max);
        
      private:
        UNIQ_PTR<Wt::Dbo::SqlConnection> m_db;
        Wt::Dbo::Session                 m_session;
      };
      
    }
  }
}

#endif // H_trex_rest_db_manager
