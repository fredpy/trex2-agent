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
#ifndef H_trex_REST_service
# define H_trex_REST_service

# include <Wt/WResource>
# include <Wt/Http/Response>

# include <boost/function.hpp>
# include <boost/shared_ptr.hpp>
# include <boost/property_tree/ptree.hpp>

namespace TREX {
  namespace REST {
    
    class REST_service;
    
    class req_info {
     public:
       typedef boost::property_tree::path  path_type;

       ~req_info() {}

       Wt::Http::Request const &request() const {
         return m_req;
       }
       path_type const &call_path() const {
         return m_call_path;
       }
       path_type const &arg_path() const {
         return m_arg_path;
       }
       std::list<std::string> const &call_list() const {
         return m_call_list;
       }
       std::list<std::string> const &arg_list() const {
         return m_arg_list;
       }

     private:
       explicit req_info(Wt::Http::Request const &r);

       Wt::Http::Request const &m_req;

       std::list<std::string> m_call_list, m_arg_list;
       path_type m_call_path, m_arg_path;


       friend class REST_service;
     };
  namespace bits {
    struct handler_wrap {
        typedef boost::property_tree::ptree fn_output;
        typedef req_info::path_type path_type;



        typedef boost::function<fn_output (req_info const &)> handler_fn;
     handler_wrap() {}

       template<class Handler>
       handler_wrap(Handler cb, std::string const &info="")
       :callback(cb), help(info) {}

       void clear() {
         callback.clear();
         help.clear();
       }

       handler_wrap &swap(handler_wrap &other) {
    	   std::swap(callback, other.callback);
    	   std::swap(help, other.help);
    	   return *this;
       }

       bool active() const {
         return callback;
       }

       handler_fn  callback;
       std::string help;
     };

    }


    class REST_service :public Wt::WResource {
    public:
      typedef boost::property_tree::ptree fn_output;
      typedef req_info::path_type path_type;

      REST_service();
      ~REST_service() {}
      
      
      typedef boost::function<fn_output (req_info const &)> handler_fn;
      
      bool has_handler(std::string const &path) const;
      
      template<class Handler>
      bool add_handler(std::string const &sub_path,
                       Handler cb, std::string const &help="") {
        return add_handler_impl(path_type(sub_path, '/'),
                         bits::handler_wrap(cb, help));
      }
      
      bool remove_handler(std::string const &sub_path, bool childs=false) {
        path_type p(sub_path, '/');
        return remove_handlers_impl(p, m_handlers, childs);
      }
      
    private:
      void handleRequest(Wt::Http::Request const &req,
                         Wt::Http::Response &response);
      
      fn_output print_help(req_info const &req) const;
      
    

      bool add_handler_impl(path_type const &p, bits::handler_wrap const &cb);

      
      typedef boost::property_tree::basic_ptree<std::string, bits::handler_wrap> cb_map;
      
      bool remove_handlers_impl(path_type &p, cb_map &m, bool childs);
      fn_output &build_help(fn_output &, cb_map const &tree,
                            std::list<std::string> &p) const;
      
      cb_map m_handlers;
    };
    
  }
}

#endif
