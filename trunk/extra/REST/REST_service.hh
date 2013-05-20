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
    
    class rest_service;
    
    class rest_request {
    public:
      typedef boost::property_tree::path path_type;
      
      ~rest_request() {}
      
      Wt::Http::Request const &request() const {
        return m_req;
      }
      path_type const &arg_path() const {
        return m_arg_path;
      }
      
      
    private:
      rest_request(Wt::Http::Request const &r);
      
      Wt::Http::Request const &m_req;
      path_type m_arg_path;
      
      friend class rest_service;
    };
    
    class rest_service :public Wt::WResource {
    public:
      virtual ~rest_service() {}
    
      std::string const &help() const {
        return m_help;
      }
      
    protected:
      rest_service(std::string const &help):m_help(help) {}
      virtual void handleRequest(rest_request const &req,
                                 std::ostream &data,
                                 Wt::Http::Response &ans) =0;
      
      
    private:
      void handleRequest(Wt::Http::Request const &req,
                         Wt::Http::Response &response);
      
      std::string m_help;
    };
    
    class service_tree :public rest_service {
    public:
      typedef boost::shared_ptr<rest_service> service_ptr;

      service_tree(Wt::WServer &serv, std::string const &base_path);
      ~service_tree() {
        beingDeleted();
      }
      
      void add_handler(std::string const &path,
                       rest_service *cmd) {
        add_handler(rest_request::path_type(path, '/'), service_ptr(cmd));
      }
      void add_handler(rest_request::path_type const &path,
                       service_ptr const &cmd);
      
      rest_request::path_type const &base() const {
        return m_base;
      }
      
    private:
      void handleRequest(rest_request const &req,
                         std::ostream &data,
                         Wt::Http::Response &ans);
      
      typedef boost::property_tree::basic_ptree<std::string, service_ptr> cb_map;
      
      void build_help(cb_map const &sub,
                      rest_request::path_type path,
                      boost::property_tree::ptree &out) const;
      
      cb_map                  m_services;
      Wt::WServer            *m_server;
      rest_request::path_type m_base;
    };
    
    class json_direct :public rest_service {
    public:
      typedef boost::property_tree::ptree fn_output;
      typedef boost::function<fn_output (rest_request const &)> handler_fn;
      
      template<class Handler>
      json_direct(Handler cb, std::string const &info)
      :rest_service(info), m_handler(cb) {}
      ~json_direct() {
        beingDeleted();
      }
      
    private:
      handler_fn m_handler;
      
      void handleRequest(rest_request const &req,
                         std::ostream &data,
                         Wt::Http::Response &ans);
      
    };
    
    
//    namespace bits {
//    
//      struct handler_wrap {
//        typedef boost::property_tree::ptree fn_output;
//        typedef req_info::path_type path_type;
//        
//        typedef boost::function<fn_output (req_info const &)> handler_fn;
//        handler_wrap() {}
//        
//        template<class Handler>
//        handler_wrap(Handler cb, std::string const &info="")
//        :callback(cb), help(info) {}
//        
//        void clear() {
//          callback.clear();
//          help.clear();
//        }
//        
//        handler_wrap &swap(handler_wrap &other) {
//          std::swap(callback, other.callback);
//          std::swap(help, other.help);
//          return *this;
//        }
//        
//        bool active() const {
//          return callback;
//        }
//        
//        handler_fn  callback;
//        std::string help;
//      };
//      
//    }
//    
//    
//    class REST_service :public Wt::WResource {
//    public:
//      typedef boost::property_tree::ptree fn_output;
//      typedef req_info::path_type path_type;
//      
//      REST_service();
//      ~REST_service() {
//        beingDeleted();
//      }
//      
//      
//      typedef boost::function<fn_output (req_info const &)> handler_fn;
//      
//      bool has_handler(std::string const &path) const;
//      
//      template<class Handler>
//      bool add_handler(std::string const &sub_path,
//                       Handler cb, std::string const &help="") {
//        return add_handler_impl(path_type(sub_path, '/'),
//                                bits::handler_wrap(cb, help));
//      }
//      
//      
//      bool remove_handler(std::string const &sub_path, bool childs=false) {
//        path_type p(sub_path, '/');
//        return remove_handlers_impl(p, m_handlers, childs);
//      }
//      
//      
//      
//    private:
//      void handleRequest(Wt::Http::Request const &req,
//                         Wt::Http::Response &response);
//      
//      fn_output print_help(req_info const &req) const;
//      
//      bool add_handler_impl(path_type const &p, bits::handler_wrap const &cb);
//      
//      
//      typedef boost::property_tree::basic_ptree<std::string, bits::handler_wrap> cb_map;
//      
//      bool remove_handlers_impl(path_type &p, cb_map &m, bool childs);
//      fn_output &build_help(fn_output &, cb_map const &tree,
//                            std::list<std::string> &p) const;
//      
//      cb_map m_handlers;
//    };
    
  }
}

#endif
