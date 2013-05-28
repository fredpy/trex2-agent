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
# include <boost/iostreams/stream.hpp>

namespace TREX {
  namespace REST {
    
    namespace helpers {
      
      class unprotect_slash {
      public:
        typedef char char_type;
        typedef boost::iostreams::sink_tag category;
        
        explicit unprotect_slash(std::ostream &dest):m_dest(dest), m_protect(false) {}
        unprotect_slash(unprotect_slash const &other):m_dest(other.m_dest), m_protect(other.m_protect) {}
        ~unprotect_slash() {}
        
        std::streamsize write(char_type const *s, std::streamsize n) {
          int i=0;
          for(char_type const *p=s; i<n; ++i, ++p) {
            if( m_protect ) {
              m_protect=false;
              if( '/'!=*p )
                m_dest.put('\\');
            } else if( '\\'==*p ) {
              m_protect = true;
              continue;
            }
            m_dest.put(*p);
          }
          return n;
        }
      private:
        std::ostream &m_dest;
        bool m_protect;
      };
      
      typedef boost::iostreams::stream<unprotect_slash> json_stream;
      
    }
    
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
        
  }
}

#endif
