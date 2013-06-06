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
#include "REST_service.hh"

#include <Wt/WServer>
#include <Wt/Utils>

#include <trex/utils/ptree_io.hh>


using namespace TREX::REST;
namespace bpt=boost::property_tree;
namespace wht=Wt::Http;

using TREX::REST::helpers::json_stream;

namespace {

  rest_request::path_type cleanup(std::string p) {
    rest_request::path_type me(p, '/'), ret('/');
    
    while( !me.empty() ) {
      std::string cur = me.reduce();
      if( !cur.empty() )
        ret /= rest_request::path_type(cur, '/');
    }
    return ret;
  }
}

std::string TREX::REST::my_url_decode(std::string str) {
  std::string result;
  
  do {
    size_t pos;
    
    pos = str.find_first_of('+');
    
    result += Wt::Utils::urlDecode(str.substr(0, pos));
    if( std::string::npos==pos )
      break;
    
    result += '+';
    str = str.substr(pos+1);
  } while( !str.empty() );
  
  
  return result;
}


/*
 * class TREX::REST::rest_request
 */

rest_request::rest_request(wht::Request const &r)
:m_req(r), m_arg_path(cleanup(r.pathInfo())) {
}

/*
 * class TREX::REST::rest_service
 */

void rest_service::handleRequest(wht::Request const &req,
                                 wht::Response &response) {
  rest_request rest(req);
  try {
    std::ostringstream oss;
    handleRequest(rest, oss, response);
    if( !oss.str().empty() ) {
      response.out()<<oss.str();
    }
  } catch(rest_error const &err) {
    response.setStatus(err.get_code());
    response.setMimeType("text/plain");
    response.out()<<err.what();
  } catch(std::exception const &e) {
    response.setStatus(400);
    response.setMimeType("text/plain");
    response.out()<<"Error from service "<<req.path()<<":\n  "<<e.what();
  } catch(...) {
    response.setStatus(400);
    response.setMimeType("text/plain");
    response.out()<<"Unknown error from service "<<req.path()<<".";
  }
}

/*
 * class service_tree
 */


service_tree::service_tree(Wt::WServer &serv, std::string const &base_path)
:rest_service("List available REST commands.\n"
              "Argument restrict the help to command matching given path."),
 m_server(&serv), m_base(base_path, '/') {
   m_server->addResource(this, (m_base/rest_request::path_type("help", '/')).dump());
   m_services.add("help", service_ptr());
}

void service_tree::handleRequest(rest_request const &req,
                                 std::ostream &data,
                                 Wt::Http::Response &ans) {
  bpt::ptree list, result;
  
  // Note no help on help in this implementation ...
  if( req.arg_path().empty() )
    build_help(m_services, req.arg_path(), list);
  else
    build_help(m_services.get_child(req.arg_path()), req.arg_path(), list);
  
  ans.setMimeType("application/json");
  json_stream json(data);
  result.add_child("help", list);
  TREX::utils::write_json(json, result, true);
}

void service_tree::add_handler(rest_request::path_type const &path,
                               service_ptr const &cmd) {
  if( cmd ) {
    m_services.put(path, cmd);
    m_server->addResource(cmd.get(), (m_base/path).dump());
  }
}


void service_tree::build_help(cb_map const &sub,
                              rest_request::path_type path,
                              bpt::ptree &out) const {  
  if( !path.empty() && path.single() && path.dump()=="help" ) {
    bpt::ptree info;
    info.put("href", (m_base/path).dump());
    info.put("info", help());
    out.push_back(bpt::ptree::value_type("", info));
  }
  if( sub.data() ) {
    bpt::ptree info;
    info.put("href", (m_base/path).dump());
    info.put("info", sub.data()->help());
    out.push_back(bpt::ptree::value_type("", info));
  } 
  for(cb_map::const_iterator i=sub.begin(); sub.end()!=i; ++i)
    build_help(i->second, path/rest_request::path_type(i->first, '/'), out);
}

/*
 * class json_direct
 */

void json_direct::handleRequest(rest_request const &req,
                                std::ostream &data,
                                Wt::Http::Response &ans) {
  ans.setMimeType("application/json");
  json_stream json(data);
  TREX::utils::write_json(json, m_handler(req), true);
}

