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

#include <trex/utils/ptree_io.hh>

using namespace TREX::REST;
using namespace TREX::REST::bits;
namespace bpt=boost::property_tree;
namespace wht=Wt::Http;



namespace {
  
  template<class Path>
  std::list<typename Path::key_type> to_list(Path p) {
    std::list<typename Path::key_type> ret;
    while( !p.empty() ) {
      typename Path::key_type key = p.reduce();
      if( !key.empty() )
        ret.push_back(key);
    }
    return ret;
  }

  template<class List>
  typename List::value_type collapse(List const &l,
                                     typename List::value_type const &sep) {
    typename List::value_type ret;
    for(typename List::const_iterator i=l.begin(); l.end()!=i; ++i) {
      ret += sep + *i;
    }
    return ret;
  }
  
  
  template<class Tree, typename Pred, class Path>
  Tree const &walk_path(Tree const &t, Path &path,
                        Pred fn, Path &traversed) {
    if( path.empty() )
      return t;
    else {
      typename Path::value_type next=path.front();
      typename Tree::const_assoc_iterator pos = t.find(next);
      
      if( t.not_found()==pos ) {
        return t;
      } else {
        path.pop_front();
        traversed.push_back(next);
        Tree const &ret = walk_path(pos->second, path, fn, traversed);
        if( fn(ret.data()) )
          return ret;
        else {
          traversed.pop_back();
          path.push_front(next);
          return t;
        }
      }
    }
  }
}

/*
 * TREX::REST::REST_service
 */

// structors

REST_service::REST_service() {
  add_handler("help", boost::bind(&REST_service::print_help, this, _1),
              "List REST commands availables.\n"
              "Argument restrict the help to command matching given path."); 
}

// modifiers

bool REST_service::add_handler_impl(REST_service::path_type const &p,
                                    handler_wrap const &cb) {
  boost::optional<cb_map &> pos = m_handlers.get_child_optional(p);
  
  if( pos ) {
    if( pos->data().active() )
      return false; // Already handled : need to remove it first
    else
      pos->data() = cb;
  } else
    m_handlers.put_child(p, cb_map(cb));
  return true;
}

bool REST_service::remove_handlers_impl(REST_service::path_type &p,
                                        REST_service::cb_map &m,
                                        bool childs) {
  if( p.empty() ) {
    m.data().clear();
    return childs || m.empty();
  } else {
    path_type::key_type key = p.reduce();
    cb_map::assoc_iterator pos = m.find(key);
    
    if( m.not_found()==pos )
      return false;
    else {
      if( remove_handlers_impl(p, pos->second, childs) ) {
        m.erase(m.to_iterator(pos));
        return m.empty() && !m.data().callback;
      } else
        return false;
    }
  }
}


// observers

bool REST_service::has_handler(std::string const &path) const {
  path_type p(path, '/');
  boost::optional<cb_map const &> ret = m_handlers.get_child_optional(p);
  
  return ret && ret->data().callback;
}

REST_service::fn_output &REST_service::build_help(REST_service::fn_output &ret,
                                                  REST_service::cb_map const &t,
                                                  std::list<std::string> &p) const {
  if( t.data().callback ) {
    bpt::ptree info;
    info.put("href", collapse(p, "/"));
    info.put("info", t.data().help);
    ret.push_back(bpt::ptree::value_type("", info));
  }
  
  for(cb_map::const_iterator i=t.begin(); t.end()!=i; ++i) {
    p.push_back(i->first);
    build_help(ret, i->second, p);
    p.pop_back();
  }
  
    
  return ret;
}


// callbacks

void REST_service::handleRequest(wht::Request const &req,
                                 wht::Response &response) {
  req_info tmp(req);
  
  cb_map const &pos = walk_path(static_cast<cb_map const &>(m_handlers),
                                tmp.m_arg_list,
                                boost::bind(&handler_wrap::active, _1),
                                tmp.m_call_list);
  tmp.m_call_path = path_type(collapse(tmp.m_call_list, "/"), '/');
  tmp.m_arg_path = path_type(collapse(tmp.m_arg_list, "/"), '/');
  
  if( pos.data().active() ) {
    try {
      TREX::utils::write_json(response.out(), pos.data().callback(tmp));
      response.setMimeType("application/json");
    } catch(std::exception const &err) {
      response.setStatus(400);
      response.out()<<"Error from service "<<tmp.request().path()
      <<tmp.call_path().dump()<<":\n  "<<err.what();
    } catch(...) {
      response.setStatus(400);
      response.out()<<"Unknown error from service "<<tmp.request().path()<<tmp.call_path().dump();
    }
  } else {
    response.setStatus(404);
    response.out()<<"No service associated to \""<<req.pathInfo()<<'\"'
      <<std::endl;
  }
}

REST_service::fn_output REST_service::print_help(req_info const &req) const {
  path_type tmp(req.request().path(), '/');
  
  std::list<std::string> base(to_list(tmp)), extra(req.arg_list());
  
  cb_map const &pos = walk_path(m_handlers, extra,
                                boost::bind(&handler_wrap::active, _1),
                                base);  
  bpt::ptree ret,help;
  
  ret.add_child("help", build_help(help, pos, base));
  return ret;
}



/*
 * TREX::REST::REST_service::req_info
 */

req_info::req_info(wht::Request const &r)
:m_req(r), m_call_path('/'), m_arg_path(r.pathInfo(), '/') {
  m_arg_list = to_list(m_arg_path);
}

