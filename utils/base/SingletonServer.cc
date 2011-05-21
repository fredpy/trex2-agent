/** @file "SingletonServer.cc"
 * @brief SingletonServer internal class implmentation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include "bits/SingletonServer.hh"

# include <iostream>
# include <cxxabi.h> 

using namespace TREX::utils::internal;

// statics 

SingletonServer *SingletonServer::s_instance = 0x0;


SingletonServer &SingletonServer::instance() {
  if( 0x0==s_instance ) 
    new SingletonServer;
  return *s_instance;
}

SingletonServer::mutex_type &SingletonServer::sing_mtx() {
  static SingletonServer::mutex_type mtx;
  return mtx;
}

// structors 

SingletonServer::SingletonServer() {
  s_instance = this;
}

SingletonServer::~SingletonServer() {
  s_instance = 0x0;
}

// Modifiers :

SingletonDummy *SingletonServer::attach(std::string const &name,
                                        sdummy_factory const &factory) {
  single_map::iterator i = m_singletons.find(name);

  if( m_singletons.end()==i ) {
    // std::cerr<<name<<" = new Ty()\n";
    i = m_singletons.insert(single_map::value_type(name,
						   factory.create())).first;
  }
  i->second->incr_ref();
  return i->second;
}

bool SingletonServer::detach(std::string const &name) {
  single_map::iterator i = m_singletons.find(name);

  if( m_singletons.end()!=i ) {
    SingletonDummy *dummy = i->second;
    bool del = dummy->decr_ref();
    
    if( del ) {
      m_singletons.erase(i);

      // std::cerr<<name<<" : delete Ty\n";
      delete dummy;
      return m_singletons.empty();
    }
  }
  return false;
}
