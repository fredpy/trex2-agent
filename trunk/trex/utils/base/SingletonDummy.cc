/** @file "SingletonDummy.cc"
 * @brief SingletonDummy internal class implmentation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include "bits/SingletonServer.hh"

using namespace TREX::utils::internal;

// statics 

SingletonDummy *SingletonDummy::attach(std::string const &name, 
                                       sdummy_factory const &factory) {
  SingletonServer::lock_type locker(SingletonServer::sing_mtx());
  return SingletonServer::instance().attach(name, factory);
}

void SingletonDummy::detach(std::string const &name) {
  SingletonServer::lock_type locker(SingletonServer::sing_mtx());
  if( SingletonServer::instance().detach(name) ) {
    delete SingletonServer::s_instance;
  }
}


// structors 

SingletonDummy::SingletonDummy() 
  :ref_counter(0ul) {}

SingletonDummy::~SingletonDummy() {}

// modifers

void SingletonDummy::incr_ref() const {
  ++ref_counter;
}

bool SingletonDummy::decr_ref() const {
  return (ref_counter--)<=1;
}


