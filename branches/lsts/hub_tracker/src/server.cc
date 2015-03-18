#include "server.hh"

#include <Wt/WText>
#include <Wt/WContainerWidget>

using namespace lsts_hub;

namespace {
  template<class Map>
  void forget(Map &m) {
    client::date_type past_day = boost::posix_time::second_clock::universal_time();
    past_day -= boost::posix_time::hours(120);
    
    typename Map::iterator i = m.lower_bound(past_day);
    m.erase(m.begin(), i);
  }

}

server::server(Wt::WServer &wt)
:m_server(wt), m_client(wt.ioService()),
m_date(boost::date_time::not_a_date_time) {
  m_client.on_trex().connect(this, &server::trex_update);
  m_client.on_msg().connect(this, &server::op_update);
  m_client.on_state().connect(this, &server::state_update);
  m_client.on_fresh().connect(this, &server::date_update);
}

server::~server() {}

void server::init(std::string const &url) {
  m_client.connect(url);
}

void server::trex_update(client::date_type when,
                         boost::property_tree::ptree msg) {
  boost::recursive_mutex::scoped_lock lock(m_mtx);
  for(std::set<App *>::const_iterator i=m_clients.begin();
      m_clients.end()!=i; ++i)
    (*i)->trex_update(when, msg);
  forget(m_trex);
  m_trex[when] = msg; // risky as it may remove concurrent messages
}

void server::state_update(client::date_type date) {
  boost::recursive_mutex::scoped_lock lock(m_mtx);
  m_state = date;
  for(std::set<App *>::const_iterator i=m_clients.begin();
      m_clients.end()!=i; ++i)
    (*i)->set_state(date);
}

void server::op_update(client::date_type when,
                       std::string what) {
  boost::recursive_mutex::scoped_lock lock(m_mtx);
  for(std::set<App *>::const_iterator i=m_clients.begin();
      m_clients.end()!=i; ++i)
    (*i)->op_update(when, what);
  forget(m_messages);
  m_messages[when] = what; // risky as it may remove concurrent messages
}

void server::connect(App *cli) {
  boost::recursive_mutex::scoped_lock lock(m_mtx);
  m_clients.insert(cli);
  cli->set_state(m_state);
  for(msg_map::const_iterator i=m_messages.begin();
      m_messages.end()!=i;++i)
    cli->op_update(i->first, i->second);

  for(trex_map::const_iterator i=m_trex.begin();
      m_trex.end()!=i;++i)
    cli->trex_update(i->first, i->second);
  cli->new_date(m_date);
}

void server::disconnect(App *cli) {
  boost::recursive_mutex::scoped_lock lock(m_mtx);
  m_clients.erase(cli);
}

void server::date_update(client::date_type date) {
  boost::recursive_mutex::scoped_lock lock(m_mtx);
  m_date = date;
  for(std::set<App *>::const_iterator i=m_clients.begin();
      m_clients.end()!=i; ++i)
    (*i)->new_date(m_date);
}
