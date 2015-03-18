#include "client.hh"

# include <Wt/WIOService>
# define BOOST_SPIRIT_THREADSAFE
# include <boost/property_tree/json_parser.hpp>

using namespace lsts_hub;

namespace {

  class client_error :public boost::system::error_category {
  public:
    client_error() {}
    ~client_error() {}

    char const *name() const {
      return "hub_client errors";
    }
    std::string message(int ev) const {
      switch (ev) {
        case client::success:
          return "Success";
        case client::server_error:
          return "Received an error from http server";
        default:
          return "unknown error";
      }
    }
  };

  client_error cli_err;
  boost::asio::io_service s_io;
}

namespace bpt=boost::property_tree;
namespace json=bpt::json_parser;

/*
 * class client
 */

client::client(Wt::WIOService &io)
:m_io(io), m_http(io), m_timer(s_io), m_work(s_io),
 m_polling(false), m_last(boost::date_time::not_a_date_time) {
   m_http.setTimeout(30);
   m_http.setMaximumResponseSize(1024*1024);
}

client::~client() {
  m_timer.cancel();
}

void client::connect(std::string const &url) {
  m_base_url = url;
  m_http.done().connect(this, &client::process_request);
  send_request();
}

void client::poll() {
  m_timer.cancel();
  send_request();
}

void client::send_request() {
  if( !m_polling.exchange(true) ) {
    std::string req = m_base_url;
  
    if( !m_last.is_special() )
      req += std::string("?since=")+boost::posix_time::to_iso_extended_string(m_last)+"Z";
    std::cerr<<"Sending request: "<<req<<std::endl;
    m_http.get(req);
    m_state(boost::posix_time::not_a_date_time);
  }
}

void client::process_request(boost::system::error_code ec,
                             Wt::Http::Message msg) {
  if( !ec ) {
    if( msg.status()==200 ) {
      // Deport the processing to when needed
      std::cerr<<"Received "<<msg.body().size()<<" bytes"<<std::endl;
      m_io.post(boost::bind(&client::process_body, this, msg.body()));
    } else {
      m_polling.store(false);
      std::cerr<<"Server error "<<msg.status()<<":\n"
      <<msg.body()<<std::endl;
      m_on_error(boost::system::error_code(server_error, cli_err));
    }
  } else {
    m_polling.store(false);
    std::cerr<<"Http get error: "<<ec.message()<<std::endl;
    m_on_error(ec);
  }
}

void client::process_body(std::string txt) {
  size_t updated = 0;
  try {
    std::istringstream iss(txt);
    bpt::ptree body;
    
    json::read_json(iss, body);
    txt.clear();
    
    for(bpt::ptree::reverse_iterator i=body.rbegin(); body.rend()!=i; ++i) {
      boost::posix_time::ptime msg_date = boost::date_time::not_a_date_time;
      std::string date_str = i->second.get<std::string>("updated_at");
      
      typedef boost::posix_time::time_input_facet facet;
      std::istringstream iss(date_str);
      
      iss.imbue(std::locale(iss.getloc(),
                            new facet("%Y-%m-%dT%H:%M:%S%F")));
      iss>>msg_date;
      
      if( !msg_date.is_special() &&
         (m_last.is_special() || m_last<msg_date) ) {
        m_last = msg_date;
        if( (++updated)>=10 ) {
          m_fresh(m_last);
          updated = 0;
        }
      }
      
      if( DUNE::IMC::ID_IMCMESSAGE==i->second.get<size_t>("type") ) {
        message_handler::return_type
        msg = handle(i->second.get<std::string>("msg"));
        
        if( msg ) {
          std::cerr<<"Processing Dune message"<<std::endl;
          
          uint16_t msg_id = msg->getId();
          
          if( DUNE::IMC::TrexOperation::getIdStatic()==msg_id ) {
            std::cerr<<"Compacting trex message"<<std::endl;
            bpt::ptree tmp = lsts_hub::compact_json(msg);
            if( updated ) {
              // Push the update here to give consistent dates
              m_fresh(m_last);
              updated = 0;
            }
            m_trex(msg_date, tmp);
          } else if( DUNE::IMC::LogBookEntry::getIdStatic()==msg_id ) {
            DUNE::IMC::LogBookEntry *e = dynamic_cast<DUNE::IMC::LogBookEntry *>(msg.get());
            date_type when = boost::posix_time::from_time_t(e->getTimeStamp());
            std::cerr<<"Notifying of op: "<<e->text<<std::endl;
            if( updated ) {
              // Push the update here to give consistent dates
              m_fresh(m_last);
              updated = 0;
            }
            m_msg(when, e->text);
          }
        }
      }
    }
    
  } catch(std::exception const &e) {
    std::cerr<<"Exception during hub response parsing: "<<e.what()<<std::endl;
  } catch(...) {
    std::cerr<<"Unknowm exception during hub response parsing."<<std::endl;
  }
  m_polling.store(false);
  if( updated )
    m_fresh(m_last);
  m_timer.expires_from_now(boost::posix_time::minutes(5));
  m_timer.async_wait(boost::bind(&client::on_timeout, this, _1));
  m_state(m_timer.expires_at());
}

void client::on_timeout(boost::system::error_code const &ec) {
  if( ec!=boost::asio::error::operation_aborted ) {
    send_request();
  }
}

boost::asio::io_service &client::service() {
  return s_io;
}
