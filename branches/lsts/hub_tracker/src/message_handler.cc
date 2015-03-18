#include "message_handler.hh"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>

#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace bpt=boost::property_tree;
namespace json=bpt::json_parser;

namespace {
  std::ostream &print_angle(std::ostream &out, long double a) {
    if( a<0 ) {
      a = -a;
      out<<'-';
    }
    
    unsigned val = std::floor(a);
    a -= std::floor(a);
    a *= 60;
    out<<val<<"&#186; ";
    val = std::floor(a);
    a -= std::floor(a);
    a *= 60;
    return out<<val<<"&#39; "<<std::setprecision(4)<<a<<"&#34;";
  }
}

bpt::ptree lsts_hub::compact_json(message_handler::return_type const &ptr) {
  bpt::ptree base, result;
  {
    // convert the message into a ptree through json
    // this is kind of an heavy handed way to do so
    // but it is robust and was obviously the quick
    // and dirty lazy approach
    std::ostringstream oss;
    ptr->toJSON(oss);
    std::istringstream iss(oss.str());
    json::read_json(iss, base);
  }
  // Does the message have a proper timestamp
  boost::optional<std::string> date_str = base.get_optional<std::string>("timestamp");
  
  if( date_str && !date_str->empty() ) {
    try {
      long long int date = boost::lexical_cast<long long int>(*date_str);
      
      if( date>0 ) {
	boost::posix_time::ptime pdate = boost::posix_time::from_time_t(date);
	result.put("timestamp", to_iso_extended_string(pdate));
      }
    } catch(...) {
    }
  }
  
  // Get the type of operation made in Trex
  switch( base.get<int>("op") ) {
  case DUNE::IMC::TrexOperation::OP_POST_TOKEN:
    result.put("op", "observation");
    break;
  case DUNE::IMC::TrexOperation::OP_POST_GOAL:
    result.put("op", "request");
    break;
  case DUNE::IMC::TrexOperation::OP_RECALL_GOAL:
    result.put("op", "recall");
    break;
  default:
    // do not really care of the others
    result.put("op", "unknown");
    return result;
  }
  // if the token have an id get it
  boost::optional<std::string> id = base.get_optional<std::string>("goal_id");
  if( id && id->size()>0 ) 
    result.put("goal_id", *id);
  
  // Now process the token
  bpt::ptree const &token = base.get_child("token");

  result.put("timeline", token.get<std::string>("timeline"));
  result.put("predicate", token.get<std::string>("predicate"));

  // Get the attributs og the token if any
  boost::optional<bpt::ptree const &> attr = token.get_child_optional("attributes");

  if( attr ) {
    // iterate through attributes
    bpt::ptree attrs;

    for(bpt::ptree::const_iterator i=attr->begin(); attr->end()!=i; ++i) {
      std::string name = i->second.get<std::string>("name");
      
      // by convention attributes starting with '_' are intrnal
      // stuff and can be hidden to the user
      if( !name.empty() && name[0]!='_' ) {
	bpt::ptree var;
	var.put("name", name);
        
	std::string lo, hi;
        lo = i->second.get<std::string>("min");
        hi = i->second.get<std::string>("max");

        if( name=="center_lat" || name=="center_lon" || name=="heading" ) {
          // Convert lat, lon and heading in degrees
          long double flo = i->second.get<long double>("min"),
          fhi = i->second.get<long double>("max");
          
          flo *= 180.0*M_1_PI;
          fhi *= 180.0*M_1_PI;
          
          
          std::ostringstream oss;
          oss.precision(8);
          print_angle(oss<<flo<<" (", flo)<<')';
          lo = oss.str();
          oss.str(std::string());
          oss.precision(8);
          print_angle(oss<<fhi<<" (", fhi)<<')';
          hi = oss.str();
        }
	

        // if "min"=="max" then it is a single "value" and I can display
        // it this way
	if( lo==hi || hi.empty() )
	  var.put("value", lo);
	else {
	  var.put("min", lo);
	  var.put("max", hi);
	}

	attrs.push_back(bpt::ptree::value_type("", var));
      }
      result.put_child("attributes", attrs);
    }
  }
  return result;
}

using namespace lsts_hub;

/*
 * class lsts_hub::message_handler
 */ 

// statics 

uint8_t message_handler::hex_char(char c) {
  uint8_t ret = std::tolower(c);
  // this switch may not be very efficient
  // it is probably better to use a if with comparison
  // against the bounds
  switch( ret ) {
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
    ret -= '0';
    break;
  case 'a':
  case 'b':
  case 'c':
  case 'd':
  case 'e':
  case 'f':
    ret -= 'a';
    ret += 10;
    break;
  default:
    throw std::runtime_error(std::string("Cannot parse ")+c);
  }
  return ret;
}

std::vector<uint8_t> message_handler::decode_hex(std::string const &val) {
  size_t len = (val.size()+1)/2; // number of bytes required to decode 
  std::vector<uint8_t> result(len);
  
  for(size_t pos=0, idx=0; pos<len; ++pos, idx+=2) {
    uint8_t a=hex_char(val[idx]), b=0;
    
    if( idx+1<val.size() )
      b=hex_char(val[idx+1]);    
    
    result[pos] = (a<<4)+b;
  }
  return result;
}

// structors

message_handler::~message_handler() {
  if( !m_pending.empty() ) 
    std::cerr<<"WARNING: "<<m_pending.size()
             <<" multipart messages are incomplete."
	     <<std::endl;
}

// Manipulators 

message_handler::return_type message_handler::process_part
(DUNE::IMC::MessagePart *part) {
  std::auto_ptr<DUNE::IMC::MessagePart> ref(part);
  parts_cache::iterator pos;
  
  pos = m_pending.find(ref->uid);
  if( m_pending.end()==pos ) {
    boost::shared_ptr<defrag_type> tmp(new defrag_type);
    
    pos = m_pending.insert(std::make_pair(ref->uid, tmp)).first;
  }
  return_type msg(pos->second->setFragment(ref.get()));
  if( msg )
    m_pending.erase(pos);
  return msg;
}


message_handler::return_type message_handler::process
(message_handler::argument_type content) {
  std::auto_ptr<DUNE::IMC::Message> ptr;
  
  {
    DUNE::IMC::ImcIridiumMessage imsg;
    std::vector<uint8_t> msg = decode_hex(content);

    imsg.deserialize(msg.data(), msg.size());
    ptr.reset(imsg.msg);
    imsg.msg = NULL;
  }
  
  if( DUNE::IMC::MessagePart::getIdStatic()==ptr->getId() ) 
    return process_part(dynamic_cast<DUNE::IMC::MessagePart *>(ptr.release()));
  else
    return return_type(ptr.release());
}
