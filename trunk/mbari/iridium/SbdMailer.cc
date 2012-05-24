#include "SbdMailer.hh"

#include <Poco/Net/MailMessage.h>
#include <Poco/Net/FilePartSource.h>
#include <Poco/Net/StringPartSource.h>

using namespace mbari::iridium;

/*
 * class mbari::iridium::SbdMailer
 */ 

// statics

std::string const SbdMailer::s_iridium_address("Data@SBD.Iridium.com");
std::string const SbdMailer::s_sbd_ext("sbd");
size_t SbdMailer::counter(0);

std::string SbdMailer::new_file() {
  std::ostringstream oss;
  oss<<"msg_"<<(++counter)<<'.'<<s_sbd_ext;
  return oss.str();
}


// structors 

SbdMailer::SbdMailer(std::string const &host, Poco::UInt16 port)
  :m_server(host, port) {}

SbdMailer::~SbdMailer() {}

// observers 

size_t SbdMailer::recipients() const {
  return m_recipients.size();
}

bool SbdMailer::has_recipient(std::string const &address) const {
  return m_recipients.find(address)!=m_recipients.end();
}

// modifiers 

void SbdMailer::add_recipient(std::string const &address) {
  m_recipients.insert(address);
}

void SbdMailer::remove_recipient(std::string const &address) {
  m_recipients.erase(address);
}

// manipulators

void SbdMailer::login() {
  m_server.login();
}

void SbdMailer::login(LoginMethod method, std::string const &user_name, 
		      std::string const &password) {
  m_server.login(method, user_name, password);
}

void SbdMailer::close() {
  m_server.close();
}



void SbdMailer::send(std::string imei, char const *data, size_t size,
                     std::string const &comment) {
  Poco::Net::MailMessage msg;
  msg.setSubject(imei);
  for(std::set<std::string>::const_iterator i=m_recipients.begin();
      m_recipients.end()!=i; ++i) {
    Poco::Net::MailRecipient to(Poco::Net::MailRecipient::PRIMARY_RECIPIENT, *i);
    msg.addRecipient(to);
  }
  msg.setSender(sender());
  if( comment.size() > 0 ) {
    Poco::Net::StringPartSource *text = new Poco::Net::StringPartSource(comment);
    msg.addContent(text);
  }
  std::string fname = new_file(), full_name;
  full_name = m_log->file_name(fname).string();
  std::ofstream out(full_name.c_str(), std::ios::binary);
  out.write(data, size);
  out.close();
  Poco::Net::FilePartSource *attachment = new Poco::Net::FilePartSource(full_name);
  msg.addAttachment(fname, attachment);
  m_server.sendMessage(msg);
}




