#include "../SbdMailer.hh"

using namespace mbari::iridium;

int main(int argc, char *arg[]) {
  std::cout<<"Server ... "<<std::endl;
  SbdMailer mailer("mbarimail.mbari.org");
  
  std::cout<<"Login ... "<<std::endl;
  mailer.login(); // Poco::Net::SMTPClientSession::AUTH_NONE, "fpy", "t0bapny.");
  
  std::cout<<"Configure ... "<<std::endl;
  mailer.add_recipient("fpy@mbari.org");
  mailer.set_sender("fpy@mbari.org");
  
  std::string test_msg("<Test number=\"0\" value=\"blah\"/>");
  std::string imei("300025010809770");

  std::cout<<"Send ... "<<std::endl;
  mailer.send(imei, test_msg.c_str(), test_msg.size(), "Ceci est un test");
  
  return 0;
}
