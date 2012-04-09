#ifndef H_DoradoHandler 
# define H_DoradoHandler

# include "MessageHandler.hh"

namespace mbari {

  class DoradoHandler :public MessageHandler {
  public:
    DoradoHandler(xml_arg const &arg);
    ~DoradoHandler() {}

  private:
    bool handleMessage(amqp::queue::message &msg);
    bool synchronize();
  }; // mbari::DoradoHandler

} // mbari

#endif // H_DoradoHandler
