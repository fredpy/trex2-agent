#ifndef H_DoradoHandler 
# define H_DoradoHandler

# include "MessageHandler.hh"
# include "serie.hh"

namespace mbari {

  class DoradoHandler :public MessageHandler {
  public:
    DoradoHandler(xml_arg const &arg);
    ~DoradoHandler() {}

  private:
    bool handleMessage(amqp::queue::message &msg);
    bool synchronize();

    typedef point<4> sensor_data;
    serie<sensor_data> m_serie;

  }; // mbari::DoradoHandler

} // mbari

#endif // H_DoradoHandler
