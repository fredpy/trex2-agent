#ifndef H_DoradoHandler 
# define H_DoradoHandler

# include <string>

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

    typedef std::map<std::string, double> sensor_data;
    serie<sensor_data> m_serie;
    bool m_updated;
  }; // mbari::DoradoHandler

} // mbari

#endif // H_DoradoHandler
