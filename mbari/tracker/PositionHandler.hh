#ifndef H_PositionHandler 
# define H_PositionHandler

# include "MessageHandler.hh"
# include "location.hh"

namespace mbari {

  class PositionHandler :public MessageHandler {
  public:
    PositionHandler(xml_arg const &arg);
    ~PositionHandler() {}
    
  private:
    bool handleMessage(amqp::queue::message &msg);
    bool synchronize();

    bool        m_fresh;
    std::string m_asset;
    location    m_position;
  }; // mbari::PositionHandler

} // mbari 

#endif // H_PositionHandler
