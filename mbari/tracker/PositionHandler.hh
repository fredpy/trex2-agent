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

    typedef std::pair<bool, location> asset_info;
    typedef std::map<std::string, asset_info> asset_map;
    asset_map m_assets;
  }; // mbari::PositionHandler

} // mbari 

#endif // H_PositionHandler
