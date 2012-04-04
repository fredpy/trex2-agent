#ifndef H_MessageHandler 
# define H_MessageHandler

# include "AMQP_queue.hh"
# include <trex/utils/Factory.hh>

namespace mbari {

  class DrifterTracker;

  class MessageHandler {
  public:
    typedef TREX::utils::Factory<MessageHandler, std::string, 
				 DrifterTracker const &> factory;

    MessageHandler(DrifterTracker const &tracker):m_tracker(tracker) {}
    ~MessageHandler() {}

  protected:
    virtual bool handleMessage(amqp::queue::message &message) =0;

    DrifterTracker const &m_tracker;

    friend class DrifterTracker;
  }; // mbari::MessageHandler

} // mbari

#endif // H_MessageHandler
