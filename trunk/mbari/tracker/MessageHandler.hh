#ifndef H_MessageHandler 
# define H_MessageHandler

# include "AMQP_queue.hh"
# include <trex/utils/XmlFactory.hh>
# include <trex/transaction/Observation.hh>
# include <trex/transaction/Goal.hh>

namespace mbari {

  class DrifterTracker;

  class MessageHandler {
  public:
    typedef TREX::utils::XmlFactory<MessageHandler, boost::shared_ptr<MessageHandler>, 
				    DrifterTracker *> factory;
    typedef factory::argument_type  xml_arg;
    
    std::string const &exchange() const {
      return m_exchange;
    }
    std::string const &route() const {
      return m_route;
    }

    MessageHandler(xml_arg const &arg);
    ~MessageHandler() {}

  protected:    
    virtual bool handleMessage(amqp::queue::message &message) =0;
    virtual bool handleRequest(TREX::transaction::goal_id const &g) {
      return false;
    }
    virtual bool synchronize() {
      return true;
    }

    TREX::transaction::TICK now() const;
    double tickToTime(TREX::transaction::TICK date) const;
    double tickDuration() const;

    bool provide(std::string const &timeline, bool control=false);
    void notify(TREX::transaction::Observation const &obs);

    std::string m_exchange;
    std::string m_route;

  private:
    DrifterTracker &m_tracker;

    friend class DrifterTracker;
  }; // mbari::MessageHandler

} // mbari

#endif // H_MessageHandler
