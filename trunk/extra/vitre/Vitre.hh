#ifndef H_Vitre
# define H_Vitre

# include <boost/asio.hpp> 

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace vitre {

    /** @brief The vitre interface reactor
     *
     * This class is used as a proxy for the Vitre java interface. It is a
     * simple  reactor that will send through a socket all the observations
     * it received.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup vitre
     */
    class VitreReactor :public TREX::transaction::TeleoReactor {
    public:
      VitreReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~VitreReactor();

    private:
      void handleInit();
      void handleTickStart();
      void notify(TREX::transaction::Observation const &obs);
      bool synchronize();
      void send(std::string const &str);
      std::string m_host;
      std::string m_port;
      boost::asio::ip::tcp::socket m_socket;

    }; // TREX::vitre::VitreReactor

  } // TREX::vitre
} // TREX

#endif // H_Vitre
