/** @file "Platform.hh"
 *  @brief first attempt at LSTS domain model and execution in TREX - platform reactor
 *
 *  @ingroup lsts
 *  @author Jose Pinto <zepinto@gmail.com>
 */

#ifndef H_Platform
# define H_Platform

# include <trex/transaction/TeleoReactor.hh>
# include <Dune/Dune.hpp>
# include <extra/lsts/ControlInterface.hh>
# include "SharedEnvironment.hh"

using DUNE_NAMESPACES;


namespace TREX {
  /** @brief lsts plug-in
   *
   * This namespace is shared by all LSTS reactors/plugins
   * @ingroup lsts
   *
   * @author Jose Pinto <zepinto@gmail.com>
   */
  namespace LSTS {


    /** @brief LSTS platform reactor definition
     *
     * This class implements a reactor that allows access to an LSTS AUV.
     * It provides State, Mode and Payload timelines for the vehicle and listens to the Command timeline
     *
     * @author Jose Pinto <zepinto@gmail.com>
     * @ingroup lsts
     */
    class Platform :public TREX::transaction::TeleoReactor {
    public:
      /** @brief XML constructor
       * @param arg An XML node definition
       *
       * This constructor is called to generate a Platform reactor
       * based on an XML node. The expected XML format is the following:
       * @code
       * <Platform name="<name>" latency="<int>" lookahead="<int>" state="<bool>" duneport="<int>" localport="<int>"/>
       * @endcode
       */
      Platform(TREX::transaction::TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~Platform();

      static void setControlInterface(TREX::LSTS::ControlInterface * itf);
      bool reportToDune(int type, const std::string &context, const std::string &text);
      bool reportToDune(int type, const std::string &message);
      bool reportToDune(const std::string &message);
      bool reportErrorToDune(const std::string &message);
      bool sendMsg(Message& msg);
    private:
      class log_proxy :public TREX::utils::TextLog::handler {
      public:
        explicit log_proxy(Platform &me)
        :m_platform(&me) {}
        log_proxy(log_proxy const &other)
        :m_platform(other.m_platform) {}
        ~log_proxy();

      private:
        void message(boost::optional<date_type> const &date, id_type const &who, id_type const &kind, msg_type const &what);
        Platform *m_platform;
      };
      log_proxy *m_active_proxy;



      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);
      bool sendMsg(Message& msg, std::string ip, int port);
      bool sendMsg(Message& msg, Address &dest);
      bool commandManeuver(const std::string &man_name, IMC::Message * maneuver);

      IMC::Message * commandGoto(const std::string &man_name, double lat, double lon, double depth, double speed, boost::optional<long long> timeout);
      IMC::Message * commandLoiter(const std::string &man_name, double lat, double lon, double depth, double radius, double speed, int seconds);
      IMC::Message * commandStationKeeping(const std::string &man_name, double lat, double lon, double speed, int seconds);
      IMC::Message * commandIdle(const std::string &man_name);
      //IMC::Message commandCalibration();
      void convertToAbsolute(double northing, double easting, double &lat, double &lon);
      void convertToRelative(double lat, double lon, double &x, double &y);
      void processState();
      void handleEntityStates(std::vector<IMC::EntityState> entityStates, IMC::EntityList lastEntityList);
      TREX::transaction::Observation estate(IMC::EstimatedState &msg);
      TREX::transaction::Observation vstate(IMC::VehicleState &msg);
      TREX::transaction::Observation mstate(IMC::ManeuverControlState &msg);
      TREX::utils::SingletonUse<SharedEnvironment> m_env;


      static ControlInterface * controlInterfaceInstance;


      // Network related
      UDPSocket send, receive;
      IOMultiplexing iom;
      uint8_t* bfr;

      /** @brief State of the timeline */
      bool m_on;
      TREX::transaction::TICK m_nextSwitch;
      /** @brief Is the state already posted as observation ? */
      bool m_firstTick;

      Concurrency::Mutex m_mutex;

      /** @brief ip where dune is listening */
      std::string duneip;

      /** @brief port where dune is listening */
      int duneport;

      /** @brief port to be used for listening for incoming IMC messages */
      int localport;

      /** @brief use debug output */
      bool debug;



      void setValue(bool val);

      std::list<TREX::transaction::goal_id> m_pending;

    };

  }
}

#endif // H_Platform
