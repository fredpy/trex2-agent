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
      void handleTickStart();
      bool sendMsg(Message& msg, std::string ip, int port);
      bool sendMsg(Message& msg, Address &dest);
//      bool commandManeuver(const std::string &man_name, IMC::Message * maneuver);
      TREX::transaction::Observation maneuverObservation(IMC::Message * man);
      IMC::Message * getManeuverCommand(const std::string &man_name, IMC::Message * maneuver);

      IMC::Message * gotoCommand(const std::string &man_name, double lat, double lon, double depth, double speed, boost::optional<long long> timeout);
      IMC::Message * loiterCommand(const std::string &man_name, double lat, double lon, double depth, double radius, double speed, int seconds);
      IMC::Message * skeepingCommand(const std::string &man_name, double lat, double lon, double speed, int seconds);
      IMC::Message * idleCommand(const std::string &man_name);
      IMC::Message * elevatorCommand(const std::string &man_name, double lat, double lon, double target_depth, double speed, boost::optional<long long> timeout);
      IMC::Message * elevatorCommand(const std::string &man_name, double target_depth, boost::optional<long long> timeout);

      //IMC::Message commandCalibration();
      void convertToAbsolute(double northing, double easting, double &lat, double &lon);
      void convertToRelative(double lat, double lon, double &x, double &y);
      void processState();
      bool uniqueObservation(TREX::transaction::Observation obs);
      typedef std::map<std::string, boost::shared_ptr<TREX::transaction::Observation> > obs_map;
      obs_map postedObservations;
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


      /** @brief whether to use RPM speeds or not */
      bool m_use_rpm;

      /** @brief value of conversion between m/s to RPM speeds */
      double m_rpm_speed_factor;

      /** @brief Default station keeping radius */
      double m_skeeping_radius;

      /** @brief Default loiter radius */
      double m_loiter_radius;

      /** @brief current vehicle position */
      double m_latitude, m_longitude, m_depth;

      /** @brief last sent command (Maneuver message) */
      std::auto_ptr <IMC::Message> sent_command;

      /** @brief whether TREX is currently blocked or not */
      bool m_blocked;

      /** @brief map of received messages       */
      std::map<uint16_t, IMC::Message *> received;

      /** @brief map of received messages (aggregated) */
      std::map<uint16_t, IMC::Message *> aggregate;
      IMC::VehicleCommand lastCommand;

      IMC::Message * commandToBePosted, * postedCommand;
      long long tickWhenToPost;
      long long tickWhenToStop;

      void setValue(bool val);

      std::list<TREX::transaction::goal_id> m_pending;

    };

  }
}

#endif // H_Platform
