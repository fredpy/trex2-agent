/** @file "Platform.hh"
 *  @brief first attempt at LSTS domain model and execution in TREX - platform reactor
 *
 *  @ingroup lsts
 *  @author Jose Pinto <zepinto@gmail.com>
 */

#ifndef H_Platform
# define H_Platform

# include <iostream>

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/IntegerDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/BooleanDomain.hh>
# include <trex/domain/EnumDomain.hh>
# include <boost/thread/thread.hpp>

# include <DUNE/DUNE.hpp>
# include <DUNE/Math/Angles.hpp>
# include <DUNE/Coordinates/WGS84.hpp>

# include <trex/lsts/EuropaExtensions.hh>
# include <trex/lsts/ImcAdapter.hh>
# include "../shared/LstsReactor.hh"
# include "ControlInterface.hh"
# include "SharedEnvironment.hh"


using namespace TREX::transaction;
using namespace TREX::utils;

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
    class Platform : public LstsReactor {
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
      Platform(TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~Platform();

      bool reportToDune(int type, const std::string &context, const std::string &text);
      bool reportToDune(int type, const std::string &message);
      bool reportToDune(const std::string &message);
      bool reportErrorToDune(const std::string &message);
      bool sendMsg(Message& msg);
    private:
      
      template<class IMCMsg>
      IMCMsg *get_msg() {
        return static_cast<IMCMsg *>(received[IMCMsg::getIdStatic()]);
      }
      
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);
      void handleTickStart();
      void handleInit();
      bool sendMsg(Message& msg, std::string ip, int port);

      void processState();
      void handleTrexOperation(TrexOperation trexOp);
      void enqueueGoalToken(std::string goal_id, TrexToken token);
      void postObservationToken(TrexToken token);
      typedef std::map<std::string, SHARED_PTR<Observation> > obs_map;
      typedef std::map<std::string, Announce *> m_links;
      obs_map postedObservations;
      void handleEntityStates(std::vector<IMC::EntityState> entityStates, IMC::EntityList lastEntityList);
      bool handleGoingRequest(goal_id const & g);
      bool handleAtRequest(goal_id const & g);
      bool handleYoYoRequest(goal_id const &goal);
      void handleGoingRecall(goal_id const & g);
      bool handleSurveilRequest(goal_id const &g);

      bool atDestination(FollowRefState * frefstate);
      bool sameReference(const IMC::Reference *msg1, const IMC::Reference *msg2);

      TREX::utils::SingletonUse<SharedEnvironment> m_env;

      //static ControlInterface * controlInterfaceInstance;

      /** @brief Is the state already posted as observation ? */
      bool m_firstTick;

      /** @brief ip where dune is listening */
      std::string duneip;

      /** @brief port where dune is listening */
      int duneport;

      /** @brief port to be used for listening for incoming IMC messages */
      int localport;

      /** @brief IMC ID to use when sending IMC messages */
      int imcid;

      /** @brief use debug output */
      bool debug;

      /** @brief whether TREX is currently blocked or not */
      bool m_blocked;

      /** @brief are we controlling an auv? */
      bool m_auv;

      /** @brief Whether it was connected on last tick */
      bool m_connected;
      TICK m_last_msg, m_max_delta;

      /** @brief map of received messages       */
      std::map<uint16_t, IMC::Message *> received;

      /** @brif vector of received goals */
      std::queue< std::pair<std::string, TREX::transaction::goal_id> > receivedGoals;
      std::queue<Observation> referenceObservations;

      IMC::Reference goingRef;
      bool m_reference_initialized;

      boost::function<bool (goal_id)> m_going_platform;

      /** @brief map of received messages (aggregated) */
      std::map<uint16_t, IMC::Message *> aggregate;

      void setValue(bool val);
      
      bool goingAUV(goal_id goal);
      bool goingUAV(goal_id g);
      //Observation* updateRefAtObservation(FollowRefState* frefstate);
      void insertIntoReceived(IMC::Message* msg);
      void postGoalToken();
      bool isActiveInPlanControlStateMsg(PlanControlState* previous_pcstate);
      void announce(double lat, double lon);
      void enqueueReferenceAtObs();
      DesiredZ setUavRefZ(const double z);

      std::list<TREX::transaction::goal_id> m_goals_pending;
      std::list<TREX::transaction::Observation> m_observations_pending;

      /** @brief received announces since last tick */
      std::map<std::string, Announce *> m_receivedAnnounces;

      //! System resources.
      DUNE::System::Resources m_sys_resources;
    };

  }
}

#endif // H_Platform
