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

# include <DUNE/DUNE.hpp>
# include <DUNE/Math/Angles.hpp>
# include <DUNE/Coordinates/WGS84.hpp>

# include "../shared/EuropaExtensions.hh"
# include "../shared/ImcAdapter.hh"
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

    static const int TREX_ID = 65000;

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

      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);
      void handleTickStart();
      void handleInit();
      bool sendMsg(Message& msg, std::string ip, int port);

      void processState();
      void handleTrexOperation(TrexOperation trexOp);
      void postGoalToken(std::string goal_id, TrexToken token);
      void postObservationToken(TrexToken token);
      typedef std::map<std::string, boost::shared_ptr<Observation> > obs_map;
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

      /** @brief use debug output */
      bool debug;

      /** @brief whether TREX is currently blocked or not */
      bool m_blocked;

      /** @brief Whether it was connected on last tick */
      bool m_connected;

      /** @brief map of received messages       */
      std::map<uint16_t, IMC::Message *> received;

      /** @brief map of received messages (aggregated) */
      std::map<uint16_t, IMC::Message *> aggregate;

      void setValue(bool val);
      
      void goingAUV(const goal_id& goal);
      void goingUAV(const goal_id& g);
  void postRefAtObservation(FollowRefState* frefstate);

      std::list<TREX::transaction::goal_id> m_goals_pending;
      std::list<TREX::transaction::Observation> m_observations_pending;

      /** @brief received announces since last tick */
      std::map<std::string, Announce *> m_receivedAnnounces;
    };

  }
}

#endif // H_Platform
