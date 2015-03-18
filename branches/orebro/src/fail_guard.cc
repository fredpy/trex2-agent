#include <trex/transaction/TeleoReactor.hh>

using namespace TREX::transaction;
namespace tlog=TREX::utils::log;

/** @brief A simple reactor that will kill trex as soon as it observes Failed
 *
 * A very simple reactor that will terminate t-REX is it receive the 
 * Failed observation from one of the timelines it observes 
 */
class fail_guard :public TeleoReactor {
public:
  /** @brief constructor 
   * @param[in] arg A XML structure
   * Create a new fail guard reactor using the description given by the 
   * XML node @p arg
   *
   * The format of the XML node is as follow 
   * <FailSafe name="name" lookahead="0" latency="0">
   *   <External name="a_timeline_name" />
   *   <External name="another_timeline" />
   *   <!-- ... -->
   * </FailSafe>
   */
  fail_guard(xml_arg_type arg)
 :TeleoReactor(arg, true, false),m_failed(false) {}
  /** @brief Destructor
   */
  ~fail_guard() {}

private:
  /** @brief External observation reception
   * @param[in] obs A new observation
   *
   * This method is called by T-REX whenever an external timeline of 
   * this reactor did produce a new observation @p obs
   * 
   * If @p obs is Failed then this reactor will display an error message
   * and schedule an @c exit() for the next time its synchronize is called
   */
  void notify(Observation const &obs);
  /** @brief stynchronization method
   *
   * The method called by T_REX during synchronization of each ticks.
   * If this reactor observed a Failed since last synchronization it will 
   * then force T_REX to exit
   */
  bool synchronize();

  bool m_failed;
}; // fail_guard

void fail_guard::notify(Observation const &obs) {
  if( obs.predicate()==Predicate::failed_pred() ) {
    // Write this on TREX.log as an error
    syslog(tlog::error)<<"Failed observation from "<<obs.object()
		     <<"I will kill T-REX";
    // Write this message en standard error output
    std::cerr<<"Terminating T-REX due to "<<obs<<std::endl;
    // set me up for killin trex
    m_failed = true;
  }
}

bool fail_guard::synchronize() {
  if( m_failed )
    exit(1); // When m_failed just do a dirty exit
  // return 
  //  - true to indicate that synchronization succeded
  //  - false to indicate that you failed to synchronize 
  //          (but on that case we did call exit already)
  return !m_failed;
}

namespace {
  // Declare this reactor for XML as a <FailSafe> tag
  TeleoReactor::xml_factory::declare<fail_guard> decl("FailSafe");
}
