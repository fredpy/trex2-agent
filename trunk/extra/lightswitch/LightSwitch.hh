/** @file "LightSwitch.hh"
 *  @brief lightswitch plug-in utilities
 *
 *  This file defines the utilities provided by the lightswitch plug-in. 
 *  
 *  @ingroup lightswitch
 *  @author Frederic Py <fpy@mbari.org>
 */
#ifndef H_LightSwitch
# define H_LightSwitch

# include "TeleoReactor.hh"

namespace TREX {
  /** @brief lightswitch plug-in utilities
   *
   * This namespace embeds the classes and functions provided by the
   * lightswitch plug-in
   * 
   * @ingroup lightswitch
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  namespace lightswitch {

    /** @brief Light reactor definition
     *
     * This class implements a very simple reactor that emulates a light
     * switch. It provides a @c light timeline that can be either on
     * (@c Light.On) or off (@c Light.Off)
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup lightswitch
     */
    class Light :public TREX::transaction::TeleoReactor {
    public:
      /** @brief XML constructor
       * @param arg An XML node definition
       *
       * This constructor is called to generate a Light reactor
       * based on an XML node. The expected XML format is the following:
       * @code
       * <Light name="<name>" latency="<int>" lookahead="<int>" state="<bool>"/>
       * @endcode
       *
       * Where @c state is a boolean value indicating which is the initial
       * state of the @c light timeline
       */
      Light(TREX::transaction::TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~Light();      

    private:
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);

      /** @brief State of the timeline */
      bool m_on;
      /** @brief Is the state already posted as observation ? */
      bool m_firstTick;

      void setValue(bool val);
      
      std::list<TREX::transaction::goal_id> m_pending;

      /** @brief Name of the predicate on */
      static TREX::utils::Symbol const onPred;
      /** @brief Name of the predicate off */
      static TREX::utils::Symbol const offPred;
      /** @brief Name of the predicate up */
      static TREX::utils::Symbol const upPred;
      /** @brief Name of the predicate down */
      static TREX::utils::Symbol const downPred;

      /** @brief Name of the timeline for light */
      static TREX::utils::Symbol const lightObj;
      /** @brief Name of the timeline for light */
      static TREX::utils::Symbol const switchObj;
    };

  }
}

#endif // H_LightSwitch
