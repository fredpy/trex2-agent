/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef H_TREX_witre_LogProxy 
# define H_TREX_witre_LogProxy

# include <trex/utils/LogManager.hh>
# include <Wt/WObject>
# include <Wt/WSignal>
# include <Wt/Dbo/Dbo>

namespace TREX {
  namespace witre {
    namespace dbo {

      class Msg;
      
      class MsgType {
      public:
        std::string identifier;
        Wt::Dbo::collection< Wt::Dbo::ptr<Msg> > messages;
        
        template<class Action>
        void persist(Action &a) {
          Wt::Dbo::id(a, identifier, "type_id");
          Wt::Dbo::hasMany(a, messages, Wt::Dbo::ManyToOne, "type");
        }
      }; // TREX::witre::dbo::MsgType
      
    } // TREX::witre::dbo
  } // TREX::witre
} // TREX

namespace Wt {
  namespace Dbo {

    template<>
    struct dbo_traits<TREX::witre::dbo::MsgType>
      :public dbo_default_traits {
      typedef std::string IdType;
      
      static IdType invalidId() {
        return std::string();
      }
      
      static const char *surrogateIdField() { return 0; }
    }; // Wt::Dbo::dbo_traits<TREX::witre::dbo::MsgType>
    
  } // Wt::Dbo
} // Wt

namespace TREX {
  namespace witre {
    namespace dbo {
      
      class Msg {
      public:        
        boost::optional<long long> date;
        Wt::Dbo::ptr<MsgType>      type;
        std::string                source;
        std::string                content;
        
        template<class Action>
        void persist(Action &a) {
          Wt::Dbo::field(a, date, "tick");
          Wt::Dbo::belongsTo(a, type, "type");
          Wt::Dbo::field(a, source, "src");
          Wt::Dbo::field(a, content, "content");
        }
      }; // TREX::witre::dbo::Msg

    } // TREX::witre::dbo
    
    class WitreServer;

    class LogProxy :public utils::TextLog::handler, Wt::WObject {
    public:
      LogProxy(LogProxy const &other);
      ~LogProxy();
      
      typedef Wt::Signal<std::string, boost::optional<long long>, std::string, std::string> log_signal;
      
      log_signal &new_entry() {
        return m_log;
      }
      Wt::Dbo::Session &db() const;
            
    private:
      explicit LogProxy(WitreServer &creator);
      
      void message(boost::optional<date_type> const &date, 
                   id_type const &who, id_type const &kind,
                   msg_type const &what);
            
      mutable WitreServer *me;
      log_signal m_log;
      
      friend class WitreServer;
    }; // TREX::witre::LogProxy
    
  } // TREX::witre
} // TREX


#endif // H_TREX_witre_LogProxy
