/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#ifndef H_trex_ros_cpp_topic
# define H_trex_ros_cpp_topic

# include "roscpp_inject.hh"
# include "bits/ros_timeline.hh"
# include "msg_cvt_traits.hh"
# include "ros_error.hh"

# include <trex/utils/platform/cpp11_deleted.hh>

# include <boost/static_assert.hpp>

# include <ros/ros.h>

namespace TREX {
  namespace ROS {
    
    namespace details {
    
      class handle_proxy:boost::noncopyable {
      public:
        typedef boost::function<utils::log::stream (utils::Symbol)> log_fn;
        
        ~handle_proxy() {}
        
        ros::NodeHandle &handle() {
          return m_handle;
        }
        
      protected:
        explicit handle_proxy(roscpp_initializer &init)
        :m_handle(init.handle()) {}
        
        utils::log::stream log(utils::Symbol const &kind = utils::log::info) {
          return m_log(kind);
        }
        
        void set_log(log_fn f) {
          m_log = f;
        }
        
        ros::NodeHandle &m_handle;
      private:
        log_fn m_log;
        
        handle_proxy() DELETED;
      };
      
      
      template<class Message, bool Goals, class Cvt>
      class publisher_proxy :public handle_proxy {
      public:
        typedef Cvt translator;
        typedef typename translator::message     message_type;
        typedef typename translator::message_ptr message_ptr;

        ~publisher_proxy() {
          if( m_pub )
            m_pub.shutdown();
        }
        
      protected:
        typedef boost::function<void (message_type const &)> dispatch_fn;

        using handle_proxy::set_log;
        using handle_proxy::log;
        
        explicit publisher_proxy(roscpp_initializer &init)
        :handle_proxy(init) {}
        
        void advertise(std::string const &topic) {
          m_pub = m_handle.advertise<Message>(topic, 10, true);
        }
        void publish(Message const &msg) {
          m_pub.publish(msg);
        }
        
        void update_tick(transaction::TICK date) {
          m_next = date+1;
        }
        
        bool add_goal(transaction::goal_id g) {
          // check if the goal can start at next tick
          if( g->getStart().closestTo(m_next)==m_next ) {
            bool should_post = true;
            
            // is there an active goal ?
            if( m_active ) {
              // can the active goal terminate at next tick ?
              if( m_pending.front()->getEnd().closestTo(m_next)==m_next ) {
                // then I could post it if there are no more pending goals
                should_post = (1==m_pending.size());
              }
            }
            if( should_post ) {
              message_ptr tmp = translator::to_ros(*g);
              if( tmp ) {
                m_dispatch(*tmp);
                if( m_active ) {
                  // remove previously active goal
                  m_pending.pop_front();
                }
                m_active = tmp;
                m_pending.push_front(g);
                // make this goal the new active one
                return true; // the goal is now active
              }
              return false; // the goal did not convert somehow
            }
          }
          m_pending.push_back(g); // goal is queued
          return true;
        }
        
        void remove_goal(transaction::goal_id g) {
          if( !m_pending.empty() ) {
            if( m_pending.front()==g ) {
              m_pending.pop_front();
              m_active.reset();
            } else {
              for(std::list<transaction::goal_id>::iterator i=m_pending.begin();
                  m_pending.end()!=i; ++i) {
                if( g==*i ) {
                  m_pending.erase(i);
                  return;
                }
              }
            }
          }
        }
        
        void process_pending(transaction::observation_id obs,
                             transaction::TICK obs_since) {
          transaction::goal_id cur;
          
          if( m_active ) {
            // A goal is currently active
            cur = m_pending.front();
            m_pending.pop_front();
            
            // is my active goal consistent with obs
            if( obs && obs->consistentWith(*cur) &&
               cur->getStart().contains(obs_since) ) {
              // constrain my start
              cur->restrictStart(transaction::IntegerDomain(obs_since));
              
              // check if my goal should terminate
              transaction::TICK end_nxt = cur->getEnd().closestTo(m_next);
              if( end_nxt<m_next ) {
                // it should have ended in the past ...
                log(utils::log::warn)<<"Goal ["<<cur<<"] lasted longer than expected";
                cur.reset();
                m_active.reset();
              } else if( m_next<end_nxt ) {
                // it has to hold a little longer
                publish(**m_active);
                m_pending.push_front(cur);
                return; // I am done here
              } else {
                // I know it could end now: but must it ?
                if( cur->getEnd().closestTo(m_next+1)==m_next ) {
                  log()<<"Stop pusblishing ["<<cur<<"]";
                  // yes it must finish
                  cur.reset();
                  m_active.reset();
                }
              }
            } else {
              if( obs )
                log()<<"Goal ["<<cur<<"] is inconsitent with new state: "<<(*obs)
                  <<" (start="<<obs_since<<")";
              else
                log()<<"Goal ["<<cur<<"] is inconsitent with new state: undefined "
                <<" (start="<<obs_since<<")";
              cur.reset();
              m_active.reset();
            }
          }
          // At this point I need to check if there is a goal that can be started
          while( !m_pending.empty() ) {
            transaction::goal_id cand = m_pending.front();
            transaction::TICK start_nxt = cand->getStart().closestTo(m_next);
            
            if( start_nxt<m_next ) {
              // It had to start in the past
              // TODO: make a more complex test that checks if constent with obs
              m_pending.pop_front();
            } else if( m_next<start_nxt ) {
              // it starts in the future
              break;
            } else {
              // it can start now
              message_ptr tmp = translator::to_ros(*cand);
              
              if( tmp ) {
                m_dispatch(*tmp);
                m_active = tmp;
                return;
              }
            }
          }
          // If I am here and cur is still set: then I can continue publishing m_active
          if( cur ) {
            publish(**m_active);
            m_pending.push_front(cur);
          }
        }
        
        
        void set_dispatch(dispatch_fn const &f) {
          m_dispatch = f;
        }
        
        transaction::goal_id active() {
          transaction::goal_id ret;
          if( !m_pending.empty() ) {
            ret = m_pending.front();
            log()<<"active = ["<<ret<<"]";
          } else
            log()<<"empty goal queue";
          return ret;
        }
        
        
      private:
        ros::Publisher m_pub;
        boost::optional<message_ptr>    m_active;
        std::list<transaction::goal_id> m_pending;
        dispatch_fn m_dispatch;
        
        transaction::TICK m_next;

        publisher_proxy() DELETED;
      };
      
      
      template<class Message, class Cvt>
      class publisher_proxy<Message, false, Cvt>: public handle_proxy {
      public:
        typedef Cvt translator;
        typedef typename translator::message     message_type;
        typedef typename translator::message_ptr message_ptr;

        ~publisher_proxy() {}
        
      protected:
        typedef boost::function<void (message_type const &)> dispatch_fn;

        explicit publisher_proxy(roscpp_initializer &init)
        :handle_proxy(init) {}
        
        using handle_proxy::set_log;
        
        void advertise(std::string const &topic) {
          throw ros_error("Attempted to publish topic \""+topic
                          +"\" without implementation for it");
        }
        void publish(Message const &msg) {
          throw ros_error("attempted to publish on a read-only topic");
        }
        
        void update_tick(transaction::TICK) {}
        bool add_goal(transaction::goal_id) {
          return false;
        }
        void remove_goal(transaction::goal_id g) {}
        void process_pending(transaction::observation_id obs,
                             transaction::TICK obs_since) {}
        
        void set_dispatch(dispatch_fn const &) {}
        transaction::goal_id active() {
          return transaction::goal_id();
        }
      private:
        publisher_proxy() DELETED;
      };
      
    }
    
    
    template<typename Message, bool Goals,
             class Convert = msg_cvt_traits<Message, Goals> >
    class cpp_topic :public details::ros_timeline,
    public details::publisher_proxy<Message, Goals, Convert>  {
      
      BOOST_STATIC_ASSERT(::ros::message_traits::IsMessage<Message>::value);
      typedef details::publisher_proxy<Message, Goals, Convert> publisher;
        
      using publisher::handle;
      using typename publisher::translator;
    
    public:
      using typename publisher::message_type;
      using typename publisher::message_ptr;
      
      
      using details::ros_timeline::xml_factory;
      using details::ros_timeline::xml_arg;

      cpp_topic(xml_arg arg);
      ~cpp_topic();
      
      std::string const &topic() const {
        return m_topic;
      }
      
    private:
      void message(message_ptr msg);
      void dispatch(message_type const &msg);
      
      bool handle_request(transaction::goal_id g) {
        if( g && g->predicate()==m_pred )
          return publisher::add_goal(g);
        return false;
      }
      void handle_recall(transaction::goal_id g) {
        if( g && g->predicate()==m_pred )
          publisher::remove_goal(g);
      }
      void synchronize(transaction::TICK date);

      transaction::observation_id m_last_obs;
      transaction::TICK           m_obs_since;
      bool const m_merge;
      bool m_extend;
      
      std::string const m_topic;
      std::string m_pred;
      
      ::ros::Subscriber m_sub;
    }; // TREX::ROS::cpp_topic<>
    
# define In_H_trex_ros_cpp_topic
#  include "bits/cpp_topic.tcc"
# undef In_H_trex_ros_cpp_topic
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_cpp_topic
