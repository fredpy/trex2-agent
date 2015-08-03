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
# include "ros_error.hh"
# include "bits/cpp_topic_base.hh"
# include "bits/handle_proxy.hh"
# include "msg_cvt_traits.hh"

# include <trex/utils/platform/cpp11_deleted.hh>
# include <boost/static_assert.hpp>

# include <ros/ros.h>

namespace TREX {
  namespace ROS {
    
    namespace details {
      
      
      template<typename Message, bool Goals, typename Cvt>
      class publish_proxy :public handle_proxy {
      public:
        typedef Cvt translator;
        typedef typename translator::message     message_type;
        typedef typename translator::message_ptr message_ptr;
        
        ~publish_proxy() {
          if( m_pub )
            m_pub.shutdown();
        }
        
      protected:
        publish_proxy(roscpp_initializer &init)
        :handle_proxy(init) {}
        
        void advertise(std::string const &topic) {
          ros::NodeHandle &h = handle();
          m_pub = h.advertise<Message>(topic, 10, true);
        }
        
        void update_next(transaction::TICK date,
                         boost::optional<transaction::Observation> const &obs,
                         transaction::TICK since) {
          transaction::goal_id cur;
          m_next = date+1;
          
          
          if( m_active_cmd ) {
            cur = m_pending.front();
            m_pending.pop_front();
          
            // check if my active goal is still compatible
            if( obs && obs->consistentWith(*cur) &&
               cur->getStart().contains(since) ) {
              cur->restrictStart(transaction::IntegerDomain(since));
              
              transaction::TICK end_nxt = cur->getEnd().closestTo(m_next);
              if( end_nxt<m_next ) {
                log(utils::log::warn)<<"goal ["<<cur<<"] lasted longer than expected.";
                cur.reset();
                m_active_cmd.reset();
              } else if( m_next<end_nxt ) {
                m_pub.publish(**m_active_cmd);
                m_pending.push_front(cur);
                return;
              } else {
                // it can end now but can it end later ?
                if( cur->getEnd().closestTo(m_next+1)==m_next ) {
                  // no it must end now !
                  cur.reset();
                  m_active_cmd.reset();
                }
              }
            } else {
              log(utils::log::warn)<<"["<<cur<<"] is cinconsitent with current state";
              cur.reset();
              m_active_cmd.reset();
            }
          }
          // at this point I need to check if any goal can be started
          while( !m_pending.empty() ) {
            transaction::goal_id cand = m_pending.front();
            transaction::TICK start_nxt = cand->getStart().closestTo(m_next);
            
            if( start_nxt<m_next ) {
              // TODO: make more refined test to see if it could be merged
              //       with obs
              m_pending.pop_front();
            } else if( m_next<start_nxt ) {
              // TODO: may need to check followers
              break;
            } else {
              message_ptr tmp =  translator::to_ros(*cand);
              
              if( tmp ) {
                log(utils::log::info)<<"Sending ["<<cand<<"] to ROS:\n"<<(*tmp);
                m_pub.publish(tmp);
                m_active_cmd = tmp;
                return;
              }
            }
          }
          // At this point if cur is still set I can post it
          if( cur ) {
            m_pub.publish(**m_active_cmd);
            m_pending.push_front(cur);
          }
        }
        
        bool push_goal(transaction::goal_id g) {
          if( g->getStart().closestTo(m_next)==m_next ) {
            bool should_post = true;
            
            if( m_active_cmd ) {
              if( m_pending.front()->getEnd().closestTo(m_next)==m_next ) {
                should_post = (1==m_pending.size());
              }
            }
            
            if( should_post ) {
              message_ptr tmp = translator::to_ros(*g);
              
              if(tmp) {
                log(utils::log::info)<<"Sending ["<<g<<"] to ROS:\n"<<(*tmp);
                m_pub.publish(tmp);
                if( m_active_cmd )
                  m_pending.pop_front();
                m_active_cmd = tmp;
                m_pending.push_front(g);
                return true;
              }
              return false;
            }
          }
          m_pending.push_back(g);
          return true;
        }
        void goal_rm(transaction::goal_id g) {
          if( !m_pending.empty() ) {
            if( m_pending.front()==g ) {
              m_pending.pop_front();
              m_active_cmd.reset();
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

        transaction::TICK next() const {
          return m_next;
        }
        
      private:
        ros::Publisher m_pub;
        
        transaction::TICK m_next;
        boost::optional<message_ptr>    m_active_cmd;
        std::list<transaction::goal_id> m_pending;
        
      };
      
      
      template<typename Message, typename Cvt>
      class publish_proxy<Message, false, Cvt> {
      public:
        typedef Cvt translator;
        typedef typename translator::message     message_type;
        typedef typename translator::message_ptr message_ptr;

        ~publish_proxy() {}
        
        void advertise(std::string const &topic) {
          throw ros_error("This topic handler do not implement goals to ros");
        }
      protected:
        publish_proxy(roscpp_initializer &) {}
        
        void set_log(handle_proxy::log_fn const &) {}
        
        void update_next(transaction::TICK,
                         boost::optional<transaction::Observation> const &,
                         transaction::TICK) {}
        transaction::TICK next() const {
          return 0;
        }
        bool push_goal(transaction::goal_id) {}
        void goal_rm(transaction::goal_id) {}
        
        
      };
      
    }
    
    
    template<typename Message, bool GoalImplemented,
             class Convert = msg_cvt_traits<Message, GoalImplemented> >
    class cpp_topic :public details::cpp_topic_base,
    details::publish_proxy<Message, GoalImplemented, Convert> {
      BOOST_STATIC_ASSERT(::ros::message_traits::IsMessage<Message>::value);
      typedef details::publish_proxy<Message, GoalImplemented, Convert> publisher;
      using typename publisher::translator;
    public:
      using typename publisher::message_type;
      using typename publisher::message_ptr;
      
      cpp_topic(xml_arg arg)
      :details::cpp_topic_base(arg, GoalImplemented, Convert::msg_type()),
      publisher(ros()) {
        publisher::set_log(boost::bind(&details::cpp_topic_base::syslog, this, _1));
      }
      
      ~cpp_topic() {}
      
      using details::cpp_topic_base::topic;
      
    private:
      void init_publisher() {
        publisher::advertise(topic());
      }
      void process_pending(transaction::TICK date) {
        publisher::update_next(date, details::cpp_topic_base::last_obs(),
                               details::cpp_topic_base::obs_since());
      }
      
      void init_subscriber(ros::Subscriber &sub) {
        WEAK_PTR<details::ros_timeline> me(shared_from_this());
        boost::function<void (message_ptr const &)> cb = boost::bind(&cpp_topic::message, me, _1);
        
        sub = ros().handle().subscribe(topic(), 10, cb);
      }
      
      void add_goal(transaction::goal_id g) {
        publisher::push_goal(g);
      }
      void remove_goal(transaction::goal_id g) {
        publisher::goal_rm(g);
      }
      
      void handle_message(message_ptr msg) {
        transaction::Observation obs(new_obs());
        // syslog()<<"Received ros message: "<<(*msg);
        translator::to_trex(msg, obs);
        do_notify(obs);
      }
                          
      static void message(WEAK_PTR<details::ros_timeline> p,
                          message_ptr const &msg) {
        SHARED_PTR<details::ros_timeline> ptr = p.lock();
        if( ptr ) {
          cpp_topic *me = dynamic_cast<cpp_topic *>(ptr.get());
          if( NULL!=me )
            me->handle_message(msg);
        }
      }
    
    };
    
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_cpp_topic
