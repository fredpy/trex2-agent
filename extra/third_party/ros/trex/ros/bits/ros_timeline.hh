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
#ifndef H_trex_ros_bits_ros_timeline
# define H_trex_ros_bits_ros_timeline

# include <trex/utils/platform/cpp11_deleted.hh>
# include <trex/transaction/Goal.hh>
# include <trex/transaction/Observation.hh>
# include <trex/utils/log/stream.hh>
# include <trex/utils/priority_strand.hh>
# include <trex/utils/id_mapper.hh>

namespace TREX {
  namespace ROS {
    
    class ros_reactor;
    class roscpp_initializer;
    
    namespace details {
      
      class ros_timeline :boost::noncopyable {
      public:
        typedef SHARED_PTR<ros_timeline> pointer;
        // TODO: find a way to replace ros_reactor * by WEAK_PTR
        typedef utils::XmlFactory<ros_timeline, pointer, ros_reactor *> xml_factory;
        typedef xml_factory::argument_type                              xml_arg;
        
        typedef pointer       base_type;
        typedef utils::Symbol id_type;
        static id_type get_id(base_type const &elem);
        
        
        virtual ~ros_timeline();
        
        utils::Symbol const &name() const {
          return m_name;
        }
        bool controllable() const {
          return m_controllable;
        }
        
        bool request(transaction::goal_id g);
        void recall(transaction::goal_id g);
        void do_init();
        void do_synchronize();
        
        utils::priority_strand &strand();
        
      protected:
        ros_timeline(xml_arg const &arg, bool control);
        ros_timeline(ros_reactor *r, utils::Symbol const &tl,
                     bool init, bool control);
        
        virtual bool handle_request(transaction::goal_id g) =0;
        virtual void handle_recall(transaction::goal_id g) {}
        
        virtual void synchronize(transaction::TICK date) =0;
        
        bool updated() const {
          return m_updated;
        }
        void notify(transaction::Observation const &obs);
        
        utils::log::stream syslog(utils::log::id_type const &kind=utils::log::null) const;
        
        transaction::Observation new_obs(utils::Symbol const &pred) const {
          return transaction::Observation(name(), pred);
        }
        
        ros_reactor &reactor() {
          return m_reactor;
        }
        roscpp_initializer &ros();
        
      private:
        void init_timeline();
        
        ros_reactor  &m_reactor;
        utils::Symbol m_name;
        bool const m_controllable;
        bool const m_init;
        bool m_undefined, m_updated;
        
        ros_timeline() DELETED;
        
        friend class ros_reactor;
      };
      
    }
    
    typedef details::ros_timeline::xml_factory ros_factory;
  }
}

#endif // H_trex_ros_bits_ros_timeline