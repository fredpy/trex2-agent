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
#ifndef H_trex_europa_TimePoint 
# define H_trex_europa_TimePoint

# include <trex/europa/config.hh>
# include <PLASMA/Variable.hh>
# include <PLASMA/Token.hh>

namespace TREX {
  namespace europa {
    namespace details {

      class CurrentState;

      class TimePoint :public EUROPA::Variable<EUROPA::IntervalIntDomain> {
      public:
	TimePoint(EUROPA::TokenId const &tok,
		  EUROPA::eint now,
		  EUROPA::LabelStr const &name);
	virtual ~TimePoint();
	
	bool set_date(EUROPA::eint now);
	void relax_date();
	bool commit_date(EUROPA::eint now);
        bool commit_end(EUROPA::eint now);

	void setToken(EUROPA::TokenId tok);
	EUROPA::TokenId const &token() const {
	  return m_tok;
	}
        void pending_discard();

      private:
        void notifyDiscarded(Entity const *entity);
        void updateBase(EUROPA::IntervalIntDomain const &dom);
        
	static EUROPA::IntervalIntDomain future(EUROPA::eint now);
	void create_constraint();
        void detach(bool cstr=true);

	EUROPA::ConstraintId m_dur_cstr, m_end_cstr;
	EUROPA::TokenId      m_tok;
        bool m_pending;

      }; // TREX::europa::details::TimePoint

    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_trex_europa_TimePoint
