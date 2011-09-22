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
#ifndef H_TransactionPlayer
# define H_TransactionPlayer

# include "TeleoReactor.hh"

namespace TREX {
  namespace transaction {

    class Player :public TeleoReactor {
    public:
      Player(TeleoReactor::xml_arg_type arg);
      ~Player();

    private:
      void handleTickStart();
      bool synchronize();

      void loadTransactions(rapidxml::xml_node<> &root);

      class transaction {
      public:
	transaction() {}
	virtual ~transaction() {}

	virtual void accept(Player &) = 0;
      };

      class timeline_transaction :public transaction {
      public:
	timeline_transaction(rapidxml::xml_node<> &node);
	virtual ~timeline_transaction() {}

	TREX::utils::Symbol const &name() const {
	  return m_name;
	}	
      protected:
	TREX::utils::Symbol m_name;
      };

      class op_provide :public timeline_transaction {
      public:
	op_provide(rapidxml::xml_node<> &node)
	  :timeline_transaction(node) {}
	~op_provide() {}

	void accept(Player &p) {
	  p.provide(name());
	}
      };

      class op_use :public timeline_transaction {
      public:
	op_use(rapidxml::xml_node<> &node)
	  :timeline_transaction(node) {}
	~op_use() {}

	void accept(Player &p) {
	  p.use(name());
	}
      };

      class op_unprovide :public timeline_transaction {
      public:
	op_unprovide(rapidxml::xml_node<> &node)
	  :timeline_transaction(node) {}
	~op_unprovide() {}

	void accept(Player &p) {
	  p.syslog("WARN")<<"unprovide not replayable yet : I am killing myself.";
	  p.m_continue = false;
	}
      };

      class op_unuse :public timeline_transaction {
      public:
	op_unuse(rapidxml::xml_node<> &node)
	  :timeline_transaction(node) {}
	~op_unuse() {}

	void accept(Player &p) {
	  p.syslog("WARN")<<"unprovide not replayable yet : I'll just do nothing.";
	}
      };

      class op_assert :public transaction {
      public:
	op_assert(rapidxml::xml_node<> &node);
	~op_assert() {}

	void accept(Player &p) {
	  p.postObservation(m_obs);
	}
	  
      private:
	Observation m_obs;
      };

      static void clear(std::list<transaction *> &l);

      std::map< TICK, std::list<transaction *> > m_exec;
      bool m_continue;
      
      friend class op_provide;
      friend class op_use;
      friend class op_assert;
    }; // TREX::transaction::Player 

  } // TREX::transaction
} // TREX

#endif // H_TransactionPlayer
