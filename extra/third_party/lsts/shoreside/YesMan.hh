/*
 * YesMan.h
 *
 *  Created on: May 9, 2013
 *      Author: Margarida
 */

#ifndef H_lsts_YESMAN
#define H_lsts_YESMAN

#include <trex/transaction/reactor.hh>

namespace TREX {
	namespace LSTS {

		class YesMan :public TREX::transaction::reactor {
		public:
			YesMan(TREX::transaction::reactor::xml_arg_type arg);
			virtual ~YesMan();
		private:
		      bool synchronize();
		      void handle_request(TREX::transaction::token_id const &g);
		};
	}
}

#endif /* H_lsts_YESMAN */
