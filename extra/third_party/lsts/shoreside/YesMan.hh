/*
 * YesMan.h
 *
 *  Created on: May 9, 2013
 *      Author: Margarida
 */

#ifndef H_lsts_YESMAN
#define H_lsts_YESMAN

#include <trex/transaction/TeleoReactor.hh>

namespace TREX {
	namespace LSTS {

		class YesMan :public TREX::transaction::TeleoReactor {
		public:
			YesMan(TREX::transaction::TeleoReactor::xml_arg_type arg);
			virtual ~YesMan();
		private:
		      bool synchronize();
		      void handleRequest(TREX::transaction::goal_id const &g);
		};
	}
}

#endif /* H_lsts_YESMAN */
