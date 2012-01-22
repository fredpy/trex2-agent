#ifndef WITREGRAPH
#define WITREGRAPH

#include <trex/transaction/TeleoReactor.hh>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <map>
#include <string>
#include <iostream>
using namespace std;

using namespace TREX::transaction;

namespace TREX {
    namespace witre {

    class WitreGraph :public boost::dfs_visitor<> {
        public:

            WitreGraph(std::map<utils::Symbol, int> &rel)
                :relation(rel), vertex() {};
            virtual ~WitreGraph() {};

            void initialize_vertex(graph::reactor_id r, graph const &g)
            {
                vertex[r] = 0;
            }

            void examine_edge(graph::relation_type const & rel, graph const &g)
            {
                if(rel->active())
                {
                    graph::reactor_id source = boost::source(rel,g);
                    graph::reactor_id target = boost::target(rel, g);
                    vertex[target] = vertex[source]+1;
                    relation.insert( std::pair<utils::Symbol, int>(rel->name(), vertex[source]));
                }
            };

        protected:
            bool is_valid(graph::reactor_id r) const
            {
                return graph::null_reactor()!=r;
            };

        private:
            std::map<graph::reactor_id, int> vertex;
            std::map<utils::Symbol, int>& relation;

    };

    }
}

#endif
