#ifndef WITREGRAPH
#define WITREGRAPH

#include<Wt/WPaintedWidget>
#include<Wt/WPainter>
#include<Wt/WPaintDevice>
#include<Wt/WPointF>
#include <Wt/WRectArea>

#include "WitreServer.hh"

#include <trex/transaction/TeleoReactor.hh>

# if defined(__clang__)
// clang annoys me by putting plenty of warnings on unused variables from boost
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-variable"
#  pragma clang diagnostic ignored "-Wunneeded-internal-declaration"
# endif

# include <boost/graph/depth_first_search.hpp>

# if defined(__clang__)
#  pragma clang diagnostic pop
# endif

#include <map>
#include <string>
#include <cmath>
#include <limits>
using namespace std;

using namespace TREX::transaction;

namespace TREX {
    namespace witre {

    class WitreGraph :public boost::dfs_visitor<> {
        public:

            WitreGraph(std::map<utils::Symbol, int> &rel)
                :relation(rel) {};
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

    class WitrePaintSearch :public boost::dfs_visitor<> {
            public:

                class nodeObject
                {
                    public:
                        utils::Symbol name;
                        std::list<utils::Symbol> connections;
                        int level;

                        nodeObject(utils::Symbol name, int level)
                        {
                            this->name = name;
                            this->level = level;
                        }

                        void addConnection(utils::Symbol name)
                        {
                            connections.push_back(name);
                        }
                };

                typedef std::map<utils::Symbol, nodeObject> GraphMap;

                WitrePaintSearch(GraphMap& temp)
                    :graphMap(temp), level(0), highestLevel(0) {};

                virtual ~WitrePaintSearch() {};

                void discover_vertex(graph::reactor_id r, graph const &g)
                {
                    if(is_valid(r))
                    {
                        std::pair<TREX::transaction::TeleoReactor::internal_iterator,
                                    TREX::transaction::TeleoReactor::internal_iterator> internal;
                        internal = boost::in_edges(r,g);
                        TREX::transaction::TeleoReactor::internal_iterator it;
                        for(it = internal.first; it!=internal.second; ++it)
                        {
                            utils::Symbol name = it->name();
                            if(graphMap.find(name)==graphMap.end())
                            {
                                nodeObject node(name,0);
                                graphMap.insert(std::pair<utils::Symbol,nodeObject>(name, node));
                            }
                            for(TeleoReactor::external_iterator temp = r->ext_begin(); temp!=r->ext_end(); ++temp)
                            {
                                if(temp->active())
                                {
                                    utils::Symbol externalName = temp->name();
                                    if(!findName(graphMap.find(name)->second.connections,externalName))
                                    {
                                        graphMap.find(name)->second.addConnection(externalName);
                                    }
                                    if(graphMap.find(externalName)==graphMap.end())
                                    {
                                        nodeObject node(externalName, graphMap.find(name)->second.level+1);
                                        graphMap.insert(std::pair<utils::Symbol,nodeObject>(externalName, node));
                                    }
                                    else
                                    {
                                        int oldLevel = graphMap.find(externalName)->second.level;
                                        int newLevel = graphMap.find(name)->second.level+1;
                                        if(newLevel>oldLevel)
                                        {
                                            int difference = newLevel-oldLevel;
                                            upLevel(externalName, difference);
                                        }
                                    }
                                }
                            }
                        }
                        level++;
                        highestLevel = (level>highestLevel)?level:highestLevel;
                    }
                }

                void upLevel(utils::Symbol name, int difference)
                {
                    std::list<utils::Symbol> called;
                    graphMap.find(name)->second.level += difference;
                    nodeObject* temp = &graphMap.find(name)->second;
                    std::list<utils::Symbol>::iterator it;
                    for(it = temp->connections.begin(); it!= temp->connections.end(); it++)
                    {
                        if(!findName(called,*it))
                        {
                            upLevel(*it,difference);
                            called.push_back(*it);
                        }
                    }
                }

                bool findName(std::list<utils::Symbol> list, utils::Symbol name)
                {
                    std::list<utils::Symbol>::iterator it;
                    for(it = list.begin(); it!=list.end(); it++)
                    {
                        if(*it==name)
                        {
                            return true;
                        }
                    }
                    return false;
                }

                void finish_vertex(graph::reactor_id r, graph const &g)
                {
                    level--;
                }

                int getMaxLevel()
                {
                    return highestLevel;
                }

            private:
                GraphMap& graphMap;
                int level;
                int highestLevel;

                bool is_valid(graph::reactor_id r) const
                {
                    return graph::null_reactor()!=r;
                }

    };

    class PaintedGraph : public Wt::WPaintedWidget
    {
        private:
            const graph& pGraph;
            std::map<utils::Symbol, Wt::WRectF> placement;
            // Wt::WPainter* paint;

        protected:
            void paintEvent(Wt::WPaintDevice* temp)
            {
                Wt::WPainter painter(temp);
                //Getting the graph information to build the image
                WitrePaintSearch::GraphMap graphMap;
                WitrePaintSearch vis(graphMap);
                boost::depth_first_search(pGraph, boost::visitor(vis));

                std::map<int,int> num;

                WitrePaintSearch::GraphMap::iterator it;

                //Drawing the circles and their names
                for(it = graphMap.begin(); it!=graphMap.end(); it++)
                {
                    int level = (*it).second.level;
                    if(num.find(level)==num.end())
                        num[level]=0;
                    //Building rectangle and storing it
                    Wt::WRectF box(num[level]*130,level*130,70,70);
                    placement[(*it).second.name] = box;
                    //Drawing circle with name
                    painter.drawEllipse(box);
                    std::string name = ((*it).second.name.str().size()<8)?(*it).second.name.str():(*it).second.name.str().substr(0,8);
                    painter.drawText(box, Wt::AlignCenter | Wt::AlignMiddle, name);
                    //Increaing number of nodes in level
                    num[level]++;
                }

                //Drawing the lines and the arrows
                for(it = graphMap.begin(); it!=graphMap.end(); it++)
                {
                    std::list<utils::Symbol>& connections = (*it).second.connections;

                    std::list<utils::Symbol>::iterator connect;
                    for(connect = connections.begin(); connect!=connections.end(); connect++)
                    {
                        Wt::WPointF tCenter = placement[(*it).first].center();
                        Wt::WPointF bCenter = placement[(*connect)].center();
                        double slope = getSlope(tCenter, bCenter);
                        Wt::WPainterPath line = drawLine(tCenter,bCenter,slope);
                        painter.drawPath(line);
                        drawArrow(painter,bCenter,slope);
                    }
                }

                //Adds interactive areas to the graph picture
                addInteractiveAreas();
            }

            double getSlope(Wt::WPointF& top, Wt::WPointF& bottom)
            {
                return (bottom.y()-top.y())/(bottom.x()-top.x());
            }

            double shortestPoint(Wt::WPointF& center, const Wt::WPointF& outside)
            {
                if(center.x()==outside.x())
                {
                    if(center.y()>outside.y())
                    {
                        center.setY(center.y()-35);
                    }
                    else
                    {
                        center.setY(center.y()+35);
                    }
                    return numeric_limits<double>::infinity();
                }
                else if(center.y()==outside.y())
                {
                    if(center.x()<outside.x())
                    {
                        center.setX(center.x()-35);
                    }
                    else
                    {
                        center.setX(center.x()+35);
                    }
                    return 0;
                }
                else
                {
                    double slope = (outside.y()-center.y())/(outside.x()-center.x());
                    if(slope>0)
                    {
                        if(center.y()>outside.y())
                        {
                            center.setX(center.x()-(center.x()+35*cos(atan(slope))-center.x()));
                            center.setY(center.y()-(center.y()+35*sin(atan(slope))-center.y()));
                        }
                        else
                        {
                            center.setX(center.x()+35*cos(atan(slope)));
                            center.setY(center.y()+35*sin(atan(slope)));
                        }
                    }
                    else
                    {
                        if(center.y()>outside.y())
                        {
                            center.setX(center.x()+35*cos(atan(slope)));
                            center.setY(center.y()+35*sin(atan(slope)));
                        }
                        else
                        {
                            center.setX(center.x()-(center.x()+35*cos(atan(slope))-center.x()));
                            center.setY(center.y()-(center.y()+35*sin(atan(slope))-center.y()));
                        }
                    }
                    return slope;
                }
            }

            Wt::WPainterPath drawLine(Wt::WPointF& tCenter, Wt::WPointF& bCenter, double& slope)
            {
                Wt::WPointF nullPoint;
                Wt::WPointF topPoint;
                Wt::WPointF bottomPoint;
                std::map<utils::Symbol, Wt::WRectF>::iterator it;
                for(it=placement.begin(); it!=placement.end(); it++)
                {
                    //True if the rect does not contain the two points and any points in slope
                    if((!(*it).second.contains(tCenter) && !(*it).second.contains(bCenter))
                           && intersectRect(tCenter,bCenter,slope,(*it).second))
                    {
                        Wt::WPointF tpoint = (*it).second.topRight();
                        tpoint.setX(tpoint.x()+(*it).second.width()/1.5);
                        Wt::WPointF bpoint = (*it).second.bottomRight();
                        bpoint.setX(bpoint.x()+(*it).second.width()/1.5);
                        if(topPoint==nullPoint || tpoint.y()<topPoint.y())
                        {
                            topPoint = tpoint;
                        }
                        if(bottomPoint==nullPoint || bpoint.y()>bottomPoint.y())
                        {
                            bottomPoint = bpoint;
                        }
                    }
                }
                if(topPoint!=nullPoint && bottomPoint!=nullPoint)
                {
                    shortestPoint(tCenter,topPoint);
                    shortestPoint(bCenter,bottomPoint);
                    slope = getSlope(bCenter, bottomPoint);
                    Wt::WPainterPath* line = new Wt::WPainterPath(tCenter);
                    line->cubicTo(topPoint,bottomPoint,bCenter);
                    return *line;
                }
                else
                {
                    shortestPoint(tCenter,bCenter);
                    shortestPoint(bCenter,tCenter);
                    Wt::WPainterPath* line = new Wt::WPainterPath(tCenter);
                    line->lineTo(bCenter);
                    return *line;
                }
            }

            bool intersectRect(Wt::WPointF& tCenter, Wt::WPointF& bCenter, double& slope, Wt::WRectF& rect)
            {
                if(slope!=numeric_limits<double>::infinity())
                {
                    bool test = false;
                    if(slope>0)
                    {
                        if(rect.x()>=tCenter.x() && rect.x()+rect.width()<=bCenter.x())
                        {
                            test = true;
                        }
                    }
                    else
                    {
                        if(rect.x()>=bCenter.x() && rect.x()+rect.width()<=tCenter.x())
                        {
                            test = true;
                        }
                    }
                    if(test)
                    {
                        double b = tCenter.y() - tCenter.x()*slope;
                        for(int i=rect.x(); i!=rect.x()+rect.width(); i++)
                        {
                            int y = slope*i+b;
                            if(rect.contains(i,y))
                            {
                                return true;
                            }
                        }
                    }
                    return false;
                }
                else
                {
                    if(rect.x()<=tCenter.x()&&rect.x()+rect.width()>=bCenter.x())
                    {
                        if(rect.y()>tCenter.y()&&rect.y()+rect.height()<bCenter.y())
                        {
                            return true;
                        }
                        return false;
                    }
                }
                return false; // fpy : I added this return at the end ...
                              //       I gueess it should return false if all the above ifs failed
            }

            void drawArrow(Wt::WPainter& painter, Wt::WPointF& center, const double& slope)
            {
                const double cosVar = 0.866;
                const double sinVar = 0.500;
                Wt::WPointF tail;
                if(slope == numeric_limits<double>::infinity())
                {
                    tail.setX(center.x());
                    tail.setY(center.y()+10);
                }
                else if(slope>=0)
                {
                    tail.setX(center.x()+10*cos(atan(slope)));
                    tail.setY(center.y()+10*sin(atan(slope)));
                }
                else
                {
                    tail.setX(center.x()-(center.x()+10*cos(atan(slope))-center.x()));
                    tail.setY(center.y()-(center.y()+10*sin(atan(slope))-center.y()));
                }
                double dx = center.x()-tail.x();
                double dy = center.y()-tail.y();
                Wt::WPointF line1;
                line1.setX(center.x()+(dx*cosVar+dy*-sinVar));
                line1.setY(center.y()+(dx*sinVar+dy*cosVar));
                Wt::WPointF line2;
                line2.setX(center.x()+(dx*cosVar+dy*sinVar));
                line2.setY(center.y()+(dx*-sinVar+dy*cosVar));
                painter.drawLine(center,line1);
                painter.drawLine(center,line2);
            }

            void addInteractiveAreas()
            {
                std::map<utils::Symbol, Wt::WRectF>::iterator it;
                for(it=placement.begin(); it!=placement.end(); it++)
                {
                    Wt::WRectArea* area = new Wt::WRectArea((*it).second);
                    area->setToolTip((*it).first.str());
                    this->addArea(area);
                }
            }

        public:
            PaintedGraph(const graph& temp)
                :pGraph(temp)
            {
                resize(1000,1000);
            }

    };

    class WitreGraphContainer : public Wt::WContainerWidget
    {
      public:
          WitreGraphContainer(Wt::WContainerWidget* parent, const graph& temp)
            : Wt::WContainerWidget(parent), paintedGraph(new PaintedGraph(temp))
          {
            this->addWidget(paintedGraph);
          }

          Wt::WContainerWidget* getWidget()
          {
              return this;
          }

      private:
        PaintedGraph * const paintedGraph;
    };

    }
}

#endif
