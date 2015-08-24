// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include <fstream>
#include <sstream>

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>

#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>

#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/roadmap.hh>
#include <hpp/manipulation/graph/node-selector.hh>
#include <hpp/manipulation/graph/guided-node-selector.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/edge.hh>

#include "graph.impl.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using core::SteeringMethodPtr_t;
      using core::SteeringMethodStraight;
      using core::WeighedDistance;
      using core::WeighedDistancePtr_t;
      using CORBA::ULong;

      std::vector <std::string> expandPassiveDofsNameVector (
          const hpp::Names_t& names, const size_t& s)
      {
        assert (s >= names.length ());
        std::vector <std::string> ret (s, std::string ());
        for (CORBA::ULong i=0; i<names.length (); ++i)
          ret [i] = std::string (names[i]);
        return ret;
      }

      Graph::Graph () :
        problemSolver_ (0x0), graph_ ()
      {}

      Long Graph::createGraph(const char* graphName)
        throw (hpp::Error)
      {
        DevicePtr_t robot = problemSolver_->robot ();
        if (!robot) throw Error ("Build the robot first.");
	// Create default steering method to store in edges, until we define a
	// factory for steering methods.
	WeighedDistancePtr_t distance
	  (HPP_DYNAMIC_PTR_CAST (WeighedDistance,
				 problemSolver_->problem ()->distance ()));
	SteeringMethodPtr_t sm;
	if  (distance) {
	  sm = SteeringMethodStraight::create (robot, distance);
	} else {
	  sm = SteeringMethodStraight::create (robot);
	}
        graph_ = graph::Graph::create(graphName, robot, sm);
        graph_->maxIterations (problemSolver_->maxIterations ());
        graph_->errorThreshold (problemSolver_->errorThreshold ());
        problemSolver_->constraintGraph (graph_);
        problemSolver_->problem()->constraintGraph (graph_);
        return (Long) graph_->id ();
      }

      Long Graph::createSubGraph(const char* subgraphName)
        throw (hpp::Error)
      {
        if (!graph_)
          throw Error ("You should create the graph"
              " before creating subgraph.");
        graph::GuidedNodeSelectorPtr_t ns = graph::GuidedNodeSelector::create
          (subgraphName, problemSolver_->roadmap ());
        graph_->nodeSelector(ns);
        return (Long) ns->id ();
      }

      void Graph::setTargetNodeList(const ID subgraph, const hpp::IDseq& nodes)
        throw (hpp::Error)
      {
        graph::GuidedNodeSelectorPtr_t ns;
        try {
          ns = HPP_DYNAMIC_PTR_CAST(graph::GuidedNodeSelector,
              graph::GraphComponent::get(subgraph).lock());
          if (!ns) throw Error ("Not a subgraph");
          graph::Nodes_t nl;
          graph::NodePtr_t node;
          for (int i = 0; i < nodes.length(); ++i) {
            node = HPP_DYNAMIC_PTR_CAST(graph::Node,
                graph::GraphComponent::get(nodes[i]).lock());
            if (!node) throw Error ("The nodes could not be found.");
            nl.push_back (node);
          }
          ns->setNodeList (nl);
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
      }

      Long Graph::createNode(const Long subgraphId, const char* nodeName)
        throw (hpp::Error)
      {
        graph::NodeSelectorPtr_t ns;
        try {
          ns = HPP_DYNAMIC_PTR_CAST(graph::NodeSelector,
              graph::GraphComponent::get(subgraphId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!ns)
          throw Error ("You should create a subgraph "
              " before creating nodes.");

        graph::NodePtr_t node = ns->createNode (nodeName);
        return (Long) node->id ();
      }

      Long Graph::createEdge(const Long nodeFromId, const Long nodeToId, const char* edgeName, const Long w, const bool isInNodeFrom)
        throw (hpp::Error)
      {
        graph::NodePtr_t from, to;
        try {
          from = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeFromId).lock());
          to   = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeToId  ).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!from || !to)
          throw Error ("The nodes could not be found.");

        graph::EdgePtr_t edge = from->linkTo (edgeName, to, w, isInNodeFrom);
        return (Long) edge->id ();
      }

      void Graph::createWaypointEdge(const Long nodeFromId, const Long nodeToId,
          const char* edgeBaseName, const Long nb, const Long w, const bool isInNodeFrom, GraphElements_out out_elmts)
        throw (hpp::Error)
      {
        graph::NodePtr_t from, to;
        try {
          from = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeFromId).lock());
          to   = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeToId  ).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!from || !to)
          throw Error ("The nodes could not be found.");

        std::ostringstream ss; ss << edgeBaseName << "_e" << nb;
        graph::EdgePtr_t edge_pc = from->linkTo (ss.str (), to, w, isInNodeFrom,
						 graph::WaypointEdge::create);
        graph::WaypointEdgePtr_t edge = HPP_DYNAMIC_PTR_CAST (graph::WaypointEdge, edge_pc);
        edge->createWaypoint (nb - 1, edgeBaseName);
        std::list <graph::EdgePtr_t> edges;
        graph::WaypointEdgePtr_t cur = edge;
        while (cur->waypoint <graph::WaypointEdge> ()) {
          cur = cur->waypoint <graph::WaypointEdge> ();
          edges.push_front (cur);
        }
        edges.push_front (cur->waypoint <graph::Edge> ());

        GraphComps_t n, e;
        GraphComp gc;
        e.length ((ULong) edges.size () + 1);
        n.length ((ULong) edges.size ());
        size_t r = 0;
        for (std::list <graph::EdgePtr_t>::const_iterator it = edges.begin ();
            it != edges.end (); it++) {
          gc.name = (*it)->name ().c_str ();
          gc.id = (Long) (*it)->id ();
          e[(ULong) r] = gc;
          gc.name = (*it)->to ()->name ().c_str ();
          gc.id = (Long) (*it)->to ()->id ();
          n[(ULong) r] = gc;
          r++;
        }
        gc.name = edge->name ().c_str ();
        gc.id = (Long) edge->id ();
        e[(ULong) r] = gc;
        out_elmts = new GraphElements;
        out_elmts->nodes = n;
        out_elmts->edges = e;
      }

      void Graph::getGraph (GraphComp_out graph_out, GraphElements_out elmts)
        throw (hpp::Error)
      {
        GraphComps_t comp_n, comp_e;
        GraphComp comp_g, current;

        graph::NodePtr_t n;
        graph::EdgePtr_t e;

        CORBA::ULong len_edges = 0;
        CORBA::ULong len_nodes = 0;
        try {
          // Set the graph values
          graph_out = new GraphComp ();
          graph_out->name = graph_->name ().c_str();
          graph_out->id = (Long) graph_->id ();

          for (std::size_t i = 0;
	       i < graph::GraphComponent::components().size (); ++i) {
            if (i == graph_->id ()) continue;
            graph::GraphComponentPtr_t gcomponent = graph::GraphComponent::get(i).lock();
            if (!gcomponent) continue;
            current.name = gcomponent->name ().c_str ();
            current.id   = (Long) gcomponent->id ();
            n = HPP_DYNAMIC_PTR_CAST(graph::Node, gcomponent);
            e = HPP_DYNAMIC_PTR_CAST(graph::Edge, gcomponent);
            if (n) {
              comp_n.length (len_nodes + 1);
              comp_n[len_nodes] = current;
              len_nodes++;
            } else if (e) {
              comp_e.length (len_edges + 1);
              graph::WaypointEdgePtr_t we = HPP_DYNAMIC_PTR_CAST (
                  graph::WaypointEdge, e);
              if (we) {
                current.start = (Long)we->waypoint<graph::Edge>()->to ()->id ();
                current.end = (Long) e->to ()->id ();
              } else {
                current.start = (Long) e->from ()->id ();
                current.end = (Long) e->to ()->id ();
              }
              comp_e[len_edges] = current;
              len_edges++;
            }
          }
          elmts = new GraphElements;
          elmts->nodes = comp_n;
          elmts->edges = comp_e;
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
      }

      bool Graph::getConfigProjectorStats (ID elmt, ConfigProjStat_out config,
          ConfigProjStat_out path)
        throw (hpp::Error)
      {
        if (!graph_)
          throw Error ("You should create the graph");
        graph::NodePtr_t node;
        graph::EdgePtr_t edge;
        try {
          node = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(elmt).lock());
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, graph::GraphComponent::get(elmt).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (node) {
          ConfigProjectorPtr_t proj =
            graph_->configConstraint (node)->configProjector ();
          if (proj) {
            config.success = (Long) proj->statistics().nbSuccess();
            config.error = (Long) proj->statistics().nbFailure();
            config.nbObs = (Long) proj->statistics().numberOfObservations();
          }
          path.success = 0;
          path.error = 0;
          path.nbObs = 0;
          return true;
        } else if (edge) {
          ConfigProjectorPtr_t proj =
            graph_->configConstraint (edge)->configProjector ();
          if (proj) {
            config.success = (Long) proj->statistics().nbSuccess();
            config.error = (Long) proj->statistics().nbFailure();
            config.nbObs = (Long) proj->statistics().numberOfObservations();
          }
          proj = graph_->pathConstraint (edge)->configProjector ();
          if (proj) {
            path.success = (Long) proj->statistics().nbSuccess();
            path.error = (Long) proj->statistics().nbFailure();
            path.nbObs = (Long) proj->statistics().numberOfObservations();
          }
          return true;
        } else {
          throw Error ("The ID does not exist.");
        }
        return false;
      }

      Long Graph::getWaypoint (const Long edgeId, hpp::ID_out nodeId)
        throw (hpp::Error)
      {
        graph::WaypointEdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::WaypointEdge, graph::GraphComponent::get(edgeId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!edge)
          throw Error ("The edge could not be found.");
        graph::EdgePtr_t waypoint = edge->waypoint <graph::Edge> ();
        nodeId = (Long) waypoint->to ()->id ();
        return (Long) waypoint->id ();
      }

      Long Graph::createLevelSetEdge(const Long nodeFromId, const Long nodeToId, const char* edgeName, const Long w, const bool isInNodeFrom)
        throw (hpp::Error)
      {
        graph::NodePtr_t from, to;
        try {
          from = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeFromId).lock());
          to   = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeToId  ).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!from || !to)
          throw Error ("The nodes could not be found.");

        graph::EdgePtr_t edge = from->linkTo (edgeName, to, w, isInNodeFrom,
					      graph::LevelSetEdge::create);
        return (Long) edge->id ();
      }

      void Graph::setLevelSetConstraints (const Long edgeId,
          const hpp::Names_t& numericalConstraintNames,
          const hpp::Names_t& passiveDofsNames,
          const hpp::Names_t& lockedDofNames)
        throw (hpp::Error)
      {
        graph::LevelSetEdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge, graph::GraphComponent::get(edgeId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!edge)
          throw Error ("The edge could not be found.");
        try {
          std::vector <std::string> pdofNames = expandPassiveDofsNameVector
            (passiveDofsNames, numericalConstraintNames.length ());
          for (CORBA::ULong i=0; i<numericalConstraintNames.length (); ++i) {
            std::string name (numericalConstraintNames [i]),
              pdofName (pdofNames[i]);
            edge->insertConfigConstraint (
                  NumericalConstraint::create (
                    problemSolver_->numericalConstraint(name),
                    problemSolver_->comparisonType (name)
                    ),
                  problemSolver_->passiveDofs (pdofNames [i])
                  );
          }
          for (CORBA::ULong i=0; i<lockedDofNames.length (); ++i) {
            std::string name (lockedDofNames [i]);
            edge->insertConfigConstraint
              (problemSolver_->get <LockedJointPtr_t> (name));
          }
          RoadmapPtr_t roadmap = HPP_DYNAMIC_PTR_CAST (Roadmap, problemSolver_->roadmap());
          if (!roadmap)
            throw Error ("The roadmap is not of type hpp::manipulation::Roadmap.");
          edge->buildHistogram ();
          roadmap->insertHistogram (edge->histogram ());
        } catch (std::exception& err) {
          throw Error (err.what());
        }
      }

      void Graph::isInNodeFrom (const Long edgeId, const bool isInNodeFrom)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, graph::GraphComponent::get(edgeId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!edge)
          throw Error ("The edge could not be found.");
        try {
          edge->isInNodeFrom (isInNodeFrom);
        } catch (std::exception& err) {
          throw Error (err.what());
        }
      }

      void Graph::setNumericalConstraints (const Long graphComponentId,
          const hpp::Names_t& constraintNames,
          const hpp::Names_t& passiveDofsNames)
        throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component = graph::GraphComponent::get(graphComponentId).lock();
        if (!component)
          throw Error ("The ID does not exist.");

        if (constraintNames.length () > 0) {
          try {
            std::vector <std::string> pdofNames = expandPassiveDofsNameVector
              (passiveDofsNames, constraintNames.length ());
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              if (!problemSolver_->numericalConstraint (name))
                throw Error ("The numerical function does not exist.");
              component->addNumericalConstraint (
                  NumericalConstraint::create (
                    problemSolver_->numericalConstraint(name),
                    problemSolver_->comparisonType (name)
                    ),
                  problemSolver_->passiveDofs (pdofNames [i])
                  );
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::setNumericalConstraintsForPath (const Long nodeId,
          const hpp::Names_t& constraintNames,
          const hpp::Names_t& passiveDofsNames)
        throw (hpp::Error)
      {
        graph::NodePtr_t n;
        try {
          n = HPP_DYNAMIC_PTR_CAST(graph::Node, graph::GraphComponent::get(nodeId).lock());
        } catch (std::out_of_range& e) {
          throw Error (e.what());
        }
        if (!n)
          throw Error ("The nodes could not be found.");

        if (constraintNames.length () > 0) {
          try {
            std::vector <std::string> pdofNames = expandPassiveDofsNameVector
              (passiveDofsNames, constraintNames.length ());
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              n->addNumericalConstraintForPath (
                  NumericalConstraint::create (
                    problemSolver_->numericalConstraint(name),
                    problemSolver_->comparisonType (name)
                    ),
                  problemSolver_->passiveDofs (pdofNames [i])
                  );
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::setLockedDofConstraints (const Long graphComponentId,
          const hpp::Names_t& constraintNames)
        throw (hpp::Error)
      {
        graph::GraphComponentPtr_t component = graph::GraphComponent::get(graphComponentId).lock();
        if (!component)
          throw Error ("The ID does not exist.");

        if (constraintNames.length () > 0) {
          try {
            for (CORBA::ULong i=0; i<constraintNames.length (); ++i) {
              std::string name (constraintNames [i]);
              component->addLockedJointConstraint
		(problemSolver_->get <LockedJointPtr_t> (name));
            }
          } catch (std::exception& err) {
            throw Error (err.what());
          }
        }
      }

      void Graph::statOnConstraint (hpp::ID IDedge)
        throw (hpp::Error)
      {
        graph::EdgePtr_t edge;
        size_t id (IDedge);
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge,
              graph::GraphComponent::get(id).lock ());
        } catch (std::exception& e ) {
          throw Error (e.what());
        }
        if (!edge) {
          std::stringstream ss;
          ss << "ID " << id << " is not an edge";
          std::string errmsg = ss.str();
          throw Error (errmsg.c_str());
        }
        try {
          RoadmapPtr_t roadmap = HPP_DYNAMIC_PTR_CAST (Roadmap, problemSolver_->roadmap());
          if (!roadmap)
            throw Error ("The roadmap is not of type hpp::manipulation::Roadmap.");
          roadmap->statAddFoliation (graph_->configConstraint (edge));
        } catch (std::exception& e) {
          throw Error (e.what());
        }
      }

      void Graph::getNode (const hpp::floatSeq& dofArray, ID_out output)
        throw (hpp::Error)
      {
        try {
          vector_t config; config.resize (dofArray.length());
          for (int iDof = 0; iDof < config.size(); iDof++) {
            config [iDof] = dofArray[iDof];
          }
          graph::NodePtr_t node = graph_->getNode (config);
          output = (Long) node->id();
        } catch (std::exception& e) {
          throw Error (e.what());
        }
      }

      CORBA::Boolean Graph::getConfigErrorForNode
      (const hpp::floatSeq& dofArray, ID nodeId, hpp::floatSeq_out error)
	throw (hpp::Error)
      {
	try {
	  vector_t err;
	  graph::GraphComponentPtr_t gc (graph::GraphComponent::get (nodeId));
	  graph::NodePtr_t node (HPP_DYNAMIC_PTR_CAST (graph::Node, gc));
	  if (!node) {
	    std::ostringstream oss;
	    oss << "Graph component " << nodeId << " is not a node.";
	    throw std::logic_error (oss.str ().c_str ());
	  }
          Configuration_t config; config.resize (dofArray.length());
          for (std::size_t iDof = 0; iDof < (std::size_t)config.size();
	       ++iDof) {
            config [iDof] = dofArray[(ULong) iDof];
          }
	  bool res = graph_->getConfigErrorForNode (config, node, err);
	  floatSeq* e = new floatSeq ();
	  e->length ((ULong) err.size ());
	  for (std::size_t i=0; i < (std::size_t) err.size (); ++i) {
	    (*e) [(ULong) i] = err [i];
	  }
	  error = e;
	  return res;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Graph::displayNodeConstraints
      (hpp::ID nodeId, CORBA::String_out constraints) throw (Error)
      {
	graph::GraphComponentPtr_t gc (graph::GraphComponent::get (nodeId));
	graph::NodePtr_t node (HPP_DYNAMIC_PTR_CAST (graph::Node, gc));
	if (!node) {
	  std::ostringstream oss;
	  oss << "Graph component " << nodeId << " is not a node.";
	  throw std::logic_error (oss.str ().c_str ());
	}
	ConstraintSetPtr_t cs (graph_->configConstraint (node));
	std::ostringstream oss;
	oss << (*cs);
	constraints = oss.str ().c_str ();
      }

      void Graph::displayEdgeConstraints
      (hpp::ID edgeId, CORBA::String_out constraints) throw (Error)
      {
	graph::GraphComponentPtr_t gc (graph::GraphComponent::get (edgeId));
	graph::EdgePtr_t edge (HPP_DYNAMIC_PTR_CAST (graph::Edge, gc));
	if (!edge) {
	  std::ostringstream oss;
	  oss << "Graph component " << edgeId << " is not an edge.";
	  throw std::logic_error (oss.str ().c_str ());
	}
	ConstraintSetPtr_t cs (graph_->configConstraint (edge));
	std::ostringstream oss;
	oss << (*cs);
	constraints = oss.str ().c_str ();
      }

      void Graph::display (const char* filename)
        throw (hpp::Error)
      {
        std::cout << *graph_;
        std::ofstream dotfile;
        dotfile.open (filename);
        graph_->dotPrint (dotfile);
        dotfile.close();
      }

      void Graph::getHistogramValue (ID edgeId, hpp::floatSeq_out freq,
          hpp::floatSeqSeq_out values)
        throw (hpp::Error)
      {
	try {
          graph::LevelSetEdgePtr_t edge =
            HPP_DYNAMIC_PTR_CAST(graph::LevelSetEdge,
                graph::GraphComponent::get (edgeId).lock ());
          if (!edge) throw Error ("The edge could not be found.");
          graph::LeafHistogramPtr_t hist = edge->histogram ();
	  floatSeq* _freq = new floatSeq ();
          floatSeqSeq *_values = new floatSeqSeq ();
          _freq->length (hist->numberOfBins ());
          _values->length (hist->numberOfBins ());
          size_type i = 0;
          for (graph::LeafHistogram::const_iterator it = hist->begin ();
              it != hist->end (); ++it) {
            (*_freq)[i] = it->freq ();
            floatSeq v;
            const vector_t& offset = it->value();
            v.length (offset.size());
            for (size_type j = 0; j < offset.size(); ++j)
              v[j] = offset [j];
            (*_values)[i] = v;
            i++;
          }
          freq = _freq;
          values = _values;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
