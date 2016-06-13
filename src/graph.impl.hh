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

#ifndef HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
# define HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH

# include <hpp/manipulation/problem-solver.hh>
# include <hpp/manipulation/graph/graph.hh>

# include "hpp/corbaserver/manipulation/fwd.hh"
# include "hpp/corbaserver/manipulation/graph.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using hpp::corbaserver::manipulation::Namess_t;
      using hpp::corbaserver::manipulation::Rules;
      using CORBA::Long;

      class Graph : public virtual POA_hpp::corbaserver::manipulation::Graph
      {
        public:
          Graph ();
          void setServer (Server* server)
          {
            server_ = server;
          }

          virtual Long createGraph(const char* graphName)
            throw (hpp::Error);

          virtual Long createSubGraph(const char* subgraphName)
            throw (hpp::Error);

          virtual void setTargetNodeList(const ID subgraph, const hpp::IDseq& nodes)
            throw (hpp::Error);

          virtual Long createNode (const Long subGraphId,
                                   const char* nodeName,
                                   const bool waypoint)
            throw (hpp::Error);

          virtual Long createEdge (const Long nodeFromId,
                                   const Long nodeToId,
                                   const char* edgeName,
                                   const Long weight,
                                   const bool isInNodeFrom)
            throw (hpp::Error);

          virtual Long createWaypointEdge (const Long nodeFromId,
                                           const Long nodeToId,
                                           const char* edgeBaseName,
                                           const Long number,
                                           const Long weight,
                                           const bool isInNodeFrom)
            throw (hpp::Error);

          virtual void setWaypoint (const ID waypointEdgeId, const Long index,
              const ID edgeId, const ID nodeId)
            throw (hpp::Error);

          virtual void getGraph (GraphComp_out graph, GraphElements_out elmts)
            throw (hpp::Error);

          virtual void getEdgeStat (ID edgeId,
              Names_t_out reasons, intSeq_out freqs)
            throw (hpp::Error);

          virtual bool getConfigProjectorStats (ID elmt, ConfigProjStat_out config,
              ConfigProjStat_out path)
            throw (hpp::Error);

          virtual Long getWaypoint (const Long edgeId, const Long index,
              hpp::ID_out nodeId)
            throw (hpp::Error);

          virtual Long createLevelSetEdge(const Long nodeFromId,
                                          const Long nodeToId,
                                          const char* edgeName,
                                          const Long w,
                                          const bool isInNodeFrom)
            throw (hpp::Error);

          virtual void isInNodeFrom (const Long edgeId,
                                     const bool isInNodeFrom)
            throw (hpp::Error);

          virtual void setContainingNode (const ID edgeId, const ID nodeId)
            throw (hpp::Error);

          virtual void setLevelSetFoliation (const Long edgeId,
                                             const hpp::Names_t& condNC,
                                             const hpp::Names_t& condLJ,
                                             const hpp::Names_t& paramNC,
                                             const hpp::Names_t& paramPDOF,
                                             const hpp::Names_t& paramLJ)
            throw (hpp::Error);

          virtual void setNumericalConstraints (const Long graphComponentId,
                                       const hpp::Names_t& constraintNames,
                                       const hpp::Names_t& passiveDofsNames)
            throw (hpp::Error);

	  virtual void resetConstraints(const Long graphComponentId) 
	    throw (hpp::Error);

          virtual void setNumericalConstraintsForPath (const Long nodeId,
              const hpp::Names_t& constraintNames,
              const hpp::Names_t& passiveDofsNames)
            throw (hpp::Error);

          virtual void setLockedDofConstraints (const Long graphComponentId,
                                       const hpp::Names_t& constraintNames)
            throw (hpp::Error);

          virtual void getNode (const hpp::floatSeq& dofArray, ID_out output)
            throw (hpp::Error);

	virtual CORBA::Boolean getConfigErrorForNode
	(const hpp::floatSeq& dofArray, ID nodeId, hpp::floatSeq_out error)
	  throw (hpp::Error);

	virtual void displayNodeConstraints
	(hpp::ID nodeId, CORBA::String_out constraints) throw (Error);

	virtual void displayEdgeConstraints
	(hpp::ID edgeId, CORBA::String_out constraints) throw (Error);

          virtual void display (const char* filename)
            throw (hpp::Error);

          virtual void getHistogramValue (ID edgeId, hpp::floatSeq_out freq,
              hpp::floatSeqSeq_out values)
            throw (hpp::Error);

          virtual void setShort (ID edgeId, CORBA::Boolean isShort)
            throw (hpp::Error);

          virtual intSeq* autoBuild (const char* graphName,
              const Names_t& grippers, const Names_t& objects,
              const Namess_t& handlesPerObject, const Namess_t& shapesPreObject,
	      const Names_t& envNames, const Rules& rulesList)
            throw (hpp::Error);

          virtual void setWeight (ID edgeId, const Long weight)
            throw (hpp::Error);

          virtual Long getWeight (ID edgeId)
            throw (hpp::Error);

        private:
          ProblemSolverPtr_t problemSolver();
          Server* server_;
          graph::GraphPtr_t graph_;
      }; // class Graph
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_GRAPH_IMPL_HH
