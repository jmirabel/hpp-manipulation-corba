// Copyright (c) 2014 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include "problem.impl.hh"

#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/corbaserver/conversions.hh>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/constraints/convex-shape-contact.hh>
#ifdef HPP_CONSTRAINTS_USE_QPOASES
# include <hpp/constraints/qp-static-stability.hh>
#endif
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/graph/node.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/axial-handle.hh>
#include <hpp/manipulation/graph-steering-method.hh>

#include "tools.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      namespace {
        using corbaServer::floatSeqToConfigPtr;
        typedef core::ProblemSolver CPs_t;

        Names_t* jointAndShapes (const JointAndShapes_t& js,
            intSeq_out indexes_out, floatSeqSeq_out points) {
          char** nameList = Names_t::allocbuf((ULong) js.size ());
          Names_t *jointNames = new Names_t ((ULong) js.size(), (ULong) js.size(),
              nameList);

          intSeq* indexes = new intSeq ();
          indexes->length ((ULong) js.size ());
          indexes_out = indexes;

          CORBA::Long rank = 0;
          std::size_t nbPts = 0;
          for (JointAndShapes_t::const_iterator itJs = js.begin ();
              itJs != js.end (); ++itJs) {
            if (itJs->first) {
              nameList[rank] = new char[itJs->first->name().length ()+1];
              strcpy (nameList[rank], itJs->first->name().c_str ());
            } else {
              nameList[rank] = new char[5];
              strcpy (nameList[rank], "NONE");
            }
            nbPts += itJs->second.size ();
            (*indexes)[rank] = (int) nbPts;
            ++rank;
          }

          floatSeqSeq* pts = new hpp::floatSeqSeq ();
          points = pts;
          pts->length ((CORBA::ULong) nbPts);

          rank = 0;
          ULong iJs = 0;
          for (JointAndShapes_t::const_iterator itJs = js.begin ();
              itJs != js.end (); ++itJs) {
            for (std::size_t i = 0; i < itJs->second.size (); ++i) {
              floatSeq p; p.length (3);
              p[0] = itJs->second[i][0];
              p[1] = itJs->second[i][1];
              p[2] = itJs->second[i][2];
              (*pts)[rank] = p;
              ++rank;
            }
            assert ((*indexes)[iJs] == rank);
            ++iJs;
          }
          return jointNames;
        }
      }

      Problem::Problem () : server_ (0x0)
      {
      }

      ProblemSolverPtr_t Problem::problemSolver ()
      {
        return server_->problemSolver();
      }

      graph::GraphPtr_t Problem::graph (bool throwIfNull)
      {
        graph::GraphPtr_t g = problemSolver()->constraintGraph();
        if (throwIfNull && !g)
          throw Error ("You should create the graph");
        return g;
      }

      bool Problem::selectProblem (const char* name)
        throw (hpp::Error)
      {
        std::string psName (name);
        corbaServer::ProblemSolverMapPtr_t psMap (server_->problemSolverMap());
        bool has = psMap->has (psName);
        if (!has) psMap->map_[psName] = ProblemSolver::create ();
        psMap->selected_ = psName;
        return !has;
      }

      void Problem::resetProblem () throw (hpp::Error)
      {
        corbaServer::ProblemSolverMapPtr_t psMap (server_->problemSolverMap());
        delete psMap->map_ [ psMap->selected_ ];
        psMap->map_ [ psMap->selected_ ]
          = manipulation::ProblemSolver::create ();
      }

      Names_t* Problem::getAvailable (const char* what) throw (hpp::Error)
      {
        std::string w (what);
        boost::algorithm::to_lower(w);
        typedef std::list <std::string> Ret_t;
        Ret_t ret;

        if (w == "gripper") {
          ret = getRobotOrThrow (problemSolver())->getKeys <pinocchio::GripperPtr_t, Ret_t> ();
        } else if (w == "handle") {
          ret = getRobotOrThrow (problemSolver())->getKeys <HandlePtr_t, Ret_t> ();
        } else if (w == "robotcontact") {
          ret = getRobotOrThrow (problemSolver())->getKeys <JointAndShapes_t, Ret_t> ();
        } else if (w == "envcontact") {
          ret = problemSolver()->core::ProblemSolver::getKeys <JointAndShapes_t, Ret_t> ();
        } else if (w == "graph") {
          ret = problemSolver()->ProblemSolver::container_t::getKeys <graph::GraphPtr_t, Ret_t> ();
        } else if (w == "type") {
          ret = boost::assign::list_of ("Gripper") ("Handle") ("RobotContact")
            ("EnvContact") ("Graph");
        } else {
          throw Error (("Type \"" + std::string(what) + "\" not known").c_str());
        }

        return toNames_t (ret.begin(), ret.end());
      }

      void Problem::createGrasp (const char* graspName,
				 const char* gripperName,
				 const char* handleName)
	throw (hpp::Error)
      {
	try {
          problemSolver()->createGraspConstraint
            (graspName, gripperName, handleName);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::createPreGrasp (const char* graspName,
                                    const char* gripperName,
                                    const char* handleName)
	throw (hpp::Error)
      {
	try {
          problemSolver()->createPreGraspConstraint
            (graspName, gripperName, handleName);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      Names_t* Problem::getEnvironmentContactNames ()
        throw (hpp::Error)
      {
        try {
	  typedef std::map<std::string, JointAndShapes_t> ShapeMap;
	  const ShapeMap& m = problemSolver()->core::ProblemSolver::map <JointAndShapes_t> ();

	  char** nameList = Names_t::allocbuf((ULong) m.size ());
	  Names_t *jointNames = new Names_t ((ULong) m.size(), (ULong) m.size(),
					     nameList);

	  std::size_t rank = 0;
          for (ShapeMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            nameList[rank] = (char*) malloc (sizeof(char)*(it->first.length ()+1));
            strcpy (nameList [rank], it->first.c_str ());
            rank++;
          }
	  return jointNames;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
        }
      }

      Names_t* Problem::getRobotContactNames ()
        throw (hpp::Error)
      {
        try {
          typedef std::map<std::string, JointAndShapes_t> ShapeMap;
          DevicePtr_t r = getRobotOrThrow (problemSolver());
	  const ShapeMap& m = r->map <JointAndShapes_t> ();

	  char** nameList = Names_t::allocbuf((ULong) m.size ());
	  Names_t *jointNames = new Names_t ((ULong) m.size(), (ULong) m.size(),
					     nameList);

	  std::size_t rank = 0;
          for (ShapeMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            nameList[rank] = (char*) malloc (sizeof(char)*(it->first.length ()+1));
            strcpy (nameList [rank], it->first.c_str ());
            rank++;
          }
	  return jointNames;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
        }
      }

      Names_t* Problem::getEnvironmentContact (const char* name,
            intSeq_out indexes, floatSeqSeq_out points)
        throw (hpp::Error)
      {
        try {
	  const JointAndShapes_t& js =
            problemSolver()->core::ProblemSolver::get <JointAndShapes_t> (name);

          return jointAndShapes (js, indexes, points);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
        }
      }

      Names_t* Problem::getRobotContact (const char* name,
            intSeq_out indexes, hpp::floatSeqSeq_out points)
        throw (hpp::Error)
      {
        try {
          DevicePtr_t r = getRobotOrThrow (problemSolver());
	  const JointAndShapes_t& js = r->get <JointAndShapes_t> (name);

          return jointAndShapes (js, indexes, points);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
        }
      }

      void Problem::createPlacementConstraint (const char* placName,
					       const Names_t& surface1,
					       const Names_t& surface2)
        throw (hpp::Error)
      {
	try {
	  problemSolver()->createPlacementConstraint (placName,
              toStringList(surface1), toStringList(surface2), 1e-3);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::createPrePlacementConstraint (const char* placName,
					          const Names_t& surface1,
					          const Names_t& surface2,
                                                  CORBA::Double width)
        throw (hpp::Error)
      {
	try {
	  problemSolver()->createPrePlacementConstraint (placName,
              toStringList(surface1), toStringList(surface2), width, 1e-3);
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::createQPStabilityConstraint (const char* placName,
          const Names_t& shapesName)
        throw (hpp::Error)
      {
	try {
#ifdef HPP_CONSTRAINTS_USE_QPOASES
	  // Get robot in hppPlanner object.
          DevicePtr_t robot = getRobotOrThrow (problemSolver());

          using constraints::ConvexShape;
          using constraints::QPStaticStability;
          using constraints::QPStaticStabilityPtr_t;
          typedef constraints::QPStaticStability::ForceData ForceData;
          using pinocchio::CenterOfMassComputation;
          using pinocchio::CenterOfMassComputationPtr_t;

          std::vector <ForceData> fds;
          std::size_t nbPoints = 0;

          for (CORBA::ULong i = 0; i < shapesName.length(); ++i) {
            JointAndShapes_t l = robot->get <JointAndShapes_t>
              (std::string (shapesName[i]));
            if (l.empty ()) throw Error ("Robot shapes not found.");
            for (JointAndShapes_t::const_iterator it = l.begin ();
                it != l.end(); it++) {
              ConvexShape c (ConvexShape (it->second, it->first));
              ForceData fd;
              fd.joint = c.joint_;
              fd.supportJoint = JointPtr_t();
              fd.normal = - c.N_;
              fd.points = c.Pts_;
              nbPoints += c.Pts_.size ();
              fds.push_back (fd);
            }
          }

          CenterOfMassComputationPtr_t com = CenterOfMassComputation::create
            (robot);
          com->add (robot->rootJoint ());
          QPStaticStabilityPtr_t c = QPStaticStability::create (placName, robot,
              fds, com);
          problemSolver()->addNumericalConstraint (placName,
              NumericalConstraint::create (c, core::EqualToZero::create())
              );
#else
          // Avoid unused-variable compilation warnings
          (void)placName;
          (void)shapesName;
          throw std::runtime_error
            ("Problem::createQPStabilityConstraint is not implemented");
#endif
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      bool Problem::applyConstraints (hpp::ID id,
          const hpp::floatSeq& input,
          hpp::floatSeq_out output,
          double& residualError)
        throw (hpp::Error)
      {
        /// First get the constraint.
        ConstraintSetPtr_t constraint;
        try {
          graph::GraphComponentPtr_t comp = graph()->get ((size_t)id).lock ();
          graph::EdgePtr_t edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, comp);
          graph::StatePtr_t state = HPP_DYNAMIC_PTR_CAST(graph::State, comp);
          if (edge) {
            constraint = graph(false)->configConstraint (edge);
            DevicePtr_t robot = getRobotOrThrow (problemSolver());
	    if (core::ConfigProjectorPtr_t cp =
		constraint->configProjector ()) {
	      cp->rightHandSideFromConfig (robot->currentConfiguration());
	    }
          } else if (state)
            constraint = graph(false)->configConstraint (state);
          else {
            std::stringstream ss;
            ss << "ID " << id << " is neither an edge nor a state";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
	  bool success = false;
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  ConfigurationPtr_t config = floatSeqToConfigPtr (robot, input, true);
	  success = constraint->apply (*config);
	  if (hpp::core::ConfigProjectorPtr_t configProjector =
	      constraint ->configProjector ()) {
	    residualError = configProjector->residualError ();
	  }
	  output = vectorToFloatSeq(*config);
	  return success;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      bool Problem::applyConstraintsWithOffset (hpp::ID IDedge,
          const hpp::floatSeq& qnear,
          const hpp::floatSeq& input,
          hpp::floatSeq_out output,
          double& residualError)
        throw (hpp::Error)
      {
        /// First get the constraint.
        graph::EdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, graph()->get((size_t)IDedge).lock ());
          if (!edge) {
            std::stringstream ss;
            ss << "ID " << IDedge << " is not an edge";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
	  bool success = false;
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  ConfigurationPtr_t config = floatSeqToConfigPtr (robot, input, true);
	  ConfigurationPtr_t qoffset = floatSeqToConfigPtr (robot, qnear, true);
          value_type dist = 0;
          core::NodePtr_t nNode = problemSolver()->roadmap()->nearestNode
	    (qoffset, dist);
          if (dist < 1e-8)
            success = edge->applyConstraints (nNode, *config);
          else
            success = edge->applyConstraints (*qoffset, *config);

	  hpp::core::ConfigProjectorPtr_t configProjector
	    (edge->configConstraint ()->configProjector ());
	  if (configProjector) {
	    residualError = configProjector->residualError ();
	  } else {
	    hppDout (info, "No config projector.");
	  }
	  ULong size = (ULong) config->size ();
	  hpp::floatSeq* q_ptr = new hpp::floatSeq ();
	  q_ptr->length (size);

	  for (std::size_t i=0; i<size; ++i) {
	    (*q_ptr) [(ULong) i] = (*config) [i];
	  }
	  output = q_ptr;
	  return success;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      bool Problem::buildAndProjectPath (hpp::ID IDedge,
          const hpp::floatSeq& qb,
          const hpp::floatSeq& qe,
          CORBA::Long& indexNotProj,
          CORBA::Long& indexProj)
        throw (hpp::Error)
      {
        /// First get the constraint.
        graph::EdgePtr_t edge;
        try {
          edge = HPP_DYNAMIC_PTR_CAST(graph::Edge, graph()->get((size_t)IDedge).lock ());
          if (!edge) {
            std::stringstream ss;
            ss << "ID " << IDedge << " is not an edge";
            std::string errmsg = ss.str();
            throw Error (errmsg.c_str());
          }
	  // If steering method is not completely set in the graph, create
	  // one.
	  if (!edge->parentGraph ()->problem ()->steeringMethod () ||
	      !edge->parentGraph ()->problem ()->steeringMethod ()
	      ->innerSteeringMethod()) {
	    problemSolver ()->initSteeringMethod ();
	  }
	  bool success = false;
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  ConfigurationPtr_t q1 = floatSeqToConfigPtr (robot, qb, true);
	  ConfigurationPtr_t q2 = floatSeqToConfigPtr (robot, qe, true);
	  core::PathVectorPtr_t pv;
	  indexNotProj = -1;
	  indexProj = -1;
          core::PathPtr_t path;
	  success = edge->build (path, *q1, *q2);
          if (!success) return false;
          pv = HPP_DYNAMIC_PTR_CAST (core::PathVector, path);
          indexNotProj = (CORBA::Long) problemSolver()->paths ().size ();
          if (!pv) {
            pv = core::PathVector::create (path->outputSize (),
                path->outputDerivativeSize ());
            pv->appendPath (path);
          }
          problemSolver()->addPath (pv);

          core::PathPtr_t projPath;
	  PathProjectorPtr_t pathProjector
	    (problemSolver()->problem()->pathProjector ());
	  if (!pathProjector) {
	    problemSolver ()->initPathProjector ();
	    pathProjector =
	      problemSolver()->problem()->pathProjector ();
	  }
	  if (pathProjector) {
	    success = pathProjector->apply (path, projPath);
	  } else {
	    success = true;
	    projPath = path->copy ();
	  }

          if (!success) {
            if (!projPath || projPath->length () == 0)
              return false;
          }
          pv = HPP_DYNAMIC_PTR_CAST (core::PathVector, projPath);
          indexProj = (CORBA::Long) problemSolver()->paths ().size ();
          if (!pv) {
            pv = core::PathVector::create (projPath->outputSize (),
                projPath->outputDerivativeSize ());
            pv->appendPath (projPath);
          }
          problemSolver()->addPath (pv);
	  return success;
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Problem::setTargetState (hpp::ID IDstate)
      {
        try {
          graph::GraphComponentPtr_t comp = graph()->get ((size_t)IDstate).lock ();
          graph::StatePtr_t state = HPP_DYNAMIC_PTR_CAST(graph::State, comp);
          if (!state) {
            HPP_THROW(Error, "ID " << IDstate << " is not a state.");
          }
          problemSolver()->setTargetState(state);
        } catch (const std::exception& exc) {
          throw hpp::Error (exc.what ());
        }
      }

      ID Problem::edgeAtParam (UShort pathId, Double param)
        throw (Error)
      {
	try {
	  if (pathId >= problemSolver()->paths ().size ()) {
            HPP_THROW (Error, "Wrong path id: " << pathId << ", number path: "
		<< problemSolver()->paths ().size () << ".");
	  }
          core::PathVectorPtr_t path = problemSolver()->paths () [pathId];
          core::PathVectorPtr_t flat = core::PathVector::create(path->outputSize(), path->outputDerivativeSize());
          path->flatten(flat);
          value_type unused;
          std::size_t r = flat->rankAtParam(param, unused);
          PathPtr_t p = flat->pathAtRank (r);
          manipulation::ConstraintSetPtr_t constraint = 
            HPP_DYNAMIC_PTR_CAST (manipulation::ConstraintSet, p->constraints());
          if (!constraint) {
            HPP_THROW (Error, "Path constraint is not of the good type "
                << "at id " << pathId << ", param " << param
                << " (rank: " << r << ")");
          }
          if (!constraint->edge()) {
            HPP_THROW (Error, "Path constraint does not contain edge information "
                << "at id " << pathId << ", param " << param
                << " (rank: " << r << ")");
          }
          return (ID)constraint->edge()->id();
	}
	catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }
    } // namespace impl
  } // namespace manipulation
} // namespace hpp
