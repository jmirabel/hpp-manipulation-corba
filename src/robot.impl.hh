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

#ifndef HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
# define HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH

# include <hpp/corbaserver/manipulation/fwd.hh>
# include <hpp/manipulation/problem-solver.hh>
# include "hpp/corbaserver/manipulation/robot.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::Short;

      class Robot : public virtual POA_hpp::corbaserver::manipulation::Robot
      {
        public:
          Robot ();

          void setProblemSolver (const ProblemSolverPtr_t& problemSolver)
          {
            problemSolver_ = problemSolver;
          }

          virtual void create (const char* robotName)
            throw (hpp::Error);

          virtual void insertRobotModel (const char* robotName,
              const char* rootJointType, const char* packageName,
              const char* modelName, const char* urdfSuffix,
              const char* srdfSuffix)
            throw (hpp::Error);

          virtual void insertObjectModel (const char* objectName,
              const char* rootJointType, const char* packageName,
              const char* modelName, const char* urdfSuffix,
              const char* srdfSuffix)
            throw (hpp::Error);

          virtual void insertHumanoidModel (const char* robotName,
              const char* rootJointType, const char* packageName,
              const char* modelName, const char* urdfSuffix,
              const char* srdfSuffix)
            throw (hpp::Error);

          virtual void loadEnvironmentModel (const char* package,
              const char* envModelName, const char* urdfSuffix,
              const char* srdfSuffix, const char* prefix)
            throw (hpp::Error);

          virtual Transform__slice* getRootJointPosition (const char* robotName)
            throw (hpp::Error);

          virtual void setRootJointPosition (const char* robotName,
              const ::hpp::Transform_ position)
            throw (hpp::Error);

          virtual void addHandle (const char* linkName, const char* handleName,
              const ::hpp::Transform_ localPosition)
            throw (hpp::Error);

          virtual void addGripper(const char* linkName, const char* gripperName,
              const ::hpp::Transform_ handlePositioninJoint,
              const hpp::Names_t& bodyInCollisionNames)
            throw (hpp::Error);

          virtual void addAxialHandle (const char* linkName,
              const char* handleName, const ::hpp::Transform_ localPosition)
            throw (hpp::Error);

        private:
          ProblemSolverPtr_t problemSolver_;
      }; // class Robot
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_ROBOT_IMPL_HH
