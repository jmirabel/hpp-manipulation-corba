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

#ifndef HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH
# define HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH

# include <hpp/corbaserver/manipulation/fwd.hh>
# include <hpp/manipulation/problem-solver.hh>
# include "hpp/corbaserver/manipulation/problem.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      using CORBA::Short;
      using CORBA::UShort;
      using CORBA::ULong;
      using CORBA::Double;

      class Problem : public virtual POA_hpp::corbaserver::manipulation::Problem
      {
      public:
	Problem ();
	void setProblemSolver (const ProblemSolverPtr_t& problemSolver)
	{
	  problemSolver_ = problemSolver;
	}

        virtual void createGrasp (const char* graspName,
            const char* gripperName, const char* handleName)
	  throw (hpp::Error);

        virtual void createPreGrasp (const char* graspName,
            const char* gripperName, const char* handleName)
	  throw (hpp::Error);

        virtual void createLockedJoint (const char* lockedJointName,
            const char* jointName, const hpp::floatSeq& value)
	  throw (hpp::Error);

        virtual Names_t* getEnvironmentContactNames ()
          throw (hpp::Error);

        virtual Names_t* getRobotContactNames ()
          throw (hpp::Error);

        virtual void createPlacementConstraint (const char* placName,
            const char* jointName, const char* triangleName,
            const char* envContactName)
	  throw (hpp::Error);

        virtual bool applyConstraints (hpp::ID id, const hpp::floatSeq& input,
            hpp::floatSeq_out output, double& residualError)
          throw (hpp::Error);

        virtual bool applyConstraintsWithOffset (hpp::ID IDedge,
            const hpp::floatSeq& qnear, const hpp::floatSeq& input,
            hpp::floatSeq_out output, double& residualError)
          throw (hpp::Error);

      private:
	ProblemSolverPtr_t problemSolver_;
      }; // class Problem
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_PROBLEM_IMPL_HH
