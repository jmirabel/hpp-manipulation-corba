// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
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
// hpp-manipulation-corba. If not, see <http://www.gnu.org/licenses/>.

#include "tools.hh"

namespace hpp {
  DevicePtr_t getRobotOrThrow (ProblemSolverPtr_t p)
  {
    DevicePtr_t robot = p->robot ();
    if (!robot) throw Error ("Robot not found.");
    return robot;
  }
} // namespace hpp
