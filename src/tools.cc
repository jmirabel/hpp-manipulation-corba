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
  namespace corbaserver {
    void Transform3fTohppTransform (const hpp::manipulation::Transform3f& transform,
        CORBA::Double* config)
    {
      fcl::Quaternion3f Q = transform.getQuatRotation ();
      fcl::Vec3f T = transform.getTranslation ();
      for(int i=0; i<3; i++)
        config [i] = T [i];
      config[3] = Q.w();
      config[4] = Q.x();
      config[5] = Q.y();
      config[6] = Q.z();
    }
  } // namespace corbaserver 
} // namespace hpp
