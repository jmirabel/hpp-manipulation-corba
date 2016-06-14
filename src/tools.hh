// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel
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

#ifndef HPP_MANIPULATION_CORBA_TOOLS_HH
# define HPP_MANIPULATION_CORBA_TOOLS_HH

# include <hpp/corbaserver/common.hh>

# include <hpp/corbaserver/manipulation/config.hh>
# include <hpp/manipulation/problem-solver.hh>

namespace hpp {
  namespace corbaserver {
    inline std::list<std::string> toStringList (const Names_t& names) {
      std::list<std::string> ret;
      for (CORBA::ULong i = 0; i < names.length(); ++i)
        ret.push_back (std::string(names[i]));
      return ret;
    }

    template <typename InputIt>
    inline Names_t* toNames_t (InputIt begin, InputIt end) {
      std::size_t len = std::distance (begin, end);
      char** nameList = Names_t::allocbuf((CORBA::ULong) len);
      Names_t *ret = new Names_t ((CORBA::ULong) len, (CORBA::ULong) len, nameList);

      std::size_t i = 0;
      while (begin != end) {
        nameList[i] = new char[begin->length ()+1];
        strcpy (nameList[i], begin->c_str ());
        ++begin;
        ++i;
      }
      return ret;
    }

    template <typename InputIt>
    inline intSeq* toIntSeq (InputIt begin, InputIt end) {
      std::size_t len = std::distance (begin, end);
      intSeq* indexes = new intSeq ();
      indexes->length ((CORBA::ULong) len);

      std::size_t i = 0;
      while (begin != end) {
        (*indexes)[i] = *begin;
        ++begin;
        ++i;
      }
      return indexes;
    }

    void HPP_MANIPULATION_CORBA_LOCAL Transform3fTohppTransform (const hpp::manipulation::Transform3f& transform,
        CORBA::Double* config);
  } // namespace corbaserver 
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_TOOLS_HH
