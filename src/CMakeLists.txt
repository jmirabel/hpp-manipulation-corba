# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})
SET(IDL_SOURCES
  gcommon
  graph
  problem
  robot
)

OMNIIDL_INCLUDE_DIRECTORIES(
  ${HPP_CORBASERVER_DATAROOTDIR}/idl ${CMAKE_SOURCE_DIR}/idl
  )

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_FILE (hpp/corbaserver/manipulation/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation
    )
ENDFOREACH()

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommon.hh
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graph.hh
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robot.hh
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problem.hh
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommonSK.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graphSK.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robotSK.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problemSK.cc
  graph.impl.cc
  graph.impl.hh
  problem.impl.cc
  problem.impl.hh
  robot.impl.cc
  robot.impl.hh
  server.cc
  client.cc
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommon_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graph_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problem_idl.py
  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robot_idl.py
)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation-urdf)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

# search for python
FINDPYTHON()

# Generate and install python code for each idl file
FOREACH (IDL ${IDL_SOURCES})
  ADD_CUSTOM_COMMAND(
    OUTPUT
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
    COMMAND ${OMNIIDL}
    ARGS -bpython -Wbpackage="" -I${CMAKE_SOURCE_DIR}/idl
    -I${HPP_CORBASERVER_DATAROOTDIR}/idl
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation/${IDL}.idl
    MAIN_DEPENDENCY
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation/${IDL}.idl
    )
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
    )
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/manipulation
    )
ENDFOREACH()

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/client.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/problem_solver.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/robot.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/constraint_graph.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )

# Stand alone corba server
ADD_EXECUTABLE (hpp-manipulation-server hpp-manipulation-corba.cc)
TARGET_LINK_LIBRARIES (hpp-manipulation-server ${LIBRARY_NAME})
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation)
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation-urdf)
PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-wholebody-step-corba)

INSTALL (TARGETS hpp-manipulation-server DESTINATION ${CMAKE_INSTALL_BINDIR})
