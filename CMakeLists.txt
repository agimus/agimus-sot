#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-core
# hpp-core is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-core  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

set(CXX_DISABLE_WERROR TRUE)

set(PROJECT_NAME agimus-sot)
set(PROJECT_DESCRIPTION "Package to link SoT to the Agimus framework.")
set(PROJECT_URL "https://github.com/agimus/agimus-sot")
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)
include(cmake/test.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON(REQUIRED EXACT 2.7)

add_project_dependency (dynamic_graph_bridge 3 REQUIRED)
# Needs only the Python classes related to ConstraintGraphFactory.
add_project_dependency (hpp-manipulation-corba 4 REQUIRED)
add_project_dependency (sot-core 4 REQUIRED)
add_project_dependency (dynamic-graph-python 4 REQUIRED)
add_project_dependency (agimus_sot_msgs REQUIRED)

INSTALL(FILES
  package.xml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
INSTALL(PROGRAMS
  scripts/start_supervisor.py
  scripts/simulation.py
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/scripts)
INSTALL(DIRECTORY launch
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
