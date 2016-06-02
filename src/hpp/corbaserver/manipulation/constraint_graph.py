#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Joseph Mirabel
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

from subprocess import Popen

## Association of a numerical constraint with the associated passive joints
#
#  Passive joints are information provided to the constraint solver to get
#  better performance and behavior in the resolution.
class _ConstraintAndPassiveJoints (object):
    def __init__ (self, constraint, passiveJoints):
        self.constraint_ = constraint
        self.passiveJoints_ = passiveJoints
    @property
    def constraint (self):
        return self.constraint_
    @property
    def passiveJoints (self):
        return self.passiveJoints_

### Definition of a constraint graph.
##
##  This class wraps the Corba client to the server implemented by
##  libhpp-manipulation-corba.so
##
##  Some method implemented by the server can be considered as private. The
##  goal of this class is to hide them and to expose those that can be
##  considered as public.
class ConstraintGraph (object):
    cmdDot = {
            'pdf': ['dot', '-Gsize=7.5,10', '-Tpdf'],
            'svg': ['dot', '-Gsize=7.5,10', '-Tsvg']
            }
    cmdViewer = {
            'pdf': ['evince'],
            'svg': ['firefox']
            }

    def __init__ (self, robot, graphName, makeGraph = True):
        self.client = robot.client.manipulation
        self.clientBasic = robot.client.basic
        self.graph = robot.client.manipulation.graph
        self.name = graphName
        self.grasps = dict ()
        self.pregrasps = dict ()
        self.nodes = dict ()
        self.edges = dict ()
        if makeGraph:
            self.graphId = self.graph.createGraph (graphName)
            self.subGraphId = self.graph.createSubGraph (graphName + "_sg")
        else:
            # fetch graph
            try:
                g = self.graph.getGraph ()
                self.graphId = g[0].id
                self.subGraphId = self.graphId + 1
                for n in g[1].nodes:
                    if self.nodes.has_key(n.name):
                        print "Erasing node", n.name, "id", self.nodes[n.name]
                    self.nodes[n.name] = n.id
                for e in g[1].edges:
                    if self.edges.has_key(e.name):
                        print "Erasing edge", e.name, "id", self.edges[e.name]
                    self.edges[e.name] = e.id
            except:
                pass

        self.textToTex = dict ()

    ### Display the current graph.
    ## The graph is printed in DOT format. Command dot must be
    ## available.
    ## \param dotOut full path of the generated DOT file.
    ## \param pdfOut fill path of the generated PDF document.
    ## \param openPDF set to False if you just want to generate the PDF.
    ## \note DOT and PDF files will be overwritten and are not automatically
    ## deleted so you can keep them.
    def display (self, dotOut = '/tmp/constraintgraph.dot', pdfOut = '/tmp/constraintgraph', format = 'pdf', open = True):
        self.graph.display (dotOut)
        if not self.cmdDot.has_key (format):
            raise TypeError ("This format is not supported. See member cmdDot for supported format.")
        dotCmd = self.cmdDot [format]
        dotCmd.append ('-o' + pdfOut + '.' + format)
        dotCmd.append (dotOut)
        dot = Popen (dotCmd)
        dot.wait ()
        if open and self.cmdViewer.has_key (format):
            viewCmd = self.cmdViewer [format]
            viewCmd.append (pdfOut + '.' + format)
            Popen (viewCmd)

    ### Create one or several node
    ## \param node name (resp. list of names) of the node(s) to be created.
    ## \param waypoint set to True when creating waypoint nodes.
    ## \note The order is important. The first should be the most restrictive one as a configuration
    ## will be in the first node for which the constraint are satisfied.
    def createNode (self, node, waypoint = False):
        if type (node) is str:
            node = [node]
        for n in node:
            self.nodes [n] = self.graph.createNode (self.subGraphId, self._(n), waypoint)

    ### Create an edge
    ## \param nodeFrom, nodeTo the extremities of the edge,
    ## \param name name of the edge,
    ## \param weight see note,
    ## \param isInNodeFrom true if a path corresponding to this edge lies in nodeFrom;
    ##       this will be automatically set if you leave it.
    ## \note The weights define the probability of selecting an edge among all the
    ## outgoing edges of a node. The probability of an edge is \f$ \frac{w_i}{\sum_j{w_j}} \f$,
    ## where each \f$ w_j \f$ corresponds to an outgoing edge from a given node.
    ## To have an edge that cannot be selected by the M-RRT algorithm but is still acceptable,
    ## set its weight to zero.
    def createEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        self.edges [name] =\
            self.graph.createEdge (self.nodes[nodeFrom], self.nodes[nodeTo], self._(name), weight, isInNodeFrom)

    ### Create a WaypointEdge.
    ## \param nodeFrom, nodeTo, name, weight, isInNodeFrom see createEdge note,
    ## \param nb number of waypoints,
    ## \note See documentation of class hpp::manipulation::graph::WaypointEdge for more information.
    ##
    ## \warning Waypoint are now specified by hand to allow finer handling
    ## of edge types between waypoints. This function has been updated to be
    ## backward compatible but except for the return value.
    ## For a finer control of what you are doing, set automaticBuilder to
    ## False.
    def createWaypointEdge (self, nodeFrom, nodeTo, name, nb = 1, weight = 1,
            isInNodeFrom = None, automaticBuilder = True):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])

        if automaticBuilder:
            n = name + "_e" + str(nb)
        else:
            n = name
        wid = self.edges[n] = self.graph.createWaypointEdge (
                self.nodes[nodeFrom], self.nodes[nodeTo], self._(name),
                nb, weight, isInNodeFrom)

        if not automaticBuilder:
            return

        waypoints = list ()
        previous = self.nodes[nodeFrom]
        for i in range(nb):
            waypoints.append((name + "_e" + str(i), name + "_n" + str(i)))
            n = waypoints[-1][1]
            e = waypoints[-1][0]
            newN = self.nodes[n] = \
                    self.graph.createNode (self.subGraphId, self._(n), True)
            newE = self.edges[e] = \
                    self.graph.createEdge (previous, newN, self._(e), -1, isInNodeFrom)
            self.graph.setWaypoint (wid, i, newE, newN);
            previous = newN

    ### Create a LevelSetEdge.
    ## \param nodeFrom, nodeTo, name, weight, isInNodeFrom see createEdge note.
    ## \note See documentation of class hpp::manipulation::graph::LevelSetEdge for more information.
    def createLevelSetEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNodeFrom = None):
        if isInNodeFrom is None:
            isInNodeFrom = (self.nodes[nodeFrom] > self.nodes[nodeTo])
        self.edges [name] =\
            self.graph.createLevelSetEdge (self.nodes[nodeFrom], self.nodes[nodeTo], self._(name), weight, isInNodeFrom)

    ## Create grasp constraints between robot gripper and object handle
    #
    #  Creates two contraints between a handle and a gripper.
    #  \li The first constraint named "${name}" is defined by
    #  the type of handle. For instance, an axial handle defines
    #  a five degree of freedom constraint with free rotation
    #  around the x-axis.
    #  \li the second constraint named "${name}/complement" is
    #  the complement to the full transformation constraint. For the axial
    #  handle, it corresponds to the rotation around x.
    #
    #  \param name prefix of the constraint names for storing in
    #         ProblemSolver map,
    #  \param gripper name of the gripper used when it has been created
    #  \param handle name of the handle in the form "object/handle"
    #  where object is the name of the object owning the handle and handle
    #  is the name of the handle in this object.
    #  \param passiveJoints name of the set of passive joints associated to
    #         the grasp constraints as register in ProblemSolver
    #         \sa manipulation.problem_solver.ProblemSolver::addPassiveDofs.
    #
    #  \sa method hpp::corbaserver::manipulation::Problem::createGrasp.
    #
    #  \note Passive joints are only used for path constraints and are for
    #        computational optimization only.
    def createGrasp (self, name, gripper, handle, passiveJoints = ""):
        self.client.problem.createGrasp (self._(name), gripper, handle)
        self.grasps [name] = (_ConstraintAndPassiveJoints (self._(name),
                                                          passiveJoints),)

    ## Create pre-grasp constraints between robot gripper and object handle
    #
    #  Creates two contraints between a handle and a gripper.
    #  \li The first constraint named "${name}" is the same as the grasp
    #  defined in createGrasp, except that the translation along x is not
    #  constrained. For instance, an axial handle defines
    #  a four degree of freedom constraint with free rotation and translation
    #  around/along the x-axis,
    #  \li the second constraint named "${name}/double_ineq" is a double
    #  inequality on the relative x-position of the handle and of the gripper.
    #  the bounds of the inequality are for now [-.001 c, 2.001 c].
    #
    #  \param name prefix of the constraint names for storing in
    #         ProblemSolver map,
    #  \param gripper name of the gripper used when it has been created
    #  \param handle name of the handle in the form "object/handle"
    #  where object is the name of the object owning the handle and handle
    #  is the name of the handle in this object,
    #  \param passiveJoints name of the set of passive joints associated to
    #         the pre-grasp constraints as register in ProblemSolver.
    #         \sa manipulation.problem_solver.ProblemSolver::addPassiveDofs.
    #
    #  \sa hpp::corbaserver::manipulation::Problem::createPreGrasp
    #
    #  \note Passive joints are only used for path constraints and are for
    #        computational optimization only.
    def createPreGrasp (self, name, gripper, handle, passiveJoints = ""):
        self.client.problem.createPreGrasp (self._(name), gripper, handle)
        self.pregrasps [name] = \
            (_ConstraintAndPassiveJoints (self._(name), passiveJoints),)

    ## Set the constraints of an edge, a node or the whole graph
    #
    # This method sets the constraints of an element of the graph and handles
    # the special cases of grasp and pregrasp constraints.
    #
    ## \param graph set to true if you are defining constraints for every nodes,
    ## \param node edge name of a component of the graph,
    ## \param grasps list of names of grasp. Each grasp 
    ## \param pregrasps list of names of pregrasps
    ## \note Exaclty one of the parameter graph, node and edge must be set.
    def setConstraints (self, graph = False, node = None, edge = None, 
                        grasps = None, pregrasps = None, lockDof = [],
                        numConstraints = [], passiveJoints = [],
                        grasp = None, pregrasp = None):
        if grasp is not None:
            from warnings import warn
            warn ("grasp argument is deprecated: use grasps and provide a " +
                  "list of grasp names")
            if type (grasp) is str:
                grasps = [grasp,]
            else:
                grasps = grasp

        if pregrasp is not None:
            from warnings import warn
            warn ("pregrasp argument is deprecated: use pregrasps and provide" +
                  " a list of pregrasp names")
            if type (pregrasp) is str:
                pregrasps = [pregrasp,]
            else:
                pregrasps = pregrasp

        nc = numConstraints [:]
        pdofs = ["" for i in range (len(numConstraints))]
        pdofs [:len(passiveJoints)] = passiveJoints [:]
        if grasps is not None:
            for g in grasps:
                for pair in self.grasps [g]:
                    if edge is not None:
                        nc.append (pair.constraint + "/complement")
                    else:
                        nc.append (pair.constraint)
                    pdofs.append (pair.passiveJoints)
        if pregrasps is not None:
            for g in pregrasps:
                for pair in self.pregrasps [g]:
                    nc.append (pair.constraint)
                    pdofs.append (pair.passiveJoints)

        if node is not None:
            self.graph.setNumericalConstraints (self.nodes [node], nc, [])
            self.graph.setNumericalConstraintsForPath (self.nodes [node], nc,
                                                       pdofs)
            self.graph.setLockedDofConstraints (self.nodes [node], lockDof)
        elif edge is not None:
            self.graph.setNumericalConstraints (self.edges [edge], nc, [])
            self.graph.setLockedDofConstraints (self.edges [edge], lockDof)
        elif graph:
            self.graph.setNumericalConstraints (self.graphId, nc, [])
            self.graph.setLockedDofConstraints (self.graphId, lockDof)

    ## Set the numerical constraints of a LevelSetEdge that create the foliation.
    #  \param edge name of a LevelSetEdge of the graph.
    #  \param condGrasps, condPregrasps name, or list of names, of grasp or pregrasp that define the foliated manifold
    #  \param condNC, condLJ numerical constraints and locked joints that define the foliated manifold
    #  \param paramGrasps, paramPregrasps name, or list of names, of grasp or pregrasp that parametrize the foliation
    #  \param paramNC, paramPassiveJoints, paramLJ numerical constraints and locked joints that parametrize the foliation
    #  \note If passiveDofsNames is a shorter list than numConstraints, passiveDofsNames is extended with an empty string,
    #        which corresponds to an empty vector of passive dofs.
    def setLevelSetFoliation (self, edge,
            condGrasps = None, condPregrasps = None, condNC = [], condLJ = [],
            paramGrasps = None, paramPregrasps = None, paramNC = [], paramPassiveJoints = [], paramLJ = []):

        cond_nc = condNC [:]
        if condGrasps is not None:
            for g in condGrasps:
                for pair in self.grasps [g]:
                    cond_nc.append (pair.constraint)
        if condPregrasps is not None:
            for g in condPregrasps:
                for pair in self.pregrasps [g]:
                    cond_nc.append (pair.constraint)

        param_nc = paramNC [:]
        pdofs = ["" for i in range (len(paramNC))]
        pdofs [:len(paramPassiveJoints)] = paramPassiveJoints [:]
        if paramGrasps is not None:
            for g in paramGrasps:
                for pair in self.grasps [g]:
                    param_nc.append (pair.constraint)
                    pdofs.append (pair.passiveJoints)
        if paramPregrasps is not None:
            for g in paramPregrasps:
                for pair in self.pregrasps [g]:
                    param_nc.extend (pair.constraint)
                    pdofs.extend (pair.passiveJoints)

        self.graph.setLevelSetFoliation (self.edges [edge], cond_nc, condLJ, param_nc, pdofs, paramLJ)

    @staticmethod
    ## Build a graph
    # \return a Initialized ConstraintGraph object
    def buildGenericGraph (robot, name, grippers, objects, handlesPerObjects, shapesPerObjects, envNames, rules = []):
        robot.client.manipulation.graph.autoBuild \
                (name, grippers, objects, handlesPerObjects, shapesPerObjects, envNames, rules)
        return ConstraintGraph (robot, name, makeGraph = False);

    ## Add entry to the local dictionnary
    # \param text plain text
    # \param tex its latex translation
    # \sa ConstraintGraph.setTextToTeXTranslation
    def addTextToTeXTranslation (self, text, tex):
        self.textToTex[text] = tex
    
    ## Set the local dictionnary
    # \param textToTex a dictionnary of (plain text, TeX replacment)
    # If the name of a node or an edges is a key of the dictionnary,
    # it is replaced by the corresponding value.
    def setTextToTeXTranslation (self, textToTex):
        if type (textToTex) is not dict:
            raise TypeError ("Argument textToTex must be a dictionnary.")
        self.textToTex = textToTex

    ## get the textToTex translation
    def _ (self, text):
        return self.textToTex.get (text, text)
