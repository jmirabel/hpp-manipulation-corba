#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
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

import hpp.corbaserver.wholebody_step

## Definition of a manipulation planning problem
#
#  This class wraps the Corba client to the server implemented by
#  libhpp-manipulation-corba.so
#
#  Some method implemented by the server can be considered as private. The
#  goal of this class is to hide them and to expose those that can be
#  considered as public.
class ProblemSolver (object):
    SLIDING = hpp.corbaserver.wholebody_step.Problem.SLIDING
    ALIGNED_COM = hpp.corbaserver.wholebody_step.Problem.ALIGNED_COM

    def __init__ (self, robot):
        self.client = robot.client
        self.robot = robot

    ## \name Initial and goal configurations
    # \{

    ## Set initial configuration of specified problem.
    #	\param dofArray Array of degrees of freedom
    #	\throw Error.
    def setInitialConfig (self, dofArray):
        return self.client.basic.problem.setInitialConfig (dofArray)

    ## Get initial configuration of specified problem.
    #	\return Array of degrees of freedom
    def getInitialConfig (self):
        return self.client.basic.problem.getInitialConfig ()

    ## Add goal configuration to specified problem.
    #	\param dofArray Array of degrees of freedom
    #	\throw Error.
    def addGoalConfig (self, dofArray):
        return self.client.basic.problem.addGoalConfig (dofArray)

    ## Get goal configurations of specified problem.
    #	\return Array of degrees of freedom
    def getGoalConfigs (self):
        return self.client.basic.problem.getGoalConfigs ()

    ## Reset goal configurations
    def resetGoalConfigs (self):
        return self.client.basic.problem.resetGoalConfigs ()
    ## \}

    ## \name Obstacles
    # \{

    ## Load obstacle from urdf file
    #  \param package Name of the package containing the model,
    #  \param filename name of the urdf file in the package
    #         (without suffix .urdf)
    #  \param prefix prefix added to object names in case the same file is
    #         loaded several times
    #
    #  The ros url is built as follows:
    #  "package://${package}/urdf/${filename}.urdf"
    #
    #  The kinematic structure of the urdf file is ignored. Only the geometric
    #  objects are loaded as obstacles.
    def loadObstacleFromUrdf (self, package, filename, prefix):
        return self.client.basic.obstacle.loadObstacleModel (package, filename,
                                                       prefix)

    ## Remove an obstacle from outer objects of a joint body
    #
    #  \param objectName name of the object to remove,
    #  \param jointName name of the joint owning the body,
    #  \param collision whether collision with object should be computed,
    #  \param distance whether distance to object should be computed.
    #  \throw Error.
    def removeObstacleFromJoint (self, objectName, jointName, collision,
                                 distance):
        return self.client.basic.obstacle.removeObstacleFromJoint \
            (objectName, jointName, collision, distance)

    ## Move an obstacle to a given configuration.
    #  \param objectName name of the polyhedron.
    #  \param cfg the configuration of the obstacle.
    #  \throw Error.
    #
    #  \note The obstacle is not added to local map
    #  impl::Obstacle::collisionListMap.
    #
    #  \note Build the collision entity of polyhedron for KCD.
    def moveObstacle (self, objectName, cfg):
        return self.client.basic.obstacle.moveObstacle (objectName, cfg)
    ## Get the position of an obstacle
    #
    #  \param objectName name of the polyhedron.
    #  \retval cfg Position of the obstacle.
    #  \throw Error.
    def getObstaclePosition (self, objectName):
        return self.client.basic.obstacle.getObstaclePosition (objectName)

    ## Get list of obstacles
    #
    #  \param collision whether to return obstacle for collision,
    #  \param distance whether to return obstacles for distance computation
    # \return list of obstacles
    def getObstacleNames (self, collision, distance):
        return self.client.basic.obstacle.getObstacleNames (collision, distance)

    ##\}

    ## \name Constraints
    #  \{

    ##  Create static stability constraints
    #
    #   Call corba request
    #   hpp::corbaserver::wholebody_step::Problem::addStaticStabilityConstraints
    #
    #   The ankles are defined by members leftAnkle and rightAnkle of variable
    #   robot passed at construction of this object.
    #   \param constraintName name of the resulting constraint,
    #   \param q0 configuration that satisfies the constraints,
    #   \param comName name of a partial COM,
    #   \param type Type of static stability constraints (Default value: ProblemSolver.SLIDING)
    #
    #   \sa hpp::corbaserver::wholebody_step::Problem::StaticStabilityType
    def createStaticStabilityConstraints (self, constraintName, q0, comName = "",
            type = None):
        if type is None:
            type = self.SLIDING
        self.client.wholebodyStep.problem.addStaticStabilityConstraints \
            (constraintName, q0, self.robot.leftAnkle, self.robot.rightAnkle, comName, type)
        if type == self.SLIDING:
            self.balanceConstraints_ = [constraintName + "/relative-com",
                                        constraintName + "/relative-orientation",
                                        constraintName + "/relative-position",
                                        constraintName + "/orientation-left-foot",
                                        constraintName + "/position-left-foot"]
        elif type == self.ALIGNED_COM:
            self.balanceConstraints_ = [constraintName + "/com-between-feet",
                                        constraintName + "/orientation-right",
                                        constraintName + "/orientation-left",
                                        constraintName + "/position-right",
                                        constraintName + "/position-left"]


    ##  Create complement of static stability constraints
    #
    #   Call corba request
    #   hpp::corbaserver::wholebody_step::Problem::addComplementStaticStabilityConstraints
    #
    #   The ankles are defined by members leftAnkle and rightAnkle of variable
    #   robot passed at construction of this object.
    #   \param constraintName name of the resulting constraint,
    #   \param q0 configuration that satisfies the constraints
    def createComplementStaticStabilityConstraints (self, constraintName, q0):
        self.client.wholebodyStep.problem.addComplementStaticStabilityConstraints \
            (constraintName, q0, self.robot.leftAnkle, self.robot.rightAnkle)

    ## Create placement and pre-placement constraints
    #
    # \param width set to None to skip creation of pre-placement constraint
    #
    # See hpp::corbaserver::manipulation::Problem::createPlacementConstraint
    # and hpp::corbaserver::manipulation::Problem::createPrePlacementConstraint
    def createPlacementConstraints (self, placementName, shapeName, envContactName, width = 0.05):
        name = placementName
        self.client.manipulation.problem.createPlacementConstraint (name, shapeName, envContactName)
        if width is not None:
            prename = "pre_" + name
            self.client.manipulation.problem.createPrePlacementConstraint (prename, shapeName, envContactName, width)
            return name, prename
        return name

    ## Return balance constraints created by method
    #  ProblemSolver.createStaticStabilityConstraints
    def balanceConstraints (self):
        return self.balanceConstraints_

    ## Reset Constraints
    #

    ## Create orientation constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param p quaternion representing the desired orientation
    #         of joint2 in the frame of joint1.
    #  \param mask Select which axis to be constrained.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createOrientationConstraint (self, constraintName, joint1Name,
                                     joint2Name, p, mask):
        return self.client.basic.problem.createOrientationConstraint \
            (constraintName, joint1Name, joint2Name, p, mask)

    ## Create position constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param joint1Name name of first joint
    #  \param joint2Name name of second joint
    #  \param point1 point in local frame of joint1,
    #  \param point2 point in local frame of joint2.
    #  \param mask Select which axis to be constrained.
    #  If joint1 of joint2 is "", the corresponding joint is replaced by
    #  the global frame.
    #  constraints are stored in ProblemSolver object
    def createPositionConstraint (self, constraintName, joint1Name,
                                  joint2Name, point1, point2, mask):
        return self.client.basic.problem.createPositionConstraint \
            (constraintName, joint1Name, joint2Name, point1, point2, mask)

    ## Create RelativeCom constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param comName name of CenterOfMassComputation
    #  \param jointName name of joint
    #  \param point point in local frame of joint.
    #  \param mask Select axis to be constrained.
    #  If jointName is "", the robot root joint is used.
    #  Constraints are stored in ProblemSolver object
    def createRelativeComConstraint (self, constraintName, comName, jointLName, point, mask):
        return self.client.basic.problem.createRelativeComConstraint \
            (constraintName, comName, jointLName, point, mask)

    ## Create ComBeetweenFeet constraint between two joints
    #
    #  \param constraintName name of the constraint created,
    #  \param comName name of CenterOfMassComputation
    #  \param jointLName name of first joint
    #  \param jointRName name of second joint
    #  \param pointL point in local frame of jointL.
    #  \param pointR point in local frame of jointR.
    #  \param jointRefName name of second joint
    #  \param mask Select axis to be constrained.
    #  If jointRef is "", the robot root joint is used.
    #  Constraints are stored in ProblemSolver object
    def createComBeetweenFeet (self, constraintName, comName, jointLName, jointRName,
        pointL, pointR, jointRefName, mask):
        return self.client.basic.problem.createComBeetweenFeet \
            (constraintName, comName, jointLName, jointRName, pointL, pointR, jointRefName, mask)

    ## Add an object to compute a partial COM of the robot.
    # \param name of the partial com
    # \param jointNames list of joint name of each tree ROOT to consider.
    # \note Joints are added recursively, it is not possible so far to add a
    # joint without addind all its children.
    def addPartialCom (self, comName, jointNames):
        return self.client.basic.robot.addPartialCom (comName, jointNames);

    ## Create a vector of passive dofs.
    #
    #  \param name name of the vector in the ProblemSolver map.
    #  \param dofNames list of names of DOF that may
    def addPassiveDofs (self, name, dofNames):
        return self.client.basic.problem.addPassiveDofs (name, dofNames)

    ## Reset Constraints
    #
    #  Reset all constraints, including numerical constraints and locked
    #  degrees of freedom.
    def resetConstraints (self):
        return self.client.basic.problem.resetConstraints ()

    ## Set numerical constraints in ConfigProjector
    #
    #  \param name name of the resulting numerical constraint obtained
    #         by stacking elementary numerical constraints,
    #  \param names list of names of the numerical constraints as
    #         inserted by method hpp::core::ProblemSolver::addNumericalConstraint.
    def setNumericalConstraints (self, name, names, priorities = None):
        if priorities is None:
            priorities = [ 0 for i in names ]
        return self.client.basic.problem.setNumericalConstraints (name, names, priorities)

    ## Apply constraints
    #
    #  \param q initial configuration
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def applyConstraints (self, q):
        return self.client.basic.problem.applyConstraints (q)

    ## Generate a configuration satisfying the constraints
    #
    #  \param maxIter maximum number of tries,
    #  \return configuration projected in success,
    #  \throw Error if projection failed.
    def generateValidConfig (self, maxIter):
        return self.client.basic.problem.generateValidConfig (maxIter)

    ## Insert a new LockedDof constraint with given value in the
    #        hpp::manipulation::ProblemSolver map
    # \param lockedDofName key of the constraint in the map
    # \param jointName name of the joint
    # \param value value of the joint configuration
    def createLockedJoint (self, lockedDofName, jointName, value):
        return self.client.manipulation.problem.createLockedJoint \
            (lockedDofName, jointName, value)

    ## Lock degree of freedom of a FreeFlyer joint
    # \param freeflyerBname base name of the joint
    #        (It will be completed by '_xyz' and '_SO3'),
    # \param lockJointBname base name of the LockedJoint constraints
    #        (It will be completed by '_xyz' and '_SO3'),
    # \param values config of the locked joints (7 float)
    def lockFreeFlyerJoint (self, freeflyerBname, lockJointBname,
                            values = (0,0,0,1,0,0,0)):
        lockedJoints = list ()
        namet = lockJointBname + '_xyz'
        namer = lockJointBname + '_SO3'
        self.createLockedJoint (namet, freeflyerBname + '_xyz', values[:3])
        lockedJoints.append (namet)
        self.createLockedJoint (namer, freeflyerBname + '_SO3', values[3:])
        lockedJoints.append (namer)
        return lockedJoints

    ## Lock degree of freedom of a planar joint
    # \param planar base name of the joint
    #        (It will be completed by '_xy' and '_rz'),
    # \param lockJointBname base name of the LockedJoint constraints
    #        (It will be completed by '_xy' and '_rz'),
    # \param values config of the locked joints (4 float)
    def lockPlanarJoint (self, planarBname, lockJointBname, values = (0,0,1,0)):
        lockedJoints = list ()
        namet = lockJointBname + '_xy'
        namer = lockJointBname + '_rz'
        self.createLockedJoint (namet, planarBname + '_xy', values[:2])
        lockedJoints.append (namet)
        self.createLockedJoint (namer, planarBname + '_rz', values[2:])
        lockedJoints.append (namer)
        return lockedJoints

    ## Lock degree of freedom with given value
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    # \param rankInConfiguration rank of the locked dof in the joint
    #        configuration vector
    # \param rankInVelocity rank of the locked dof in the joint
    #        velocity vector
    def lockDof (self, jointName, value, rankInConfiguration, rankInVelocity):
        return self.client.basic.problem.lockDof \
            (jointName, value, rankInConfiguration, rankInVelocity)

    ## Lock joint with one degree of freedom with given value
    # \param jointName name of the joint
    # \param value value of the locked degree of freedom,
    def lockOneDofJoint (self, jointName, value):
        return self.client.basic.problem.lockDof (jointName, value, 0, 0)

    ## error threshold in numerical constraint resolution
    def setErrorThreshold (self, threshold):
        return self.client.basic.problem.setErrorThreshold (threshold)

    ## Set the maximal number of iterations
    def setMaxIterations (self, iterations):
	return self.client.basic.problem.setMaxIterations (iterations)


    ## \}

    ## \name Solve problem and get paths
    # \{

    ## Select path planner type
    #  \param Name of the path planner type, either "DiffusingPlanner",
    #         "VisibilityPrmPlanner", or any type added by method
    #         core::ProblemSolver::addPathPlannerType
    def selectPathPlanner (self, pathPlannerType):
        return self.client.basic.problem.selectPathPlanner (pathPlannerType)

    ## Add path optimizer type
    #  \see hpp.corbaserver.problem_solver.ProblemSolver.addPathOptimizer
    def addPathOptimizer (self, pathOptimizerType):
        return self.client.basic.problem.addPathOptimizer (pathOptimizerType)

    ## Clear path optimizers
    #  \see hpp.corbaserver.problem_solver.ProblemSolver.clearPathOptimizers
    def clearPathOptimizers (self):
        return self.client.basic.problem.clearPathOptimizers ()

    ## Select path validation method
    #  \param Name of the path validation method, either "Discretized"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathValidationType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathValidation (self, pathValidationType, tolerance):
        return self.client.basic.problem.selectPathValidation \
            (pathValidationType, tolerance)

    ## Select path projector method
    #  \param Name of the path projector method, either "None"
    #  "Progressive", "Dichotomy", or any type added by
    #  core::ProblemSolver::addPathProjectorType,
    #  \param tolerance maximal acceptable penetration.
    def selectPathProjector (self, pathProjectorType, tolerance):
        return self.client.basic.problem.selectPathProjector \
            (pathProjectorType, tolerance)

    def prepareSolveStepByStep (self):
        return self.client.basic.problem.prepareSolveStepByStep ()

    def executeOneStep (self):
        return self.client.basic.problem.executeOneStep ()

    def finishSolveStepByStep (self):
        return self.client.basic.problem.finishSolveStepByStep ()

    ## Solve the problem of corresponding ChppPlanner object
    def solve (self):
        return self.client.basic.problem.solve ()

    ## Make direct connection between two configurations
    #  \param startConfig, endConfig: the configurations to link.
    #  \throw Error if steering method fails to create a direct path of if
    #  direct path is not valid
    def directPath (self, startConfig, endConfig, validate = True):
        return self.client.basic.problem.directPath (startConfig, endConfig, validate)

    ## Get Number of paths
    def numberPaths (self):
        return self.client.basic.problem.numberPaths ()

    ## Optimize a given path
    # \param inPathId Id of the path in this problem.
    # \throw Error.
    def optimizePath(self, inPathId):
        return self.client.basic.problem.optimizePath (inPathId)

    ## Get length of path
    # \param inPathId rank of the path in the problem
    # \return length of path if path exists.
    def pathLength(self, inPathId):
        return self.client.basic.problem.pathLength(inPathId)

    ## Get the robot's config at param on the a path
    # \param inPathId rank of the path in the problem
    # \param atDistance : the user parameter choice
    # \return dofseq : the config at param
    def configAtParam(self, inPathId, atDistance):
        return self.client.basic.problem.configAtParam(inPathId, atDistance)

    ## Get way points of a path
    #  \param pathId rank of the path in the problem
    def getWaypoints (self, pathId):
        return self.client.basic.problem.getWaypoints (pathId)

    ## \name Interruption of a path planning request
    #  \{

    ## \brief Interrupt path planning activity
    #   \note this method is effective only when multi-thread policy is used
    #         by CORBA server.
    #         See constructor of class Server for details.
    def interruptPathPlanning (self):
        return self.client.basic.problem.interruptPathPlanning ()
    # \}

    ## \name exploring the roadmap
    #  \{

    ## Get nodes of the roadmap.
    def nodes(self):
	return self.client.basic.problem.nodes ()

    # the configuration of the node nodeId
    def node(self,nodeId):
      return self.client.basic.problem.node(nodeId)

    ## Number of nodes
    def numberNodes (self):
        return self.client.basic.problem.numberNodes ()

    ## Number of edges
    def numberEdges (self):
        return self.client.basic.problem.numberEdges ()

    ## Edge at given rank
    def edge (self, edgeId):
        return self.client.basic.problem.edge (edgeId)

    ## Number of connected components
    def numberConnectedComponents (self):
        return self.client.basic.problem.numberConnectedComponents ()

    ## Nodes of a connected component
    #  \param connectedComponentId index of connected component in roadmap
    #  \return list of nodes of the connected component.
    def nodesConnectedComponent (self, ccId):
        return self.client.basic.problem.nodesConnectedComponent (ccId)

    ## Clear the roadmap
    def clearRoadmap (self):
        return self.client.basic.problem.clearRoadmap ()
    ## \}
