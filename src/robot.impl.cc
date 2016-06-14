// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
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

#include "robot.impl.hh"

#include <hpp/fcl/math/transform.h>
#include <hpp/util/debug.hh>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/gripper.hh>
#include <hpp/model/body.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/manipulation/srdf/util.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/axial-handle.hh>

#include <hpp/corbaserver/manipulation/server.hh>

#include "tools.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      namespace {
        typedef core::ProblemSolver CPs_t;
        typedef ProblemSolver::ThisC_t PSC_t;

        manipulation::DevicePtr_t createRobot (const std::string& name) {
          manipulation::DevicePtr_t r = manipulation::Device::create (name);
          fcl::Transform3f t; t.setIdentity ();
          model::ObjectFactory of;
          JointPtr_t rj = of.createJointAnchor (t);
          rj->name ("base_joint");
          r->rootJoint (rj);
          return r;
        }

        manipulation::DevicePtr_t getOrCreateRobot (ProblemSolver* p,
            const std::string& name = "Robot")
        {
          manipulation::DevicePtr_t r = p->robot ();
          if (r) return r;
          r = createRobot (name);
          p->robot (r);
          return r;
        }

        manipulation::DevicePtr_t getRobotOrThrow (ProblemSolver* p)
        {
          manipulation::DevicePtr_t r = p->robot ();
          if (!r) throw hpp::Error ("Robot not found.");
          return r;
        }

        JointPtr_t getJointByBodyNameOrThrow (ProblemSolver* p,
            const std::string& n)
        {
          manipulation::DevicePtr_t r = getRobotOrThrow (p);
          JointPtr_t j = r->getJointByBodyName (n);
          if (!j) throw hpp::Error ("Joint not found.");
          return j;
        }
      }

      Robot::Robot () : server_ (0x0)
      {}

      ProblemSolverPtr_t Robot::problemSolver ()
      {
        return server_->problemSolver();
      }

      void Robot::create (const char* name)
	throw (Error)
      {
	try {
          problemSolver()->robot (createRobot (std::string (name)));
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::insertRobotModel (const char* robotName,
          const char* rootJointType, const char* packageName,
          const char* modelName, const char* urdfSuffix,
          const char* srdfSuffix)
	throw (Error)
      {
	try {
          manipulation::DevicePtr_t robot = getOrCreateRobot (problemSolver());
          robot->prepareInsertRobot ();
	  manipulation::srdf::loadRobotModel (robot, robot->rootJoint (),
              std::string (robotName), std::string (rootJointType),
              std::string (packageName), std::string (modelName),
              std::string (urdfSuffix), std::string (srdfSuffix));
          robot->didInsertRobot (robotName);
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::insertRobotSRDFModel (const char* robotName,
          const char* packageName, const char* modelName,
          const char* srdfSuffix)
	throw (Error)
      {
	try {
          manipulation::DevicePtr_t robot = getOrCreateRobot (problemSolver());
	  manipulation::srdf::addRobotSRDFModel (robot, std::string (robotName),
              std::string (packageName), std::string (modelName),
              std::string (srdfSuffix));
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::insertObjectModel (const char* objectName,
          const char* rootJointType, const char* packageName,
          const char* modelName, const char* urdfSuffix,
          const char* srdfSuffix)
	throw (Error)
      {
	try {
          manipulation::DevicePtr_t robot = getOrCreateRobot (problemSolver());
          robot->prepareInsertRobot ();
          manipulation::srdf::loadObjectModel (robot, robot->rootJoint (),
              std::string (objectName), std::string (rootJointType),
              std::string (packageName), std::string (modelName),
              std::string (urdfSuffix), std::string (srdfSuffix));
          robot->didInsertRobot (objectName);
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::insertHumanoidModel (const char* robotName,
          const char* rootJointType, const char* packageName,
          const char* modelName, const char* urdfSuffix,
          const char* srdfSuffix)
	throw (Error)
      {
	try {
          manipulation::DevicePtr_t robot = getOrCreateRobot (problemSolver());
          robot->prepareInsertRobot ();
	  manipulation::srdf::loadHumanoidModel (robot, robot->rootJoint (),
              std::string (robotName), std::string (rootJointType),
              std::string (packageName), std::string (modelName),
              std::string (urdfSuffix), std::string (srdfSuffix));
          robot->didInsertRobot (robotName);
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::loadEnvironmentModel (const char* package,
          const char* envModelName, const char* urdfSuffix,
          const char* srdfSuffix, const char* prefix)
	throw (hpp::Error)
      {
	try {
          manipulation::DevicePtr_t object =
            manipulation::Device::create (std::string (envModelName));
          manipulation::srdf::loadEnvironmentModel (object,
              std::string (package), std::string (envModelName),
              std::string (urdfSuffix), std::string (srdfSuffix));
          std::string p (prefix);

	  // Detach objects from joints
	  for (model::ObjectIterator itObj = object->objectIterator
		 (hpp::model::COLLISION); !itObj.isEnd (); ++itObj) {
            model::CollisionObjectPtr_t obj = model::CollisionObject::create
	      ((*itObj)->fcl ()->collisionGeometry(), (*itObj)->getTransform (), p + (*itObj)->name ());
	    problemSolver()->addObstacle (obj, true, true);
	    hppDout (info, "Adding obstacle " << obj->name ());
          }
          typedef CPs_t::traits<JointAndShapes_t>::Map_t ShapeMap;
          const ShapeMap& m = object->map <JointAndShapes_t> ();
          for (ShapeMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            JointAndShapes_t shapes;
            for (JointAndShapes_t::const_iterator itT = it->second.begin ();
                itT != it->second.end(); ++itT) {
              const fcl::Transform3f& M = itT->first->currentTransformation ();
              Shape_t newShape (itT->second.size());
              for (std::size_t i = 0; i < newShape.size (); ++i)
                newShape [i] = M.transform (itT->second[i]);
              shapes.push_back (JointAndShape_t (NULL, newShape));
            }
            problemSolver()->PSC_t::add (p + it->first, shapes);
          }
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }


      Transform__slice* Robot::getRootJointPosition (const char* robotName)
        throw (Error)
      {
        try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
          std::string n (robotName);
          model::JointPtr_t joint (NULL),
            root = robot->rootJoint ();
          for (size_t i = 0; i < root->numberChildJoints (); ++i)
            if (root->childJoint (i)->name ().compare (0, n.size(), n) == 0) {
              joint = root->childJoint (i);
              break;
            }
          if (!joint)
            throw hpp::Error
              ("Root of subtree with the provided prefix not found");
          const Transform3f& T = joint->positionInParentFrame ();
          double* res = new Transform_;
          corbaserver::Transform3fTohppTransform (T, res);
          return res;
        } catch (const std::exception& exc) {
          throw Error (exc.what ());
        }
      }

      void Robot::setRootJointPosition (const char* robotName,
                                        const ::hpp::Transform_ position)
        throw (Error)
      {
        try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
          std::string n (robotName);
          model::JointPtr_t joint (NULL),
            root = robot->rootJoint ();
          for (size_t i = 0; i < root->numberChildJoints (); ++i)
            if (root->childJoint (i)->name ().compare (0, n.size(), n) == 0) {
              joint = root->childJoint (i);
              break;
            }
          if (!joint)
            throw hpp::Error
              ("Root of subtree with the provided prefix not found");
	  fcl::Quaternion3f q (position [3], position [4],
			       position [5], position [6]);
	  fcl::Vec3f v (position [0], position [1],
			 position [2]);
          joint->positionInParentFrame (fcl::Transform3f (q, v));
        } catch (const std::exception& exc) {
          throw Error (exc.what ());
        }
      }

      void Robot::addHandle (const char* linkName, const char* handleName,
          const ::hpp::Transform_ localPosition)
	throw (hpp::Error)
      {
	try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint =
            getJointByBodyNameOrThrow (problemSolver(), linkName);
	  fcl::Quaternion3f q (localPosition [3], localPosition [4],
			       localPosition [5], localPosition [6]);
	  fcl::Vec3f v (localPosition [0], localPosition [1],
			 localPosition [2]);
	  HandlePtr_t handle = Handle::create (handleName, Transform3f (q, v),
					       joint);
	  robot->add (handleName, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::addGripper(const char* linkName, const char* gripperName,
          const ::hpp::Transform_ p, const Names_t& bodyInCollisionNames)
	throw (hpp::Error)
      {
	try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint =
            getJointByBodyNameOrThrow (problemSolver(), linkName);
	  fcl::Quaternion3f q (p [3], p [4], p [5], p [6]);
	  fcl::Vec3f v (p [0], p [1], p [2]);
          model::JointVector_t jointInCollision;
          for (CORBA::ULong i=0; i<bodyInCollisionNames.length (); ++i) {     
	    std::string bodyName (bodyInCollisionNames [i]);
            jointInCollision.push_back(robot->getJointByBodyName(bodyName));
          }
	  GripperPtr_t gripper = model::Gripper::create (gripperName, joint, 
                                                  Transform3f (q, v),
                                                  jointInCollision);
	  robot->add (gripperName, gripper);
          // hppDout (info, "add Gripper: " << *gripper); 
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::addAxialHandle (const char* linkName, const char* handleName,
          const ::hpp::Transform_ localPosition)
	throw (hpp::Error)
      {
	try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint =
            getJointByBodyNameOrThrow (problemSolver(), linkName);
	  fcl::Quaternion3f q (localPosition [3], localPosition [4],
			       localPosition [5], localPosition [6]);
	  fcl::Vec3f v (localPosition [0], localPosition [1],
			 localPosition [2]);
	  HandlePtr_t handle = AxialHandle::create
	    (handleName, Transform3f (q, v), joint);
	  robot->add (handleName, handle);
          hppDout (info, "add Handle: " << *handle); 
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      char* Robot::getGripperPositionInJoint (const char* gripperName,
          ::hpp::Transform__out position)
        throw (hpp::Error)
      {
	try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
          GripperPtr_t gripper = robot->get <GripperPtr_t> (gripperName);
          if (!gripper)
            throw Error ("This gripper does not exists.");
          const fcl::Transform3f& t = gripper->objectPositionInJoint ();
          corbaserver::Transform3fTohppTransform (t, position);
          char* name = new char[gripper->joint ()->name ().length()+1];
          strcpy (name, gripper->joint ()->name ().c_str ());
          return name;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      char* Robot::getHandlePositionInJoint (const char* handleName,
          ::hpp::Transform__out position)
        throw (hpp::Error)
      {
	try {
          manipulation::DevicePtr_t robot = getRobotOrThrow (problemSolver());
          HandlePtr_t handle = robot->get <HandlePtr_t> (handleName);
          if (!handle)
            throw Error ("This handle does not exists.");
          const fcl::Transform3f& t = handle->localPosition ();
          corbaserver::Transform3fTohppTransform (t, position);
          char* name = new char[handle->joint ()->name ().length()+1];
          strcpy (name, handle->joint ()->name ().c_str ());
          return name;
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
        }
      }

    } // namespace impl
  } // namespace manipulation
} // namespace hpp
