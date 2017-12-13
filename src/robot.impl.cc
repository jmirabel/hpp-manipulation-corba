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

#include <pinocchio/multibody/model.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/exception-factory.hh>
#include <hpp/pinocchio/humanoid-robot.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/manipulation/srdf/util.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>

#include <hpp/corbaserver/manipulation/server.hh>

#include "tools.hh"

namespace hpp {
  namespace manipulation {
    namespace impl {
      namespace {
        using pinocchio::Gripper;
        typedef core::ProblemSolver CPs_t;

        DevicePtr_t createRobot (const std::string& name) {
          DevicePtr_t r = Device::create (name);
          return r;
        }

        DevicePtr_t getOrCreateRobot (ProblemSolver* p,
            const std::string& name = "Robot")
        {
          DevicePtr_t r = p->robot ();
          if (r) return r;
          r = createRobot (name);
          p->robot (r);
          return r;
        }

        JointPtr_t getJointByBodyNameOrThrow (ProblemSolver* p,
            const std::string& n)
        {
          DevicePtr_t r = getRobotOrThrow (p);
          JointPtr_t j = r->getJointByBodyName (n);
          if (!j) throw hpp::Error ("Joint not found.");
          return j;
        }

        template<typename GripperOrHandle>
          GripperOrHandle copy (const GripperOrHandle& in, const DevicePtr_t& device, const std::string& p);

        template<> GripperPtr_t copy (const GripperPtr_t& in, const DevicePtr_t& device, const std::string& p) {
            Transform3f position = (in->joint()
                ? in->joint()->currentTransformation() * in->objectPositionInJoint()
                : in->objectPositionInJoint());

            se3::Model& model = device->model();
            const std::string name = p + in->name();
            if (model.existFrame(name))
              throw std::invalid_argument ("Could not add the gripper because a frame \'" + name + "\" already exists.");
            model.addFrame (se3::Frame(
                  name,
                  model.getJointId("universe"),
                  model.getFrameId("universe"),
                  position,
                  se3::OP_FRAME));

            GripperPtr_t out = Gripper::create(name, device);
            out->clearance (in->clearance());
            return out;
        }

        template<> HandlePtr_t copy (const HandlePtr_t& in, const DevicePtr_t& device, const std::string& p) {
            Transform3f position = (in->joint()
                ? in->joint()->currentTransformation() * in->localPosition()
                : in->localPosition());

            HandlePtr_t out = Handle::create(p + in->name(), position, JointPtr_t(new Joint(device, 0)));
            out->clearance (in->clearance());
            return out;
        }

        template<typename Object>
        void copy(const DevicePtr_t& from, const DevicePtr_t& to, const std::string& prefix)
        {
          typedef typename Device::Containers_t::traits<Object>::Map_t Map;
          const Map& m = from->map <Object> ();
          for (typename Map::const_iterator it = m.begin (); it != m.end (); it++) {
            Object obj = copy<Object>(it->second, to, prefix);
            to->add <Object> (obj->name(), obj);
          }
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

      void Robot::finishedRobot (const char* name)
	throw (Error)
      {
	try {
          problemSolver()->robot ()->didInsertRobot(std::string (name));
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
          DevicePtr_t robot = getOrCreateRobot (problemSolver());
          if (robot->has<FrameIndices_t> (robotName))
            HPP_THROW(std::invalid_argument, "A robot named " << robotName << " already exists");
          pinocchio::urdf::loadRobotModel (robot, 0, robotName, rootJointType,
              packageName, modelName, urdfSuffix, srdfSuffix);
	  srdf::loadModelFromFile (robot, robotName,
              packageName, modelName, srdfSuffix);
          robot->didInsertRobot (robotName);
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::insertRobotModelFromString (const char* robotName,
              const char* rootJointType,
              const char* urdfString,
              const char* srdfString)
	throw (Error)
      {
	try {
          DevicePtr_t robot = getOrCreateRobot (problemSolver());
          if (robot->has<FrameIndices_t> (robotName))
            HPP_THROW(std::invalid_argument, "A robot named " << robotName << " already exists");

          pinocchio::urdf::loadModelFromString (robot, 0, robotName,
              rootJointType, urdfString, srdfString);
	  srdf::loadModelFromXML (robot, robotName, srdfString);
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
          DevicePtr_t robot = getOrCreateRobot (problemSolver());
	  srdf::addRobotSRDFModel (robot, std::string (robotName),
              std::string (packageName), std::string (modelName),
              std::string (srdfSuffix));
          robot->didInsertRobot (robotName);
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
          DevicePtr_t robot = getOrCreateRobot (problemSolver());
          if (robot->has<FrameIndices_t> (objectName))
            HPP_THROW(std::invalid_argument, "A robot named " << objectName << " already exists");
          pinocchio::urdf::loadRobotModel (robot, 0, objectName, rootJointType,
              packageName, modelName, urdfSuffix, srdfSuffix);
          srdf::loadModelFromFile (robot, objectName,
              packageName, modelName, srdfSuffix);
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
          DevicePtr_t robot = getOrCreateRobot (problemSolver());
          if (robot->has<FrameIndices_t> (robotName))
            HPP_THROW(std::invalid_argument, "A robot named " << robotName << " already exists");
          pinocchio::urdf::loadHumanoidModel (robot, 0, robotName, rootJointType,
              packageName, modelName, urdfSuffix, srdfSuffix);
          srdf::loadModelFromFile (robot, robotName,
              packageName, modelName, srdfSuffix);
          robot->didInsertRobot (robotName);
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::insertHumanoidModelFromString (const char* robotName,
          const char* rootJointType,
          const char* urdfString,
          const char* srdfString)
	throw (Error)
      {
	try {
          DevicePtr_t robot = getOrCreateRobot (problemSolver());
          if (robot->has<FrameIndices_t> (robotName))
            HPP_THROW(std::invalid_argument, "A robot named " << robotName << " already exists");
          pinocchio::urdf::loadModelFromString (robot, 0, robotName,
              rootJointType, urdfString, srdfString);
          pinocchio::urdf::setupHumanoidRobot (robot, robotName);
	  srdf::loadModelFromXML (robot, robotName, srdfString);
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
          DevicePtr_t robot = getRobotOrThrow (problemSolver());

          std::string modelName (envModelName); 
          DevicePtr_t object = Device::create (modelName);
          pinocchio::urdf::loadUrdfModel (object, "anchor",
              package, modelName + std::string(urdfSuffix));
          srdf::loadModelFromFile (object, "",
              package, envModelName, srdfSuffix);
          std::string p (prefix);
          object->controlComputation(Device::JOINT_POSITION);
          object->computeForwardKinematics();
          object->updateGeometryPlacements();

	  // Detach objects from joints
          using pinocchio::DeviceObjectVector;
          DeviceObjectVector& objects = object->objectVector();
          for (DeviceObjectVector::iterator itObj = objects.begin();
              itObj != objects.end(); ++itObj) {
            problemSolver()->addObstacle (
                p + (*itObj)->name (),
                *(*itObj)->fcl (),
                true, true);
	    hppDout (info, "Adding obstacle " << obj->name ());
	  }
          typedef CPs_t::traits<JointAndShapes_t>::Map_t ShapeMap;
          const ShapeMap& m = object->map <JointAndShapes_t> ();
          for (ShapeMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            JointAndShapes_t shapes;
            for (JointAndShapes_t::const_iterator itT = it->second.begin ();
                itT != it->second.end(); ++itT) {
              const Transform3f& M = itT->first->currentTransformation ();
              Shape_t newShape (itT->second.size());
              for (std::size_t i = 0; i < newShape.size (); ++i)
                newShape [i] = M.act (itT->second[i]);
              shapes.push_back (JointAndShape_t (JointPtr_t(), newShape));
            }
            problemSolver()->core::ProblemSolver::add (p + it->first, shapes);
          }

          copy<HandlePtr_t > (object, robot, p);
          copy<GripperPtr_t> (object, robot, p);
          robot->didInsertRobot (p.substr(0, p.size() - 1));
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      void Robot::loadEnvironmentModelFromString (const char* urdfString,
          const char* srdfString, const char* prefix)
	throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow (problemSolver());

          std::string p (prefix);
          DevicePtr_t object = Device::create (p);
          // TODO replace "" by p and remove `p +` in what follows
          pinocchio::urdf::loadModelFromString (object, 0, "",
              "anchor", urdfString, srdfString);
          srdf::loadModelFromXML (object, "", srdfString);
          object->controlComputation(Device::JOINT_POSITION);
          object->computeForwardKinematics();
          object->updateGeometryPlacements();

	  // Detach objects from joints
          using pinocchio::DeviceObjectVector;
          DeviceObjectVector& objects = object->objectVector();
          for (DeviceObjectVector::iterator itObj = objects.begin();
              itObj != objects.end(); ++itObj) {
            problemSolver()->addObstacle (
                p + (*itObj)->name (),
                *(*itObj)->fcl (),
                true, true);
	    hppDout (info, "Adding obstacle " << obj->name ());
	  }
          typedef CPs_t::traits<JointAndShapes_t>::Map_t ShapeMap;
          const ShapeMap& m = object->map <JointAndShapes_t> ();
          for (ShapeMap::const_iterator it = m.begin ();
              it != m.end (); it++) {
            JointAndShapes_t shapes;
            for (JointAndShapes_t::const_iterator itT = it->second.begin ();
                itT != it->second.end(); ++itT) {
              const Transform3f& M = itT->first->currentTransformation ();
              Shape_t newShape (itT->second.size());
              for (std::size_t i = 0; i < newShape.size (); ++i)
                newShape [i] = M.act (itT->second[i]);
              shapes.push_back (JointAndShape_t (JointPtr_t(), newShape));
            }
            problemSolver()->core::ProblemSolver::add (p + it->first, shapes);
          }

          copy<HandlePtr_t > (object, robot, p);
          copy<GripperPtr_t> (object, robot, p);
          robot->didInsertRobot (p.substr(0, p.size() - 1));
          problemSolver()->resetProblem ();
	} catch (const std::exception& exc) {
	  throw hpp::Error (exc.what ());
	}
      }

      Transform__slice* Robot::getRootJointPosition (const char* robotName)
        throw (Error)
      {
        try {
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
          std::string n (robotName);
          if (!robot->has<FrameIndices_t> (n))
            throw hpp::Error
              ("Root of subtree with the provided prefix not found");
          const se3::Model& model = robot->model();
          const se3::Frame& rf = model.frames[
            robot->get<FrameIndices_t>(n)[0]
            ];
          double* res = new Transform_;
          if (rf.type == se3::JOINT)
            Transform3fTohppTransform (model.jointPlacements[rf.parent], res);
          else
            Transform3fTohppTransform (rf.placement, res);
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
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
          std::string n (robotName);
          Transform3f T;
          hppTransformToTransform3f (position, T);
          robot->setRobotRootPosition(n, T);
          robot->computeForwardKinematics();
        } catch (const std::exception& exc) {
          throw Error (exc.what ());
        }
      }

      void Robot::addHandle (const char* linkName, const char* handleName,
          const ::hpp::Transform_ localPosition)
	throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint =
            getJointByBodyNameOrThrow (problemSolver(), linkName);
          Transform3f T;
          hppTransformToTransform3f(localPosition, T);
	  HandlePtr_t handle = Handle::create (handleName, T, joint);
	  robot->add (handleName, handle);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Robot::addGripper(const char* linkName, const char* gripperName,
          const ::hpp::Transform_ p)
	throw (hpp::Error)
      {
	try {
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint =
            getJointByBodyNameOrThrow (problemSolver(), linkName);
          Transform3f T;
          hppTransformToTransform3f(p, T);
          robot->model().addFrame(
              se3::Frame(gripperName, joint->index(),
                robot->model().getFrameId(joint->name()),
                T, se3::OP_FRAME)
              );
	  GripperPtr_t gripper = Gripper::create (gripperName, robot);
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
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
	  JointPtr_t joint =
            getJointByBodyNameOrThrow (problemSolver(), linkName);
          Transform3f T;
          hppTransformToTransform3f(localPosition, T);
	  HandlePtr_t handle = Handle::create (handleName, T, joint);
          std::vector <bool> mask (6, true); mask [5] = false;
          handle->mask (mask);
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
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
          GripperPtr_t gripper = robot->get <GripperPtr_t> (gripperName);
          if (!gripper)
            throw Error ("This gripper does not exists.");
          const Transform3f& t = gripper->objectPositionInJoint ();
          Transform3fTohppTransform (t, position);
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
          DevicePtr_t robot = getRobotOrThrow (problemSolver());
          HandlePtr_t handle = robot->get <HandlePtr_t> (handleName);
          if (!handle)
            throw Error ("This handle does not exists.");
          const Transform3f& t = handle->localPosition ();
          Transform3fTohppTransform (t, position);
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
