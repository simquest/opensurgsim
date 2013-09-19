// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_FRAMEWORK_REPRESENTATION_H
#define SURGSIM_FRAMEWORK_REPRESENTATION_H

#include <memory>

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Framework
{

class SceneElement;

/// Representations are manifestations of a SceneElement. For example, a
/// SceneElement can be represented in graphics, physics, etc. Each of these
/// representation will be derived from this class.
class Representation : public Component
{
public:
	explicit Representation(const std::string& m_name);
	virtual ~Representation();

	/// Set the initial pose of the representation
	/// \param pose The initial pose
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Get the initial pose of the representation
	/// \return The initial pose
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const = 0;

	/// Set the pose of the representation
	/// \param pose The pose to set the representation to
	/// \note This requests the representation to set its pose to the given pose
	/// \note In physics, the actual pose of the representation might not be exactly the requested one
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Get the pose of the representation
	/// \return The pose of this representation
	/// \note getPose may or may not return the pose last sets by setPose
	/// \note In physics, the simulation will drive the pose internally
	virtual const SurgSim::Math::RigidTransform3d& getPose() const = 0;

private:
	virtual bool doInitialize();
	virtual bool doWakeUp();
};

}; // namespace Framework

}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_REPRESENTATION_H
