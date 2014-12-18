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

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{

namespace Framework
{

/// Representations are manifestations of a SceneElement. For example, a
/// SceneElement can be represented in graphics, physics, etc. Each of these
/// representation will be derived from this class.
class Representation : public Component
{
public:
	/// Constructor
	/// \param name Name of the representation
	explicit Representation(const std::string& name);
	/// Destructor
	virtual ~Representation();

	/// Set the pose of the representation with respect to the Scene Element
	/// \param pose The pose to set the representation to
	virtual void setLocalPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Get the pose of the representation with respect to the Scene Element
	/// \return The pose of this representation
	virtual SurgSim::Math::RigidTransform3d getLocalPose() const;

	/// Get the pose of the representation in world coordinates
	/// \return The pose of this representation
	virtual SurgSim::Math::RigidTransform3d getPose() const;

private:
	/// Local Pose of the Representation with respect to the SceneElement
	SurgSim::Math::RigidTransform3d m_localPose;

	bool doInitialize() override;
	bool doWakeUp() override;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_REPRESENTATION_H
