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

#ifndef SURGSIM_FRAMEWORK_POSECOMPONENT_H
#define SURGSIM_FRAMEWORK_POSECOMPONENT_H

#include <memory>

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{

namespace Framework
{
SURGSIM_STATIC_REGISTRATION(PoseComponent);

/// The PoseComponent holds a pose. It is used to group Representations that
/// share a common pose, in a SceneElement for example.
class PoseComponent : public Component
{
public:
	/// Constructor
	/// \param name Name of the representation
	explicit PoseComponent(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Framework::PoseComponent);

	/// Set the Pose of the PoseComponent
	/// \param pose The pose in world coordinates
	void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Get the pose of the PoseComponent
	/// \return The pose in world coordinates
	const SurgSim::Math::UnalignedRigidTransform3d& getPose() const;

protected:
	/// Get the PoseComponent for this component
	/// A PoseComponent cannot have a PoseComponent, so this will
	/// return nullptr.
	/// \return The PoseComponent
	std::shared_ptr<PoseComponent> getPoseComponent() override;

	/// Get the PoseComponent for this component, constant access
	/// A PoseComponent cannot have a PoseComponent, so this will
	/// return nullptr.
	/// \return The PoseComponent
	std::shared_ptr<const PoseComponent> getPoseComponent() const override;

private:
	bool doInitialize() override;
	bool doWakeUp() override;

	SurgSim::Math::UnalignedRigidTransform3d m_pose;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_POSECOMPONENT_H
