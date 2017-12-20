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

#ifndef SURGSIM_GRAPHICS_SKELETONREPRESENTATION_H
#define SURGSIM_GRAPHICS_SKELETONREPRESENTATION_H

#include <string>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Graphics/Model.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/RigidTransform.h"


namespace SurgSim
{

namespace Graphics
{

/// Skeleton representation is used to move a mesh based on the movements of
/// pre-selected control points (bones).
class SkeletonRepresentation : public virtual Representation
{
public:

	/// Constructor.
	/// \param	name	The name of the representation.
	explicit SkeletonRepresentation(const std::string& name): Representation(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(SkeletonRepresentation, std::shared_ptr<SurgSim::Framework::Asset>,
			Model, getModel, setModel);

		// Enables the alternative use of the model file instead of the actual mesh object
		DecoderType decoder = std::bind(&SkeletonRepresentation::loadModel,
			this,
			std::bind(&YAML::Node::as<std::string>, std::placeholders::_1));
		setDecoder("ModelFileName", decoder);

		SetterType setter = std::bind(&SkeletonRepresentation::loadModel,
			this,
			std::bind(SurgSim::Framework::convert<std::string>, std::placeholders::_1));

		setSetter("ModelFileName", setter);

		typedef std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d> PoseMap;
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(SkeletonRepresentation, PoseMap,
			NeutralPoses, getNeutralBonePoses, setNeutralBonePoses);
	}

	/// Convenience function to trigger the load of the model with the given filename, if successful, this will
	/// replace the old model
	/// \param	fileName Name of the file to be loaded
	virtual void loadModel(const std::string& fileName) = 0;

	/// Set the current model to the model passed
	/// \param model to be used for this scenery representation, this will replace the old model
	virtual void setModel(std::shared_ptr<SurgSim::Framework::Asset> model) = 0;

	/// \return the current model.
	virtual std::shared_ptr<Model> getModel() const = 0;

	/// Set the pose for a given bone.
	/// \param name The name of the bone.
	/// \param pose The pose of the bone.
	virtual void setBonePose(const std::string& name, const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Get the pose for a given bone.
	/// \param name The name of the bone.
	/// \return pose The pose of the bone.
	virtual SurgSim::Math::RigidTransform3d getBonePose(const std::string& name) const = 0;

	/// Set the neutral pose for a given bone.
	/// \param name The name of the bone.
	/// \param pose The neutral pose of the bone.
	virtual void setNeutralBonePose(const std::string& name, const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Get the neutral pose for a given bone.
	/// \param name The name of the bone.
	/// \return pose The neutral pose of the bone.
	virtual SurgSim::Math::RigidTransform3d getNeutralBonePose(const std::string& name) const = 0;

protected:
	/// Set neutral poses for a set of bones.
	/// \param poses A map of bone names and neutral poses
	virtual void setNeutralBonePoses(const std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d>& poses) = 0;

	/// Get all the neutral poses
	/// \return A map of bone names and neutral poses
	virtual std::map<std::string, SurgSim::Math::UnalignedRigidTransform3d> getNeutralBonePoses() const = 0;
};

};  // namespace Graphics
};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_SKELETONREPRESENTATION_H
