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

#ifndef SURGSIM_GRAPHICS_REPRESENTATION_H
#define SURGSIM_GRAPHICS_REPRESENTATION_H

#include <SurgSim/Framework/Representation.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Graphics
{

class Manager;
class Material;

/// Base graphics representation class, which defines the interface that all graphics representations must implement.
///
/// A Graphics::Representation is the visual Framework::Representation of a Framework::SceneElement in the
/// Framework::Scene.
class Representation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit Representation(const std::string& name);

	/// Sets whether the representation is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible) = 0;

	/// Gets whether the representation is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const = 0;

	/// Set the initial pose of the representation
	/// \param	pose	The initial pose
	/// \note	This will reset initial, current, and final poses all to the new initial pose.
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Get the initial pose of the representation
	/// \return	The initial pose
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const = 0;

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	virtual bool setMaterial(std::shared_ptr<Material> material) = 0;

	/// Gets the material that defines the visual appearance of the representation
	/// \return	Graphics material
	virtual std::shared_ptr<Material> getMaterial() const = 0;


	/// Removes the material from the representation
	virtual void clearMaterial() = 0;

	/// Updates the representation
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt) = 0;

	/// Add a reference to a group, this will eventual add this representation to the group with the
	/// the same name.
	/// \param	name	The name of the group.
	/// \return	true if it succeeds, false if the group reference already exists.
	virtual bool addGroupReference(const std::string& name) = 0;

	/// Adds a list of group references.
	/// \param	groups	The references.
	virtual void addGroupReferences(const std::vector<std::string>& groups) = 0;

	/// Gets group references.
	/// \return	The group references.
	virtual std::vector<std::string> getGroupReferences() = 0;

};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_REPRESENTATION_H
