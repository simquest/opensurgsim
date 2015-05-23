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

#include "SurgSim/Framework/Representation.h"

#include "SurgSim/Math/RigidTransform.h"

#include <unordered_set>

namespace SurgSim
{

namespace Graphics
{

class Manager;
class Material;

/// Base graphics representation class, which defines the interface that all graphics representations must implement.
///
/// A Graphics::Representation is the visual Framework::Representation of a Framework::SceneElement in the
/// Framework::Scene. Graphical representations can request to be assigned to groups, groups are used to select certain
/// elements for rendering in various special effects.
class Representation : public SurgSim::Framework::Representation
{
public:

	static const std::string DefaultGroupName;
	static const std::string DefaultHudGroupName;

	/// Constructor
	/// \param	name	Name of the representation
	explicit Representation(const std::string& name);

	/// Destructor
	virtual ~Representation();

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	virtual bool setMaterial(std::shared_ptr<Material> material) = 0;

	/// Gets the material that defines the visual appearance of the representation
	/// \return	Graphics material
	virtual std::shared_ptr<Material> getMaterial() const = 0;

	/// Removes the material from the representation
	virtual void clearMaterial() = 0;

	/// Sets the representation to render as a wire frame.
	/// \param	val	true if this representation should be rendered as a wireframe.
	virtual void setDrawAsWireFrame(bool val) = 0;

	/// Return if the representation is rendered as a wire frame.
	/// \return	True if this representation is rendered as a wireframe; false if not.
	virtual bool getDrawAsWireFrame() const = 0;

	/// Enable or disable the generation of tangents.
	/// \param value true enables tangent generation, false otherwise
	virtual void setGenerateTangents(bool value) = 0;

	/// \return whether tangent generation is on or off
	virtual bool isGeneratingTangents() const = 0;

	/// Updates the representation
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt) = 0;

	/// Add a reference to a group, this will eventual add this representation to the group with the
	/// the same name.
	/// \param	name	The name of the group.
	/// \return	true if it succeeds, false if the group reference already exists.
	virtual bool addGroupReference(const std::string& name);


	/// Adds a list of group references.
	/// \param	groups	The references.
	void addGroupReferences(const std::vector<std::string>& groups);

	/// Sets the list of group references. Clearing all the old references
	/// \param groups The references.
	void setGroupReferences(const std::vector<std::string>& groups);

	/// Helper functions, this clears all the references and sets, only the reference
	/// given in the parameter
	/// \param group The reference to be used for this representation
	void setGroupReference(const std::string& group);

	/// Gets group references.
	/// \return	The group references.
	std::vector<std::string> getGroupReferences() const;

	/// Function to remove an unwanted reference.
	/// \param group The name of the reference that should be removed
	/// \return true If the reference was found and removed
	bool removeGroupReference(const std::string& group);

	/// Clear all the Group references
	void clearGroupReferences();

private:
	/// List of groups that this representation would like to be added
	std::unordered_set<std::string> m_groups;

	/// Check if the representation is awake and print a warning message if it is.
	/// \param functionName the name of the calling function to be used in the error message
	/// \return the value of isAwake()
	bool checkAwake(const std::string& functionName);
};

};  // namespace Graphics
};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_REPRESENTATION_H
