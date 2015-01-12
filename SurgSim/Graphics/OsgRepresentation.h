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

#ifndef SURGSIM_GRAPHICS_OSGREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGREPRESENTATION_H

#include <memory>

#include <osg/ref_ptr>

#include "SurgSim/Graphics/Representation.h"

namespace osg
{
class Node;
class PositionAttitudeTransform;
class Switch;
}

namespace SurgSim
{
namespace Graphics
{

class OsgMaterial;

/// Base OSG implementation of a graphics representation.
///
/// Wraps an osg::Node which serves as the root for the representation's portion of the scene graph.
class OsgRepresentation : public virtual Representation
{
public:
	/// Constructor
	explicit OsgRepresentation(const std::string& name);
	/// Destructor
	virtual ~OsgRepresentation();

	/// Returns the root OSG Node for this representations portion of the scene graph
	osg::ref_ptr<osg::Node> getOsgNode() const;

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	/// \note	OsgPlaneRepresentation only accepts subclasses of OsgMaterial.
	bool setMaterial(std::shared_ptr<Material> material) override;

	/// Gets the material that defines the visual appearance of the representation
	/// \return	Graphics material
	std::shared_ptr<Material> getMaterial() const override;

	/// Removes the material from the representation
	void clearMaterial() override;

	void setDrawAsWireFrame(bool val) override;
	bool getDrawAsWireFrame() const override;

	/// Updates the representation.
	/// \param	dt	The time in seconds of the preceding timestep.
	void update(double dt) override;

protected:
	virtual void doUpdate(double dt);

	/// Set the visibility of this representation
	/// \param val The visibility
	void setVisible(bool val);

	/// Switch used to toggle the visibility of the representation
	osg::ref_ptr<osg::Switch> m_switch;
	/// Transform used to pose the representation
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;

	/// Material defining the visual appearance of the representation
	std::shared_ptr<OsgMaterial> m_material;

	/// Indicates if the representation is rendered as a wireframe.
	bool m_drawAsWireFrame;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGREPRESENTATION_H