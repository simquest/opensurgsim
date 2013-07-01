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

#ifndef SURGSIM_GRAPHICS_OSGREPRESENTATIONBASE_H
#define SURGSIM_GRAPHICS_OSGREPRESENTATIONBASE_H

#include <memory>

#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/Representation.h>

namespace osg
{
	class Switch;
	class PositionAttitudeTransform;
}

namespace SurgSim
{
namespace Graphics
{

class OsgMaterial;

class OsgRepresentationBase : public OsgRepresentation, public virtual Representation
{
public:

	/// Constructor
	OsgRepresentationBase(const std::string& name);
	~OsgRepresentationBase();

	osg::ref_ptr<osg::Node> getOsgNode() const;

	virtual void setVisible(bool visible);

	virtual bool isVisible() const;

	virtual void update(double dt);

	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose);

	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const;

	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose);

	virtual const SurgSim::Math::RigidTransform3d& getPose() const;

	virtual bool setMaterial(std::shared_ptr<Material> material);

	virtual std::shared_ptr<Material> getMaterial() const;

	virtual void clearMaterial();


protected:
	osg::ref_ptr<osg::Switch> m_switch;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;

private:
	virtual void doUpdate(double dt);

	/// Initial pose of the representation
	SurgSim::Math::RigidTransform3d m_initialPose;
	SurgSim::Math::RigidTransform3d m_pose;

	std::shared_ptr<OsgMaterial> m_material;

};

}; // Graphics
}; // SurgSim

#endif