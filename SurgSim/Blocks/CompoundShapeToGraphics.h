// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_COMPOUNDSHAPETOGRAPHICS_H
#define SURGSIM_BLOCKS_COMPOUNDSHAPETOGRAPHICS_H

#include <memory>
#include <vector>
#include <string>

#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{
namespace Graphics
{
class Representation;
}

namespace Framework
{
class Component;
class Representation;
}

namespace Math
{
class CompoundShape;
}

namespace Blocks
{

/// Keep a set of Graphics representations in sync with a CompoundShape, the shape can either be set directly or
/// pulled from a Physics or Collision Representation. The graphics pieces will be set to coincide with the appropriate
/// pieces of the compound shape in order.
class CompoundShapeToGraphics : public Framework::Behavior
{
public:
	/// Constructor
	CompoundShapeToGraphics(const std::string& name);

	/// Destructor
	~CompoundShapeToGraphics();

	void update(double dt) override;

	int getTargetManagerType() const override;

	bool doInitialize() override;

	bool doWakeUp() override;

	/// Sets the shape to be used for synchronization
	/// \param shape The shape to be used
	void setShape(const std::shared_ptr<Math::CompoundShape>& shape);

	/// Sets the source component, the components needs to provide a shape and that shape needs to be a compound shape
	/// \param component The component to be used as a shape source
	/// \throws SurgSim::AssertionFailure if the component does not meet the requirements
	void setSource(const std::shared_ptr<Framework::Component>& component);

	/// Sets the graphics targets to be used, each target will be update with the pose of the corresponding sub shape
	/// in the compound shape. The components need to be graphics representations
	/// \param components The list of graphics representations to be used as targets
	/// \throws SurgSim::AssertionFailure if one of the components is not a graphics representation
	void setTargets(const std::vector<std::shared_ptr<Framework::Component>> components);

	/// Adds a single target to the list of targets, the target needs to be a graphics representation
	/// \param component Graphics Representation to be added at the end of the list
	/// \throws SurgSim::AssertionFailure if the component is not a graphics representation
	void addTarget(const std::shared_ptr<Framework::Component>& component);

	/// \return the registered graphics targets
	std::vector<std::shared_ptr<Graphics::Representation>> getTargets() const;

	/// \return the shape that is being used, if set, nullptr otherwise
	std::shared_ptr<Math::CompoundShape> getShape() const;

	/// \return the component that is the source, if set, nullptr otherwise
	std::shared_ptr<Framework::Component> getSource() const;

private:

	/// Source shape used for updating
	std::shared_ptr<Math::CompoundShape> m_shape;

	/// Source representation if known
	std::shared_ptr<Framework::Component> m_source;

	/// List of graphics targets for updating
	std::vector<std::shared_ptr<Graphics::Representation>> m_representations;
};

}
}

#endif
