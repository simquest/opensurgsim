// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015dd, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/CompoundShapeToGraphics.h"
#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{

namespace Blocks
{

CompoundShapeToGraphics::CompoundShapeToGraphics(const std::string& name) : Framework::Behavior(name)
{

}

CompoundShapeToGraphics::~CompoundShapeToGraphics()
{

}

void CompoundShapeToGraphics::update(double dt)
{
	size_t i = 0;
	for (const auto& representation : m_representations)
	{
		representation->setLocalPose(m_shape->getPose(i++));

	}
}

int CompoundShapeToGraphics::getTargetManagerType() const
{
	return Framework::MANAGER_TYPE_GRAPHICS;
}

bool CompoundShapeToGraphics::doInitialize()
{
	return true;
}

bool CompoundShapeToGraphics::doWakeUp()
{
	if (m_source != nullptr)
	{
		std::shared_ptr<Math::Shape> shape;
		m_source->getValue<std::shared_ptr<Math::Shape>>("Shape", &shape);

		SURGSIM_ASSERT(shape != nullptr) << "Source " << m_source->getFullName() << " does not contain a shape.";
		SURGSIM_ASSERT(shape->getType() == Math::SHAPE_TYPE_COMPOUNDSHAPE) << "Source : " << m_source->getFullName()
				<< "Does not contain a compoundshape.";

		m_shape = std::dynamic_pointer_cast<Math::CompoundShape>(shape);
	}
	return true;
}

void CompoundShapeToGraphics::setShape(const std::shared_ptr<Math::CompoundShape>& shape)
{
	m_shape = shape;
	m_source = nullptr;
}

void CompoundShapeToGraphics::setSource(const std::shared_ptr<Framework::Component>& component)
{
	SURGSIM_ASSERT(component->isReadable("Shape")) << "Component : " << component->getFullName()
			<< "Does not contain a shape .";
	m_source = component;

}

void CompoundShapeToGraphics::setTargets(const std::vector<std::shared_ptr<Framework::Component>> components)
{
	m_representations.clear();
	for (const auto& c : components)
	{
		addTarget(c);
	}
}

void CompoundShapeToGraphics::addTarget(const std::shared_ptr<Framework::Component>& component)
{
	auto graphics = Framework::checkAndConvert<Graphics::Representation>(component,
					"SurgSim::Graphics::Representation");
	SURGSIM_ASSERT(graphics != nullptr)
			<< "Component " << component->getFullName() << " not a graphics representation, could not add.";
	m_representations.push_back(std::move(graphics));
}

std::vector<std::shared_ptr<Graphics::Representation>> CompoundShapeToGraphics::getTargets() const
{
	return m_representations;
}

std::shared_ptr<Math::CompoundShape> CompoundShapeToGraphics::getShape() const
{
	return m_shape;
}

std::shared_ptr<Framework::Component> CompoundShapeToGraphics::getSource() const
{
	return m_source;
}


}
}