// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Blocks/VisualizeConstraints.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/SlidingConstraintData.h"

namespace SurgSim
{

namespace Blocks
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::VisualizeConstraintsBehavior,
				 VisualizeConstraintsBehavior);

VisualizeConstraintsBehavior::VisualizeConstraintsBehavior(const std::string& name) :
	Behavior(name),
	m_logger(SurgSim::Framework::Logger::getLogger("Block/VisualizeConstraintsBehavior"))
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(VisualizeConstraintsBehavior, FieldsType, VectorFields,
									  getVectorFields, setVectorFields);
}

void VisualizeConstraintsBehavior::setVectorField(Physics::ConstraintGroupType constraintType,
		const std::shared_ptr<Framework::Component>& vectorField)
{
	SURGSIM_ASSERT(constraintType < Physics::CONSTRAINT_GROUP_TYPE_COUNT) << "Invalid constraint type";

	auto converted = Framework::checkAndConvert<Graphics::VectorFieldRepresentation>(
						 vectorField, "SurgSim::Graphics::VectorFieldRepresentation");

	boost::lock_guard<boost::mutex> lock(m_graphicsMutex);
	m_graphics[constraintType] = converted;
}

void VisualizeConstraintsBehavior::update(double dt)
{
	using DataStructures::Vertices;
	using Graphics::VectorFieldData;
	using Physics::ConstraintGroupType;
	using Physics::ContactConstraintData;

	Physics::PhysicsManagerState state;
	auto manager = m_manager.lock();
	if (manager == nullptr)
	{
		return;
	}

	manager->getFinalState(&state);

	if (state.getMlcpProblem().getSize() == 0)
	{
		return;
	}

	Math::MlcpSolution::Vector& x = state.getMlcpSolution().x;
	if (state.getMlcpProblem().getSize() != static_cast<size_t>(x.size()))
	{
		SURGSIM_LOG_WARNING(m_logger) << "mlcp solution size = " << x.size() << " while mlcp problem size = " <<
									  state.getMlcpProblem().getSize() << std::endl;
		return;
	}

	if (x.size() < 1)
	{
		return;
	}

	// For each constraint type...
	size_t constraintTypeEnd = static_cast<int>(Physics::CONSTRAINT_GROUP_TYPE_COUNT);

	std::map<int, Graphics::VectorField> vectorFields;

	for (size_t constraintType = 0; constraintType < constraintTypeEnd; constraintType++)
	{
		ConstraintGroupType constraintGroupType = static_cast<ConstraintGroupType>(constraintType);
		const std::vector<std::shared_ptr<Physics::Constraint>>& constraints =
					state.getConstraintGroup(constraintType);

		// For each constraint of that type...
		for (auto it = constraints.begin(); it != constraints.end(); it++)
		{
			// Let's gather the application point and the force vector...
			Math::Vector3d force = Math::Vector3d::Zero();
			Math::Vector3d point = (*it)->getLocalizations().first->calculatePosition();

			int mlcpConstraintIndex = state.getConstraintsMapping().getValue((*it).get());
			Math::Vector4d color = Math::Vector4d::Zero();

			switch ((*it)->getType())
			{
				case SurgSim::Physics::FIXED_3DPOINT:
				{
					force = x.segment(mlcpConstraintIndex, 3);
					color = SurgSim::Math::Vector4d(0.9, 0.9, 0.9, 1);
				}
				break;
				case SurgSim::Physics::FIXED_3DROTATION_VECTOR:
				{
					force = x.segment(mlcpConstraintIndex, 3);
					color = SurgSim::Math::Vector4d(0.0, 0.0, 0.9, 1);
				}
				break;
				case SurgSim::Physics::FRICTIONLESS_3DCONTACT:
				{
					const SurgSim::Math::Vector3d& forceNormal =
						std::static_pointer_cast<ContactConstraintData>((*it)->getData())->getNormal();
					double forceNormalIntensity = x[mlcpConstraintIndex];
					force = forceNormal * forceNormalIntensity;
					color = SurgSim::Math::Vector4d(0.9, 0.0, 0.0, 1);
				}
				break;
				case SurgSim::Physics::FRICTIONLESS_SLIDING:
				{
					auto data = std::static_pointer_cast<SurgSim::Physics::SlidingConstraintData>((*it)->getData());
					auto normals = data->getNormals();
					force = normals[0].cross(normals[1]) * 0.1;
					color = SurgSim::Math::Vector4d(0.0, 0.9, 0.0, 1);
				}
				break;
				default:
					break;
			}

			// Build the data
			VectorFieldData data;
			data.direction = force;
			data.color.setValue(color);
			Vertices<VectorFieldData>::VertexType vertex(point, data);
			vectorFields[constraintGroupType].addVertex(vertex);
		}
	}

	boost::lock_guard<boost::mutex> lock(m_graphicsMutex);
	for (const auto& graphics : m_graphics)
	{
		graphics.second->updateVectorField(vectorFields[graphics.first]);
	}
}

int VisualizeConstraintsBehavior::getTargetManagerType() const
{
	return Framework::MANAGER_TYPE_GRAPHICS;
}

bool VisualizeConstraintsBehavior::doInitialize()
{
	m_manager = getRuntime()->getManager<Physics::PhysicsManager>();
	return !m_manager.expired();
}

bool VisualizeConstraintsBehavior::doWakeUp()
{
	return true;
}

void VisualizeConstraintsBehavior::setVectorFields(const FieldsType& fields)
{
	for (const auto& field : fields)
	{
		setVectorField(static_cast<Physics::ConstraintGroupType>(field.first), field.second);
	}
}

SurgSim::Blocks::VisualizeConstraintsBehavior::FieldsType VisualizeConstraintsBehavior::getVectorFields() const
{
	FieldsType result;
	for (const auto& field : m_graphics)
	{
		result.push_back(field);
	}
	return result;
}

VisualizeConstraints::VisualizeConstraints(const std::string& name) : BasicSceneElement(name)
{
	auto visualizer = std::make_shared<VisualizeConstraintsBehavior>("Visualizer");
	addComponent(visualizer);
	{
		auto vectors = std::make_shared<Graphics::OsgVectorFieldRepresentation>("Contact");
		visualizer->setVectorField(Physics::CONSTRAINT_GROUP_TYPE_CONTACT, vectors);
		addComponent(vectors);
	}
	{
		auto vectors = std::make_shared<Graphics::OsgVectorFieldRepresentation>("Scene");
		visualizer->setVectorField(Physics::CONSTRAINT_GROUP_TYPE_SCENE, vectors);
		addComponent(vectors);
	}
}

}
}
