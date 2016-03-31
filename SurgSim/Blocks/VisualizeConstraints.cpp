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
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgVectorFieldRepresentation.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/PhysicsManager.h"

namespace SurgSim
{

namespace Blocks
{


VisualizeConstraintsBehavior::VisualizeConstraintsBehavior(const std::string& name) : Behavior(name)
{

}

void VisualizeConstraintsBehavior::setVectorField(SurgSim::Physics::ConstraintGroupType constraintType,
		std::shared_ptr<SurgSim::Graphics::VectorFieldRepresentation> vectorField)
{
	m_gfxVectorField[constraintType] = vectorField;
}

void VisualizeConstraintsBehavior::update(double dt)
{
	using SurgSim::DataStructures::Vertices;
	using SurgSim::Graphics::VectorFieldData;
	using SurgSim::Physics::ConstraintGroupType;
	using SurgSim::Physics::ContactConstraintData;

	SurgSim::Physics::PhysicsManagerState state;
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

	SurgSim::Math::MlcpSolution::Vector& x = state.getMlcpSolution().x;
	if (state.getMlcpProblem().getSize() != static_cast<size_t>(x.size()))
	{
		std::cout << "mlcp solution size = " << x.size() << " while mlcp problem size = " <<
				  state.getMlcpProblem().getSize() << std::endl;
		return;
	}

	if (x.size() < 1)
	{
		return;
	}

	// For each constraint type...
	size_t constraintTypeEnd = static_cast<int>(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_COUNT);

	std::map<int, SurgSim::Graphics::VectorField> vectorFields;

	for (size_t constraintType = 0; constraintType < constraintTypeEnd; constraintType++)
	{
		ConstraintGroupType constraintGroupType = static_cast<ConstraintGroupType>(constraintType);
		const std::vector<std::shared_ptr<SurgSim::Physics::Constraint>>& constraints =
					state.getConstraintGroup(constraintType);

		// For each constraint of that type...
		for (auto it = constraints.begin(); it != constraints.end(); it++)
		{
			// Let's gather the application point and the force vector...
			SurgSim::Math::Vector3d force = SurgSim::Math::Vector3d::Zero();
			SurgSim::Math::Vector3d point = (*it)->getLocalizations().first->calculatePosition();

			int mlcpConstraintIndex = state.getConstraintsMapping().getValue((*it).get());
			SurgSim::Math::Vector4d color = SurgSim::Math::Vector4d::Zero();

			switch ((*it)->getType())
			{
				case SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT:
					{
						force = x.segment(mlcpConstraintIndex, 3);
						color = SurgSim::Math::Vector4d(0.9, 0.9, 0.9, 1);
					}
					break;
				case SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT:
					{
						const SurgSim::Math::Vector3d& forceNormal =
							std::static_pointer_cast<ContactConstraintData>((*it)->getData())->getNormal();
						double forceNormalIntensity = x[mlcpConstraintIndex];
						force = forceNormal * forceNormalIntensity;
						color = SurgSim::Math::Vector4d(0.9, 0.0, 0.0, 1);
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

	// Update the graphics this will clear all the places where there haven't been constraints
	for (const auto& graphics : m_gfxVectorField)
	{
		graphics.second->updateVectorField(vectorFields[graphics.first]);
	}
}

int VisualizeConstraintsBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_GRAPHICS;
}

bool VisualizeConstraintsBehavior::doInitialize()
{
	m_manager = getRuntime()->getManager<SurgSim::Physics::PhysicsManager>();
	return !m_manager.expired();
}

bool VisualizeConstraintsBehavior::doWakeUp()
{
	return true;
}

VisualizeConstraints::VisualizeConstraints(const std::string& name) : BasicSceneElement(name)
{
	auto visualizer = std::make_shared<VisualizeConstraintsBehavior>("Visualizer");
	addComponent(visualizer);
	{
		auto vectors = std::make_shared<Graphics::OsgVectorFieldRepresentation>("Contact");
		visualizer->setVectorField(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_CONTACT, vectors);
		addComponent(vectors);
	}
	{
		auto vectors = std::make_shared<Graphics::OsgVectorFieldRepresentation>("Scene");
		visualizer->setVectorField(SurgSim::Physics::CONSTRAINT_GROUP_TYPE_SCENE, vectors);
		addComponent(vectors);
	}
}

}
}