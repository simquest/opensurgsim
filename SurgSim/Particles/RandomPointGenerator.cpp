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

#include "SurgSim/Particles/RandomPointGenerator.h"

#include "SurgSim/Math/Shape.h"
#include "SurgSim/Particles/DefaultPointGenerator.h"
#include "SurgSim/Particles/RandomBoxPointGenerator.h"
#include "SurgSim/Particles/RandomMeshPointGenerator.h"
#include "SurgSim/Particles/RandomSpherePointGenerator.h"


namespace SurgSim
{
namespace Particles
{

RandomPointGenerator::RandomPointGenerator()
{
	for (size_t index = 0; index < static_cast<size_t>(Math::SHAPE_TYPE_COUNT); ++index)
	{
		m_pointGenerators[index].reset(new DefaultPointGenerator());
	}

	m_pointGenerators[Math::SHAPE_TYPE_BOX].reset(new RandomBoxPointGenerator());
	m_pointGenerators[Math::SHAPE_TYPE_MESH].reset(new RandomMeshPointGenerator());
	m_pointGenerators[Math::SHAPE_TYPE_SPHERE].reset(new RandomSpherePointGenerator());
}

Math::Vector3d RandomPointGenerator::pointInShape(std::shared_ptr<Math::Shape> shape)
{
	SURGSIM_ASSERT(shape != nullptr) << "Empty shape passed in.";

	auto shapeType = shape->getType();
	SURGSIM_ASSERT(Math::SHAPE_TYPE_NONE < shapeType && shapeType < Math::SHAPE_TYPE_COUNT) <<
		"Unknown shape type passed in.";

	return m_pointGenerators[shapeType]->pointInShape(shape);
}

Math::Vector3d RandomPointGenerator::pointOnShape(std::shared_ptr<Math::Shape> shape)
{
	SURGSIM_ASSERT(shape != nullptr) << "Empty shape passed in.";

	auto shapeType = shape->getType();
	SURGSIM_ASSERT(Math::SHAPE_TYPE_NONE < shapeType && shapeType < Math::SHAPE_TYPE_COUNT) <<
		"Unknown shape type passed in.";

	return m_pointGenerators[shapeType]->pointOnShape(shape);
}

}; // namespace Particles
}; // namespace SurgSim
