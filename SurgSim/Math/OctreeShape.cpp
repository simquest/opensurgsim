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

#include "SurgSim/Math/OctreeShape.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::OctreeShape, OctreeShape);

OctreeShape::OctreeShape() :
	m_rootNode(std::make_shared<NodeType>())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(
		SurgSim::Math::OctreeShape,
		std::shared_ptr<SurgSim::Framework::Asset>,
		Octree,
		getOctree,
		setOctree);

	// Enables the alternative use of the mesh file instead of the actual mesh object
	DecoderType decoder = std::bind(&OctreeShape::loadOctree,
									this,
									std::bind(&YAML::Node::as<std::string>, std::placeholders::_1));
	setDecoder("OctreeFileName", decoder);

	SetterType setter = std::bind(&OctreeShape::loadOctree,
								  this,
								  std::bind(SurgSim::Framework::convert<std::string>, std::placeholders::_1));

	setSetter("OctreeFileName", setter);
}

OctreeShape::~OctreeShape()
{
}

int OctreeShape::getType() const
{
	return SHAPE_TYPE_OCTREE;
}

void OctreeShape::loadOctree(const std::string& filePath)
{
	auto rootNode = std::make_shared<NodeType>();
	rootNode->load(filePath);

	SURGSIM_ASSERT(isValid(rootNode))
			<< "Loading failed " << filePath << " contains an invalid octree.";

	setOctree(rootNode);
}

double OctreeShape::getVolume() const
{
	SURGSIM_FAILURE() << "OctreeShape::getVolume not implemented";
	return 0.0;
}

Vector3d OctreeShape::getCenter() const
{
	return Vector3d::Zero();
}

Matrix33d OctreeShape::getSecondMomentOfVolume() const
{
	SURGSIM_FAILURE() << "OctreeShape::getSecondMomentOfVolume not implemented";
	return Matrix33d::Zero();
}

std::shared_ptr<OctreeShape::NodeType> OctreeShape::getOctree()
{
	return m_rootNode;
}

void OctreeShape::setOctree(std::shared_ptr<SurgSim::Framework::Asset> node)
{
	SURGSIM_ASSERT(node != nullptr) << "Tried to set the shape with a nullptr";
	auto octreeNode = std::dynamic_pointer_cast<NodeType>(node);
	SURGSIM_ASSERT(octreeNode != nullptr)
			<< "OctreeShape needs OctreeNode<SurgSim::DataStructures::EmptyData> but received " << node->getClassName();
	SURGSIM_ASSERT(isValid(octreeNode))
			<< "OctreeShape was passed an invalid Octree.";
	m_rootNode = octreeNode;
}

bool OctreeShape::isValid(std::shared_ptr<NodeType> node) const
{
	return (nullptr != node) && (node->getBoundingBox().sizes().minCoeff() >= 0);
}

bool OctreeShape::isValid() const
{
	return isValid(m_rootNode);
}

}; // namespace Math
}; // namespace SurgSim
