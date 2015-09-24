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

#ifndef SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H
#define SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Macros.h"

namespace SurgSim
{

namespace Framework
{
class Component;
class Asset;
}

namespace Graphics
{
class MeshRepresentation;
}

namespace Physics
{
class DeformableRepresentation;
}

namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(TransferPhysicsToGraphicsMeshBehavior);

/// Behavior to copy positions of a PhysicsRepresentation to a GraphicsMesh.
/// By default the behavior will take each node and copy the position of that node to the vertex with the corresponding
/// index. If an index map is available, for each pair in the index map it will take the nodeId from the first
/// member of the pair and copy it to the vertex with the id of the second member of the pair.
/// The index map can be computed from meshes given to this behavior or precomputed via other means.
class TransferPhysicsToGraphicsMeshBehavior : public Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit TransferPhysicsToGraphicsMeshBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior);

	/// Set the representation from which the positions are from
	/// \param source The physics representation
	void setSource(const std::shared_ptr<Framework::Component>& source);

	/// Set the representation which will receive the positions
	/// \param target The Graphics Mesh representation
	void setTarget(const std::shared_ptr<Framework::Component>& target);

	/// Get the Physics representation which sends the positions
	/// \return The Physics representation which produces positions.
	std::shared_ptr<Physics::DeformableRepresentation> getSource() const;

	/// Get the Graphics representation which receives the positions
	/// \return The Graphics Mesh representation which receives positions.
	std::shared_ptr<Graphics::MeshRepresentation> getTarget() const;

	/// Generate a mapping, for each point in source find the points target that coincide
	/// \param source Source of search
	/// \param target Target of search
	/// \note source and target need to be triangle meshes
	void setIndexMap(
		const std::shared_ptr<DataStructures::TriangleMeshPlain>& source,
		const std::shared_ptr<DataStructures::TriangleMeshPlain>& target);

	/// Generate a mapping, for each point in source find the points target that coincide
	/// \param sourceFile filename for source mesh
	/// \param targetFile filename for target mesh
	/// \note sourceFile and targetFile need to be valid ply files
	void setIndexMap(const std::string& sourceFile, const std::string& targetFile);

	/// Set the mapping to be used if not empty. first index is the node, second index is the vertex.
	/// for all pairs copy node position to given vertex index.
	/// \param indexMap mapping to be used for this mesh
	void setIndexMap(const std::vector<std::pair<size_t, size_t>>& indexMap);

	/// \return the current mapping
	const std::vector<std::pair<size_t, size_t>> getIndexMap() const;

	void update(double dt) override;

private:
	bool doInitialize() override;
	bool doWakeUp() override;


	void setIndexMap(const std::pair<std::string, std::string>& fileName);

	void setIndexMap(const std::pair<std::shared_ptr<Framework::Asset>, std::shared_ptr<Framework::Asset>>& meshes);

	/// The DeformableRepresentation from which the Ode state comes.
	std::shared_ptr<Physics::DeformableRepresentation> m_source;

	/// The Graphics Mesh Representation to which the vertices' positions are set.
	std::shared_ptr<Graphics::MeshRepresentation> m_target;

	/// The mapping to be used if not empty.
	std::vector<std::pair<size_t, size_t>> m_indexMap;
};

/// Generate a mapping, for each point in source find the points target that coincide
/// \param source Source of search
/// \param target Target of search
/// \note source and target need to be triangle meshes
/// \return the map of matching points, pair.first are all points from source, pair.second will be points in target
std::vector<std::pair<size_t, size_t>> generateIndexMap(
										const std::shared_ptr<DataStructures::TriangleMeshPlain>& source,
										const std::shared_ptr<DataStructures::TriangleMeshPlain>& target);

};  // namespace Blocks
};  // namespace SurgSim

#endif  // SURGSIM_BLOCKS_TRANSFERPHYSICSTOGRAPHICSMESHBEHAVIOR_H