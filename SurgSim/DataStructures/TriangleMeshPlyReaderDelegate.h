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

#ifndef SURGSIM_DATASTRUCTURES_TRIANGLEMESHPLYREADERDELEGATE_H
#define SURGSIM_DATASTRUCTURES_TRIANGLEMESHPLYREADERDELEGATE_H

#include <array>

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMesh.h"

namespace SurgSim
{
namespace DataStructures
{
class TriangleMeshPlyReaderDelegate : public PlyReaderDelegate
{
public:

	typedef TriangleMesh<void, void, void> MeshType;

	TriangleMeshPlyReaderDelegate();

	TriangleMeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh);

	std::shared_ptr<TriangleMesh<void, void, void>> getMesh();

	virtual bool registerDelegate(PlyReader* reader) override;

	virtual bool fileIsAcceptable(const PlyReader& reader) override;

	void* beginVertices(const std::string& elementName, size_t vertices);

	void processVertex(const std::string& elementName);

	void endVertices(const std::string& elementName);

	void* beginFaces(const std::string& elementName, size_t faces);

	void processFace(const std::string& elementName);

	void endFaces(const std::string& elementName);

	std::shared_ptr<MeshType> mesh;

private:
	bool m_isAcceptable;

	struct VertexData
	{
		double x;
		double y;
		double z;
		int64_t overrun;
	} vertexData;

	struct FaceData
	{
		unsigned int edgeCount;
		unsigned int* indices;
		int64_t overrun;
	} faceData;

	std::shared_ptr<MeshType> m_mesh;
	std::vector<SurgSim::Math::Vector3d> m_vertexPositions;

	// Preallocate index array for
	std::array<unsigned int, 3> m_indices;

};

}
}

#endif
