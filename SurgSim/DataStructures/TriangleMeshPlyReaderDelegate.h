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

/// Implementation of PlyReaderDelegate for simple triangle meshes
class TriangleMeshPlyReaderDelegate : public PlyReaderDelegate
{
public:

	/// The Mesh Type
	typedef TriangleMesh<void, void, void> MeshType;

	/// Default constructor.
	TriangleMeshPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	TriangleMeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh);

	/// Gets the mesh.
	/// \return The mesh.
	std::shared_ptr<TriangleMesh<void, void, void>> getMesh();

	/// Registers the delegate with the reader, overridden from \sa PlyReaderDelegate.
	/// \return true if it succeeds, false otherwise.
	virtual bool registerDelegate(PlyReader* reader) override;

	/// Check whether this file is acceptable to the delegate, overridden from \sa PlyReaderDelegate.
	/// \return true if it succeeds, false otherwise.
	virtual bool fileIsAcceptable(const PlyReader& reader) override;

	/// Callback function, begin the processing of vertices.
	/// \param elementName Name of the element.
	/// \param vertexCount Number of vertices.
	/// \return memory for vertex data to the reader.
	void* beginVertices(const std::string& elementName, size_t vertexCount);

	/// Callback function to process one vertex.
	/// \param elementName Name of the element.
	void processVertex(const std::string& elementName);

	/// Callback function to finalize processing of vertices.
	/// \param elementName Name of the element.
	void endVertices(const std::string& elementName);

	/// Callback function, begin the processing of faces.
	/// \param elementName Name of the element.
	/// \param vertexCount Number of faces.
	/// \return memory for face data to the reader.
	void* beginFaces(const std::string& elementName, size_t faceCount);

	/// Callback function to process one face.
	/// \param elementName Name of the element.
	void processFace(const std::string& elementName);

	/// Callback function to finalize processing of faces.
	/// \param elementName Name of the element.
	void endFaces(const std::string& elementName);


private:
	bool m_isAcceptable;

	/// Internal structure, the receiver for data from the "vertex" element
	struct VertexData
	{
		double x;
		double y;
		double z;
		int64_t overrun; ///< Used to check for buffer overruns
	} vertexData;

	/// Internal structure, the received for data from the "face" element
	struct FaceData
	{
		unsigned int edgeCount;
		unsigned int* indices;
		int64_t overrun; ///< Used to check for buffer overruns
	} faceData;

	/// The mesh that will be created
	std::shared_ptr<MeshType> m_mesh;

	// Preallocated index array to receive data for the faces
	std::array<unsigned int, 3> m_indices;
};

}
}

#endif
