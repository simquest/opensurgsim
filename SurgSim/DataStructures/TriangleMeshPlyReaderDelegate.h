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
#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/PlyReaderDelegate.h"

namespace SurgSim
{
namespace DataStructures
{

/// Implementation of PlyReaderDelegate for simple triangle meshes
template <class M>
class TriangleMeshPlyReaderDelegate : public PlyReaderDelegate
{
public:

	typedef M MeshType;

	/// Default constructor.
	TriangleMeshPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	explicit TriangleMeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh);

	/// Gets the mesh.
	/// \return The mesh.
	std::shared_ptr<MeshType> getMesh();

	/// Registers the delegate with the reader, overridden from \sa PlyReaderDelegate.
	/// \param reader The reader that should be used.
	/// \return true if it succeeds, false otherwise.
	bool registerDelegate(PlyReader* reader) override;

	/// Check whether this file is acceptable to the delegate, overridden from \sa PlyReaderDelegate.
	/// \param reader The reader that should be used.
	/// \return true if it succeeds, false otherwise.
	bool fileIsAcceptable(const PlyReader& reader) override;

	/// Callback function, begin the processing of vertices.
	/// \param elementName Name of the element.
	/// \param vertexCount Number of vertices.
	/// \return memory for vertex data to the reader.
	void* beginVertices(const std::string& elementName, size_t vertexCount);

	/// Callback function to process one vertex.
	/// \param elementName Name of the element.
	virtual void processVertex(const std::string& elementName);

	/// Callback function to finalize processing of vertices.
	/// \param elementName Name of the element.
	void endVertices(const std::string& elementName);

	/// Callback function, begin the processing of faces.
	/// \param elementName Name of the element.
	/// \param faceCount Number of faces.
	/// \return memory for face data to the reader.
	void* beginFaces(const std::string& elementName, size_t faceCount);

	/// Callback function to process one face.
	/// \param elementName Name of the element.
	void processFace(const std::string& elementName);

	/// Callback function to finalize processing of faces.
	/// \param elementName Name of the element.
	void endFaces(const std::string& elementName);

	void* beginEdges(const std::string& elementName, size_t edgeCount);

	void processEdge(const std::string& elementName);

	void endEdges(const std::string& elementName);

	/// Callback function to finalize processing of the mesh
	void endFile();

protected:

	/// \return true if s/t coordinates where found in the ply file on registration.
	bool hasTextureCoordinates();

	/// Internal structure, the receiver for data from the "vertex" element
	/// Provide space for standard ply vertex data, x/y/z and s/t
	struct VertexData
	{
		double x;
		double y;
		double z;
		int64_t overrun1; ///< Used to check for buffer overruns
		double s;
		double t;
		int64_t overrun2; ///< Used to check for buffer overruns
	} m_vertexData;

	/// Internal structure, the received for data from the "face" element
	struct ListData
	{
		unsigned int count;
		unsigned int* indices;
		int64_t overrun; ///< Used to check for buffer overruns
	} m_listData;

	/// The mesh that will be created
	std::shared_ptr<MeshType> m_mesh;

	// Statically allocated index array to receive data for the faces
	std::array<size_t, 3> m_indices;
	std::array<size_t, 2> m_edges;

private:
	/// Set to true if s/t coordinates are found in the .ply file
	bool m_hasTextureCoordinates;

	/// Set to true if faces are found in the .ply file
	bool m_hasFaces;
};


};
};

#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate-inl.h"

#endif
