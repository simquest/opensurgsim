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

#ifndef SURGSIM_GRAPHICS_MESHPLYREADERDELEGATE_H
#define SURGSIM_GRAPHICS_MESHPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/PlyReaderDelegate.h"
#include "SurgSim/Graphics/Mesh.h"

namespace SurgSim
{
namespace Graphics
{

/// Implementation of PlyReaderDelegate for graphicsmeshes
class MeshPlyReaderDelegate : public SurgSim::DataStructures::PlyReaderDelegate
{
public:

	/// The Mesh Type
	typedef SurgSim::Graphics::Mesh MeshType;

	/// Default constructor.
	MeshPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	explicit MeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh);

	/// Gets the mesh.
	/// \return The mesh.
	std::shared_ptr<Mesh> getMesh();

	/// Registers the delegate with the reader, overridden from \sa PlyReaderDelegate.
	/// \param reader The reader that should be used.
	/// \return true if it succeeds, false otherwise.
	virtual bool registerDelegate(SurgSim::DataStructures::PlyReader* reader) override;

	/// Check whether this file is acceptable to the delegate, overridden from \sa PlyReaderDelegate.
	/// \param reader The reader that should be used.
	/// \return true if it succeeds, false otherwise.
	virtual bool fileIsAcceptable(const SurgSim::DataStructures::PlyReader& reader) override;

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
	/// \param faceCount Number of faces.
	/// \return memory for face data to the reader.
	void* beginFaces(const std::string& elementName, size_t faceCount);

	/// Callback function to process one face.
	/// \param elementName Name of the element.
	void processFace(const std::string& elementName);

	/// Callback function to finalize processing of faces.
	/// \param elementName Name of the element.
	void endFaces(const std::string& elementName);


private:
	/// Internal structure, the receiver for data from the "vertex" element
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
	struct FaceData
	{
		unsigned int edgeCount;
		unsigned int* indices;
		int64_t overrun; ///< Used to check for buffer overruns
	} m_faceData;

	/// The mesh that will be created
	std::shared_ptr<MeshType> m_mesh;

	// Preallocated index array to receive data for the faces
	std::array<size_t, 3> m_indices;

	bool m_hasNormals;
};

}
}

#endif
