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

#ifndef SURGSIM_GRAPHICS_MESHREPRESENTATION_H
#define SURGSIM_GRAPHICS_MESHREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{
namespace Graphics
{
class Mesh;

/// Graphics representation of a mesh, can be initialized from a Mesh structure
class MeshRepresentation : public virtual Representation
{
public:

	enum UpdateOption
	{
		UPDATE_OPTION_NONE = 0,
		UPDATE_OPTION_VERTICES = 0x01,
		UPDATE_OPTION_COLORS = 0x02,
		UPDATE_OPTION_TEXTURES = 0x04,
		UPDATE_OPTION_TRIANGLES = 0x08,
		UPDATE_OPTION_ALL = UPDATE_OPTION_VERTICES | UPDATE_OPTION_COLORS |
							UPDATE_OPTION_TEXTURES | UPDATE_OPTION_TRIANGLES
	};

	/// Constructor.
	/// \param	name	The name of the representation.
	explicit MeshRepresentation(const std::string& name) : Representation(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(MeshRepresentation, std::string, Filename, getFilename, setFilename);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(MeshRepresentation, bool, DrawAsWireFrame,
										  drawAsWireFrame, setDrawAsWireFrame);
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(MeshRepresentation, int, UpdateOptions, getUpdateOptions, setUpdateOptions);
	}

	virtual ~MeshRepresentation() {}

	/// Gets the mesh.
	/// \return	The mesh.
	virtual std::shared_ptr<Mesh> getMesh() = 0;

	/// Set loading filename
	/// \param filename	The filename to load
	/// \note The mesh will be loaded right after the file name is set,
	///       if 'filename' indicates a file containing a valid mesh.
	/// \note If the valid file contains an empty mesh, i.e. no vertex is specified in that file,
	///       an empty mesh will be held.
	virtual void setFilename(std::string filename) = 0;

	/// Get the file name of the external file which contains the triangle mesh.
	/// \return File name of the external file which contains the triangle mesh.
	virtual std::string getFilename() const = 0;

	/// Sets the mesh to render as a wire frame.
	/// \param	val	true if this mesh should be rendered as a wireframe.
	virtual void setDrawAsWireFrame(bool val) = 0;

	/// Return if the mesh is rendered as a wire frame.
	/// \return	True if this mesh is rendered as a wireframe; false if not.
	virtual bool drawAsWireFrame() const = 0;

	/// Sets the structures that are expected to change during the lifetime of the mesh, these will be updated
	/// every frame, independent of a structural change in the mesh. UPDATE_OPTION_VERTICES is set in the constructor
	/// as a default value.
	/// \param	val	Boolean or expression of UpdateOption enum.
	virtual void setUpdateOptions(int val) = 0;

	/// Gets update options for this mesh.
	/// \return	The update options.
	virtual int getUpdateOptions() const = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_MESHREPRESENTATION_H