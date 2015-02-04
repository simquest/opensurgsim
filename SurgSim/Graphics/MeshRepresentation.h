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

#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/Representation.h"


namespace SurgSim
{

namespace Graphics
{

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
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(MeshRepresentation, std::shared_ptr<SurgSim::Framework::Asset>, Mesh, getMesh,
			setMesh);

		// Enables the alternative use of the mesh file instead of the actual mesh object
		DecoderType decoder = std::bind(&MeshRepresentation::loadMesh, this,
										std::bind(&YAML::Node::as<std::string>, std::placeholders::_1));
		setDecoder("MeshFileName", decoder);

		SetterType setter = std::bind(&MeshRepresentation::loadMesh, this,
									  std::bind(SurgSim::Framework::convert<std::string>, std::placeholders::_1));
		setSetter("MeshFileName", setter);

		SURGSIM_ADD_SERIALIZABLE_PROPERTY(MeshRepresentation, int, UpdateOptions, getUpdateOptions, setUpdateOptions);
	}

	/// Destructor
	virtual ~MeshRepresentation() {}

	/// Gets the mesh.
	/// \return	The mesh.
	virtual std::shared_ptr<Mesh> getMesh() = 0;

	/// Sets the mesh.
	/// \param mesh the graphics mesh to be used
	virtual void setMesh(std::shared_ptr<SurgSim::Framework::Asset> mesh) = 0;

	/// Convenience function to trigger the load of the mesh with the given filename
	/// \param	fileName name of the file to be loaded
	virtual void loadMesh(const std::string& fileName) = 0;

	/// Sets the shape of the representation
	/// param shape the shape of this representation
	/// \note only shapes of type MeshShape are supported
	virtual void setShape(std::shared_ptr<SurgSim::Math::Shape> shape) = 0;

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
